`include "riscv_define.v"

module ROB #(
    parameter ROB_SIZE      = 32,
    parameter ROB_IDX_WIDTH = 5   // log2(ROB_SIZE)
)(
    input  wire                      clk,
    input  wire                      rst_n,

    // ======================
    // 1) 分配接口（2 发射）
    // ======================
    input  wire                      alloc0_valid,
    input  wire [`INST_ADDR_WIDTH-1:0] alloc0_pc,
    input  wire                      alloc0_has_dest,
    input  wire                      alloc0_is_float,     // 1: 浮点寄存器；0: 整数寄存器
    input  wire [`REG_ADDR_WIDTH-1:0] alloc0_arch_rd,
    input  wire [`REG_ADDR_WIDTH-1:0] alloc0_new_preg,
    input  wire [`REG_ADDR_WIDTH-1:0] alloc0_old_preg,

    output wire                      alloc0_ready,
    output wire [ROB_IDX_WIDTH-1:0]  alloc0_rob_idx,

    input  wire                      alloc1_valid,
    input  wire [`INST_ADDR_WIDTH-1:0] alloc1_pc,
    input  wire                      alloc1_has_dest,
    input  wire                      alloc1_is_float,
    input  wire [`REG_ADDR_WIDTH-1:0] alloc1_arch_rd,
    input  wire [`REG_ADDR_WIDTH-1:0] alloc1_new_preg,
    input  wire [`REG_ADDR_WIDTH-1:0] alloc1_old_preg,

    output wire                      alloc1_ready,
    output wire [ROB_IDX_WIDTH-1:0]  alloc1_rob_idx,

    // ======================
    // 2) 写回接口（2 写回）
    //    如果有 #FU_Finished >= 2，需要仲裁
    // ======================
    input  wire                      wb0_valid,
    input  wire [ROB_IDX_WIDTH-1:0]  wb0_rob_idx,
    input  wire [`DATA_WIDTH-1:0]    wb0_value,
    input  wire                      wb0_exception,

    input  wire                      wb1_valid,
    input  wire [ROB_IDX_WIDTH-1:0]  wb1_rob_idx,
    input  wire [`DATA_WIDTH-1:0]    wb1_value,
    input  wire                      wb1_exception,

    // ======================
    // 3) 提交接口（2 提交）
    // ======================
    output reg                       commit0_valid,
    output reg  [`INST_ADDR_WIDTH-1:0] commit0_pc,
    output reg                       commit0_has_dest,
    output reg                       commit0_is_float,
    output reg  [`REG_ADDR_WIDTH-1:0] commit0_arch_rd,
    output reg  [`REG_ADDR_WIDTH-1:0] commit0_new_preg,
    output reg  [`REG_ADDR_WIDTH-1:0] commit0_old_preg,
    output reg  [`DATA_WIDTH-1:0]     commit0_value,
    output reg                       commit0_exception,

    output reg                       commit1_valid,
    output reg  [`INST_ADDR_WIDTH-1:0] commit1_pc,
    output reg                       commit1_has_dest,
    output reg                       commit1_is_float,
    output reg  [`REG_ADDR_WIDTH-1:0] commit1_arch_rd,
    output reg  [`REG_ADDR_WIDTH-1:0] commit1_new_preg,
    output reg  [`REG_ADDR_WIDTH-1:0] commit1_old_preg,
    output reg  [`DATA_WIDTH-1:0]     commit1_value,
    output reg                       commit1_exception,

    // ======================
    // 4) Flush 接口
    // ======================
    input  wire                      flush,
    output wire                      rob_empty
);

    // ======================================================
    // 1. ROB entry 定义
    // ======================================================

    // Entry:
    //  valid: 是否有效
    //  ready: 是否已写回
    //  pc
    //  has_dest: 是否有寄存器写入
    //  is_float: 目标寄存器类型（浮点/整数）
    //  arch_rd: 目标架构寄存器号
    //  new_preg: 重命名后物理寄存器号
    //  old_preg: 重命名前物理寄存器号
    //  value: 写回值
    //  exception: 是否有异常

    reg               entry_valid        [0:ROB_SIZE-1];
    reg               entry_ready        [0:ROB_SIZE-1];

    reg [`INST_ADDR_WIDTH-1:0]  entry_pc         [0:ROB_SIZE-1];
    reg                         entry_has_dest   [0:ROB_SIZE-1];
    reg                         entry_is_float   [0:ROB_SIZE-1];
    reg [`REG_ADDR_WIDTH-1:0]   entry_arch_rd    [0:ROB_SIZE-1];
    reg [`REG_ADDR_WIDTH-1:0]   entry_new_preg   [0:ROB_SIZE-1];
    reg [`REG_ADDR_WIDTH-1:0]   entry_old_preg   [0:ROB_SIZE-1];
    reg [`DATA_WIDTH-1:0]       entry_value      [0:ROB_SIZE-1];
    reg                         entry_exception  [0:ROB_SIZE-1];

    // ======================================================
    // 2. head / tail 指针 & 计数
    // ======================================================

    reg [ROB_IDX_WIDTH-1:0] head;   // 指向下一个提交的 entry
    reg [ROB_IDX_WIDTH-1:0] tail;   // 指向下一个空闲的 entry
    reg [ROB_IDX_WIDTH:0]   count;  // Valid: 0 ~ ROB_SIZE

    wire rob_full;
    assign rob_full  = (count == ROB_SIZE);
    assign rob_empty = (count == 0);

    // 指针 +1（Rolling形式 ROB_SIZE - 1 -> 0）
    function [ROB_IDX_WIDTH-1:0] ptr_inc;
        input [ROB_IDX_WIDTH-1:0] ptr;
        begin
            if (ptr == ROB_SIZE-1)
                ptr_inc = {ROB_IDX_WIDTH{1'b0}};
            else
                ptr_inc = ptr + 1'b1;
        end
    endfunction

    // ======================================================
    // 3. 分配握手逻辑（2 发射）
    //    根据当前空位数，决定 alloc0 / alloc1 是否能进 ROB
    // ======================================================

    wire [ROB_IDX_WIDTH:0] free_slots = ROB_SIZE - count;

    // 是否有足够空位分配
    wire alloc0_can = (free_slots >= 1);
    // 如果 alloc0 也要进，则 alloc1 需要至少 2 个空位（一个已经给了0）；反之只要 1 个空位
    wire alloc1_can = (free_slots >= (alloc0_valid ? 2 : 1));

    // 是否进行分配
    wire do_alloc0 = alloc0_valid && alloc0_can;
    wire do_alloc1 = alloc1_valid && alloc1_can;

    assign alloc0_ready = alloc0_can;
    assign alloc1_ready = alloc1_can;

    // 分配的 ROB idx
    assign alloc0_rob_idx = tail;
    assign alloc1_rob_idx = (do_alloc0) ? ptr_inc(tail) : tail;

    // ======================================================
    // 4. 写回 / 提交 / Flush 主时序逻辑
    // ======================================================

    integer i; // Entry index

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            head  <= {ROB_IDX_WIDTH{1'b0}};
            tail  <= {ROB_IDX_WIDTH{1'b0}};
            count <= { (ROB_IDX_WIDTH+1){1'b0} };

            commit0_valid      <= 1'b0;
            commit1_valid      <= 1'b0;
            commit0_exception  <= 1'b0;
            commit1_exception  <= 1'b0;

            for (i = 0; i < ROB_SIZE; i = i + 1) begin
                entry_valid[i]       <= 1'b0;
                entry_ready[i]       <= 1'b0;
                entry_pc[i]          <= {`INST_ADDR_WIDTH{1'b0}};
                entry_has_dest[i]    <= 1'b0;
                entry_is_float[i]    <= 1'b0;
                entry_arch_rd[i]     <= {`REG_ADDR_WIDTH{1'b0}};
                entry_new_preg[i]    <= {`REG_ADDR_WIDTH{1'b0}};
                entry_old_preg[i]    <= {`REG_ADDR_WIDTH{1'b0}};
                entry_value[i]       <= {`DATA_WIDTH{1'b0}};
                entry_exception[i]   <= 1'b0;
            end
        end else begin
            // 默认情况：本周期不提交
            commit0_valid     <= 1'b0;
            commit1_valid     <= 1'b0;
            commit0_exception <= 1'b0;
            commit1_exception <= 1'b0;

            if (flush) begin
                // 需要刷新ROB，直接清空整个 ROB
                head  <= {ROB_IDX_WIDTH{1'b0}};
                tail  <= {ROB_IDX_WIDTH{1'b0}};
                count <= { (ROB_IDX_WIDTH+1){1'b0} };
                for (i = 0; i < ROB_SIZE; i = i + 1) begin
                    entry_valid[i]      <= 1'b0;
                    entry_ready[i]      <= 1'b0;
                    entry_exception[i]  <= 1'b0;
                end
            end else begin
                // --------------------------------
                // 4.1 写回阶段（2 写回）
                // --------------------------------
                // 注意：wb1 会覆盖 wb0 对同一 entry 的写入（很少发生）
                if (wb0_valid) begin
                    entry_value[wb0_rob_idx]     <= wb0_value;
                    entry_ready[wb0_rob_idx]     <= 1'b1;
                    entry_exception[wb0_rob_idx] <= wb0_exception;
                end

                if (wb1_valid) begin
                    entry_value[wb1_rob_idx]     <= wb1_value;
                    entry_ready[wb1_rob_idx]     <= 1'b1;
                    entry_exception[wb1_rob_idx] <= wb1_exception;
                end

                // --------------------------------
                // 4.2 分配阶段（2 发射）
                // --------------------------------
                // 利用临时指针和计数，避免顺序依赖
                reg [ROB_IDX_WIDTH-1:0] tail_next;
                reg [ROB_IDX_WIDTH:0]   count_next;

                tail_next  = tail;
                count_next = count;

                if (do_alloc0) begin
                    entry_valid[tail_next]      <= 1'b1;
                    entry_ready[tail_next]      <= 1'b0;
                    entry_pc[tail_next]         <= alloc0_pc;
                    entry_has_dest[tail_next]   <= alloc0_has_dest;
                    entry_is_float[tail_next]   <= alloc0_is_float;
                    entry_arch_rd[tail_next]    <= alloc0_arch_rd;
                    entry_new_preg[tail_next]   <= alloc0_new_preg;
                    entry_old_preg[tail_next]   <= alloc0_old_preg;
                    entry_value[tail_next]      <= {`DATA_WIDTH{1'b0}};
                    entry_exception[tail_next]  <= 1'b0;

                    tail_next  = ptr_inc(tail_next);
                    count_next = count_next + 1'b1;
                end

                if (do_alloc1) begin
                    entry_valid[tail_next]      <= 1'b1;
                    entry_ready[tail_next]      <= 1'b0;
                    entry_pc[tail_next]         <= alloc1_pc;
                    entry_has_dest[tail_next]   <= alloc1_has_dest;
                    entry_is_float[tail_next]   <= alloc1_is_float;
                    entry_arch_rd[tail_next]    <= alloc1_arch_rd;
                    entry_new_preg[tail_next]   <= alloc1_new_preg;
                    entry_old_preg[tail_next]   <= alloc1_old_preg;
                    entry_value[tail_next]      <= {`DATA_WIDTH{1'b0}};
                    entry_exception[tail_next]  <= 1'b0;

                    tail_next  = ptr_inc(tail_next);
                    count_next = count_next + 1'b1;
                end

                tail  <= tail_next;
                count <= count_next;

                // --------------------------------
                // 4.3 提交阶段（2 提交）
                // --------------------------------
                reg [ROB_IDX_WIDTH-1:0] head_idx0;
                reg [ROB_IDX_WIDTH-1:0] head_idx1;
                reg [1:0]               commit_num; // 0, 1, 2 提交指令数

                head_idx0  = head;
                head_idx1  = ptr_inc(head);
                commit_num = 2'd0;

                // can_commit0: head entry 有效且 ready
                if (!rob_empty && entry_valid[head_idx0] && entry_ready[head_idx0]) begin
                    // 提交第 0 条
                    commit0_valid      <= 1'b1;
                    commit0_pc         <= entry_pc[head_idx0];
                    commit0_has_dest   <= entry_has_dest[head_idx0];
                    commit0_is_float   <= entry_is_float[head_idx0];
                    commit0_arch_rd    <= entry_arch_rd[head_idx0];
                    commit0_new_preg   <= entry_new_preg[head_idx0];
                    commit0_old_preg   <= entry_old_preg[head_idx0];
                    commit0_value      <= entry_value[head_idx0];
                    commit0_exception  <= entry_exception[head_idx0];

                    // 清空 head entry
                    entry_valid[head_idx0]      <= 1'b0;
                    entry_ready[head_idx0]      <= 1'b0;
                    entry_exception[head_idx0]  <= 1'b0;

                    commit_num = commit_num + 2'd1;

                    // 如果 head 没异常，且还有第二条 ready，可以提交第 1 条
                    if (!entry_exception[head_idx0] &&
                        (count_next > 1) &&
                        entry_valid[head_idx1] && entry_ready[head_idx1] &&
                        !entry_exception[head_idx1]) begin // todo: entry_exception[head_idx1] 是否有要判断

                        commit1_valid      <= 1'b1;
                        commit1_pc         <= entry_pc[head_idx1];
                        commit1_has_dest   <= entry_has_dest[head_idx1];
                        commit1_is_float   <= entry_is_float[head_idx1];
                        commit1_arch_rd    <= entry_arch_rd[head_idx1];
                        commit1_new_preg   <= entry_new_preg[head_idx1];
                        commit1_old_preg   <= entry_old_preg[head_idx1];
                        commit1_value      <= entry_value[head_idx1];
                        commit1_exception  <= entry_exception[head_idx1];

                        entry_valid[head_idx1]      <= 1'b0;
                        entry_ready[head_idx1]      <= 1'b0;
                        entry_exception[head_idx1]  <= 1'b0;

                        commit_num = commit_num + 2'd1;
                    end
                end

                // 根据 commit_num 更新 head & count
                case (commit_num)
                    2'd0: begin
                        // 不变
                    end
                    2'd1: begin
                        head  <= ptr_inc(head);
                        count <= count_next - 1'b1;
                    end
                    2'd2: begin
                        head  <= ptr_inc(ptr_inc(head));
                        count <= count_next - 2'd2;
                    end
                    default: ; // 3 should not happen
                endcase

            end // !flush
        end // !rst
    end // always

endmodule

// TODO: Input: logical register addr
module Rename(

);



endmodule