`include "riscv_define.v"

// ============================================================
// Reorder Buffer (ROB)
// ============================================================
module ROB #(
    parameter ROB_SIZE       = `ROB_SIZE,
    parameter ROB_IDX_WIDTH  = `ROB_IDX_WIDTH,
    parameter PREG_IDX_WIDTH = `PREG_IDX_WIDTH
)(
    input  wire                        clk,
    input  wire                        rst_n,

    // Alloc (2 per cycle)
    input  wire                        alloc0_valid,
    input  wire [`INST_ADDR_WIDTH-1:0] alloc0_pc,
    input  wire                        alloc0_has_dest,
    input  wire                        alloc0_is_float,
    input  wire [`REG_ADDR_WIDTH-1:0]  alloc0_arch_rd,
    input  wire [PREG_IDX_WIDTH-1:0]   alloc0_new_preg,
    input  wire [PREG_IDX_WIDTH-1:0]   alloc0_old_preg,
    output wire                        alloc0_ready,
    output wire [ROB_IDX_WIDTH-1:0]    alloc0_rob_idx,

    input  wire                        alloc1_valid,
    input  wire [`INST_ADDR_WIDTH-1:0] alloc1_pc,
    input  wire                        alloc1_has_dest,
    input  wire                        alloc1_is_float,
    input  wire [`REG_ADDR_WIDTH-1:0]  alloc1_arch_rd,
    input  wire [PREG_IDX_WIDTH-1:0]   alloc1_new_preg,
    input  wire [PREG_IDX_WIDTH-1:0]   alloc1_old_preg,
    output wire                        alloc1_ready,
    output wire [ROB_IDX_WIDTH-1:0]    alloc1_rob_idx,

    // Writeback (2 per cycle)
    input  wire                        wb0_valid,
    input  wire [ROB_IDX_WIDTH-1:0]    wb0_rob_idx,
    input  wire [`DATA_WIDTH-1:0]      wb0_value,
    input  wire                        wb0_exception,

    input  wire                        wb1_valid,
    input  wire [ROB_IDX_WIDTH-1:0]    wb1_rob_idx,
    input  wire [`DATA_WIDTH-1:0]      wb1_value,
    input  wire                        wb1_exception,

    // Commit (2 per cycle)
    output reg                         commit0_valid,
    output reg  [ROB_IDX_WIDTH-1:0]    commit0_rob_idx,
    output reg  [`INST_ADDR_WIDTH-1:0] commit0_pc,
    output reg                         commit0_has_dest,
    output reg                         commit0_is_float,
    output reg  [`REG_ADDR_WIDTH-1:0]  commit0_arch_rd,
    output reg  [PREG_IDX_WIDTH-1:0]   commit0_new_preg,
    output reg  [PREG_IDX_WIDTH-1:0]   commit0_old_preg,
    output reg  [`DATA_WIDTH-1:0]      commit0_value,
    output reg                         commit0_exception,

    output reg                         commit1_valid,
    output reg  [ROB_IDX_WIDTH-1:0]    commit1_rob_idx,
    output reg  [`INST_ADDR_WIDTH-1:0] commit1_pc,
    output reg                         commit1_has_dest,
    output reg                         commit1_is_float,
    output reg  [`REG_ADDR_WIDTH-1:0]  commit1_arch_rd,
    output reg  [PREG_IDX_WIDTH-1:0]   commit1_new_preg,
    output reg  [PREG_IDX_WIDTH-1:0]   commit1_old_preg,
    output reg  [`DATA_WIDTH-1:0]      commit1_value,
    output reg                         commit1_exception,

    // Flush
    input  wire                        flush,
    output wire                        rob_empty
);
    reg                        entry_valid       [0:ROB_SIZE-1];
    reg                        entry_ready       [0:ROB_SIZE-1];
    reg [`INST_ADDR_WIDTH-1:0] entry_pc          [0:ROB_SIZE-1];
    reg                        entry_has_dest    [0:ROB_SIZE-1];
    reg                        entry_is_float    [0:ROB_SIZE-1];
    reg [`REG_ADDR_WIDTH-1:0]  entry_arch_rd     [0:ROB_SIZE-1];
    reg [PREG_IDX_WIDTH-1:0]   entry_new_preg    [0:ROB_SIZE-1];
    reg [PREG_IDX_WIDTH-1:0]   entry_old_preg    [0:ROB_SIZE-1];
    reg [`DATA_WIDTH-1:0]      entry_value       [0:ROB_SIZE-1];
    reg                        entry_exception   [0:ROB_SIZE-1];

    reg [ROB_IDX_WIDTH-1:0] head;
    reg [ROB_IDX_WIDTH-1:0] tail;
    reg [ROB_IDX_WIDTH:0]   count;
    reg [ROB_IDX_WIDTH-1:0] head_idx0;
    reg [ROB_IDX_WIDTH-1:0] head_idx1;
    reg [ROB_IDX_WIDTH-1:0] tail_next;
    reg [ROB_IDX_WIDTH:0]   count_next;
    reg [1:0]               commit_num;

    assign rob_empty = (count == 0);

    function [ROB_IDX_WIDTH-1:0] ptr_inc;
        input [ROB_IDX_WIDTH-1:0] ptr;
        begin
            ptr_inc = (ptr == ROB_SIZE-1) ? {ROB_IDX_WIDTH{1'b0}} : ptr + 1'b1;
        end
    endfunction

    wire [ROB_IDX_WIDTH:0] free_slots = ROB_SIZE - count;
    wire alloc0_can = (free_slots >= 1);
    wire alloc1_can = (free_slots >= (alloc0_valid ? 2 : 1));
    wire do_alloc0  = alloc0_valid && alloc0_can;
    wire do_alloc1  = alloc1_valid && alloc1_can;
    assign alloc0_ready   = alloc0_can;
    assign alloc1_ready   = alloc1_can;
    assign alloc0_rob_idx = tail;
    assign alloc1_rob_idx = do_alloc0 ? ptr_inc(tail) : tail;

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            head  <= {ROB_IDX_WIDTH{1'b0}};
            tail  <= {ROB_IDX_WIDTH{1'b0}};
            count <= {(ROB_IDX_WIDTH+1){1'b0}};
            commit0_valid <= 1'b0; commit1_valid <= 1'b0;
            commit0_exception <= 1'b0; commit1_exception <= 1'b0;
            commit0_rob_idx <= {ROB_IDX_WIDTH{1'b0}};
            commit1_rob_idx <= {ROB_IDX_WIDTH{1'b0}};
            for (i=0;i<ROB_SIZE;i=i+1) begin
                entry_valid[i]<=1'b0; entry_ready[i]<=1'b0; entry_exception[i]<=1'b0;
                entry_pc[i]<= {`INST_ADDR_WIDTH{1'b0}}; entry_has_dest[i]<=1'b0; entry_is_float[i]<=1'b0;
                entry_arch_rd[i]<= {`REG_ADDR_WIDTH{1'b0}}; entry_new_preg[i]<= {PREG_IDX_WIDTH{1'b0}};
                entry_old_preg[i]<= {PREG_IDX_WIDTH{1'b0}}; entry_value[i]<= {`DATA_WIDTH{1'b0}};
            end
        end else begin
            commit0_valid <= 1'b0; commit1_valid <= 1'b0;
            commit0_exception <= 1'b0; commit1_exception <= 1'b0;
            if (flush) begin
                head <= {ROB_IDX_WIDTH{1'b0}}; tail <= {ROB_IDX_WIDTH{1'b0}}; count <= {(ROB_IDX_WIDTH+1){1'b0}};
                commit0_rob_idx <= {ROB_IDX_WIDTH{1'b0}};
                commit1_rob_idx <= {ROB_IDX_WIDTH{1'b0}};
                for (i=0;i<ROB_SIZE;i=i+1) begin
                    entry_valid[i]<=1'b0; entry_ready[i]<=1'b0; entry_exception[i]<=1'b0;
                end
            end else begin
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

                head_idx0  = head;
                head_idx1  = ptr_inc(head);
                commit_num = 2'd0;
                if (!rob_empty && entry_valid[head_idx0] && entry_ready[head_idx0]) begin
                    commit0_valid     <= 1'b1;
                    commit0_rob_idx   <= head_idx0;
                    commit0_pc        <= entry_pc[head_idx0];
                    commit0_has_dest  <= entry_has_dest[head_idx0];
                    commit0_is_float  <= entry_is_float[head_idx0];
                    commit0_arch_rd   <= entry_arch_rd[head_idx0];
                    commit0_new_preg  <= entry_new_preg[head_idx0];
                    commit0_old_preg  <= entry_old_preg[head_idx0];
                    commit0_value     <= entry_value[head_idx0];
                    commit0_exception <= entry_exception[head_idx0];
                    entry_valid[head_idx0]     <= 1'b0;
                    entry_ready[head_idx0]     <= 1'b0;
                    entry_exception[head_idx0] <= 1'b0;
                    commit_num = commit_num + 1'b1;
                    if (!entry_exception[head_idx0] && (count_next > 1) &&
                        entry_valid[head_idx1] && entry_ready[head_idx1] && !entry_exception[head_idx1]) begin
                        commit1_valid     <= 1'b1;
                        commit1_rob_idx   <= head_idx1;
                        commit1_pc        <= entry_pc[head_idx1];
                        commit1_has_dest  <= entry_has_dest[head_idx1];
                        commit1_is_float  <= entry_is_float[head_idx1];
                        commit1_arch_rd   <= entry_arch_rd[head_idx1];
                        commit1_new_preg  <= entry_new_preg[head_idx1];
                        commit1_old_preg  <= entry_old_preg[head_idx1];
                        commit1_value     <= entry_value[head_idx1];
                        commit1_exception <= entry_exception[head_idx1];
                        entry_valid[head_idx1]     <= 1'b0;
                        entry_ready[head_idx1]     <= 1'b0;
                        entry_exception[head_idx1] <= 1'b0;
                        commit_num = commit_num + 1'b1;
                    end
                end
                case (commit_num)
                    2'd1: begin head <= ptr_inc(head); count <= count_next - 1'b1; end
                    2'd2: begin head <= ptr_inc(ptr_inc(head)); count <= count_next - 2'd2; end
                    default: ;
                endcase
            end
        end
    end
endmodule

// ============================================================
// Register Rename: logical -> physical regs + ROB bookkeeping
// ============================================================
// TODO: Wire up the mem buffers (store buffer and read buffer) in MEM ALU at commit stage
module RegRename #(
    parameter ARCH_REGS = 32
)(
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        flush,
    input  wire [`ROB_IDX_WIDTH-1:0]   flush_rob_idx,

    // Incoming pre-decode info (2 instructions)
    input  wire [`IF_BATCH_SIZE-1:0]   in_inst_valid,
    input  wire [`INST_ADDR_WIDTH-1:0] in_pc_0,
    input  wire [`INST_ADDR_WIDTH-1:0] in_pc_1,
    input  wire [`INST_WIDTH-1:0]      in_inst_0,
    input  wire [`INST_WIDTH-1:0]      in_inst_1,
    input  wire [1:0]                  in_fu_type_0,
    input  wire [1:0]                  in_fu_type_1,
    input  wire                        in_pred_taken_0,
    input  wire [`INST_ADDR_WIDTH-1:0] in_pred_target_0,
    input  wire [`BP_GHR_BITS-1:0]     in_pred_hist_0,
    input  wire                        in_pred_taken_1,
    input  wire [`INST_ADDR_WIDTH-1:0] in_pred_target_1,
    input  wire [`BP_GHR_BITS-1:0]     in_pred_hist_1,
    input  wire [`REG_ADDR_WIDTH-1:0]  in_rs1_0,
    input  wire [`REG_ADDR_WIDTH-1:0]  in_rs2_0,
    input  wire [`REG_ADDR_WIDTH-1:0]  in_rd_0,
    input  wire [`DATA_WIDTH-1:0]      in_imm_0,
    input  wire                        in_use_imm_0,
    input  wire                        in_rs1_is_fp_0,
    input  wire                        in_rs2_is_fp_0,
    input  wire                        in_rd_is_fp_0,
    input  wire [`REG_ADDR_WIDTH-1:0]  in_rs1_1,
    input  wire [`REG_ADDR_WIDTH-1:0]  in_rs2_1,
    input  wire [`REG_ADDR_WIDTH-1:0]  in_rd_1,
    input  wire [`DATA_WIDTH-1:0]      in_imm_1,
    input  wire                        in_use_imm_1,
    input  wire                        in_rs1_is_fp_1,
    input  wire                        in_rs2_is_fp_1,
    input  wire                        in_rd_is_fp_1,

    // Writeback path into ROB
    input  wire                        wb0_valid,
    input  wire [`ROB_IDX_WIDTH-1:0]   wb0_rob_idx,
    input  wire [`DATA_WIDTH-1:0]      wb0_value,
    input  wire                        wb0_exception,
    input  wire                        wb1_valid,
    input  wire [`ROB_IDX_WIDTH-1:0]   wb1_rob_idx,
    input  wire [`DATA_WIDTH-1:0]      wb1_value,
    input  wire                        wb1_exception,

    // Renamed outputs to dispatch
    output reg  [`IF_BATCH_SIZE-1:0]   out_inst_valid,
    output reg  [`INST_WIDTH-1:0]      out_inst_0,
    output reg  [`INST_WIDTH-1:0]      out_inst_1,
    output reg  [1:0]                  out_fu_type_0,
    output reg  [1:0]                  out_fu_type_1,
    output reg  [`INST_ADDR_WIDTH-1:0] out_pc_0,
    output reg  [`INST_ADDR_WIDTH-1:0] out_pc_1,
    output reg                         out_pred_taken_0,
    output reg [`INST_ADDR_WIDTH-1:0]  out_pred_target_0,
    output reg [`BP_GHR_BITS-1:0]      out_pred_hist_0,
    output reg                         out_pred_taken_1,
    output reg [`INST_ADDR_WIDTH-1:0]  out_pred_target_1,
    output reg [`BP_GHR_BITS-1:0]      out_pred_hist_1,
    output reg  [`REG_ADDR_WIDTH-1:0]  out_rs1_0,
    output reg  [`REG_ADDR_WIDTH-1:0]  out_rs2_0,
    output reg  [`REG_ADDR_WIDTH-1:0]  out_rd_0,
    output reg  [`DATA_WIDTH-1:0]      out_imm_0,
    output reg                         out_use_imm_0,
    output reg                         out_rs1_is_fp_0,
    output reg                         out_rs2_is_fp_0,
    output reg                         out_rd_is_fp_0,
    output reg  [`PREG_IDX_WIDTH-1:0]  out_rs1_preg_0,
    output reg                         out_rs1_preg_valid_0,
    output reg  [`PREG_IDX_WIDTH-1:0]  out_rs2_preg_0,
    output reg                         out_rs2_preg_valid_0,
    output reg  [`PREG_IDX_WIDTH-1:0]  out_rd_preg_0,
    output reg                         out_rd_preg_valid_0,
    output reg  [`PREG_IDX_WIDTH-1:0]  out_old_rd_preg_0,
    output reg                         out_old_rd_preg_valid_0,
    output reg  [`REG_ADDR_WIDTH-1:0]  out_rs1_1,
    output reg  [`REG_ADDR_WIDTH-1:0]  out_rs2_1,
    output reg  [`REG_ADDR_WIDTH-1:0]  out_rd_1,
    output reg  [`DATA_WIDTH-1:0]      out_imm_1,
    output reg                         out_use_imm_1,
    output reg                         out_rs1_is_fp_1,
    output reg                         out_rs2_is_fp_1,
    output reg                         out_rd_is_fp_1,
    output reg  [`ROB_IDX_WIDTH-1:0]   out_rob_idx_0,
    output reg                         out_rob_idx_valid_0,
    output reg  [`ROB_IDX_WIDTH-1:0]   out_rob_idx_1,
    output reg                         out_rob_idx_valid_1,
    output reg  [`PREG_IDX_WIDTH-1:0]  out_rs1_preg_1,
    output reg                         out_rs1_preg_valid_1,
    output reg  [`PREG_IDX_WIDTH-1:0]  out_rs2_preg_1,
    output reg                         out_rs2_preg_valid_1,
    output reg  [`PREG_IDX_WIDTH-1:0]  out_rd_preg_1,
    output reg                         out_rd_preg_valid_1,
    output reg  [`PREG_IDX_WIDTH-1:0]  out_old_rd_preg_1,
    output reg                         out_old_rd_preg_valid_1,

    // ROB commit outputs (pass-through)
    output wire                        commit0_valid,
    output wire [`ROB_IDX_WIDTH-1:0]   commit0_rob_idx,
    output wire [`INST_ADDR_WIDTH-1:0] commit0_pc,
    output wire                        commit0_has_dest,
    output wire                        commit0_is_float,
    output wire [`REG_ADDR_WIDTH-1:0]  commit0_arch_rd,
    output wire [`PREG_IDX_WIDTH-1:0]  commit0_new_preg,
    output wire [`PREG_IDX_WIDTH-1:0]  commit0_old_preg,
    output wire [`DATA_WIDTH-1:0]      commit0_value,
    output wire                        commit0_exception,
    output wire                        commit1_valid,
    output wire [`ROB_IDX_WIDTH-1:0]   commit1_rob_idx,
    output wire [`INST_ADDR_WIDTH-1:0] commit1_pc,
    output wire                        commit1_has_dest,
    output wire                        commit1_is_float,
    output wire [`REG_ADDR_WIDTH-1:0]  commit1_arch_rd,
    output wire [`PREG_IDX_WIDTH-1:0]  commit1_new_preg,
    output wire [`PREG_IDX_WIDTH-1:0]  commit1_old_preg,
    output wire [`DATA_WIDTH-1:0]      commit1_value,
    output wire                        commit1_exception,
    output wire                        rob_empty,

    output wire                        rename_stall
);

localparam INT_PREG_COUNT = `INT_PREG_NUM;
localparam FP_PREG_COUNT  = `FP_PREG_NUM;
localparam [`PREG_IDX_WIDTH:0] INT_FREE_INIT_TAIL = (INT_PREG_COUNT > ARCH_REGS) ? (INT_PREG_COUNT-ARCH_REGS) : 0;
localparam [`PREG_IDX_WIDTH:0] FP_FREE_INIT_TAIL  = (FP_PREG_COUNT > ARCH_REGS) ? (FP_PREG_COUNT-ARCH_REGS) : 0;

// Logical reg -> physical reg maps
reg [`PREG_IDX_WIDTH-1:0] int_map [0:ARCH_REGS-1];
reg                       int_map_valid [0:ARCH_REGS-1];
reg [`PREG_IDX_WIDTH-1:0] fp_map  [0:ARCH_REGS-1];
reg                       fp_map_valid  [0:ARCH_REGS-1];

// Checkpoints per ROB entry (coarse: snapshot before allocation)
reg [`PREG_IDX_WIDTH-1:0] int_map_cp    [0:`ROB_SIZE-1][0:ARCH_REGS-1];
reg                       int_map_v_cp  [0:`ROB_SIZE-1][0:ARCH_REGS-1];
reg [`PREG_IDX_WIDTH-1:0] fp_map_cp     [0:`ROB_SIZE-1][0:ARCH_REGS-1];
reg                       fp_map_v_cp   [0:`ROB_SIZE-1][0:ARCH_REGS-1];
reg [`PREG_IDX_WIDTH:0]   int_head_cp   [0:`ROB_SIZE-1];
reg [`PREG_IDX_WIDTH:0]   int_tail_cp   [0:`ROB_SIZE-1];
reg [`PREG_IDX_WIDTH:0]   int_count_cp  [0:`ROB_SIZE-1];
reg [`PREG_IDX_WIDTH:0]   fp_head_cp    [0:`ROB_SIZE-1];
reg [`PREG_IDX_WIDTH:0]   fp_tail_cp    [0:`ROB_SIZE-1];
reg [`PREG_IDX_WIDTH:0]   fp_count_cp   [0:`ROB_SIZE-1];

// Free lists
reg [`PREG_IDX_WIDTH-1:0] int_free [0:INT_PREG_COUNT-1];
reg [`PREG_IDX_WIDTH-1:0] fp_free  [0:FP_PREG_COUNT-1];
reg [`PREG_IDX_WIDTH:0]   int_free_head, int_free_tail, int_free_count;
reg [`PREG_IDX_WIDTH:0]   fp_free_head,  fp_free_tail,  fp_free_count;
reg [`PREG_IDX_WIDTH:0]   int_tail_next;
reg [`PREG_IDX_WIDTH:0]   fp_tail_next;
reg [`PREG_IDX_WIDTH:0]   int_free_head_next;
reg [`PREG_IDX_WIDTH:0]   fp_free_head_next;
reg [`PREG_IDX_WIDTH:0]   int_free_count_next;
reg [`PREG_IDX_WIDTH:0]   fp_free_count_next;

// ROB alloc wires
wire alloc0_ready;
wire alloc1_ready;
wire [`ROB_IDX_WIDTH-1:0] alloc0_rob_idx;
wire [`ROB_IDX_WIDTH-1:0] alloc1_rob_idx;

// Internal allocation bookkeeping
wire has_dest0 = in_inst_valid[0] && (in_rd_is_fp_0 || (in_rd_0 != {`REG_ADDR_WIDTH{1'b0}}));
wire has_dest1 = in_inst_valid[1] && (in_rd_is_fp_1 || (in_rd_1 != {`REG_ADDR_WIDTH{1'b0}}));
wire rob_req0  = in_inst_valid[0];
wire rob_req1  = in_inst_valid[1];
wire need_int0 = has_dest0 && !in_rd_is_fp_0;
wire need_fp0  = has_dest0 && in_rd_is_fp_0;
wire need_int1 = has_dest1 && !in_rd_is_fp_1;
wire need_fp1  = has_dest1 && in_rd_is_fp_1;

wire [1:0] int_need_count = (need_int0 ? 2'd1 : 2'd0) + (need_int1 ? 2'd1 : 2'd0);
wire [1:0] fp_need_count  = (need_fp0  ? 2'd1 : 2'd0) + (need_fp1  ? 2'd1 : 2'd0);

wire int_can_alloc = (int_free_count >= int_need_count);
wire fp_can_alloc  = (fp_free_count  >= fp_need_count);

assign rename_stall = (!int_can_alloc) || (!fp_can_alloc) ||
                      (rob_req0 && !alloc0_ready) || (rob_req1 && !alloc1_ready);

function [`PREG_IDX_WIDTH-1:0] int_free_peek;
    input [`PREG_IDX_WIDTH:0] head;
    input integer offset;
    integer idx;
    begin
        idx = head + offset;
        if (idx >= INT_PREG_COUNT) idx = idx - INT_PREG_COUNT;
        int_free_peek = int_free[idx[`PREG_IDX_WIDTH-1:0]];
    end
endfunction

function [`PREG_IDX_WIDTH-1:0] fp_free_peek;
    input [`PREG_IDX_WIDTH:0] head;
    input integer offset;
    integer idx;
    begin
        idx = head + offset;
        if (idx >= FP_PREG_COUNT) idx = idx - FP_PREG_COUNT;
        fp_free_peek = fp_free[idx[`PREG_IDX_WIDTH-1:0]];
    end
endfunction

// New/old physical regs chosen this cycle
wire [`PREG_IDX_WIDTH-1:0] new_preg0 = in_rd_is_fp_0 ? fp_free_peek(fp_free_head, 0) : int_free_peek(int_free_head, 0);
wire [`PREG_IDX_WIDTH-1:0] new_preg1_int = need_int0 ? int_free_peek(int_free_head, 1) : int_free_peek(int_free_head, 0);
wire [`PREG_IDX_WIDTH-1:0] new_preg1_fp  = need_fp0  ? fp_free_peek(fp_free_head, 1)  : fp_free_peek(fp_free_head, 0);
wire [`PREG_IDX_WIDTH-1:0] new_preg1 = in_rd_is_fp_1 ? new_preg1_fp : new_preg1_int;

wire [`PREG_IDX_WIDTH-1:0] old_preg0 = has_dest0 ? (in_rd_is_fp_0 ? fp_map[in_rd_0] : int_map[in_rd_0]) : {`PREG_IDX_WIDTH{1'b0}};
wire [`PREG_IDX_WIDTH-1:0] old_preg1_pre = has_dest1 ? (in_rd_is_fp_1 ? fp_map[in_rd_1] : int_map[in_rd_1]) : {`PREG_IDX_WIDTH{1'b0}};
wire same_dest_type = (in_rd_is_fp_1 == in_rd_is_fp_0);
wire same_dest_reg  = (in_rd_1 == in_rd_0);
wire [`PREG_IDX_WIDTH-1:0] old_preg1 = (has_dest1 && has_dest0 && same_dest_type && same_dest_reg) ? new_preg0 : old_preg1_pre;

// Source physicals
wire [`PREG_IDX_WIDTH-1:0] rs1_tag0 = in_rs1_is_fp_0 ? fp_map[in_rs1_0] : int_map[in_rs1_0];
wire [`PREG_IDX_WIDTH-1:0] rs2_tag0 = in_rs2_is_fp_0 ? fp_map[in_rs2_0] : int_map[in_rs2_0];
wire [`PREG_IDX_WIDTH-1:0] rs1_tag1 = (has_dest0 && same_dest_type && (in_rs1_1 == in_rd_0)) ? new_preg0 :
                                      (in_rs1_is_fp_1 ? fp_map[in_rs1_1] : int_map[in_rs1_1]);
wire [`PREG_IDX_WIDTH-1:0] rs2_tag1 = (has_dest0 && same_dest_type && (in_rs2_1 == in_rd_0)) ? new_preg0 :
                                      (in_rs2_is_fp_1 ? fp_map[in_rs2_1] : int_map[in_rs2_1]);

wire rs1_tag_valid0 = in_inst_valid[0] && (in_rs1_is_fp_0 ? fp_map_valid[in_rs1_0] : int_map_valid[in_rs1_0]) &&
                     (!(~in_rs1_is_fp_0 && (in_rs1_0 == {`REG_ADDR_WIDTH{1'b0}})));
wire rs2_tag_valid0 = in_inst_valid[0] && (in_rs2_is_fp_0 ? fp_map_valid[in_rs2_0] : int_map_valid[in_rs2_0]) &&
                     (!(~in_rs2_is_fp_0 && (in_rs2_0 == {`REG_ADDR_WIDTH{1'b0}})));
wire rs1_dep0 = has_dest0 && same_dest_type && (in_rs1_1 == in_rd_0);
wire rs2_dep0 = has_dest0 && same_dest_type && (in_rs2_1 == in_rd_0);
wire rs1_tag_valid1 = in_inst_valid[1] && !rs1_dep0 &&
                     (in_rs1_is_fp_1 ? fp_map_valid[in_rs1_1] : int_map_valid[in_rs1_1]) &&
                     (!(~in_rs1_is_fp_1 && (in_rs1_1 == {`REG_ADDR_WIDTH{1'b0}})));
wire rs2_tag_valid1 = in_inst_valid[1] && !rs2_dep0 &&
                     (in_rs2_is_fp_1 ? fp_map_valid[in_rs2_1] : int_map_valid[in_rs2_1]) &&
                     (!(~in_rs2_is_fp_1 && (in_rs2_1 == {`REG_ADDR_WIDTH{1'b0}})));

wire [`ROB_IDX_WIDTH-1:0] rob_commit0_idx, rob_commit1_idx;
assign commit0_rob_idx = rob_commit0_idx;
assign commit1_rob_idx = rob_commit1_idx;

// Commit push bookkeeping
wire commit0_push_int = commit0_valid && commit0_has_dest && !commit0_is_float && (commit0_arch_rd != {`REG_ADDR_WIDTH{1'b0}});
wire commit1_push_int = commit1_valid && commit1_has_dest && !commit1_is_float && (commit1_arch_rd != {`REG_ADDR_WIDTH{1'b0}});
wire commit0_push_fp  = commit0_valid && commit0_has_dest && commit0_is_float;
wire commit1_push_fp  = commit1_valid && commit1_has_dest && commit1_is_float;
wire [1:0] int_push_count = commit0_push_int + commit1_push_int;
wire [1:0] fp_push_count  = commit0_push_fp  + commit1_push_fp;

// ROB instance
ROB #(.PREG_IDX_WIDTH(`PREG_IDX_WIDTH)) u_rob (
    .clk(clk),
    .rst_n(rst_n),
    .alloc0_valid    (rob_req0 && !flush && int_can_alloc && fp_can_alloc),
    .alloc0_pc       (in_pc_0),
    .alloc0_has_dest (has_dest0),
    .alloc0_is_float (in_rd_is_fp_0),
    .alloc0_arch_rd  (in_rd_0),
    .alloc0_new_preg (new_preg0),
    .alloc0_old_preg (old_preg0),
    .alloc0_ready    (alloc0_ready),
    .alloc0_rob_idx  (alloc0_rob_idx),

    .alloc1_valid    (rob_req1 && !flush && int_can_alloc && fp_can_alloc),
    .alloc1_pc       (in_pc_1),
    .alloc1_has_dest (has_dest1),
    .alloc1_is_float (in_rd_is_fp_1),
    .alloc1_arch_rd  (in_rd_1),
    .alloc1_new_preg (new_preg1),
    .alloc1_old_preg (old_preg1),
    .alloc1_ready    (alloc1_ready),
    .alloc1_rob_idx  (alloc1_rob_idx),

    .wb0_valid       (wb0_valid),
    .wb0_rob_idx     (wb0_rob_idx),
    .wb0_value       (wb0_value),
    .wb0_exception   (wb0_exception),

    .wb1_valid       (wb1_valid),
    .wb1_rob_idx     (wb1_rob_idx),
    .wb1_value       (wb1_value),
    .wb1_exception   (wb1_exception),

    .commit0_valid   (commit0_valid),
    .commit0_rob_idx (rob_commit0_idx),
    .commit0_pc      (commit0_pc),
    .commit0_has_dest(commit0_has_dest),
    .commit0_is_float(commit0_is_float),
    .commit0_arch_rd (commit0_arch_rd),
    .commit0_new_preg(commit0_new_preg),
    .commit0_old_preg(commit0_old_preg),
    .commit0_value   (commit0_value),
    .commit0_exception(commit0_exception),

    .commit1_valid   (commit1_valid),
    .commit1_rob_idx (rob_commit1_idx),
    .commit1_pc      (commit1_pc),
    .commit1_has_dest(commit1_has_dest),
    .commit1_is_float(commit1_is_float),
    .commit1_arch_rd (commit1_arch_rd),
    .commit1_new_preg(commit1_new_preg),
    .commit1_old_preg(commit1_old_preg),
    .commit1_value   (commit1_value),
    .commit1_exception(commit1_exception),

    .flush           (flush),
    .rob_empty       (rob_empty)
);

integer i, j;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        out_inst_valid <= {`IF_BATCH_SIZE{1'b0}};
        out_inst_0 <= {`INST_WIDTH{1'b0}};
        out_inst_1 <= {`INST_WIDTH{1'b0}};
        out_fu_type_0 <= {2{1'b0}};
        out_fu_type_1 <= {2{1'b0}};
        out_pc_0 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pc_1 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pred_taken_0 <= 1'b0;
        out_pred_target_0 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pred_hist_0 <= {`BP_GHR_BITS{1'b0}};
        out_pred_taken_1 <= 1'b0;
        out_pred_target_1 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pred_hist_1 <= {`BP_GHR_BITS{1'b0}};
        out_rs1_0 <= {`REG_ADDR_WIDTH{1'b0}};
        out_rs2_0 <= {`REG_ADDR_WIDTH{1'b0}};
        out_rd_0  <= {`REG_ADDR_WIDTH{1'b0}};
        out_imm_0 <= {`DATA_WIDTH{1'b0}};
        out_use_imm_0 <= 1'b0;
        out_rs1_is_fp_0 <= 1'b0;
        out_rs2_is_fp_0 <= 1'b0;
        out_rd_is_fp_0  <= 1'b0;
        out_rs1_preg_0 <= {`PREG_IDX_WIDTH{1'b0}};
        out_rs1_preg_valid_0 <= 1'b0;
        out_rs2_preg_0 <= {`PREG_IDX_WIDTH{1'b0}};
        out_rs2_preg_valid_0 <= 1'b0;
        out_rd_preg_0 <= {`PREG_IDX_WIDTH{1'b0}};
        out_rd_preg_valid_0 <= 1'b0;
        out_old_rd_preg_0 <= {`PREG_IDX_WIDTH{1'b0}};
        out_old_rd_preg_valid_0 <= 1'b0;
        out_rs1_1 <= {`REG_ADDR_WIDTH{1'b0}};
        out_rs2_1 <= {`REG_ADDR_WIDTH{1'b0}};
        out_rd_1  <= {`REG_ADDR_WIDTH{1'b0}};
        out_imm_1 <= {`DATA_WIDTH{1'b0}};
        out_use_imm_1 <= 1'b0;
        out_rs1_is_fp_1 <= 1'b0;
        out_rs2_is_fp_1 <= 1'b0;
        out_rd_is_fp_1  <= 1'b0;
        out_rob_idx_0 <= {`ROB_IDX_WIDTH{1'b0}};
        out_rob_idx_valid_0 <= 1'b0;
        out_rob_idx_1 <= {`ROB_IDX_WIDTH{1'b0}};
        out_rob_idx_valid_1 <= 1'b0;
        out_rs1_preg_1 <= {`PREG_IDX_WIDTH{1'b0}};
        out_rs1_preg_valid_1 <= 1'b0;
        out_rs2_preg_1 <= {`PREG_IDX_WIDTH{1'b0}};
        out_rs2_preg_valid_1 <= 1'b0;
        out_rd_preg_1 <= {`PREG_IDX_WIDTH{1'b0}};
        out_rd_preg_valid_1 <= 1'b0;
        out_old_rd_preg_1 <= {`PREG_IDX_WIDTH{1'b0}};
        out_old_rd_preg_valid_1 <= 1'b0;
        out_rob_idx_0 <= {`ROB_IDX_WIDTH{1'b0}};
        out_rob_idx_valid_0 <= 1'b0;
        out_rob_idx_1 <= {`ROB_IDX_WIDTH{1'b0}};
        out_rob_idx_valid_1 <= 1'b0;
        int_free_head <= 0;
        int_free_tail <= INT_FREE_INIT_TAIL;
        int_free_count <= INT_FREE_INIT_TAIL;
        fp_free_head  <= 0;
        fp_free_tail  <= FP_FREE_INIT_TAIL;
        fp_free_count <= FP_FREE_INIT_TAIL;
        for (i=0; i<ARCH_REGS; i=i+1) begin
            int_map[i] <= i[`PREG_IDX_WIDTH-1:0];
            fp_map[i]  <= i[`PREG_IDX_WIDTH-1:0];
            int_map_valid[i] <= 1'b1;
            fp_map_valid[i]  <= 1'b1;
        end
        for (i=ARCH_REGS; i<INT_PREG_COUNT; i=i+1) begin
            int_free[i-ARCH_REGS] <= i[`PREG_IDX_WIDTH-1:0];
        end
        for (i=ARCH_REGS; i<FP_PREG_COUNT; i=i+1) begin
            fp_free[i-ARCH_REGS] <= i[`PREG_IDX_WIDTH-1:0];
        end
        for (j=0; j<`ROB_SIZE; j=j+1) begin
            int_head_cp[j]  <= 0;
            int_tail_cp[j]  <= INT_FREE_INIT_TAIL;
            int_count_cp[j] <= INT_FREE_INIT_TAIL;
            fp_head_cp[j]   <= 0;
            fp_tail_cp[j]   <= FP_FREE_INIT_TAIL;
            fp_count_cp[j]  <= FP_FREE_INIT_TAIL;
            for (i=0; i<ARCH_REGS; i=i+1) begin
                int_map_cp[j][i] <= i[`PREG_IDX_WIDTH-1:0];
                fp_map_cp[j][i]  <= i[`PREG_IDX_WIDTH-1:0];
                int_map_v_cp[j][i] <= 1'b1;
                fp_map_v_cp[j][i]  <= 1'b1;
            end
        end
    end else if (flush) begin
        out_inst_valid <= {`IF_BATCH_SIZE{1'b0}};
        out_inst_0 <= {`INST_WIDTH{1'b0}};
        out_inst_1 <= {`INST_WIDTH{1'b0}};
        out_fu_type_0 <= {2{1'b0}};
        out_fu_type_1 <= {2{1'b0}};
        out_pc_0 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pc_1 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pred_taken_0 <= 1'b0;
        out_pred_target_0 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pred_hist_0 <= {`BP_GHR_BITS{1'b0}};
        out_pred_taken_1 <= 1'b0;
        out_pred_target_1 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pred_hist_1 <= {`BP_GHR_BITS{1'b0}};
        out_rs1_preg_valid_0 <= 1'b0; out_rs2_preg_valid_0 <= 1'b0; out_rd_preg_valid_0 <= 1'b0; out_old_rd_preg_valid_0 <= 1'b0;
        out_rs1_preg_valid_1 <= 1'b0; out_rs2_preg_valid_1 <= 1'b0; out_rd_preg_valid_1 <= 1'b0; out_old_rd_preg_valid_1 <= 1'b0;
        out_rob_idx_0 <= {`ROB_IDX_WIDTH{1'b0}};
        out_rob_idx_valid_0 <= 1'b0;
        out_rob_idx_1 <= {`ROB_IDX_WIDTH{1'b0}};
        out_rob_idx_valid_1 <= 1'b0;
        int_free_head <= 0;
        int_free_tail <= INT_FREE_INIT_TAIL;
        int_free_count <= INT_FREE_INIT_TAIL;
        fp_free_head  <= 0;
        fp_free_tail  <= FP_FREE_INIT_TAIL;
        fp_free_count <= FP_FREE_INIT_TAIL;
        for (i=0; i<ARCH_REGS; i=i+1) begin
            int_map[i] <= i[`PREG_IDX_WIDTH-1:0];
            fp_map[i]  <= i[`PREG_IDX_WIDTH-1:0];
            int_map_valid[i] <= 1'b1;
            fp_map_valid[i]  <= 1'b1;
        end
        for (i=ARCH_REGS; i<INT_PREG_COUNT; i=i+1) begin
            int_free[i-ARCH_REGS] <= i[`PREG_IDX_WIDTH-1:0];
        end
        for (i=ARCH_REGS; i<FP_PREG_COUNT; i=i+1) begin
            fp_free[i-ARCH_REGS] <= i[`PREG_IDX_WIDTH-1:0];
        end
        for (j=0; j<`ROB_SIZE; j=j+1) begin
            int_head_cp[j]  <= 0;
            int_tail_cp[j]  <= INT_FREE_INIT_TAIL;
            int_count_cp[j] <= INT_FREE_INIT_TAIL;
            fp_head_cp[j]   <= 0;
            fp_tail_cp[j]   <= FP_FREE_INIT_TAIL;
            fp_count_cp[j]  <= FP_FREE_INIT_TAIL;
            for (i=0; i<ARCH_REGS; i=i+1) begin
                int_map_cp[j][i] <= i[`PREG_IDX_WIDTH-1:0];
                fp_map_cp[j][i]  <= i[`PREG_IDX_WIDTH-1:0];
                int_map_v_cp[j][i] <= 1'b1;
                fp_map_v_cp[j][i]  <= 1'b1;
            end
        end
    end else begin
        int_free_head_next  = int_free_head;
        fp_free_head_next   = fp_free_head;
        int_free_count_next = int_free_count;
        fp_free_count_next  = fp_free_count;

        if (rename_stall) begin
            out_inst_valid <= {`IF_BATCH_SIZE{1'b0}};
            out_rob_idx_0 <= {`ROB_IDX_WIDTH{1'b0}};
            out_rob_idx_valid_0 <= 1'b0;
            out_rob_idx_1 <= {`ROB_IDX_WIDTH{1'b0}};
            out_rob_idx_valid_1 <= 1'b0;
            out_pc_0 <= {`INST_ADDR_WIDTH{1'b0}};
            out_pc_1 <= {`INST_ADDR_WIDTH{1'b0}};
            out_pred_taken_0 <= 1'b0;
            out_pred_target_0 <= {`INST_ADDR_WIDTH{1'b0}};
            out_pred_hist_0 <= {`BP_GHR_BITS{1'b0}};
            out_pred_taken_1 <= 1'b0;
            out_pred_target_1 <= {`INST_ADDR_WIDTH{1'b0}};
            out_pred_hist_1 <= {`BP_GHR_BITS{1'b0}};
        end else begin
            if (rob_req0) begin
                int_head_cp[alloc0_rob_idx]  <= int_free_head;
                int_tail_cp[alloc0_rob_idx]  <= int_free_tail;
                int_count_cp[alloc0_rob_idx] <= int_free_count;
                fp_head_cp[alloc0_rob_idx]   <= fp_free_head;
                fp_tail_cp[alloc0_rob_idx]   <= fp_free_tail;
                fp_count_cp[alloc0_rob_idx]  <= fp_free_count;
                for (i=0; i<ARCH_REGS; i=i+1) begin
                    int_map_cp[alloc0_rob_idx][i] <= int_map[i];
                    fp_map_cp[alloc0_rob_idx][i]  <= fp_map[i];
                    int_map_v_cp[alloc0_rob_idx][i] <= int_map_valid[i];
                    fp_map_v_cp[alloc0_rob_idx][i]  <= fp_map_valid[i];
                end
            end
            if (rob_req1) begin
                int_head_cp[alloc1_rob_idx]  <= int_free_head;
                int_tail_cp[alloc1_rob_idx]  <= int_free_tail;
                int_count_cp[alloc1_rob_idx] <= int_free_count;
                fp_head_cp[alloc1_rob_idx]   <= fp_free_head;
                fp_tail_cp[alloc1_rob_idx]   <= fp_free_tail;
                fp_count_cp[alloc1_rob_idx]  <= fp_free_count;
                for (i=0; i<ARCH_REGS; i=i+1) begin
                    int_map_cp[alloc1_rob_idx][i] <= int_map[i];
                    fp_map_cp[alloc1_rob_idx][i]  <= fp_map[i];
                    int_map_v_cp[alloc1_rob_idx][i] <= int_map_valid[i];
                    fp_map_v_cp[alloc1_rob_idx][i]  <= fp_map_valid[i];
                end
            end
            out_inst_valid <= in_inst_valid;
            if (in_inst_valid[0]) begin
                out_inst_0 <= in_inst_0;
                out_pc_0 <= in_pc_0;
                out_pred_taken_0 <= in_pred_taken_0;
                out_pred_target_0 <= in_pred_target_0;
                out_pred_hist_0 <= in_pred_hist_0;
                out_fu_type_0 <= in_fu_type_0;
                out_rs1_0 <= in_rs1_0;
                out_rs2_0 <= in_rs2_0;
                out_rd_0  <= in_rd_0;
                out_imm_0 <= in_imm_0;
                out_use_imm_0 <= in_use_imm_0;
                out_rs1_is_fp_0 <= in_rs1_is_fp_0;
                out_rs2_is_fp_0 <= in_rs2_is_fp_0;
                out_rd_is_fp_0  <= in_rd_is_fp_0;
                out_rs1_preg_0 <= rs1_tag0;
                out_rs1_preg_valid_0 <= rs1_tag_valid0;
                out_rs2_preg_0 <= rs2_tag0;
                out_rs2_preg_valid_0 <= rs2_tag_valid0;
                out_rd_preg_0 <= has_dest0 ? new_preg0 : {`PREG_IDX_WIDTH{1'b0}};
                out_rd_preg_valid_0 <= has_dest0;
                out_old_rd_preg_0 <= old_preg0;
                out_old_rd_preg_valid_0 <= has_dest0;
                out_rob_idx_0 <= rob_req0 ? alloc0_rob_idx : {`ROB_IDX_WIDTH{1'b0}};
                out_rob_idx_valid_0 <= rob_req0;
            end else begin
                out_inst_0 <= {`INST_WIDTH{1'b0}};
                out_pc_0 <= {`INST_ADDR_WIDTH{1'b0}};
                out_fu_type_0 <= 2'b0;
                out_rs1_0 <= {`REG_ADDR_WIDTH{1'b0}};
                out_rs2_0 <= {`REG_ADDR_WIDTH{1'b0}};
                out_rd_0  <= {`REG_ADDR_WIDTH{1'b0}};
                out_imm_0 <= {`DATA_WIDTH{1'b0}};
                out_use_imm_0 <= 1'b0;
                out_rs1_is_fp_0 <= 1'b0;
                out_rs2_is_fp_0 <= 1'b0;
                out_rd_is_fp_0  <= 1'b0;
                out_pred_taken_0 <= 1'b0;
                out_pred_target_0 <= {`INST_ADDR_WIDTH{1'b0}};
                out_pred_hist_0 <= {`BP_GHR_BITS{1'b0}};
                out_rs1_preg_0 <= {`PREG_IDX_WIDTH{1'b0}};
                out_rs1_preg_valid_0 <= 1'b0;
                out_rs2_preg_0 <= {`PREG_IDX_WIDTH{1'b0}};
                out_rs2_preg_valid_0 <= 1'b0;
                out_rd_preg_0 <= {`PREG_IDX_WIDTH{1'b0}};
                out_rd_preg_valid_0 <= 1'b0;
                out_old_rd_preg_0 <= {`PREG_IDX_WIDTH{1'b0}};
                out_old_rd_preg_valid_0 <= 1'b0;
                out_rob_idx_0 <= {`ROB_IDX_WIDTH{1'b0}};
                out_rob_idx_valid_0 <= 1'b0;
            end

            if (in_inst_valid[1]) begin
                out_inst_1 <= in_inst_1;
                out_pc_1 <= in_pc_1;
                out_fu_type_1 <= in_fu_type_1;
                out_rs1_1 <= in_rs1_1;
                out_rs2_1 <= in_rs2_1;
                out_rd_1  <= in_rd_1;
                out_imm_1 <= in_imm_1;
                out_use_imm_1 <= in_use_imm_1;
                out_rs1_is_fp_1 <= in_rs1_is_fp_1;
                out_rs2_is_fp_1 <= in_rs2_is_fp_1;
                out_rd_is_fp_1  <= in_rd_is_fp_1;
                out_pred_taken_1 <= in_pred_taken_1;
                out_pred_target_1 <= in_pred_target_1;
                out_pred_hist_1 <= in_pred_hist_1;
                out_rs1_preg_1 <= rs1_tag1;
                out_rs1_preg_valid_1 <= rs1_tag_valid1;
                out_rs2_preg_1 <= rs2_tag1;
                out_rs2_preg_valid_1 <= rs2_tag_valid1;
                out_rd_preg_1 <= has_dest1 ? new_preg1 : {`PREG_IDX_WIDTH{1'b0}};
                out_rd_preg_valid_1 <= has_dest1;
                out_old_rd_preg_1 <= old_preg1;
                out_old_rd_preg_valid_1 <= has_dest1;
                out_rob_idx_1 <= rob_req1 ? alloc1_rob_idx : {`ROB_IDX_WIDTH{1'b0}};
                out_rob_idx_valid_1 <= rob_req1;
            end else begin
                out_inst_1 <= {`INST_WIDTH{1'b0}};
                out_pc_1 <= {`INST_ADDR_WIDTH{1'b0}};
                out_fu_type_1 <= 2'b0;
                out_rs1_1 <= {`REG_ADDR_WIDTH{1'b0}};
                out_rs2_1 <= {`REG_ADDR_WIDTH{1'b0}};
                out_rd_1  <= {`REG_ADDR_WIDTH{1'b0}};
                out_imm_1 <= {`DATA_WIDTH{1'b0}};
                out_use_imm_1 <= 1'b0;
                out_rs1_is_fp_1 <= 1'b0;
                out_rs2_is_fp_1 <= 1'b0;
                out_rd_is_fp_1  <= 1'b0;
                out_pred_taken_1 <= 1'b0;
                out_pred_target_1 <= {`INST_ADDR_WIDTH{1'b0}};
                out_pred_hist_1 <= {`BP_GHR_BITS{1'b0}};
                out_rs1_preg_1 <= {`PREG_IDX_WIDTH{1'b0}};
                out_rs1_preg_valid_1 <= 1'b0;
                out_rs2_preg_1 <= {`PREG_IDX_WIDTH{1'b0}};
                out_rs2_preg_valid_1 <= 1'b0;
                out_rd_preg_1 <= {`PREG_IDX_WIDTH{1'b0}};
                out_rd_preg_valid_1 <= 1'b0;
                out_old_rd_preg_1 <= {`PREG_IDX_WIDTH{1'b0}};
                out_old_rd_preg_valid_1 <= 1'b0;
                out_rob_idx_1 <= {`ROB_IDX_WIDTH{1'b0}};
                out_rob_idx_valid_1 <= 1'b0;
            end
        end

        // Pop free lists on allocation
        if (!rename_stall) begin
            if (int_need_count != 0) begin
                int_free_head_next = (int_free_head + int_need_count >= INT_PREG_COUNT) ?
                                      int_free_head + int_need_count - INT_PREG_COUNT :
                                      int_free_head + int_need_count;
                int_free_count_next = int_free_count_next - int_need_count;
            end
            if (fp_need_count != 0) begin
                fp_free_head_next = (fp_free_head + fp_need_count >= FP_PREG_COUNT) ?
                                     fp_free_head + fp_need_count - FP_PREG_COUNT :
                                     fp_free_head + fp_need_count;
                fp_free_count_next = fp_free_count_next - fp_need_count;
            end
        end

        // Update maps on allocation
        if (!rename_stall && has_dest0) begin
            if (in_rd_is_fp_0) begin
                fp_map[in_rd_0] <= new_preg0;
                fp_map_valid[in_rd_0] <= 1'b1;
            end else if (in_rd_0 != {`REG_ADDR_WIDTH{1'b0}}) begin
                int_map[in_rd_0] <= new_preg0;
                int_map_valid[in_rd_0] <= 1'b1;
            end
        end
        if (!rename_stall && has_dest1) begin
            if (in_rd_is_fp_1) begin
                fp_map[in_rd_1] <= new_preg1;
                fp_map_valid[in_rd_1] <= 1'b1;
            end else if (in_rd_1 != {`REG_ADDR_WIDTH{1'b0}}) begin
                int_map[in_rd_1] <= new_preg1;
                int_map_valid[in_rd_1] <= 1'b1;
            end
        end

        // Free old physical regs on commit
        int_tail_next = int_free_tail;
        fp_tail_next  = fp_free_tail;
        if (commit0_push_int) begin
            int_free[int_tail_next[`PREG_IDX_WIDTH-1:0]] <= commit0_old_preg;
            int_tail_next = (int_tail_next == INT_PREG_COUNT-1) ? 0 : int_tail_next + 1'b1;
        end
        if (commit1_push_int) begin
            int_free[int_tail_next[`PREG_IDX_WIDTH-1:0]] <= commit1_old_preg;
            int_tail_next = (int_tail_next == INT_PREG_COUNT-1) ? 0 : int_tail_next + 1'b1;
        end
        if (commit0_push_fp) begin
            fp_free[fp_tail_next[`PREG_IDX_WIDTH-1:0]] <= commit0_old_preg;
            fp_tail_next = (fp_tail_next == FP_PREG_COUNT-1) ? 0 : fp_tail_next + 1'b1;
        end
        if (commit1_push_fp) begin
            fp_free[fp_tail_next[`PREG_IDX_WIDTH-1:0]] <= commit1_old_preg;
            fp_tail_next = (fp_tail_next == FP_PREG_COUNT-1) ? 0 : fp_tail_next + 1'b1;
        end
        int_free_tail <= int_tail_next;
        fp_free_tail  <= fp_tail_next;
        int_free_head <= int_free_head_next;
        fp_free_head  <= fp_free_head_next;
        int_free_count <= int_free_count_next + int_push_count;
        fp_free_count  <= fp_free_count_next + fp_push_count;
    end
end

endmodule
