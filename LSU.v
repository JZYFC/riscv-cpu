`include "riscv_define.v"

module LSU (
    input wire clk,
    input wire rst_n,
    input wire flush,

    // 来自 IssueBuffer 的请求
    input wire valid_in,
    input wire [31:0] addr,
    input wire [31:0] wdata,
    input wire [`MEM_OP_WIDTH-1:0] mem_op,
    input wire mem_is_load,
    input wire mem_unsigned,
    input wire [`ROB_IDX_WIDTH-1:0] rob_idx_in,
    input wire [`ROB_GEN_WIDTH-1:0] rob_gen_in,
    input wire [`PREG_IDX_WIDTH-1:0] rd_tag_in,
    input wire rd_is_fp_in,

    // 反馈给 IssueBuffer 的状态
    output wire busy,             // 单元忙，无法接收新请求
    output reg  wb_valid,         // 写回有效
    output reg  [31:0] wb_value,  // 写回数据
    output reg  [`ROB_IDX_WIDTH-1:0] wb_rob_idx,
    output reg  [`ROB_GEN_WIDTH-1:0] wb_rob_gen,
    output reg  [`PREG_IDX_WIDTH-1:0] wb_dest_tag,
    output reg  wb_dest_is_fp,
    output reg  wb_exception,     // 发生异常（如非法访问）

    // 连接到 MainMemory 的接口 (穿透 Cache)
    output wire mem_req,
    output wire mem_we,
    output wire [31:0] mem_addr,
    output wire [127:0] mem_wdata,
    input  wire [127:0] mem_rdata,
    input  wire mem_ready
);

    // 状态机定义
    localparam IDLE = 2'd0;
    localparam WAIT_CACHE = 2'd1;
    localparam DONE = 2'd2;
    reg [1:0] state;

    // 锁存输入信号
    reg [31:0] addr_reg;
    reg [31:0] wdata_reg;
    reg [`MEM_OP_WIDTH-1:0] mem_op_reg;
    reg mem_is_load_reg;
    reg mem_unsigned_reg;
    reg [`ROB_IDX_WIDTH-1:0] rob_idx_reg;
    reg [`ROB_GEN_WIDTH-1:0] rob_gen_reg;
    reg [`PREG_IDX_WIDTH-1:0] rd_tag_reg;
    reg rd_is_fp_reg;

    assign busy = (state != IDLE);

    // === TLB 实例 ===
    wire [31:0] tlb_paddr;
    wire tlb_hit, tlb_miss;
    TLB u_tlb (
        .clk(clk), .rst_n(rst_n),
        .vaddr(addr_reg), .req(state == WAIT_CACHE),
        .paddr(tlb_paddr), .hit(tlb_hit), .miss(tlb_miss),
        .we(1'b0), .w_vaddr(32'b0), .w_paddr(32'b0)
    );

    // 如果 TLB 没命中，默认物理地址 = 虚拟地址 (Bare-metal 模式)
    wire [31:0] effective_paddr = tlb_hit ? tlb_paddr : addr_reg;

    // === Cache 实例 ===
    wire [31:0] cache_rdata;
    wire cache_valid_out;
    wire cache_stall;
    
    // 生成 Cache 写选通 (wstrb)
    reg [3:0] wstrb;
    always @(*) begin
        case (mem_op_reg)
            `MEM_OP_SB: wstrb = 4'b0001 << addr_reg[1:0];
            `MEM_OP_SH: wstrb = 4'b0011 << addr_reg[1:0];
            `MEM_OP_SW: wstrb = 4'b1111;
            default:    wstrb = 4'b0000;
        endcase
    end

    // 对齐写数据
    reg [31:0] aligned_wdata;
    always @(*) begin
        case (mem_op_reg)
            `MEM_OP_SB: aligned_wdata = {4{wdata_reg[7:0]}};
            `MEM_OP_SH: aligned_wdata = {2{wdata_reg[15:0]}};
            default:    aligned_wdata = wdata_reg;
        endcase
    end

    // Misaligned access detection
    reg misaligned_in;
    always @(*) begin
        case (mem_op)
            `MEM_OP_LW, `MEM_OP_SW: misaligned_in = |addr[1:0];
            `MEM_OP_LH, `MEM_OP_LHU, `MEM_OP_SH: misaligned_in = addr[0];
            default: misaligned_in = 1'b0;
        endcase
    end

    Cache u_cache (
        .clk(clk), .rst_n(rst_n),
        // CPU 侧
        .paddr(effective_paddr),
        .req(state == WAIT_CACHE),
        .we(!mem_is_load_reg && (state == WAIT_CACHE)),
        .wdata(aligned_wdata),
        .wstrb(wstrb),
        .rdata(cache_rdata),
        .valid_out(cache_valid_out),
        .stall_cpu(cache_stall),
        // 主存侧
        .mem_req(mem_req), .mem_we(mem_we),
        .mem_addr(mem_addr), .mem_wdata(mem_wdata),
        .mem_rdata(mem_rdata), .mem_ready(mem_ready)
    );

    // === Load 数据扩展逻辑 ===
    reg [31:0] final_rdata;
    reg [7:0]  b_data;
    reg [15:0] h_data;
    
    always @(*) begin
        // 根据地址低位选择字节/半字
        case (addr_reg[1:0])
            2'b00: b_data = cache_rdata[7:0];
            2'b01: b_data = cache_rdata[15:8];
            2'b10: b_data = cache_rdata[23:16];
            2'b11: b_data = cache_rdata[31:24];
        endcase
        case (addr_reg[1])
            1'b0: h_data = cache_rdata[15:0];
            1'b1: h_data = cache_rdata[31:16];
        endcase

        case (mem_op_reg)
            `MEM_OP_LB:  final_rdata = {{24{b_data[7]}}, b_data};
            `MEM_OP_LBU: final_rdata = {24'b0, b_data};
            `MEM_OP_LH:  final_rdata = {{16{h_data[15]}}, h_data};
            `MEM_OP_LHU: final_rdata = {16'b0, h_data};
            `MEM_OP_LW:  final_rdata = cache_rdata;
            default:     final_rdata = cache_rdata;
        endcase
    end

    // === 主控状态机 ===
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            wb_valid <= 0;
            wb_exception <= 0;
            wb_rob_gen <= {`ROB_GEN_WIDTH{1'b0}};
        end else begin
            if (flush) begin
                state <= IDLE;
                wb_valid <= 0;
                wb_exception <= 0;
                wb_rob_gen <= {`ROB_GEN_WIDTH{1'b0}};
            end else begin
                case (state)
                    IDLE: begin
                        wb_valid <= 0;
                        if (valid_in) begin
                            if (misaligned_in) begin
                                wb_valid <= 1;
                                wb_value <= 32'b0;
                                wb_rob_idx <= rob_idx_in;
                                wb_rob_gen <= rob_gen_in;
                                wb_dest_tag <= rd_tag_in;
                                wb_dest_is_fp <= rd_is_fp_in;
                                wb_exception <= 1'b1;
                                state <= IDLE;
                            end else begin
                                addr_reg <= addr;
                                wdata_reg <= wdata;
                                mem_op_reg <= mem_op;
                                mem_is_load_reg <= mem_is_load;
                                mem_unsigned_reg <= mem_unsigned;
                                rob_idx_reg <= rob_idx_in;
                                rob_gen_reg <= rob_gen_in;
                                rd_tag_reg <= rd_tag_in;
                                rd_is_fp_reg <= rd_is_fp_in;
                                state <= WAIT_CACHE;
                            end
                        end
                    end
                    WAIT_CACHE: begin
                        // Cache 命中且返回有效数据 
                        if (cache_valid_out) begin
                            wb_valid <= 1;
                            // 如果是 Load，写回读取的数据；如果是 Store，写回 0 
                            wb_value <= mem_is_load_reg ? final_rdata : 32'b0;
                            wb_rob_idx <= rob_idx_reg;
                            wb_rob_gen <= rob_gen_reg;
                            wb_dest_tag <= rd_tag_reg;
                            wb_dest_is_fp <= rd_is_fp_reg;
                            wb_exception <= 0;
                            state <= IDLE; // 任务完成，回到 IDLE
                        end
                    end
                endcase
            end
        end
    end

endmodule