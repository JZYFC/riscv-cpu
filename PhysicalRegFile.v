`include "riscv_define.v"

module PhysicalRegFile #(
    parameter INT_PREG_NUMS = 64,
    parameter F_PREG_NUMS = 64
)
(
    // common
    input clk,
    input rst_n,
    
    // --- int reg files interface ---
    input [`REG_ADDR_WIDTH-1:0] i_rs1_addr,
    input [`REG_ADDR_WIDTH-1:0] i_rs2_addr,
    
    output [`DATA_WIDTH-1:0]    i_rs1_data,
    output [`DATA_WIDTH-1:0]    i_rs2_data,
    
    input [`REG_ADDR_WIDTH-1:0] i_rd_addr,
    input [`DATA_WIDTH-1:0]     i_rd_data,
    input                       i_rd_we, // write enable
    
    // --- float reg files interface ---
    input [`REG_ADDR_WIDTH-1:0] f_rs1_addr,
    input [`REG_ADDR_WIDTH-1:0] f_rs2_addr,
    
    output [`DATA_WIDTH-1:0]    f_rs1_data,
    output [`DATA_WIDTH-1:0]    f_rs2_data,
    
    input [`REG_ADDR_WIDTH-1:0] f_rd_addr,
    input [`DATA_WIDTH-1:0]     f_rd_data,
    input                       f_rd_we // write enable
);
    // --- regs ---
    reg [`DATA_WIDTH-1:0] int_prf   [0:INT_PREG_NUMS-1];
    reg [`DATA_WIDTH-1:0] float_prf [0:F_PREG_NUMS-1];
    
    // --- reads ---
    assign i_rs1_data = int_prf[i_rs1_addr];
    assign i_rs2_data = int_prf[i_rs2_addr];
    
    assign f_rs1_data = float_prf[f_rs1_addr];
    assign f_rs2_data = float_prf[f_rs2_addr];
    
    // --- writes ---
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            for (i = 0; i < INT_PREG_NUMS; i = i + 1)
                int_prf[i] <= {`DATA_WIDTH{1'b0}};
            for (i = 0; i < F_PREG_NUMS; i = i + 1)
                float_prf[i] <= {`DATA_WIDTH{1'b0}};
        end else begin
            if (i_rd_we) int_prf[i_rd_addr] <= i_rd_data;
            if (f_rd_we) float_prf[f_rd_addr] <= f_rd_data;
        end
    end
endmodule
