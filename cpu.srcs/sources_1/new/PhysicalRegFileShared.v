`include "riscv_define.v"

// Shared physical register file (int + fp), dual read pairs and dual write ports each.
module PhysicalRegFileShared #(
    parameter INT_PREG_NUMS = `INT_PREG_NUM,
    parameter F_PREG_NUMS   = `FP_PREG_NUM
)(
    input clk,
    input rst_n,

    // int reads
    input [`PREG_IDX_WIDTH-1:0] i_rs1_addr0,
    input [`PREG_IDX_WIDTH-1:0] i_rs2_addr0,
    input [`PREG_IDX_WIDTH-1:0] i_rs1_addr1,
    input [`PREG_IDX_WIDTH-1:0] i_rs2_addr1,
    output [`DATA_WIDTH-1:0]    i_rs1_data0,
    output [`DATA_WIDTH-1:0]    i_rs2_data0,
    output [`DATA_WIDTH-1:0]    i_rs1_data1,
    output [`DATA_WIDTH-1:0]    i_rs2_data1,

    // int writes
    input [`PREG_IDX_WIDTH-1:0] i_rd0_addr,
    input [`DATA_WIDTH-1:0]     i_rd0_data,
    input                       i_rd0_we,
    input [`PREG_IDX_WIDTH-1:0] i_rd1_addr,
    input [`DATA_WIDTH-1:0]     i_rd1_data,
    input                       i_rd1_we,

    // fp reads
    input [`PREG_IDX_WIDTH-1:0] f_rs1_addr0,
    input [`PREG_IDX_WIDTH-1:0] f_rs2_addr0,
    input [`PREG_IDX_WIDTH-1:0] f_rs1_addr1,
    input [`PREG_IDX_WIDTH-1:0] f_rs2_addr1,
    output [`DATA_WIDTH-1:0]    f_rs1_data0,
    output [`DATA_WIDTH-1:0]    f_rs2_data0,
    output [`DATA_WIDTH-1:0]    f_rs1_data1,
    output [`DATA_WIDTH-1:0]    f_rs2_data1,

    // fp writes
    input [`PREG_IDX_WIDTH-1:0] f_rd0_addr,
    input [`DATA_WIDTH-1:0]     f_rd0_data,
    input                       f_rd0_we,
    input [`PREG_IDX_WIDTH-1:0] f_rd1_addr,
    input [`DATA_WIDTH-1:0]     f_rd1_data,
    input                       f_rd1_we
);
    reg [`DATA_WIDTH-1:0] int_prf   [0:INT_PREG_NUMS-1];
    reg [`DATA_WIDTH-1:0] float_prf [0:F_PREG_NUMS-1];

    // Reads with bypass (write port1 > port0 priority)
    assign i_rs1_data0 = (i_rd1_we && (i_rd1_addr == i_rs1_addr0)) ? i_rd1_data :
                         (i_rd0_we && (i_rd0_addr == i_rs1_addr0)) ? i_rd0_data :
                         int_prf[i_rs1_addr0];
    assign i_rs2_data0 = (i_rd1_we && (i_rd1_addr == i_rs2_addr0)) ? i_rd1_data :
                         (i_rd0_we && (i_rd0_addr == i_rs2_addr0)) ? i_rd0_data :
                         int_prf[i_rs2_addr0];
    assign i_rs1_data1 = (i_rd1_we && (i_rd1_addr == i_rs1_addr1)) ? i_rd1_data :
                         (i_rd0_we && (i_rd0_addr == i_rs1_addr1)) ? i_rd0_data :
                         int_prf[i_rs1_addr1];
    assign i_rs2_data1 = (i_rd1_we && (i_rd1_addr == i_rs2_addr1)) ? i_rd1_data :
                         (i_rd0_we && (i_rd0_addr == i_rs2_addr1)) ? i_rd0_data :
                         int_prf[i_rs2_addr1];

    assign f_rs1_data0 = (f_rd1_we && (f_rd1_addr == f_rs1_addr0)) ? f_rd1_data :
                         (f_rd0_we && (f_rd0_addr == f_rs1_addr0)) ? f_rd0_data :
                         float_prf[f_rs1_addr0];
    assign f_rs2_data0 = (f_rd1_we && (f_rd1_addr == f_rs2_addr0)) ? f_rd1_data :
                         (f_rd0_we && (f_rd0_addr == f_rs2_addr0)) ? f_rd0_data :
                         float_prf[f_rs2_addr0];
    assign f_rs1_data1 = (f_rd1_we && (f_rd1_addr == f_rs1_addr1)) ? f_rd1_data :
                         (f_rd0_we && (f_rd0_addr == f_rs1_addr1)) ? f_rd0_data :
                         float_prf[f_rs1_addr1];
    assign f_rs2_data1 = (f_rd1_we && (f_rd1_addr == f_rs2_addr1)) ? f_rd1_data :
                         (f_rd0_we && (f_rd0_addr == f_rs2_addr1)) ? f_rd0_data :
                         float_prf[f_rs2_addr1];

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i=0;i<INT_PREG_NUMS;i=i+1) int_prf[i] <= {`DATA_WIDTH{1'b0}};
            for (i=0;i<F_PREG_NUMS;i=i+1)   float_prf[i] <= {`DATA_WIDTH{1'b0}};
        end else begin
            if (i_rd0_we) int_prf[i_rd0_addr] <= i_rd0_data;
            if (i_rd1_we) int_prf[i_rd1_addr] <= i_rd1_data;
            if (f_rd0_we) float_prf[f_rd0_addr] <= f_rd0_data;
            if (f_rd1_we) float_prf[f_rd1_addr] <= f_rd1_data;
        end
    end
endmodule
