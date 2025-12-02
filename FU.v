`include "riscv_define.v"


// Integer ALU Module
// Operations cover RISCV's I ISA
// Not including MUL/DIV !
module IntAlu(
    input clk,      // clock
    input rst_n,    // reset when low

    // Inputs
    input [`ALU_OP_WIDTH-1:0] ALU_op,

    input [`DATA_WIDTH-1:0] rs1,
    input [`DATA_WIDTH-1:0] rs2,

    input                   add_c_in,   // add: carry in
    input                   cmp_signed, // cmp: signed or unsigned

    // Outputs
    output [`DATA_WIDTH-1:0] ALU_result,
);
// -- add --
wire [`DATA_WIDTH-1:0] inner_add_rs1;
wire [`DATA_WIDTH-1:0] inner_add_rs2;
wire                   inner_add_c_in;
wire                   inner_add_res;
assign inner_add_rs1 = ALU_op[`ALU_OP_ADD] ? rs1 : `DATA_WIDTH'b0;
assign inner_add_rs2 = ALU_op[`ALU_OP_ADD] ? rs2 : `DATA_WIDTH'b0;
assign inner_add_c_in = ALU_op[`ALU_OP_ADD] ? add_c_in
assign inner_add_res = inner_add_rs1 + inner_add_rs2 + inner_add_c_in;

// -- cmp --
wire [`DATA_WIDTH-1:0] inner_cmp_rs1;
wire [`DATA_WIDTH-1:0] inner_cmp_rs2;
wire                   inner_cmp_res;
assign inner_cmp_rs1 = ALU_op[`ALU_OP_CMP] ? rs1 : `DATA_WIDTH'b0;
assign inner_cmp_rs2 = ALU_op[`ALU_OP_CMP] ? rs2 : `DATA_WIDTH'b0;
assign inner_cmp_res = cmp_signed ? (
    $signed(inner_cmp_rs1) < $signed(inner_cmp_rs2))
    : ($unsigned(inner_cmp_rs1) < $unsigned(inner_cmp_rs2)
);

// -- and --
wire [`DATA_WIDTH-1:0] inner_and_rs1;
wire [`DATA_WIDTH-1:0] inner_and_rs2;
wire [`DATA_WIDTH-1:0] inner_and_res;
assign inner_and_rs1 = ALU_op[`ALU_OP_AND] ? rs1 : `DATA_WIDTH'b0;
assign inner_and_rs2 = ALU_op[`ALU_OP_AND] ? rs2 : `DATA_WIDTH'b0;
assign inner_and_res = inner_and_rs1 & inner_and_rs2;

// --- or ---
wire [`DATA_WIDTH-1:0] inner_or_rs1;
wire [`DATA_WIDTH-1:0] inner_or_rs2;
wire [`DATA_WIDTH-1:0] inner_or_res;
assign inner_or_rs1 = ALU_op[`ALU_OP_OR] ? rs1 : `DATA_WIDTH'b0;
assign inner_or_rs2 = ALU_op[`ALU_OP_OR] ? rs2 : `DATA_WIDTH'b0;
assign inner_or_res = inner_or_rs1 | inner_or_rs2;

// --- xor ---
wire [`DATA_WIDTH-1:0] inner_xor_rs1;
wire [`DATA_WIDTH-1:0] inner_xor_rs2;
wire [`DATA_WIDTH-1:0] inner_xor_res;
assign inner_xor_rs1 = ALU_op[`ALU_OP_XOR] ? rs1 : `DATA_WIDTH'b0;
assign inner_xor_rs2 = ALU_op[`ALU_OP_XOR] ? rs2 : `DATA_WIDTH'b0;
assign inner_xor_res = inner_xor_rs1 ^ inner_xor_rs2;

// --- logic shift: sll srl ---
wire [`DATA_WIDTH-1:0] inner_sll_rs1;
wire [`DATA_WIDTH-1:0] inner_sll_rs2;
wire [`DATA_WIDTH-1:0] inner_sll_res;
assign inner_sll_rs1 = ALU_op[`ALU_OP_SLL] ? rs1 : `DATA_WIDTH'b0;
assign inner_sll_rs2 = ALU_op[`ALU_OP_SLL] ? rs2[4:0] : `DATA_WIDTH'b0;
assign inner_sll_res = inner_sll_rs1 << inner_sll_rs2;

wire [`DATA_WIDTH-1:0] inner_srl_rs1;
wire [`DATA_WIDTH-1:0] inner_srl_rs2;
wire [`DATA_WIDTH-1:0] inner_srl_res;
assign inner_srl_rs1 = ALU_op[`ALU_OP_SRL] ? rs1 : `DATA_WIDTH'b0;
assign inner_srl_rs2 = ALU_op[`ALU_OP_SRL] ? rs2[4:0] : `DATA_WIDTH'b0;
assign inner_srl_res = inner_srl_rs1 >> inner_srl_rs2;


// --- arithmetic shift: sra ---
wire [`DATA_WIDTH-1:0] inner_sra_rs1;
wire [`DATA_WIDTH-1:0] inner_sra_rs2;
wire [`DATA_WIDTH-1:0] inner_sra_res;
assign inner_sra_rs1 = ALU_op[`ALU_OP_SRA] ? rs1 : `DATA_WIDTH'b0;
assign inner_sra_rs2 = ALU_op[`ALU_OP_SRA] ? rs2[4:0] : `DATA_WIDTH'b0;
assign inner_sra_res = $signed(inner_sra_rs1) >>> inner_sra_rs2;


// Output mux
assign ALU_result = ALU_op[`ALU_OP_ADD] ? inner_add_res :
                    ALU_op[`ALU_OP_CMP] ? inner_cmp_res :
                    ALU_op[`ALU_OP_AND] ? inner_and_res :
                    ALU_op[`ALU_OP_OR]  ? inner_or_res :
                    ALU_op[`ALU_OP_XOR] ? inner_xor_res :
                    ALU_op[`ALU_OP_SLL] ? inner_sll_res :
                    ALU_op[`ALU_OP_SRL] ? inner_srl_res :
                    ALU_op[`ALU_OP_SRA] ? inner_sra_res :
                    `DATA_WIDTH'b0; // default
endmodule

// Region: Integer Multiplier and Divider ALU Module

// Top IntMulDivALU
// For mul operation, decode MUL MULH MULHSU MULHU to corresponding control signals(signed/unsigned for rs1/rs2, low/high part)
// For div operation, decode DIV DIVU REM REMU to corresponding control signals(signed/unsigned, div/rem)
module IntMulDivALU(
    input clk,      // clock
    input rst_n,    // reset when low

    // Inputs
    input [`ALU_OP_WIDTH-1:0] ALU_op,

    input [`DATA_WIDTH-1:0] rs1,
    input [`DATA_WIDTH-1:0] rs2,

    // mul specific
    input                   require_low_part, // whether low part is required, if not, then high part is required
    input                   mul_signed_rs1,  // rs1 signed or unsigned
    input                   mul_signed_rs2,  // rs2 signed or unsigned

    // div/rem specific. rs1 is the dividend, rs2 is the divisor
    input                   require_div_result, // whether division result is required, if not, then remainder is required
    input                   div_signed,         // signed/unsigned division for both rs1 and rs2
    
    // TODO: Add clock signals for mul and div's sequential operations

    // Outputs
    output [`DATA_WIDTH-1:0] ALU_result,
);
// -- muls --

// we'll do 33bit multiplication to unify the signed and unsigned multiplication
wire [`DATA_WIDTH:0] inner_mul_rs1;
wire [`DATA_WIDTH:0] inner_mul_rs2;
wire [`DOUBLE_DATA_WIDTH+1:0] inner_mul_res; // Internal extended result 65bits
wire [`DATA_WIDTH-1:0] inner_mul_rd; // The actual result we need

assign inner_mul_rs1 = mul_signed_rs1 ? {rs1[`DATA_WIDTH-1], rs1} : {1'b0, rs1};
assign inner_mul_rs2 = mul_signed_rs2 ? {rs2[`DATA_WIDTH-1], rs2} : {1'b0, rs2};

// use generated multiplier
assign inner_mul_res = $signed(inner_mul_rs1) * $signed(inner_mul_rs2); // force signed multiplication

assign inner_mul_rd = require_low_part ? inner_mul_res[`DATA_WIDTH-1:0] : inner_mul_res[`DOUBLE_DATA_WIDTH-1:`DATA_WIDTH];

// -- divs --
// general division cannot be generated on some platforms, we'll use our own divider module
// TODO: implement divider module

endmodule
// EndRegion: Integer Multiplier and Divider ALU Module


// Dummy Top Level
// 4 kinds of FUs: IntALU(Done, NoTest), IntMulDivFU(MulDone, DIV TODO, NoTest), FloatALU, MemFU, Branch FU?
module FUs(

);
endmodule