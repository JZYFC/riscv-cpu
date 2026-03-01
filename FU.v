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
    output [`DATA_WIDTH-1:0] ALU_result
);
// -- add --
wire [`DATA_WIDTH-1:0] inner_add_rs1;
wire [`DATA_WIDTH-1:0] inner_add_rs2;
wire                   inner_add_c_in;
wire [`DATA_WIDTH-1:0] inner_add_res;
assign inner_add_rs1 = ALU_op[`ALU_OP_ADD] ? rs1 : `DATA_WIDTH'b0;
assign inner_add_rs2 = ALU_op[`ALU_OP_ADD] ? rs2 : `DATA_WIDTH'b0;
assign inner_add_c_in = ALU_op[`ALU_OP_ADD] ? add_c_in : 1'b0;
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
    output [`DATA_WIDTH-1:0] ALU_result
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
// Division is handled by IntDivider module (instantiated in IssueBuffer)

endmodule
// EndRegion: Integer Multiplier and Divider ALU Module


// 32-cycle restoring integer divider
// Handles DIV, DIVU, REM, REMU per RISC-V spec edge cases
module IntDivider (
    input            clk,
    input            rst_n,
    input            start,       // 1-cycle pulse to begin computation
    input  [31:0]    dividend,    // rs1
    input  [31:0]    divisor,     // rs2
    input            is_signed,   // 1 = signed (DIV/REM), 0 = unsigned (DIVU/REMU)
    input            is_rem,      // 1 = output remainder, 0 = output quotient
    output reg [31:0] result,
    output reg        done        // 1-cycle pulse when result is ready
);

    reg         running;
    reg [4:0]   count;        // 0..31, 32 steps total
    reg [31:0]  q_reg;        // quotient accumulator; initialized to unsigned dividend
    reg [31:0]  r_reg;        // partial remainder
    reg [31:0]  udivisor_r;   // latched unsigned divisor
    reg         quot_neg;     // negate quotient at end (signed division)
    reg         rem_neg;      // negate remainder at end (signed division)
    reg         latch_is_rem; // latched is_rem at start

    // Combinational restoring-division step:
    // shift partial remainder left by 1, bringing in the MSB of the dividend register
    wire [31:0] p_shift = {r_reg[30:0], q_reg[31]};
    // 33-bit subtract; p_sub[32]=1 means borrow (p_shift < udivisor_r)
    wire [32:0] p_sub   = {1'b0, p_shift} - {1'b0, udivisor_r};

    // Next-step values (used at count==31 to compute result before registers update)
    wire [31:0] nxt_q = p_sub[32] ? {q_reg[30:0], 1'b0} : {q_reg[30:0], 1'b1};
    wire [31:0] nxt_r = p_sub[32] ? p_shift              : p_sub[31:0];
    // Sign-corrected final outputs
    wire [31:0] final_q = quot_neg ? (~nxt_q + 1'b1) : nxt_q;
    wire [31:0] final_r = rem_neg  ? (~nxt_r + 1'b1) : nxt_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            running      <= 1'b0;
            done         <= 1'b0;
            result       <= 32'b0;
            count        <= 5'b0;
            q_reg        <= 32'b0;
            r_reg        <= 32'b0;
            udivisor_r   <= 32'b0;
            quot_neg     <= 1'b0;
            rem_neg      <= 1'b0;
            latch_is_rem <= 1'b0;
        end else begin
            done <= 1'b0; // default: not done

            if (start) begin
                latch_is_rem <= is_rem;

                if (divisor == 32'b0) begin
                    // RISC-V spec: divide by zero
                    // quotient = 0xFFFFFFFF (-1), remainder = dividend
                    done    <= 1'b1;
                    result  <= is_rem ? dividend : 32'hFFFFFFFF;
                    running <= 1'b0;
                end else if (is_signed && (dividend == 32'h80000000) && (divisor == 32'hFFFFFFFF)) begin
                    // RISC-V spec: signed overflow (INT_MIN / -1)
                    // quotient = INT_MIN (0x80000000), remainder = 0
                    done    <= 1'b1;
                    result  <= is_rem ? 32'b0 : 32'h80000000;
                    running <= 1'b0;
                end else begin
                    // Normal case: 32-cycle restoring division
                    count   <= 5'd0;
                    r_reg   <= 32'b0;
                    running <= 1'b1;
                    if (is_signed) begin
                        quot_neg   <= dividend[31] ^ divisor[31];
                        rem_neg    <= dividend[31];
                        q_reg      <= dividend[31] ? (~dividend + 1'b1) : dividend;
                        udivisor_r <= divisor[31]  ? (~divisor  + 1'b1) : divisor;
                    end else begin
                        quot_neg   <= 1'b0;
                        rem_neg    <= 1'b0;
                        q_reg      <= dividend;
                        udivisor_r <= divisor;
                    end
                end
            end else if (running) begin
                if (count == 5'd31) begin
                    // Final step: compute sign-corrected result
                    running <= 1'b0;
                    done    <= 1'b1;
                    result  <= latch_is_rem ? final_r : final_q;
                    q_reg   <= nxt_q;
                    r_reg   <= nxt_r;
                end else begin
                    // Regular restoring step
                    if (!p_sub[32]) begin
                        // p_shift >= udivisor_r: quotient bit = 1
                        r_reg <= p_sub[31:0];
                        q_reg <= {q_reg[30:0], 1'b1};
                    end else begin
                        // p_shift < udivisor_r: quotient bit = 0, restore
                        r_reg <= p_shift;
                        q_reg <= {q_reg[30:0], 1'b0};
                    end
                    count <= count + 1'b1;
                end
            end
        end
    end

endmodule


// Dummy Top Level
// 4 kinds of FUs: IntALU(Done, NoTest), IntMulDivFU(MulDone, DIV TODO, NoTest), FloatALU, MemFU, Branch FU?
module FUs(

);
endmodule
