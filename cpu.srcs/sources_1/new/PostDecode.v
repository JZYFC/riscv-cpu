`include "riscv_define.v"

// Post-decode (after register rename) to generate detailed micro-ops for issue.
// Takes physical register info from the rename stage and classifies each
// instruction into a target functional unit with per-unit micro-op controls.
module PostDecode (
    input clk,
    input rst_n,
    input stall,
    input flush,

    // Incoming bundle from rename
    input  wire [`IF_BATCH_SIZE-1:0]   in_inst_valid,
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
    input  wire [`INST_ADDR_WIDTH-1:0] in_pc_0,
    input  wire [`REG_ADDR_WIDTH-1:0]  in_rs1_0,
    input  wire [`REG_ADDR_WIDTH-1:0]  in_rs2_0,
    input  wire [`REG_ADDR_WIDTH-1:0]  in_rd_0,
    input  wire [`DATA_WIDTH-1:0]      in_imm_0,
    input  wire                        in_use_imm_0,
    input  wire                        in_rs1_is_fp_0,
    input  wire                        in_rs2_is_fp_0,
    input  wire                        in_rd_is_fp_0,
    input  wire [`ROB_IDX_WIDTH-1:0]   in_rob_idx_0,
    input  wire                        in_rob_idx_valid_0,
    input  wire [`ROB_GEN_WIDTH-1:0]   in_rob_gen_0,
    input  wire [`PREG_IDX_WIDTH-1:0]  in_rs1_preg_0,
    input  wire                        in_rs1_preg_valid_0,
    input  wire [`PREG_IDX_WIDTH-1:0]  in_rs2_preg_0,
    input  wire                        in_rs2_preg_valid_0,
    input  wire [`PREG_IDX_WIDTH-1:0]  in_rd_preg_0,
    input  wire                        in_rd_preg_valid_0,
    input  wire [`PREG_IDX_WIDTH-1:0]  in_old_rd_preg_0,
    input  wire                        in_old_rd_preg_valid_0,

    input  wire [`INST_ADDR_WIDTH-1:0] in_pc_1,
    input  wire [`REG_ADDR_WIDTH-1:0]  in_rs1_1,
    input  wire [`REG_ADDR_WIDTH-1:0]  in_rs2_1,
    input  wire [`REG_ADDR_WIDTH-1:0]  in_rd_1,
    input  wire [`DATA_WIDTH-1:0]      in_imm_1,
    input  wire                        in_use_imm_1,
    input  wire                        in_rs1_is_fp_1,
    input  wire                        in_rs2_is_fp_1,
    input  wire                        in_rd_is_fp_1,
    input  wire [`ROB_IDX_WIDTH-1:0]   in_rob_idx_1,
    input  wire                        in_rob_idx_valid_1,
    input  wire [`ROB_GEN_WIDTH-1:0]   in_rob_gen_1,
    input  wire [`PREG_IDX_WIDTH-1:0]  in_rs1_preg_1,
    input  wire                        in_rs1_preg_valid_1,
    input  wire [`PREG_IDX_WIDTH-1:0]  in_rs2_preg_1,
    input  wire                        in_rs2_preg_valid_1,
    input  wire [`PREG_IDX_WIDTH-1:0]  in_rd_preg_1,
    input  wire                        in_rd_preg_valid_1,
    input  wire [`PREG_IDX_WIDTH-1:0]  in_old_rd_preg_1,
    input  wire                        in_old_rd_preg_valid_1,

    // Registered outputs for dispatch/issue
    output reg  [`IF_BATCH_SIZE-1:0]   out_inst_valid,
    output reg  [`INST_WIDTH-1:0]      out_inst_0,
    output reg  [`INST_WIDTH-1:0]      out_inst_1,
    output reg  [1:0]                  out_fu_type_0,
    output reg  [1:0]                  out_fu_type_1,
    output reg  [`INST_ADDR_WIDTH-1:0] out_pc_0,
    output reg                         out_pred_taken_0,
    output reg [`INST_ADDR_WIDTH-1:0]  out_pred_target_0,
    output reg [`BP_GHR_BITS-1:0]      out_pred_hist_0,
    output reg  [`REG_ADDR_WIDTH-1:0]  out_rs1_0,
    output reg  [`REG_ADDR_WIDTH-1:0]  out_rs2_0,
    output reg  [`REG_ADDR_WIDTH-1:0]  out_rd_0,
    output reg  [`DATA_WIDTH-1:0]      out_imm_0,
    output reg                         out_use_imm_0,
    output reg                         out_rs1_is_fp_0,
    output reg                         out_rs2_is_fp_0,
    output reg                         out_rd_is_fp_0,
    output reg  [`ROB_IDX_WIDTH-1:0]   out_rob_idx_0,
    output reg                         out_rob_idx_valid_0,
    output reg  [`ROB_GEN_WIDTH-1:0]   out_rob_gen_0,
    output reg  [`PREG_IDX_WIDTH-1:0]  out_rs1_preg_0,
    output reg                         out_rs1_preg_valid_0,
    output reg  [`PREG_IDX_WIDTH-1:0]  out_rs2_preg_0,
    output reg                         out_rs2_preg_valid_0,
    output reg  [`PREG_IDX_WIDTH-1:0]  out_rd_preg_0,
    output reg                         out_rd_preg_valid_0,
    output reg  [`PREG_IDX_WIDTH-1:0]  out_old_rd_preg_0,
    output reg                         out_old_rd_preg_valid_0,

    output reg  [`INST_ADDR_WIDTH-1:0] out_pc_1,
    output reg  [`REG_ADDR_WIDTH-1:0]  out_rs1_1,
    output reg  [`REG_ADDR_WIDTH-1:0]  out_rs2_1,
    output reg  [`REG_ADDR_WIDTH-1:0]  out_rd_1,
    output reg  [`DATA_WIDTH-1:0]      out_imm_1,
    output reg                         out_use_imm_1,
    output reg                         out_rs1_is_fp_1,
    output reg                         out_rs2_is_fp_1,
    output reg                         out_rd_is_fp_1,
    output reg                         out_pred_taken_1,
    output reg [`INST_ADDR_WIDTH-1:0]  out_pred_target_1,
    output reg [`BP_GHR_BITS-1:0]      out_pred_hist_1,
    output reg  [`ROB_IDX_WIDTH-1:0]   out_rob_idx_1,
    output reg                         out_rob_idx_valid_1,
    output reg  [`ROB_GEN_WIDTH-1:0]   out_rob_gen_1,
    output reg  [`PREG_IDX_WIDTH-1:0]  out_rs1_preg_1,
    output reg                         out_rs1_preg_valid_1,
    output reg  [`PREG_IDX_WIDTH-1:0]  out_rs2_preg_1,
    output reg                         out_rs2_preg_valid_1,
    output reg  [`PREG_IDX_WIDTH-1:0]  out_rd_preg_1,
    output reg                         out_rd_preg_valid_1,
    output reg  [`PREG_IDX_WIDTH-1:0]  out_old_rd_preg_1,
    output reg                         out_old_rd_preg_valid_1,

    // Detailed decode outputs
    output reg  [`FU_DEC_WIDTH-1:0]    out_fu_sel_0,
    output reg  [`FU_DEC_WIDTH-1:0]    out_fu_sel_1,
    output reg  [`ALU_OP_WIDTH-1:0]    out_int_op_0,
    output reg  [`ALU_OP_WIDTH-1:0]    out_int_op_1,
    output reg                         out_int_is_sub_0,
    output reg                         out_int_is_sub_1,
    output reg                         out_cmp_signed_0,
    output reg                         out_cmp_signed_1,
    output reg  [`ALU_OP_WIDTH-1:0]    out_muldiv_op_0,
    output reg  [`ALU_OP_WIDTH-1:0]    out_muldiv_op_1,
    output reg                         out_mul_high_0,
    output reg                         out_mul_high_1,
    output reg                         out_mul_signed_rs1_0,
    output reg                         out_mul_signed_rs1_1,
    output reg                         out_mul_signed_rs2_0,
    output reg                         out_mul_signed_rs2_1,
    output reg                         out_div_signed_0,
    output reg                         out_div_signed_1,
    output reg                         out_div_is_rem_0,
    output reg                         out_div_is_rem_1,
    output reg  [`BR_OP_WIDTH-1:0]     out_branch_op_0,
    output reg  [`BR_OP_WIDTH-1:0]     out_branch_op_1,
    output reg  [`MEM_OP_WIDTH-1:0]    out_mem_op_0,
    output reg  [`MEM_OP_WIDTH-1:0]    out_mem_op_1,
    output reg                         out_mem_is_load_0,
    output reg                         out_mem_is_load_1,
    output reg                         out_mem_unsigned_0,
    output reg                         out_mem_unsigned_1,
    output reg  [`CSR_OP_WIDTH-1:0]    out_csr_op_0,
    output reg  [`CSR_OP_WIDTH-1:0]    out_csr_op_1,
    output reg  [11:0]                 out_csr_addr_0,
    output reg  [11:0]                 out_csr_addr_1,
    output reg  [`FP_OP_WIDTH-1:0]     out_fp_op_0,
    output reg  [`FP_OP_WIDTH-1:0]     out_fp_op_1,
    output reg                         out_illegal_0,
    output reg                         out_illegal_1
);

// RISC-V base opcodes
localparam [6:0] OPCODE_LUI      = 7'b0110111;
localparam [6:0] OPCODE_AUIPC    = 7'b0010111;
localparam [6:0] OPCODE_JAL      = 7'b1101111;
localparam [6:0] OPCODE_JALR     = 7'b1100111;
localparam [6:0] OPCODE_BRANCH   = 7'b1100011;
localparam [6:0] OPCODE_LOAD     = 7'b0000011;
localparam [6:0] OPCODE_STORE    = 7'b0100011;
localparam [6:0] OPCODE_OP_IMM   = 7'b0010011;
localparam [6:0] OPCODE_OP       = 7'b0110011;
localparam [6:0] OPCODE_MISC_MEM = 7'b0001111;
localparam [6:0] OPCODE_SYSTEM   = 7'b1110011;

// Floating point opcodes
localparam [6:0] OPCODE_LOAD_FP  = 7'b0000111;
localparam [6:0] OPCODE_STORE_FP = 7'b0100111;
localparam [6:0] OPCODE_OP_FP    = 7'b1010011;
localparam [6:0] OPCODE_MADD     = 7'b1000011;
localparam [6:0] OPCODE_MSUB     = 7'b1000111;
localparam [6:0] OPCODE_NMSUB    = 7'b1001011;
localparam [6:0] OPCODE_NMADD    = 7'b1001111;

// Decode helper: classify a single instruction into per-FU micro-ops.
task automatic decode_one;
    input  [`INST_WIDTH-1:0]     inst;
    output [`FU_DEC_WIDTH-1:0]   fu_sel;
    output [`ALU_OP_WIDTH-1:0]   int_op;
    output                       int_is_sub;
    output                       cmp_signed;
    output [`ALU_OP_WIDTH-1:0]   muldiv_op;
    output                       mul_high;
    output                       mul_signed_rs1;
    output                       mul_signed_rs2;
    output                       div_signed;
    output                       div_is_rem;
    output [`BR_OP_WIDTH-1:0]    br_op;
    output [`MEM_OP_WIDTH-1:0]   mem_op;
    output                       mem_is_load;
    output                       mem_unsigned;
    output [`CSR_OP_WIDTH-1:0]   csr_op;
    output [11:0]                csr_addr;
    output [`FP_OP_WIDTH-1:0]    fp_op;
    output                       illegal;
    reg [6:0] opcode;
    reg [2:0] funct3;
    reg [6:0] funct7;
begin
    opcode = inst[6:0];
    funct3 = inst[14:12];
    funct7 = inst[31:25];

    fu_sel         = `FU_DEC_INT;
    int_op         = {`ALU_OP_WIDTH{1'b0}};
    int_is_sub     = 1'b0;
    cmp_signed     = 1'b0;
    muldiv_op      = {`ALU_OP_WIDTH{1'b0}};
    mul_high       = 1'b0;
    mul_signed_rs1 = 1'b0;
    mul_signed_rs2 = 1'b0;
    div_signed     = 1'b0;
    div_is_rem     = 1'b0;
    br_op          = `BR_OP_NONE;
    mem_op         = {`MEM_OP_WIDTH{1'b0}};
    mem_is_load    = 1'b0;
    mem_unsigned   = 1'b0;
    csr_op         = `CSR_OP_NONE;
    csr_addr       = 12'b0;
    fp_op          = `FP_OP_DUMMY;
    illegal        = 1'b0;

    case (opcode)
        OPCODE_OP: begin
            if (funct7 == 7'b0000001) begin
                fu_sel = `FU_DEC_MULDIV;
                case (funct3)
                    3'b000: begin // MUL
                        muldiv_op[`ALU_OP_MUL] = 1'b1;
                        mul_high       = 1'b0;
                        mul_signed_rs1 = 1'b1;
                        mul_signed_rs2 = 1'b1;
                    end
                    3'b001: begin // MULH
                        muldiv_op[`ALU_OP_MUL] = 1'b1;
                        mul_high       = 1'b1;
                        mul_signed_rs1 = 1'b1;
                        mul_signed_rs2 = 1'b1;
                    end
                    3'b010: begin // MULHSU
                        muldiv_op[`ALU_OP_MUL] = 1'b1;
                        mul_high       = 1'b1;
                        mul_signed_rs1 = 1'b1;
                        mul_signed_rs2 = 1'b0;
                    end
                    3'b011: begin // MULHU
                        muldiv_op[`ALU_OP_MUL] = 1'b1;
                        mul_high       = 1'b1;
                        mul_signed_rs1 = 1'b0;
                        mul_signed_rs2 = 1'b0;
                    end
                    3'b100: begin // DIV
                        muldiv_op[`ALU_OP_DIV] = 1'b1;
                        div_signed   = 1'b1;
                        div_is_rem   = 1'b0;
                    end
                    3'b101: begin // DIVU
                        muldiv_op[`ALU_OP_DIV] = 1'b1;
                        div_signed   = 1'b0;
                        div_is_rem   = 1'b0;
                    end
                    3'b110: begin // REM
                        muldiv_op[`ALU_OP_DIV] = 1'b1;
                        div_signed   = 1'b1;
                        div_is_rem   = 1'b1;
                    end
                    3'b111: begin // REMU
                        muldiv_op[`ALU_OP_DIV] = 1'b1;
                        div_signed   = 1'b0;
                        div_is_rem   = 1'b1;
                    end
                    default: illegal = 1'b1;
                endcase
            end else begin
                fu_sel = `FU_DEC_INT;
                case (funct3)
                    3'b000: begin
                        int_op[`ALU_OP_ADD] = 1'b1;
                        int_is_sub = (funct7 == 7'b0100000); // SUB
                    end
                    3'b001: int_op[`ALU_OP_SLL] = 1'b1;
                    3'b010: begin int_op[`ALU_OP_CMP] = 1'b1; cmp_signed = 1'b1; end // SLT
                    3'b011: begin int_op[`ALU_OP_CMP] = 1'b1; cmp_signed = 1'b0; end // SLTU
                    3'b100: int_op[`ALU_OP_XOR] = 1'b1;
                    3'b101: begin
                        if (funct7 == 7'b0100000) int_op[`ALU_OP_SRA] = 1'b1;
                        else                       int_op[`ALU_OP_SRL] = 1'b1;
                    end
                    3'b110: int_op[`ALU_OP_OR]  = 1'b1;
                    3'b111: int_op[`ALU_OP_AND] = 1'b1;
                    default: illegal = 1'b1;
                endcase
            end
        end
        OPCODE_OP_IMM: begin
            fu_sel = `FU_DEC_INT;
            case (funct3)
                3'b000: begin int_op[`ALU_OP_ADD] = 1'b1; int_is_sub = 1'b0; end // ADDI
                3'b010: begin int_op[`ALU_OP_CMP] = 1'b1; cmp_signed = 1'b1; end // SLTI
                3'b011: begin int_op[`ALU_OP_CMP] = 1'b1; cmp_signed = 1'b0; end // SLTIU
                3'b100: int_op[`ALU_OP_XOR] = 1'b1;
                3'b110: int_op[`ALU_OP_OR]  = 1'b1;
                3'b111: int_op[`ALU_OP_AND] = 1'b1;
                3'b001: int_op[`ALU_OP_SLL] = 1'b1;
                3'b101: begin
                    if (funct7 == 7'b0100000) int_op[`ALU_OP_SRA] = 1'b1;
                    else                       int_op[`ALU_OP_SRL] = 1'b1;
                end
                default: illegal = 1'b1;
            endcase
        end
        OPCODE_LUI: begin
            fu_sel = `FU_DEC_INT;
            int_op[`ALU_OP_ADD] = 1'b1; // imm -> rd
        end
        OPCODE_AUIPC: begin
            fu_sel = `FU_DEC_INT;
            int_op[`ALU_OP_ADD] = 1'b1; // PC + imm
        end
        OPCODE_JAL: begin
            fu_sel = `FU_DEC_BRANCH;
            br_op  = `BR_OP_JAL;
        end
        OPCODE_JALR: begin
            fu_sel = `FU_DEC_BRANCH;
            br_op  = `BR_OP_JALR;
        end
        OPCODE_BRANCH: begin
            fu_sel = `FU_DEC_BRANCH;
            case (funct3)
                3'b000: br_op = `BR_OP_BEQ;
                3'b001: br_op = `BR_OP_BNE;
                3'b100: br_op = `BR_OP_BLT;
                3'b101: br_op = `BR_OP_BGE;
                3'b110: br_op = `BR_OP_BLTU;
                3'b111: br_op = `BR_OP_BGEU;
                default: illegal = 1'b1;
            endcase
        end
        OPCODE_LOAD: begin
            fu_sel      = `FU_DEC_LSU;
            mem_is_load = 1'b1;
            case (funct3)
                3'b000: mem_op = `MEM_OP_LB;
                3'b001: mem_op = `MEM_OP_LH;
                3'b010: mem_op = `MEM_OP_LW;
                3'b100: begin mem_op = `MEM_OP_LBU; mem_unsigned = 1'b1; end
                3'b101: begin mem_op = `MEM_OP_LHU; mem_unsigned = 1'b1; end
                default: illegal = 1'b1;
            endcase
        end
        OPCODE_STORE: begin
            fu_sel      = `FU_DEC_LSU;
            mem_is_load = 1'b0;
            case (funct3)
                3'b000: mem_op = `MEM_OP_SB;
                3'b001: mem_op = `MEM_OP_SH;
                3'b010: mem_op = `MEM_OP_SW;
                default: illegal = 1'b1;
            endcase
        end
        OPCODE_LOAD_FP: begin
            fu_sel      = `FU_DEC_LSU;
            mem_is_load = 1'b1;
            mem_op      = `MEM_OP_LW;
        end
        OPCODE_STORE_FP: begin
            fu_sel      = `FU_DEC_LSU;
            mem_is_load = 1'b0;
            mem_op      = `MEM_OP_SW;
        end
        OPCODE_OP_FP: begin
            fu_sel = `FU_DEC_FP;
            case (inst[31:27])
                5'b00000: fp_op = `FP_OP_ADD;
                5'b00001: fp_op = `FP_OP_SUB;
                5'b00010: fp_op = `FP_OP_MUL;
                5'b00011: fp_op = `FP_OP_DIV;
                5'b01011: fp_op = `FP_OP_SQRT;
                5'b00100: fp_op = `FP_OP_MV;   // fsgnj/mv-style
                5'b10000: fp_op = `FP_OP_CVT;  // convert to/from int
                5'b10100: fp_op = `FP_OP_CMP;  // feq/flt/fle
                default:  fp_op = `FP_OP_DUMMY;
            endcase
        end
        OPCODE_MADD,
        OPCODE_MSUB,
        OPCODE_NMSUB,
        OPCODE_NMADD: begin
            fu_sel = `FU_DEC_FP;
            fp_op  = `FP_OP_FMA;
        end
        OPCODE_MISC_MEM: begin
            fu_sel = `FU_DEC_SYSTEM; // fence-style ops
        end
        OPCODE_SYSTEM: begin
            fu_sel   = `FU_DEC_SYSTEM;
            csr_addr = inst[31:20];
            case (funct3)
                3'b001: csr_op = `CSR_OP_RW; // CSRRW
                3'b010: csr_op = `CSR_OP_RS; // CSRRS
                3'b011: csr_op = `CSR_OP_RC; // CSRRC
                3'b101: csr_op = `CSR_OP_RW; // CSRRWI
                3'b110: csr_op = `CSR_OP_RS; // CSRRSI
                3'b111: csr_op = `CSR_OP_RC; // CSRRCI
                default: csr_op = `CSR_OP_NONE; // ecall/ebreak/mret/wfi
            endcase
        end
        default: begin
            fu_sel  = `FU_DEC_DUMMY;
            illegal = 1'b1;
        end
    endcase

    if (illegal) fu_sel = `FU_DEC_DUMMY;
end
endtask

// Combinational decode results for the two slots
reg [`FU_DEC_WIDTH-1:0]  fu_dec_0, fu_dec_1;
reg [`ALU_OP_WIDTH-1:0]  int_op_dec_0, int_op_dec_1;
reg                      int_sub_dec_0, int_sub_dec_1;
reg                      cmp_signed_dec_0, cmp_signed_dec_1;
reg [`ALU_OP_WIDTH-1:0]  muldiv_op_dec_0, muldiv_op_dec_1;
reg                      mul_high_dec_0, mul_high_dec_1;
reg                      mul_signed_rs1_dec_0, mul_signed_rs1_dec_1;
reg                      mul_signed_rs2_dec_0, mul_signed_rs2_dec_1;
reg                      div_signed_dec_0, div_signed_dec_1;
reg                      div_is_rem_dec_0, div_is_rem_dec_1;
reg [`BR_OP_WIDTH-1:0]   br_op_dec_0, br_op_dec_1;
reg [`MEM_OP_WIDTH-1:0]  mem_op_dec_0, mem_op_dec_1;
reg                      mem_is_load_dec_0, mem_is_load_dec_1;
reg                      mem_unsigned_dec_0, mem_unsigned_dec_1;
reg [`CSR_OP_WIDTH-1:0]  csr_op_dec_0, csr_op_dec_1;
reg [11:0]               csr_addr_dec_0, csr_addr_dec_1;
reg [`FP_OP_WIDTH-1:0]   fp_op_dec_0, fp_op_dec_1;
reg                      illegal_dec_0, illegal_dec_1;

always @(*) begin
    decode_one(in_inst_0, fu_dec_0, int_op_dec_0, int_sub_dec_0, cmp_signed_dec_0,
               muldiv_op_dec_0, mul_high_dec_0, mul_signed_rs1_dec_0, mul_signed_rs2_dec_0,
               div_signed_dec_0, div_is_rem_dec_0, br_op_dec_0, mem_op_dec_0,
               mem_is_load_dec_0, mem_unsigned_dec_0, csr_op_dec_0, csr_addr_dec_0,
               fp_op_dec_0, illegal_dec_0);
    decode_one(in_inst_1, fu_dec_1, int_op_dec_1, int_sub_dec_1, cmp_signed_dec_1,
               muldiv_op_dec_1, mul_high_dec_1, mul_signed_rs1_dec_1, mul_signed_rs2_dec_1,
               div_signed_dec_1, div_is_rem_dec_1, br_op_dec_1, mem_op_dec_1,
               mem_is_load_dec_1, mem_unsigned_dec_1, csr_op_dec_1, csr_addr_dec_1,
               fp_op_dec_1, illegal_dec_1);
end

// Registered outputs
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        out_inst_valid <= {`IF_BATCH_SIZE{1'b0}};
        out_inst_0 <= {`INST_WIDTH{1'b0}};
        out_inst_1 <= {`INST_WIDTH{1'b0}};
        out_fu_type_0 <= 2'b0;
        out_fu_type_1 <= 2'b0;
        out_pc_0 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pc_1 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pred_taken_0 <= 1'b0;
        out_pred_target_0 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pred_hist_0 <= {`BP_GHR_BITS{1'b0}};
        out_rs1_0 <= {`REG_ADDR_WIDTH{1'b0}};
        out_rs2_0 <= {`REG_ADDR_WIDTH{1'b0}};
        out_rd_0  <= {`REG_ADDR_WIDTH{1'b0}};
        out_imm_0 <= {`DATA_WIDTH{1'b0}};
        out_use_imm_0 <= 1'b0;
        out_rs1_is_fp_0 <= 1'b0;
        out_rs2_is_fp_0 <= 1'b0;
        out_rd_is_fp_0  <= 1'b0;
        out_rob_idx_0 <= {`ROB_IDX_WIDTH{1'b0}};
        out_rob_idx_valid_0 <= 1'b0;
        out_rob_gen_0 <= {`ROB_GEN_WIDTH{1'b0}};
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
        out_pred_taken_1 <= 1'b0;
        out_pred_target_1 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pred_hist_1 <= {`BP_GHR_BITS{1'b0}};
        out_rob_idx_1 <= {`ROB_IDX_WIDTH{1'b0}};
        out_rob_idx_valid_1 <= 1'b0;
        out_rob_gen_1 <= {`ROB_GEN_WIDTH{1'b0}};
        out_rs1_preg_1 <= {`PREG_IDX_WIDTH{1'b0}};
        out_rs1_preg_valid_1 <= 1'b0;
        out_rs2_preg_1 <= {`PREG_IDX_WIDTH{1'b0}};
        out_rs2_preg_valid_1 <= 1'b0;
        out_rd_preg_1 <= {`PREG_IDX_WIDTH{1'b0}};
        out_rd_preg_valid_1 <= 1'b0;
        out_old_rd_preg_1 <= {`PREG_IDX_WIDTH{1'b0}};
        out_old_rd_preg_valid_1 <= 1'b0;

        out_fu_sel_0 <= `FU_DEC_DUMMY;
        out_fu_sel_1 <= `FU_DEC_DUMMY;
        out_int_op_0 <= {`ALU_OP_WIDTH{1'b0}};
        out_int_op_1 <= {`ALU_OP_WIDTH{1'b0}};
        out_int_is_sub_0 <= 1'b0;
        out_int_is_sub_1 <= 1'b0;
        out_cmp_signed_0 <= 1'b0;
        out_cmp_signed_1 <= 1'b0;
        out_muldiv_op_0 <= {`ALU_OP_WIDTH{1'b0}};
        out_muldiv_op_1 <= {`ALU_OP_WIDTH{1'b0}};
        out_mul_high_0 <= 1'b0;
        out_mul_high_1 <= 1'b0;
        out_mul_signed_rs1_0 <= 1'b0;
        out_mul_signed_rs1_1 <= 1'b0;
        out_mul_signed_rs2_0 <= 1'b0;
        out_mul_signed_rs2_1 <= 1'b0;
        out_div_signed_0 <= 1'b0;
        out_div_signed_1 <= 1'b0;
        out_div_is_rem_0 <= 1'b0;
        out_div_is_rem_1 <= 1'b0;
        out_branch_op_0 <= `BR_OP_NONE;
        out_branch_op_1 <= `BR_OP_NONE;
        out_mem_op_0 <= {`MEM_OP_WIDTH{1'b0}};
        out_mem_op_1 <= {`MEM_OP_WIDTH{1'b0}};
        out_mem_is_load_0 <= 1'b0;
        out_mem_is_load_1 <= 1'b0;
        out_mem_unsigned_0 <= 1'b0;
        out_mem_unsigned_1 <= 1'b0;
        out_csr_op_0 <= `CSR_OP_NONE;
        out_csr_op_1 <= `CSR_OP_NONE;
        out_csr_addr_0 <= 12'b0;
        out_csr_addr_1 <= 12'b0;
        out_fp_op_0 <= `FP_OP_DUMMY;
        out_fp_op_1 <= `FP_OP_DUMMY;
        out_illegal_0 <= 1'b0;
        out_illegal_1 <= 1'b0;
    end else if (flush) begin
        out_inst_valid <= {`IF_BATCH_SIZE{1'b0}};
        out_inst_0 <= {`INST_WIDTH{1'b0}};
        out_inst_1 <= {`INST_WIDTH{1'b0}};
        out_fu_type_0 <= 2'b0;
        out_fu_type_1 <= 2'b0;
        out_pc_0 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pc_1 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pred_taken_0 <= 1'b0;
        out_pred_target_0 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pred_hist_0 <= {`BP_GHR_BITS{1'b0}};
        out_rs1_0 <= {`REG_ADDR_WIDTH{1'b0}};
        out_rs2_0 <= {`REG_ADDR_WIDTH{1'b0}};
        out_rd_0  <= {`REG_ADDR_WIDTH{1'b0}};
        out_imm_0 <= {`DATA_WIDTH{1'b0}};
        out_use_imm_0 <= 1'b0;
        out_rs1_is_fp_0 <= 1'b0;
        out_rs2_is_fp_0 <= 1'b0;
        out_rd_is_fp_0  <= 1'b0;
        out_rob_idx_0 <= {`ROB_IDX_WIDTH{1'b0}};
        out_rob_idx_valid_0 <= 1'b0;
        out_rob_gen_0 <= {`ROB_GEN_WIDTH{1'b0}};
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
        out_pred_taken_1 <= 1'b0;
        out_pred_target_1 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pred_hist_1 <= {`BP_GHR_BITS{1'b0}};
        out_rob_idx_1 <= {`ROB_IDX_WIDTH{1'b0}};
        out_rob_idx_valid_1 <= 1'b0;
        out_rob_gen_1 <= {`ROB_GEN_WIDTH{1'b0}};
        out_rs1_preg_1 <= {`PREG_IDX_WIDTH{1'b0}};
        out_rs1_preg_valid_1 <= 1'b0;
        out_rs2_preg_1 <= {`PREG_IDX_WIDTH{1'b0}};
        out_rs2_preg_valid_1 <= 1'b0;
        out_rd_preg_1 <= {`PREG_IDX_WIDTH{1'b0}};
        out_rd_preg_valid_1 <= 1'b0;
        out_old_rd_preg_1 <= {`PREG_IDX_WIDTH{1'b0}};
        out_old_rd_preg_valid_1 <= 1'b0;

        out_fu_sel_0 <= `FU_DEC_DUMMY;
        out_fu_sel_1 <= `FU_DEC_DUMMY;
        out_int_op_0 <= {`ALU_OP_WIDTH{1'b0}};
        out_int_op_1 <= {`ALU_OP_WIDTH{1'b0}};
        out_int_is_sub_0 <= 1'b0;
        out_int_is_sub_1 <= 1'b0;
        out_cmp_signed_0 <= 1'b0;
        out_cmp_signed_1 <= 1'b0;
        out_muldiv_op_0 <= {`ALU_OP_WIDTH{1'b0}};
        out_muldiv_op_1 <= {`ALU_OP_WIDTH{1'b0}};
        out_mul_high_0 <= 1'b0;
        out_mul_high_1 <= 1'b0;
        out_mul_signed_rs1_0 <= 1'b0;
        out_mul_signed_rs1_1 <= 1'b0;
        out_mul_signed_rs2_0 <= 1'b0;
        out_mul_signed_rs2_1 <= 1'b0;
        out_div_signed_0 <= 1'b0;
        out_div_signed_1 <= 1'b0;
        out_div_is_rem_0 <= 1'b0;
        out_div_is_rem_1 <= 1'b0;
        out_branch_op_0 <= `BR_OP_NONE;
        out_branch_op_1 <= `BR_OP_NONE;
        out_mem_op_0 <= {`MEM_OP_WIDTH{1'b0}};
        out_mem_op_1 <= {`MEM_OP_WIDTH{1'b0}};
        out_mem_is_load_0 <= 1'b0;
        out_mem_is_load_1 <= 1'b0;
        out_mem_unsigned_0 <= 1'b0;
        out_mem_unsigned_1 <= 1'b0;
        out_csr_op_0 <= `CSR_OP_NONE;
        out_csr_op_1 <= `CSR_OP_NONE;
        out_csr_addr_0 <= 12'b0;
        out_csr_addr_1 <= 12'b0;
        out_fp_op_0 <= `FP_OP_DUMMY;
        out_fp_op_1 <= `FP_OP_DUMMY;
        out_illegal_0 <= 1'b0;
        out_illegal_1 <= 1'b0;
    end else if (!stall) begin
        out_inst_valid <= in_inst_valid;

        if (in_inst_valid[0]) begin
            out_inst_0 <= in_inst_0;
            out_fu_type_0 <= in_fu_type_0;
            out_pc_0 <= in_pc_0;
            out_pred_taken_0 <= in_pred_taken_0;
            out_pred_target_0 <= in_pred_target_0;
            out_pred_hist_0 <= in_pred_hist_0;
            out_rs1_0 <= in_rs1_0;
            out_rs2_0 <= in_rs2_0;
            out_rd_0  <= in_rd_0;
            out_imm_0 <= in_imm_0;
            out_use_imm_0 <= in_use_imm_0;
            out_rs1_is_fp_0 <= in_rs1_is_fp_0;
            out_rs2_is_fp_0 <= in_rs2_is_fp_0;
            out_rd_is_fp_0  <= in_rd_is_fp_0;
            out_rs1_preg_0 <= in_rs1_preg_0;
            out_rs1_preg_valid_0 <= in_rs1_preg_valid_0;
            out_rs2_preg_0 <= in_rs2_preg_0;
            out_rs2_preg_valid_0 <= in_rs2_preg_valid_0;
            out_rd_preg_0 <= in_rd_preg_0;
            out_rd_preg_valid_0 <= in_rd_preg_valid_0;
            out_old_rd_preg_0 <= in_old_rd_preg_0;
            out_old_rd_preg_valid_0 <= in_old_rd_preg_valid_0;
            out_rob_idx_0 <= in_rob_idx_0;
            out_rob_idx_valid_0 <= in_rob_idx_valid_0;
            out_rob_gen_0 <= in_rob_gen_0;

            out_fu_sel_0 <= fu_dec_0;
            out_int_op_0 <= int_op_dec_0;
            out_int_is_sub_0 <= int_sub_dec_0;
            out_cmp_signed_0 <= cmp_signed_dec_0;
            out_muldiv_op_0 <= muldiv_op_dec_0;
            out_mul_high_0 <= mul_high_dec_0;
            out_mul_signed_rs1_0 <= mul_signed_rs1_dec_0;
            out_mul_signed_rs2_0 <= mul_signed_rs2_dec_0;
            out_div_signed_0 <= div_signed_dec_0;
            out_div_is_rem_0 <= div_is_rem_dec_0;
            out_branch_op_0 <= br_op_dec_0;
            out_mem_op_0 <= mem_op_dec_0;
            out_mem_is_load_0 <= mem_is_load_dec_0;
            out_mem_unsigned_0 <= mem_unsigned_dec_0;
            out_csr_op_0 <= csr_op_dec_0;
            out_csr_addr_0 <= csr_addr_dec_0;
            out_fp_op_0 <= fp_op_dec_0;
            out_illegal_0 <= illegal_dec_0;
        end else begin
            out_inst_0 <= {`INST_WIDTH{1'b0}};
            out_fu_type_0 <= 2'b0;
            out_rs1_0 <= {`REG_ADDR_WIDTH{1'b0}};
            out_rs2_0 <= {`REG_ADDR_WIDTH{1'b0}};
            out_rd_0  <= {`REG_ADDR_WIDTH{1'b0}};
            out_imm_0 <= {`DATA_WIDTH{1'b0}};
            out_use_imm_0 <= 1'b0;
            out_rs1_is_fp_0 <= 1'b0;
            out_rs2_is_fp_0 <= 1'b0;
            out_rd_is_fp_0  <= 1'b0;
            out_pc_0 <= {`INST_ADDR_WIDTH{1'b0}};
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
            out_rob_gen_0 <= {`ROB_GEN_WIDTH{1'b0}};

            out_fu_sel_0 <= `FU_DEC_DUMMY;
            out_int_op_0 <= {`ALU_OP_WIDTH{1'b0}};
            out_int_is_sub_0 <= 1'b0;
            out_cmp_signed_0 <= 1'b0;
            out_muldiv_op_0 <= {`ALU_OP_WIDTH{1'b0}};
            out_mul_high_0 <= 1'b0;
            out_mul_signed_rs1_0 <= 1'b0;
            out_mul_signed_rs2_0 <= 1'b0;
            out_div_signed_0 <= 1'b0;
            out_div_is_rem_0 <= 1'b0;
            out_branch_op_0 <= `BR_OP_NONE;
            out_mem_op_0 <= {`MEM_OP_WIDTH{1'b0}};
            out_mem_is_load_0 <= 1'b0;
            out_mem_unsigned_0 <= 1'b0;
            out_csr_op_0 <= `CSR_OP_NONE;
            out_csr_addr_0 <= 12'b0;
            out_fp_op_0 <= `FP_OP_DUMMY;
            out_illegal_0 <= 1'b0;
        end

        if (in_inst_valid[1]) begin
            out_inst_1 <= in_inst_1;
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
            out_pc_1 <= in_pc_1;
            out_rs1_preg_1 <= in_rs1_preg_1;
            out_rs1_preg_valid_1 <= in_rs1_preg_valid_1;
            out_rs2_preg_1 <= in_rs2_preg_1;
            out_rs2_preg_valid_1 <= in_rs2_preg_valid_1;
            out_rd_preg_1 <= in_rd_preg_1;
            out_rd_preg_valid_1 <= in_rd_preg_valid_1;
            out_old_rd_preg_1 <= in_old_rd_preg_1;
            out_old_rd_preg_valid_1 <= in_old_rd_preg_valid_1;
            out_rob_idx_1 <= in_rob_idx_1;
            out_rob_idx_valid_1 <= in_rob_idx_valid_1;
            out_rob_gen_1 <= in_rob_gen_1;

            out_fu_sel_1 <= fu_dec_1;
            out_int_op_1 <= int_op_dec_1;
            out_int_is_sub_1 <= int_sub_dec_1;
            out_cmp_signed_1 <= cmp_signed_dec_1;
            out_muldiv_op_1 <= muldiv_op_dec_1;
            out_mul_high_1 <= mul_high_dec_1;
            out_mul_signed_rs1_1 <= mul_signed_rs1_dec_1;
            out_mul_signed_rs2_1 <= mul_signed_rs2_dec_1;
            out_div_signed_1 <= div_signed_dec_1;
            out_div_is_rem_1 <= div_is_rem_dec_1;
            out_branch_op_1 <= br_op_dec_1;
            out_mem_op_1 <= mem_op_dec_1;
            out_mem_is_load_1 <= mem_is_load_dec_1;
            out_mem_unsigned_1 <= mem_unsigned_dec_1;
            out_csr_op_1 <= csr_op_dec_1;
            out_csr_addr_1 <= csr_addr_dec_1;
            out_fp_op_1 <= fp_op_dec_1;
            out_illegal_1 <= illegal_dec_1;
        end else begin
            out_inst_1 <= {`INST_WIDTH{1'b0}};
            out_fu_type_1 <= 2'b0;
            out_rs1_1 <= {`REG_ADDR_WIDTH{1'b0}};
            out_rs2_1 <= {`REG_ADDR_WIDTH{1'b0}};
            out_rd_1  <= {`REG_ADDR_WIDTH{1'b0}};
            out_imm_1 <= {`DATA_WIDTH{1'b0}};
            out_use_imm_1 <= 1'b0;
            out_rs1_is_fp_1 <= 1'b0;
            out_rs2_is_fp_1 <= 1'b0;
            out_rd_is_fp_1  <= 1'b0;
            out_pc_1 <= {`INST_ADDR_WIDTH{1'b0}};
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
            out_rob_gen_1 <= {`ROB_GEN_WIDTH{1'b0}};

            out_fu_sel_1 <= `FU_DEC_DUMMY;
            out_int_op_1 <= {`ALU_OP_WIDTH{1'b0}};
            out_int_is_sub_1 <= 1'b0;
            out_cmp_signed_1 <= 1'b0;
            out_muldiv_op_1 <= {`ALU_OP_WIDTH{1'b0}};
            out_mul_high_1 <= 1'b0;
            out_mul_signed_rs1_1 <= 1'b0;
            out_mul_signed_rs2_1 <= 1'b0;
            out_div_signed_1 <= 1'b0;
            out_div_is_rem_1 <= 1'b0;
            out_branch_op_1 <= `BR_OP_NONE;
            out_mem_op_1 <= {`MEM_OP_WIDTH{1'b0}};
            out_mem_is_load_1 <= 1'b0;
            out_mem_unsigned_1 <= 1'b0;
            out_csr_op_1 <= `CSR_OP_NONE;
            out_csr_addr_1 <= 12'b0;
            out_fp_op_1 <= `FP_OP_DUMMY;
            out_illegal_1 <= 1'b0;
        end
    end
end

endmodule
