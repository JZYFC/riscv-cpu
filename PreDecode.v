`include "riscv_define.v"

// Pre-decode RISC-V 32-bit instructions into rename-ready info (registers + imm).
// Outputs are split per-instruction to match the current fetch batch size.
module PreDecode (
    input clk,
    input rst_n,

    input  wire [`INST_WIDTH-1:0] in_inst_0,
    input  wire [`INST_WIDTH-1:0] in_inst_1,
    input  wire [`INST_ADDR_WIDTH-1:0] in_pc_0,
    input  wire [`INST_ADDR_WIDTH-1:0] in_pc_1,
    input  wire [`IF_BATCH_SIZE-1:0] in_inst_valid,
    input  wire                        in_pred_taken_0,
    input  wire [`INST_ADDR_WIDTH-1:0] in_pred_target_0,
    input  wire [`BP_GHR_BITS-1:0]     in_pred_hist_0,
    input  wire                        in_pred_taken_1,
    input  wire [`INST_ADDR_WIDTH-1:0] in_pred_target_1,
    input  wire [`BP_GHR_BITS-1:0]     in_pred_hist_1,

    output reg  [`IF_BATCH_SIZE-1:0] out_inst_valid,
    output reg  [`INST_WIDTH-1:0]     out_inst_0,
    output reg  [`INST_WIDTH-1:0]     out_inst_1,
    output reg  [`INST_ADDR_WIDTH-1:0] out_pc_0,
    output reg  [`INST_ADDR_WIDTH-1:0] out_pc_1,
    output reg                        out_pred_taken_0,
    output reg [`INST_ADDR_WIDTH-1:0] out_pred_target_0,
    output reg [`BP_GHR_BITS-1:0]     out_pred_hist_0,
    output reg                        out_pred_taken_1,
    output reg [`INST_ADDR_WIDTH-1:0] out_pred_target_1,
    output reg [`BP_GHR_BITS-1:0]     out_pred_hist_1,

    // Micro-code fields for inst0
    output reg  [1:0]                   out_fu_type_0,
    output reg  [`REG_ADDR_WIDTH-1:0]   out_rs1_0,
    output reg  [`REG_ADDR_WIDTH-1:0]   out_rs2_0,
    output reg  [`REG_ADDR_WIDTH-1:0]   out_rd_0,
    output reg  [`DATA_WIDTH-1:0]       out_imm_0,
    output reg                          out_use_imm_0,
    output reg                          out_rs1_is_fp_0,
    output reg                          out_rs2_is_fp_0,
    output reg                          out_rd_is_fp_0,

    // Micro-code fields for inst1
    output reg  [1:0]                   out_fu_type_1,
    output reg  [`REG_ADDR_WIDTH-1:0]   out_rs1_1,
    output reg  [`REG_ADDR_WIDTH-1:0]   out_rs2_1,
    output reg  [`REG_ADDR_WIDTH-1:0]   out_rd_1,
    output reg  [`DATA_WIDTH-1:0]       out_imm_1,
    output reg                          out_use_imm_1,
    output reg                          out_rs1_is_fp_1,
    output reg                          out_rs2_is_fp_1,
    output reg                          out_rd_is_fp_1
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

// Floating point opcodes (dummy decode targets for now)
localparam [6:0] OPCODE_LOAD_FP  = 7'b0000111;
localparam [6:0] OPCODE_STORE_FP = 7'b0100111;
localparam [6:0] OPCODE_OP_FP    = 7'b1010011;
localparam [6:0] OPCODE_MADD     = 7'b1000011;
localparam [6:0] OPCODE_MSUB     = 7'b1000111;
localparam [6:0] OPCODE_NMSUB    = 7'b1001011;
localparam [6:0] OPCODE_NMADD    = 7'b1001111;

// Helpers
function automatic [1:0] decode_fu_type;
    input [`INST_WIDTH-1:0] inst;
    reg [6:0] opcode;
    begin
        opcode = inst[6:0];
        case (opcode)
            OPCODE_OP: begin
                if (inst[31:25] == 7'b0000001) begin
                    decode_fu_type = `ALU_TYPE_MUL; // M extension
                end else begin
                    decode_fu_type = `ALU_TYPE_INT;
                end
            end
            OPCODE_OP_IMM,
            OPCODE_LUI,
            OPCODE_AUIPC,
            OPCODE_JAL,
            OPCODE_JALR,
            OPCODE_BRANCH,
            OPCODE_SYSTEM: decode_fu_type = `ALU_TYPE_INT;
            OPCODE_LOAD,
            OPCODE_STORE,
            OPCODE_MISC_MEM: decode_fu_type = `ALU_TYPE_MEM;
            OPCODE_LOAD_FP,
            OPCODE_STORE_FP,
            OPCODE_OP_FP,
            OPCODE_MADD,
            OPCODE_MSUB,
            OPCODE_NMSUB,
            OPCODE_NMADD: decode_fu_type = `ALU_TYPE_FO;
            default: decode_fu_type = `ALU_TYPE_INT;
        endcase
    end
endfunction

function automatic [`REG_ADDR_WIDTH-1:0] decode_rs1;
    input [`INST_WIDTH-1:0] inst;
    reg [6:0] opcode;
    begin
        opcode = inst[6:0];
        case (opcode)
            OPCODE_LUI,
            OPCODE_AUIPC,
            OPCODE_JAL: decode_rs1 = {`REG_ADDR_WIDTH{1'b0}};
            default:    decode_rs1 = inst[19:15];
        endcase
    end
endfunction

function automatic [`REG_ADDR_WIDTH-1:0] decode_rs2;
    input [`INST_WIDTH-1:0] inst;
    reg [6:0] opcode;
    begin
        opcode = inst[6:0];
        case (opcode)
            OPCODE_OP,
            OPCODE_BRANCH,
            OPCODE_STORE,
            OPCODE_OP_FP,
            OPCODE_STORE_FP,
            OPCODE_MADD,
            OPCODE_MSUB,
            OPCODE_NMSUB,
            OPCODE_NMADD: decode_rs2 = inst[24:20];
            default:      decode_rs2 = {`REG_ADDR_WIDTH{1'b0}};
        endcase
    end
endfunction

function automatic [`REG_ADDR_WIDTH-1:0] decode_rd;
    input [`INST_WIDTH-1:0] inst;
    reg [6:0] opcode;
    begin
        opcode = inst[6:0];
        case (opcode)
            OPCODE_STORE,
            OPCODE_STORE_FP,
            OPCODE_BRANCH: decode_rd = {`REG_ADDR_WIDTH{1'b0}};
            default:       decode_rd = inst[11:7];
        endcase
    end
endfunction

function automatic [`DATA_WIDTH-1:0] decode_imm;
    input [`INST_WIDTH-1:0] inst;
    reg [6:0] opcode;
    reg [`DATA_WIDTH-1:0] imm;
    begin
        opcode = inst[6:0];
        case (opcode)
            OPCODE_OP_IMM,
            OPCODE_LOAD,
            OPCODE_JALR,
            OPCODE_SYSTEM,
            OPCODE_MISC_MEM: imm = {{20{inst[31]}}, inst[31:20]}; // I-type
            OPCODE_STORE:    imm = {{20{inst[31]}}, inst[31:25], inst[11:7]}; // S-type
            OPCODE_BRANCH:   imm = {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0}; // B-type
            OPCODE_LUI,
            OPCODE_AUIPC:    imm = {inst[31:12], 12'b0}; // U-type
            OPCODE_JAL:      imm = {{11{inst[31]}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0}; // J-type
            default:         imm = {`DATA_WIDTH{1'b0}};
        endcase
        decode_imm = imm;
    end
endfunction

// Whether the immediate should be used (and rs2 may be bypassed in ALU stage).
function automatic decode_use_imm;
    input [`INST_WIDTH-1:0] inst;
    reg [6:0] opcode;
    begin
        opcode = inst[6:0];
        case (opcode)
            OPCODE_OP_IMM,
            OPCODE_LOAD,
            OPCODE_STORE,
            OPCODE_BRANCH,
            OPCODE_JAL,
            OPCODE_JALR,
            OPCODE_LUI,
            OPCODE_AUIPC,
            OPCODE_SYSTEM,
            OPCODE_MISC_MEM: decode_use_imm = 1'b1;
            default: decode_use_imm = 1'b0;
        endcase
    end
endfunction

function automatic decode_rs1_is_fp;
    input [`INST_WIDTH-1:0] inst;
    reg [6:0] opcode;
    begin
        opcode = inst[6:0];
        case (opcode)
            OPCODE_OP_FP,
            OPCODE_MADD,
            OPCODE_MSUB,
            OPCODE_NMSUB,
            OPCODE_NMADD: decode_rs1_is_fp = 1'b1;
            default: decode_rs1_is_fp = 1'b0; // FP loads/stores use integer base
        endcase
    end
endfunction

function automatic decode_rs2_is_fp;
    input [`INST_WIDTH-1:0] inst;
    reg [6:0] opcode;
    begin
        opcode = inst[6:0];
        case (opcode)
            OPCODE_OP_FP,
            OPCODE_STORE_FP,
            OPCODE_MADD,
            OPCODE_MSUB,
            OPCODE_NMSUB,
            OPCODE_NMADD: decode_rs2_is_fp = 1'b1;
            default: decode_rs2_is_fp = 1'b0;
        endcase
    end
endfunction

function automatic decode_rd_is_fp;
    input [`INST_WIDTH-1:0] inst;
    reg [6:0] opcode;
    begin
        opcode = inst[6:0];
        case (opcode)
            OPCODE_LOAD_FP,
            OPCODE_OP_FP,
            OPCODE_MADD,
            OPCODE_MSUB,
            OPCODE_NMSUB,
            OPCODE_NMADD: decode_rd_is_fp = 1'b1;
            default: decode_rd_is_fp = 1'b0;
        endcase
    end
endfunction

// Single-cycle registered decode
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        out_inst_valid <= {`IF_BATCH_SIZE{1'b0}};
        out_inst_0 <= {`INST_WIDTH{1'b0}};
        out_inst_1 <= {`INST_WIDTH{1'b0}};
        out_pc_0 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pc_1 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pred_taken_0 <= 1'b0;
        out_pred_target_0 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pred_hist_0 <= {`BP_GHR_BITS{1'b0}};
        out_pred_taken_1 <= 1'b0;
        out_pred_target_1 <= {`INST_ADDR_WIDTH{1'b0}};
        out_pred_hist_1 <= {`BP_GHR_BITS{1'b0}};

        out_fu_type_0  <= `ALU_TYPE_INT;
        out_rs1_0      <= {`REG_ADDR_WIDTH{1'b0}};
        out_rs2_0      <= {`REG_ADDR_WIDTH{1'b0}};
        out_rd_0       <= {`REG_ADDR_WIDTH{1'b0}};
        out_imm_0      <= {`DATA_WIDTH{1'b0}};
        out_use_imm_0  <= 1'b0;
        out_rs1_is_fp_0<= 1'b0;
        out_rs2_is_fp_0<= 1'b0;
        out_rd_is_fp_0 <= 1'b0;

        out_fu_type_1  <= `ALU_TYPE_INT;
        out_rs1_1      <= {`REG_ADDR_WIDTH{1'b0}};
        out_rs2_1      <= {`REG_ADDR_WIDTH{1'b0}};
        out_rd_1       <= {`REG_ADDR_WIDTH{1'b0}};
        out_imm_1      <= {`DATA_WIDTH{1'b0}};
        out_use_imm_1  <= 1'b0;
        out_rs1_is_fp_1<= 1'b0;
        out_rs2_is_fp_1<= 1'b0;
        out_rd_is_fp_1 <= 1'b0;
    end else begin
        out_inst_valid <= in_inst_valid;
        out_pred_taken_0 <= in_pred_taken_0;
        out_pred_target_0 <= in_pred_target_0;
        out_pred_hist_0 <= in_pred_hist_0;
        out_pred_taken_1 <= in_pred_taken_1;
        out_pred_target_1 <= in_pred_target_1;
        out_pred_hist_1 <= in_pred_hist_1;

        if (in_inst_valid[0]) begin
            out_inst_0 <= in_inst_0;
            out_pc_0 <= in_pc_0;
            out_fu_type_0  <= decode_fu_type(in_inst_0);
            out_rs1_0      <= decode_rs1(in_inst_0);
            out_rs2_0      <= decode_rs2(in_inst_0);
            out_rd_0       <= decode_rd(in_inst_0);
            out_imm_0      <= decode_imm(in_inst_0);
            out_use_imm_0  <= decode_use_imm(in_inst_0);
            out_rs1_is_fp_0<= decode_rs1_is_fp(in_inst_0);
            out_rs2_is_fp_0<= decode_rs2_is_fp(in_inst_0);
            out_rd_is_fp_0 <= decode_rd_is_fp(in_inst_0);
        end else begin
            out_inst_0 <= {`INST_WIDTH{1'b0}};
            out_pc_0 <= {`INST_ADDR_WIDTH{1'b0}};
            out_fu_type_0  <= `ALU_TYPE_INT;
            out_rs1_0      <= {`REG_ADDR_WIDTH{1'b0}};
            out_rs2_0      <= {`REG_ADDR_WIDTH{1'b0}};
            out_rd_0       <= {`REG_ADDR_WIDTH{1'b0}};
            out_imm_0      <= {`DATA_WIDTH{1'b0}};
            out_use_imm_0  <= 1'b0;
            out_rs1_is_fp_0<= 1'b0;
            out_rs2_is_fp_0<= 1'b0;
            out_rd_is_fp_0 <= 1'b0;
            out_pred_taken_0 <= 1'b0;
            out_pred_target_0 <= {`INST_ADDR_WIDTH{1'b0}};
            out_pred_hist_0 <= {`BP_GHR_BITS{1'b0}};
        end

        if (in_inst_valid[1]) begin
            out_inst_1 <= in_inst_1;
            out_pc_1 <= in_pc_1;
            out_fu_type_1  <= decode_fu_type(in_inst_1);
            out_rs1_1      <= decode_rs1(in_inst_1);
            out_rs2_1      <= decode_rs2(in_inst_1);
            out_rd_1       <= decode_rd(in_inst_1);
            out_imm_1      <= decode_imm(in_inst_1);
            out_use_imm_1  <= decode_use_imm(in_inst_1);
            out_rs1_is_fp_1<= decode_rs1_is_fp(in_inst_1);
            out_rs2_is_fp_1<= decode_rs2_is_fp(in_inst_1);
            out_rd_is_fp_1 <= decode_rd_is_fp(in_inst_1);
        end else begin
            out_inst_1 <= {`INST_WIDTH{1'b0}};
            out_pc_1 <= {`INST_ADDR_WIDTH{1'b0}};
            out_fu_type_1  <= `ALU_TYPE_INT;
            out_rs1_1      <= {`REG_ADDR_WIDTH{1'b0}};
            out_rs2_1      <= {`REG_ADDR_WIDTH{1'b0}};
            out_rd_1       <= {`REG_ADDR_WIDTH{1'b0}};
            out_imm_1      <= {`DATA_WIDTH{1'b0}};
            out_use_imm_1  <= 1'b0;
            out_rs1_is_fp_1<= 1'b0;
            out_rs2_is_fp_1<= 1'b0;
            out_rd_is_fp_1 <= 1'b0;
            out_pred_taken_1 <= 1'b0;
            out_pred_target_1 <= {`INST_ADDR_WIDTH{1'b0}};
            out_pred_hist_1 <= {`BP_GHR_BITS{1'b0}};
        end
    end
end

endmodule
