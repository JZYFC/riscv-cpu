`define INST_ADDR_WIDTH 32    // Instruction address width(aka PC width)
`define INST_WIDTH 32         // Instruction width, same as DATA_WIDTH
`define DATA_WIDTH 32       // Data width
`define DOUBLE_DATA_WIDTH 64 // Double data width (for mul/div)
`define REG_ADDR_WIDTH 5    // Register address width (32 logical registers)

`define INST_INIT     `INST_ADDR_WIDTH'h1000 // Initial PC value
`define TRAP_VECTOR   `INST_INIT             // Trap/exception entry
`define INST_ADD_STEP 4    // PC increment step for each instruction

`define IF_BATCH_SIZE 2    // Number of instructions fetched in one batch. Check out_inst_{0-1} in IF module

// Branch predictor parameters
`define BP_GHR_BITS  8
`define BP_IDX_BITS  10
`define BP_RAS_DEPTH 8

// ALU ops
`define ALU_OP_ADD 4'b0000 // sub can be achieved by add with c_in=1 and rs2 negated (2's complement)
                           // ISA cover: ADD SUB ADDI
`define ALU_OP_CMP 4'b0001 // ISA cover: SLT SLTU SLTI SLTIU
`define ALU_OP_AND 4'b0010 // ISA cover: AND ANDI
`define ALU_OP_OR  4'b0011 // ISA cover: OR ORI
`define ALU_OP_XOR 4'b0100 // ISA cover: XOR XORI
`define ALU_OP_SLL 4'b0101 // ISA cover: SLL SLLI
`define ALU_OP_SRL 4'b0110 // ISA cover: SRL SRLI
`define ALU_OP_SRA 4'b0111 // ISA cover: SRA SRAI

// ALU mul/div ops
`define ALU_OP_MUL 4'b1000    // ISA cover: MUL MULH MULHSU MULHU
`define ALU_OP_DIV 4'b1001    // ISA cover: DIV DIVU REM REMU

// CHANGEME: when add new operations
`define ALU_OP_WIDTH `ALU_OP_DIV+ 1

`define ALU_TYPE_INT 2'b00 // Integer ALU
`define ALU_TYPE_MUL 2'b01 // Integer Multiplier/Divider ALU
`define ALU_TYPE_FO  2'b10 // Floating-point ALU (not implemented yet)
`define ALU_TYPE_MEM 2'b11 // Memory access (not implemented yet)

// Extended decode opcodes beyond integer ALU for other functional units.
`define DECODE_OP_LOAD   `ALU_OP_WIDTH     // memory load
`define DECODE_OP_STORE  `ALU_OP_WIDTH+1   // memory store
`define DECODE_OP_BRANCH `ALU_OP_WIDTH+2   // branch (compare + PC add)
`define DECODE_OP_JUMP   `ALU_OP_WIDTH+3   // jal / jalr
`define DECODE_OP_LUI    `ALU_OP_WIDTH+4   // load upper immediate
`define DECODE_OP_AUIPC  `ALU_OP_WIDTH+5   // add upper immediate to PC
`define DECODE_OP_SYSTEM `ALU_OP_WIDTH+6   // fence / ecall / ebreak / csr
`define DECODE_OP_FP     `ALU_OP_WIDTH+7   // floating-point operation placeholder
`define DECODE_OP_DUMMY  `ALU_OP_WIDTH+8   // unknown / unsupported opcode
`define DECODE_OP_WIDTH  `ALU_OP_WIDTH+9

// Post-decode functional unit selections (after rename)
`define FU_DEC_WIDTH 3
`define FU_DEC_INT       3'd0
`define FU_DEC_MULDIV    3'd1
`define FU_DEC_LSU       3'd2
`define FU_DEC_BRANCH    3'd3
`define FU_DEC_FP        3'd4
`define FU_DEC_SYSTEM    3'd5
`define FU_DEC_DUMMY     3'd7

// Branch micro-ops
`define BR_OP_WIDTH 4
`define BR_OP_NONE  4'd0
`define BR_OP_BEQ   4'd1
`define BR_OP_BNE   4'd2
`define BR_OP_BLT   4'd3
`define BR_OP_BGE   4'd4
`define BR_OP_BLTU  4'd5
`define BR_OP_BGEU  4'd6
`define BR_OP_JAL   4'd7
`define BR_OP_JALR  4'd8

// Memory micro-ops (load/store + size/sign)
`define MEM_OP_WIDTH 3
`define MEM_OP_LB    3'd0
`define MEM_OP_LH    3'd1
`define MEM_OP_LW    3'd2
`define MEM_OP_LBU   3'd3
`define MEM_OP_LHU   3'd4
`define MEM_OP_SB    3'd5
`define MEM_OP_SH    3'd6
`define MEM_OP_SW    3'd7

// CSR/system micro-ops
`define CSR_OP_WIDTH 2
`define CSR_OP_NONE  2'd0
`define CSR_OP_RW    2'd1
`define CSR_OP_RS    2'd2
`define CSR_OP_RC    2'd3

// Floating-point micro-ops (placeholder set)
`define FP_OP_WIDTH 4
`define FP_OP_ADD   4'd0
`define FP_OP_SUB   4'd1
`define FP_OP_MUL   4'd2
`define FP_OP_MADD   4'd3
`define FP_OP_MSUB   4'd4
`define FP_OP_NMSUB  4'd5
`define FP_OP_NMADD   4'd6
`define FP_OP_SUBABS    4'd7
`define FP_OP_FEQ   4'd8
`define FP_OP_FLT 4'd9
// these micro-ops are not implemented yet
`define FP_OP_DIV   4'd10
`define FP_OP_FMA   4'd11
`define FP_OP_SQRT  4'd12
`define FP_OP_CVT   4'd13
`define FP_OP_MV    4'd14
`define FP_OP_CMP   4'd15
`define FP_OP_DUMMY 4'd15

// Rounding mode (RISC-V frm encoding)
`define ROUNDING_MODE_WIDTH 3

`define ROUNDING_MODE_RNE 3'd0  // Round to Nearest, ties to Even
`define ROUNDING_MODE_RTZ 3'd1  // Round towards Zero
`define ROUNDING_MODE_RDN 3'd2  // Round Down (towards -inf)
`define ROUNDING_MODE_RUP 3'd3  // Round Up (towards +inf)
`define ROUNDING_MODE_RMM 3'd4  // Round to Nearest, ties to Max Magnitude
`define ROUNDING_MODE_DYN 3'd7  // Dynamic (use frm CSR)

// ROB parameters
`define ROB_SIZE      32
`define ROB_IDX_WIDTH 5   // log2(ROB_SIZE)

// Physical register file parameters (can exceed architectural count)
`define INT_PREG_NUM   64
`define FP_PREG_NUM    64
`define PREG_IDX_WIDTH 6   // log2(max(INT_PREG_NUM, FP_PREG_NUM))
// 虚拟内存 Sv32 参数
`define PAGE_SIZE       4096       // 4KB 页大小
`define PAGE_OFFSET_BITS 12        // 偏移量位宽
`define VPN_WIDTH       20         // 虚拟页号 (32 - 12)
`define PPN_WIDTH       20         // 物理页号 (Sv32 PPN也是20位，简化处理)

// Cache 参数 (2路组相联，每块16字节)
`define CACHE_LINE_SIZE 128        // 128-bit (16 Bytes) 数据块
`define CACHE_INDEX_BITS 6         // 64组 (Sets)
`define CACHE_TAG_BITS  (32 - `CACHE_INDEX_BITS - 4) // 32 - 6 - 4(offset) = 22位 Tag
`define CACHE_WAY_NUM   2          // 2路
