# 08 参数速查表（riscv_define.v）

所有全局宏定义集中于 `riscv_define.v`，所有其他文件通过 `` `include "riscv_define.v" `` 引入。

---

## 8.1 基础位宽与地址

| 宏 | 值 | 说明 |
|----|----|------|
| `INST_ADDR_WIDTH` | 32 | PC / 地址位宽 |
| `INST_WIDTH` | 32 | 指令字宽度 |
| `DATA_WIDTH` | 32 | 数据路径宽度 |
| `DOUBLE_DATA_WIDTH` | 64 | 双精度宽度（用于乘法结果） |
| `REG_ADDR_WIDTH` | 5 | 架构寄存器地址位宽（对应 32 个寄存器） |
| `INST_INIT` | `32'h1000` | 复位后初始 PC |
| `TRAP_VECTOR` | `32'h1000` | 异常入口地址（与初始 PC 相同） |
| `INST_ADD_STEP` | 4 | 指令步长（字节，RV32 固定 4 字节） |
| `IF_BATCH_SIZE` | 2 | 每周期取指/发射/提交批量大小 |

---

## 8.2 ALU 操作码（one-hot 编码）

`ALU_OP_WIDTH = 10`（`ALU_OP_DIV + 1 = 9 + 1`）

| 宏 | 位 | 指令映射 |
|----|----|---------|
| `ALU_OP_ADD` | bit 0 | ADD/SUB/ADDI（SUB = ADD + rs2取反 + c_in） |
| `ALU_OP_CMP` | bit 1 | SLT/SLTU/SLTI/SLTIU |
| `ALU_OP_AND` | bit 2 | AND/ANDI |
| `ALU_OP_OR` | bit 3 | OR/ORI |
| `ALU_OP_XOR` | bit 4 | XOR/XORI |
| `ALU_OP_SLL` | bit 5 | SLL/SLLI |
| `ALU_OP_SRL` | bit 6 | SRL/SRLI |
| `ALU_OP_SRA` | bit 7 | SRA/SRAI |
| `ALU_OP_MUL` | bit 8 | MUL/MULH/MULHSU/MULHU |
| `ALU_OP_DIV` | bit 9 | DIV/DIVU/REM/REMU（32 周期恢复式迭代除法器，已实现） |

---

## 8.3 ALU 功能单元类型（2位编码，PreDecode 使用）

| 宏 | 值 | 说明 |
|----|----|------|
| `ALU_TYPE_INT` | `2'b00` | 整数 ALU |
| `ALU_TYPE_MUL` | `2'b01` | 乘除法单元 |
| `ALU_TYPE_FO`  | `2'b10` | 浮点 ALU |
| `ALU_TYPE_MEM` | `2'b11` | 内存访问 |

---

## 8.4 功能单元选择（PostDecode / IssueBuffer 使用，3位编码）

| 宏 | 值 | 说明 |
|----|----|------|
| `FU_DEC_INT` | `3'd0` | 整数 ALU（ADD/AND/OR 等） |
| `FU_DEC_MULDIV` | `3'd1` | 乘除法单元 |
| `FU_DEC_LSU` | `3'd2` | Load/Store 单元 |
| `FU_DEC_BRANCH` | `3'd3` | 分支/跳转单元 |
| `FU_DEC_FP` | `3'd4` | 浮点 FPU |
| `FU_DEC_SYSTEM` | `3'd5` | CSR / FENCE |
| `FU_DEC_DUMMY` | `3'd7` | 非法/未知指令 |

---

## 8.5 分支微操作码（4位）

| 宏 | 值 | 指令 |
|----|----|------|
| `BR_OP_NONE` | 0 | 非分支 |
| `BR_OP_BEQ` | 1 | BEQ |
| `BR_OP_BNE` | 2 | BNE |
| `BR_OP_BLT` | 3 | BLT（有符号） |
| `BR_OP_BGE` | 4 | BGE（有符号） |
| `BR_OP_BLTU` | 5 | BLTU（无符号） |
| `BR_OP_BGEU` | 6 | BGEU（无符号） |
| `BR_OP_JAL` | 7 | JAL |
| `BR_OP_JALR` | 8 | JALR |

---

## 8.6 内存微操作码（3位）

| 宏 | 值 | 指令 | 说明 |
|----|----|------|------|
| `MEM_OP_LB` | 0 | LB | 有符号字节加载 |
| `MEM_OP_LH` | 1 | LH | 有符号半字加载 |
| `MEM_OP_LW` | 2 | LW | 字加载 |
| `MEM_OP_LBU` | 3 | LBU | 无符号字节加载 |
| `MEM_OP_LHU` | 4 | LHU | 无符号半字加载 |
| `MEM_OP_SB` | 5 | SB | 字节存储 |
| `MEM_OP_SH` | 6 | SH | 半字存储 |
| `MEM_OP_SW` | 7 | SW | 字存储 |

---

## 8.7 CSR 微操作码（2位）

| 宏 | 值 | 指令 |
|----|----|------|
| `CSR_OP_NONE` | 0 | 无 CSR 操作（ECALL/EBREAK/FENCE 等） |
| `CSR_OP_RW` | 1 | CSRRW / CSRRWI |
| `CSR_OP_RS` | 2 | CSRRS / CSRRSI（set bits） |
| `CSR_OP_RC` | 3 | CSRRC / CSRRCI（clear bits） |

---

## 8.8 浮点微操作码（4位）

| 宏 | 值 | 操作 | 实现状态 |
|----|----|------|---------|
| `FP_OP_ADD` | 0 | FADD.S | 已实现（ibex_fpu） |
| `FP_OP_SUB` | 1 | FSUB.S | 已实现 |
| `FP_OP_MUL` | 2 | FMUL.S | 已实现 |
| `FP_OP_MADD` | 3 | FMADD.S | 已实现 |
| `FP_OP_MSUB` | 4 | FMSUB.S | 已实现 |
| `FP_OP_NMSUB` | 5 | FNMSUB.S | 已实现 |
| `FP_OP_NMADD` | 6 | FNMADD.S | 已实现 |
| `FP_OP_SUBABS` | 7 | （自定义） | 已实现 |
| `FP_OP_FEQ` | 8 | FEQ.S | 已实现 |
| `FP_OP_FLT` | 9 | FLT.S | 已实现 |
| `FP_OP_DIV` | 10 | FDIV.S | 未实现 |
| `FP_OP_FMA` | 11 | FMA（4操作数） | 未实现 |
| `FP_OP_SQRT` | 12 | FSQRT.S | 未实现 |
| `FP_OP_CVT` | 13 | FCVT 系列 | 未实现 |
| `FP_OP_MV` | 14 | FMV.W.X 等 | 未实现 |
| `FP_OP_CMP` | 15 | FLE.S（与 DUMMY 冲突，已知 Bug） | |
| `FP_OP_DUMMY` | 15 | 占位/无效 | **与 FP_OP_CMP 值相同，存在冲突** |

---

## 8.9 ROB 参数

| 宏 | 值 | 说明 |
|----|----|------|
| `ROB_SIZE` | 32 | ROB 条目数 |
| `ROB_IDX_WIDTH` | 5 | log2(32)，ROB 索引位宽 |
| `ROB_GEN_WIDTH` | 2 | 代号位宽，每槽独立 2 位计数器 |

---

## 8.10 物理寄存器参数

| 宏 | 值 | 说明 |
|----|----|------|
| `INT_PREG_NUM` | 64 | 整数物理寄存器数量 |
| `FP_PREG_NUM` | 64 | 浮点物理寄存器数量 |
| `PREG_IDX_WIDTH` | 6 | log2(64)，物理寄存器地址位宽 |

---

## 8.11 缓存参数

| 宏 | 值 | 说明 |
|----|----|------|
| `CACHE_LINE_SIZE` | 128 | 缓存行位宽（16 字节） |
| `CACHE_INDEX_BITS` | 6 | 组索引位数（64 组） |
| `CACHE_TAG_BITS` | 22 | `32 - 6(index) - 4(offset_bits)` |
| `CACHE_WAY_NUM` | 2 | 路数（2-way 组相联） |

---

## 8.12 分支预测器参数

| 宏 | 值 | 说明 |
|----|----|------|
| `BP_GHR_BITS` | 8 | 全局历史寄存器位宽 |
| `BP_IDX_BITS` | 10 | PHT/BTB 索引位宽（1024 项） |
| `BP_RAS_DEPTH` | 8 | 返回地址栈深度 |

---

## 8.13 虚拟内存参数（Sv32 风格，未完整实现）

| 宏 | 值 | 说明 |
|----|----|------|
| `PAGE_SIZE` | 4096 | 4KB 页大小 |
| `PAGE_OFFSET_BITS` | 12 | 页内偏移位数 |
| `VPN_WIDTH` | 20 | 虚拟页号位宽 |
| `PPN_WIDTH` | 20 | 物理页号位宽 |

---

## 8.14 舍入模式（RISC-V frm 字段）

| 宏 | 值 | 说明 |
|----|----|------|
| `ROUNDING_MODE_RNE` | `3'd0` | 向最近偶数舍入（默认） |
| `ROUNDING_MODE_RTZ` | `3'd1` | 向零舍入 |
| `ROUNDING_MODE_RDN` | `3'd2` | 向负无穷舍入 |
| `ROUNDING_MODE_RUP` | `3'd3` | 向正无穷舍入 |
| `ROUNDING_MODE_RMM` | `3'd4` | 向最近幅值最大舍入 |
| `ROUNDING_MODE_DYN` | `3'd7` | 动态（使用 frm CSR，当前未实现） |
