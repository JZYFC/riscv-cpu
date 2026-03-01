# 04 后解码与发射/执行

---

## 4.1 PostDecode（PostDecode.v）

### 职责

PostDecode 是流水线中**第二次解码**（PostDecode），在 RegRename 之后进行。此时每条指令已拥有物理寄存器标签和 ROB 索引。PostDecode 的任务是将指令字翻译成**面向具体功能单元的微操作码集合**，这些信息直接进入 IssueBuffer 并在执行时使用。

PreDecode 只确定大类（INT/MEM/MUL/FP），PostDecode 则完成精细分类：

| 功能单元 | `fu_sel` 值 | 负责指令 |
|----------|------------|---------|
| INT ALU | `FU_DEC_INT` | ADD/SUB/AND/OR/XOR/SLL/SRL/SRA/SLT/SLTU/LUI/AUIPC 及 I-type 变体 |
| MulDiv | `FU_DEC_MULDIV` | MUL/MULH/MULHSU/MULHU/DIV/DIVU/REM/REMU |
| Branch | `FU_DEC_BRANCH` | BEQ/BNE/BLT/BGE/BLTU/BGEU/JAL/JALR |
| LSU | `FU_DEC_LSU` | LB/LH/LW/LBU/LHU/SB/SH/SW 及 FP load/store |
| System | `FU_DEC_SYSTEM` | CSRRW/CSRRS/CSRRC（及立即数变体）/FENCE |
| FP | `FU_DEC_FP` | FADD/FSUB/FMUL/FEQ/FLT 等 OP_FP；FMADD/FMSUB/FNMSUB/FNMADD |
| Dummy | `FU_DEC_DUMMY` | 未知/非法指令 |

### 微操作码详解

**整数 ALU（`int_op`）**：one-hot 编码，对应 `ALU_OP_ADD/CMP/AND/OR/XOR/SLL/SRL/SRA`。`int_is_sub=1` 配合 ADD 实现减法（对 rs2 取反后 carry_in=1）；`cmp_signed=1` 区分有符号/无符号比较。

**乘除法（`muldiv_op`）**：`muldiv_op[ALU_OP_MUL]=1` 表示乘法，`mul_high` 决定取高 32 位还是低 32 位，`mul_signed_rs1/rs2` 控制有符号扩展；`muldiv_op[ALU_OP_DIV]=1` 表示除法，`div_signed` 和 `div_is_rem` 控制具体类型。

**分支（`branch_op`）**：`BR_OP_BEQ/BNE/BLT/BGE/BLTU/BGEU/JAL/JALR`。

**访存（`mem_op`）**：`MEM_OP_LB/LH/LW/LBU/LHU/SB/SH/SW`；`mem_is_load` 区分读写；`mem_unsigned` 控制零扩展。

**CSR（`csr_op`, `csr_addr`）**：`CSR_OP_RW/RS/RC`（read-write/read-set/read-clear），`csr_addr` 为 12 位 CSR 地址。

**非法指令**：`illegal=1` + `fu_sel=FU_DEC_DUMMY`，在 IssueBuffer 中将触发异常写回。

### 实现方式

PostDecode 用一个 `decode_one` task 实现单条指令的完整解码。每周期对 in_inst_0 和 in_inst_1 并行调用两次（组合逻辑），结果在时钟上升沿时寄存。flush 时清零，stall 时保持。

---

## 4.2 IssueBuffer（IssueBuffer.v）——保留站

IssueBuffer 是乱序执行的核心，集成了**保留站（RS）**、**操作数读取**、**执行单元调度**和**写回/重定向**逻辑。

### 4.2.1 保留站结构

RS 是一个并行数组，深度 8（`RS_DEPTH=8`）。每个 RS 槽包含一条指令的全部执行所需信息：

| 类别 | 字段 | 说明 |
|------|------|------|
| 状态 | `rs_valid` | 该槽是否被占用 |
| 年龄 | `rs_age` | 全局计数器快照，越小越老 |
| 功能单元 | `rs_fu_sel` | 目标功能单元 |
| 操作码 | `rs_int_op` `rs_muldiv_op` `rs_branch_op` `rs_mem_op` `rs_csr_op` `rs_fp_op` | 各 FU 操作码 |
| 标志 | `rs_int_sub` `rs_cmp_signed` `rs_mul_high` ... | 精细控制位 |
| 操作数 | `rs_rs1_ready/rs2_ready` | 是否就绪 |
| 操作数 | `rs_rs1_tag/rs2_tag` | 物理寄存器号 |
| 操作数 | `rs_rs1_val/rs2_val` | 就绪时的数值 |
| 目的 | `rs_rd_tag` `rs_rd_is_fp` | 物理目的寄存器 |
| ROB | `rs_rob_idx` `rs_rob_gen` | ROB 条目信息 |
| 预测 | `rs_pred_taken` `rs_pred_target` `rs_pred_hist` | 分支预测元数据 |
| 辅助 | `rs_arch_rd` `rs_arch_rs1` | 架构寄存器号（用于 call/return 识别） |
| 指令 | `rs_inst` `rs_pc` `rs_imm` `rs_use_imm` | 原始指令和立即数 |

### 4.2.2 入队（Enqueue）

每周期最多 2 条指令入队。入队时：

1. **背压检查**：若 RS 空闲槽数 < 入队需求，输出 `stall_dispatch`（传递回 PostDecode/RegRename）；
2. **CDB 旁路**：入队时检查 CDB（当周期是否恰好有写回能满足该指令的操作数）：

```verilog
rs1_ready_now_0 = rs1_is_x0_0 || in_rs1_ready_now_0
               || (cdb0_valid && cdb0_tag == in_rs1_preg_0)
               || (cdb1_valid && cdb1_tag == in_rs1_preg_0);
```

3. **x0 优化**：若源寄存器是整数 x0，直接标记就绪、值为 0，无需等待；
4. 找到一个空闲 RS 槽，写入所有字段；年龄计数器 `age_counter` 自增后存入 `rs_age`。

### 4.2.3 操作数唤醒（Wakeup）

每周期，当 `cdb0` 或 `cdb1` 上有有效写回时，**广播**到所有 RS 槽：

```verilog
// 对每个 RS 槽
if (!rs_rs1_ready[i] && rs_rs1_tag[i] == cdb0_tag && cdb0_valid) begin
    rs_rs1_ready[i] <= 1'b1;
    rs_rs1_val[i]   <= cdb0_value;
end
// 类似地处理 cdb1 和 rs2
```

注意 CDB 只有 2 路（对应 wb0/wb1），LSU 写回（wb2/lsu_wb）通过单独的逻辑广播唤醒等待 lsu_wb_dest_tag 的 RS 槽。

### 4.2.4 指令选择（Issue Selection）

每周期在 RS 中选出**最多 2 条**就绪且允许执行的指令（issue_idx0, issue_idx1）：

**就绪条件**（`base_ready_mask`）：`rs_valid && rs_rs1_ready && rs_rs2_ready`

**特殊约束**：
- **LSU**：只有当前最老的 LSU 指令（按 ROB 距离 head 最近）可以发射；Store 必须等到其 ROB idx == rob_head（到达 ROB 头部，即它是最老的未提交指令，确保 store 不会被错误路径执行）；
- **Branch**：branch 指令只有当 `rs_rob_idx == rob_head`（该分支是 ROB 中最老的指令）时才允许发射；这保证了分支解析是"精确的"——比它更老的指令都已提交；
- **allow_issue1**：LSU 写回的周期（`lsu_wb_valid`）不允许同时发射第二条指令（WB 端口冲突）。

选择策略：从满足 `issue_ready_mask` 的槽中选最老的（`rs_age` 最小）；issue_idx1 再从剩余满足条件的槽中选次老的（排除 issue_idx0）。若 issue_idx0 是 Branch，不选 issue_idx1（branch 单独处理）。

### 4.2.5 操作数路径

```verilog
// opA：若是 AUIPC，使用 PC；否则使用 rs1
opA_0 = (rs_inst[issue_idx0][6:0] == OPCODE_AUIPC) ? rs_pc[issue_idx0]
                                                    : rs_rs1_val[issue_idx0];
// opB：若使用立即数，用 imm；否则用 rs2
opB_0 = rs_use_imm[issue_idx0] ? rs_imm[issue_idx0] : rs_rs2_val[issue_idx0];
// SUB：对 opB 取反（~opB），carry_in=1，实现 rs1 - rs2
opB_eff_0 = rs_int_sub[issue_idx0] ? ~opB_0 : opB_0;
```

---

## 4.3 执行单元（Functional Units）

### 4.3.1 IntAlu（FU.v）

纯组合逻辑，无状态，并行计算所有运算，最后通过 one-hot `ALU_op` 选择结果：

| 操作 | 实现 |
|------|------|
| ADD | `rs1 + rs2 + add_c_in`（SUB = ADD with rs2取反，c_in=1） |
| CMP | `rs1 < rs2`，有符号或无符号，结果 0/1 |
| AND | `rs1 & rs2` |
| OR | `rs1 \| rs2` |
| XOR | `rs1 ^ rs2` |
| SLL | `rs1 << rs2[4:0]` |
| SRL | `rs1 >> rs2[4:0]`（逻辑右移） |
| SRA | `$signed(rs1) >>> rs2[4:0]`（算术右移） |

IssueBuffer 中实例化 2 个 IntAlu（`u_alu0`、`u_alu1`），分别服务 issue_idx0 和 issue_idx1。

**LUI**：实现为 `ADD x0(=0) + imm`；**AUIPC**：实现为 `ADD PC + imm`（opA 切换为 PC）。

### 4.3.2 乘法（IssueBuffer 内联）

**MUL 系列（单周期）**：在 IssueBuffer 内直接用 Verilog 乘法实现，不调用 IntMulDivALU：

```verilog
// 33 位有符号扩展后相乘，取低 32 位（MUL）或高 32 位（MULH/MULHSU/MULHU）
wire signed [32:0] mul_a0 = mul_signed_rs1 ? {rs1[31], rs1} : {1'b0, rs1};
wire signed [32:0] mul_b0 = mul_signed_rs2 ? {rs2[31], rs2} : {1'b0, rs2};
wire signed [65:0] mul_full0 = mul_a0 * mul_b0;
wire [31:0] mul_res0 = mul_high ? mul_full0[63:32] : mul_full0[31:0];
```

**DIV 系列（32 周期迭代）**：IssueBuffer 实例化 `IntDivider u_divider`（`FU.v` 中定义），采用恢复式移位-减法算法，固定 32 个迭代步骤完成一次除法，结果通过 `div_done_wire` 和 `div_result_wire` 返回。相关控制寄存器：

| 信号 | 说明 |
|------|------|
| `div_busy` | 除法器正在运行，阻止新的除法发射 |
| `div_start` | 1 周期脉冲，向 IntDivider 发起计算 |
| `div_op1_r` / `div_op2_r` | 锁存的被除数 / 除数 |
| `div_is_signed_r` | 有符号（DIV/REM）或无符号（DIVU/REMU） |
| `div_is_rem_r` | 输出余数（REM）还是商（DIV） |
| `div_done_wire` | IntDivider 完成脉冲（1 周期） |
| `div_result_wire` | IntDivider 输出结果 |
| `div_value` | 锁存最终结果，供写回使用 |

**RISC-V 规范边界情况**（在 IntDivider 内部检测，1 周期内完成）：

| 情况 | 期望商 | 期望余数 |
|------|--------|----------|
| `divisor == 0` | `0xFFFFFFFF`（DIV）/ `0xFFFFFFFF`（DIVU） | `dividend`（REM/REMU） |
| `DIV(INT_MIN, -1)`（有符号溢出） | `0x80000000`（INT_MIN） | `0` |

**flush 安全**：flush 时 `div_busy <= 0`，即使 IntDivider 稍后完成，也因 `div_busy == 0` 而被忽略，不会产生错误写回。

### 4.3.3 Branch 执行（IssueBuffer 内联）

分支在 IssueBuffer 内以纯组合逻辑解析：

```verilog
// 条件分支
BR_OP_BEQ:  br_taken = (rs1 == rs2);
BR_OP_BNE:  br_taken = (rs1 != rs2);
BR_OP_BLT:  br_taken = ($signed(rs1) < $signed(rs2));
BR_OP_BGE:  br_taken = ($signed(rs1) >= $signed(rs2));
BR_OP_BLTU: br_taken = (rs1 < rs2);   // 无符号
BR_OP_BGEU: br_taken = (rs1 >= rs2);  // 无符号

// 无条件跳转
BR_OP_JAL:  br_taken = 1; target = pc + imm;
BR_OP_JALR: br_taken = 1; target = (rs1 + imm) & ~1;  // 清除最低位
```

**错误预测检测**：

```verilog
br_mispredict = (br_taken != pred_taken) ||
                (br_taken && pred_taken && (br_target != pred_target));
```

若错误预测：发出 `redirect_valid=1` 和 `redirect_target`，ROB `flush_rob_idx` 为该分支的 ROB 索引。

**call/return 识别**（用于 BP 更新）：
- call = JAL/JALR 且 `arch_rd = x1（ra）`；
- return = JALR 且 `arch_rs1 = x1（ra）` 且 `arch_rd = x0`。

**写回值**：JAL/JALR 写回 `link = PC + 4`（返回地址），条件分支无写回（rd=x0）。

### 4.3.4 System/CSR（IssueBuffer 内联）

CSR 文件实现为 4096 项数组 `csr_file[0:4095]`。执行为纯组合逻辑：

```
csr_old = csr_file[csr_addr]   // 读旧值（作为写回结果返回给 rd）
根据 csr_op 计算 csr_new：
  CSR_OP_RW: csr_new = rs1（或 zimm）
  CSR_OP_RS: csr_new = csr_old | rs1
  CSR_OP_RC: csr_new = csr_old & ~rs1
```

CSR 写在时钟边沿执行（`if csr_we: csr_file[addr] <= csr_new`）。ECALL/EBREAK/FENCE 作为 SYSTEM 类型但 `csr_op=NONE`，不改写 CSR；FENCE 视为 NOP 但可触发异常路径（通过 `illegal` 标记）。

### 4.3.5 浮点 FPU（ibex_fpu.v）

IssueBuffer 实例化 2 个 `ibex_fpu` 单元（`u_fpu0`、`u_fpu1`），接受来自 issue_idx0/1 的操作数和 FP 操作码。ibex_fpu 是一个组合逻辑单周期 FPU，来自 lowRISC 的 ibex 项目，支持 IEEE 754 单精度浮点运算。

**已接入**：FP_OP_ADD, FP_OP_SUB, FP_OP_MUL, FP_OP_MADD（fma）, FP_OP_FEQ, FP_OP_FLT 等。
**未完全实现**：FP_OP_DIV, FP_OP_SQRT, FP_OP_CVT, FP_OP_MV（部分通过 FP_OP_DUMMY 处理）。

舍入模式当前固定为 `ROUNDING_MODE_RNE`（向最近偶数舍入），不依赖 CSR frm 字段。

### 4.3.6 写回仲裁（WB Arbitration）

每周期最多 3 路写回（wb0/wb1/wb2）：

- **wb0/wb1**：来自 cand0/cand1（issue_idx0/1 的执行结果，INT/Branch/System/FP/MUL）和 cand2/cand3（按 pick 计数轮流占用）；
- **wb2**：固定给 LSU 的无目的写回（`lsu_nodest_valid`）；
- **cand2**：`div_done && div_dest_valid`，当 IntDivider 完成时进入 wb0 或 wb1（取决于当周期 pick 数）；
- **cand3**：`lsu_wb_valid && lsu_wb_dest_valid`，LSU 有目的寄存器时进入 wb0 或 wb1。

优先级（pick 顺序）：cand3 > cand0 > cand1 > cand2；pick 满 2 时后续 cand 本周期不写回。

写回到：
1. **PRF**（通过 IssueBuffer 的 `i_wr_addr0/1`、`f_wr_addr0/1` 输出）；
2. **ROB**（通过 `wb0/1/2_rob_idx`，标记条目 ready）；
3. **记分牌**（RegRename 中的 `int_preg_ready`，通过收到 wb 信号后更新）。
