# 02 取指前端

取指前端由四个模块构成：**IF（取指级）**、**BranchPredictor（分支预测器）**、**InstructionBuffer（指令缓冲 FIFO）**、**PreDecode（预解码）**。它们负责将指令从 ICache 取出，预测控制流方向，并在进入重命名级之前完成第一轮指令字段分解。

---

## 2.1 IF 模块（IF.v）

### 职责

- 维护**程序计数器（PC）寄存器** `reg_PC`，初始值 `0x0000_1000`；
- 向 ICache 发起取指请求，每次请求一个 128 位缓存行（覆盖 4 条 32 位指令）；
- 从缓存行中提取最多 **2 条**指令字（`inst0`、`inst1`）；
- 调用 **BranchPredictor** 进行预测，根据预测结果更新 PC；
- 将指令字、PC、预测元数据输出给下游的 `InstructionBuffer`。

### 缓存行管理

IF 内部维护一个**单行行缓存**（`line_valid`、`line_base`、`line_data`），避免每周期都向 ICache 发出请求：

```
line_valid : 当前行缓存是否有效
line_base  : 缓存行地址的高位（bits[31:4]）
line_data  : 128 位指令数据
```

命中判断：`line_hit = line_valid && (line_base == reg_PC[31:4])`

若未命中（`need_line`），则向 ICache 发出 `ic_req`，等待 `ic_valid` 返回后安装到行缓存。

为了防止 **flush 后旧响应被误接受**，IF 使用一个 `fetch_epoch` 计数器和 `pending_line_epoch`，仅当 epoch 匹配时才接受 ICache 返回的行数据。

### 批量取指与跨行处理

每次取指尝试提取 2 条指令：
- `word_idx = reg_PC[3:2]`（当前行内的 32 位字偏移）
- 若 `word_idx == 2'b11`（第 4 个字，跨行边界），则第二条指令不可用（`cross_line=1`），只输出 1 条，下一周期再取下一行

```verilog
// 有效掩码：分支预测到跳转时只输出第一条（停止推测序列取指）
fetch_valid_mask = bp_pred_taken ? 2'b01 :
                  (cross_line   ? 2'b01 : 2'b11);
```

### PC 更新策略

```
chosen_target = bp_pred_taken ? bp_pred_target : seq_next_pc
// 顺序 PC：若未跨行 +8（两条），若跨行 +4（一条）
seq_next_pc   = reg_PC + (cross_line ? 4 : 8)
```

分支预测命中时，`chosen_target` 为预测目标地址，流水线继续从该地址推测取指。

### 输出信号

| 信号 | 宽度 | 说明 |
|------|------|------|
| `out_inst_addr_0/1` | 32 | 两条指令的 PC |
| `out_inst_0/1` | 32 | 指令字 |
| `out_inst_valid` | 2 | 有效掩码（bit0=inst0有效，bit1=inst1有效） |
| `out_pred_taken_0/1` | 1 | 分支预测是否跳转 |
| `out_pred_target_0/1` | 32 | 预测目标 PC |
| `out_pred_hist_0/1` | 8 | 全局历史寄存器快照（用于后续更新） |

---

## 2.2 BranchPredictor（BranchPredictor.v）

### 概述

采用经典的 **Gshare** 方案：

```
预测索引 = GHR[9:0] XOR PC[11:2]   // GHR 8位，IDX_BITS=10
```

包含三个组件：
1. **PHT（Pattern History Table）**：1024 项 2 位饱和计数器，初始为 `2'b01`（弱不跳转）；
2. **BTB（Branch Target Buffer）**：1024 项，存储目标地址、Tag、is_return 标记；
3. **RAS（Return Address Stack）**：8 项，用于加速 `JALR ra` 返回地址预测。

### 预测流程（纯组合逻辑）

```
fetch_idx = GHR XOR PC[IDX_BITS+1:2]
fetch_tag = PC[31:IDX_BITS+2]

btb_hit   = btb_valid[fetch_idx] && (btb_tag[fetch_idx] == fetch_tag)
pred_taken = fetch_valid && btb_hit && pht[fetch_idx][1]  // 强跳转（MSB=1）

// 若是 return 指令，从 RAS 顶取目标
btb_target_sel = btb_is_ret[fetch_idx] ? ras_top : btb_target[fetch_idx]
pred_target    = btb_hit ? btb_target_sel : (PC + 8)
```

**关键设计**：只有 BTB 命中才预测为跳转，避免在非分支 PC 处误抑制 slot1 的有效性。

### 更新流程（每周期最多 2 个更新，来自 IssueBuffer 执行结果）

每个周期最多处理 2 个已解析的分支结果（`update0`、`update1`）。更新在 `always @(posedge clk)` 块中**顺序**应用（先 update0，再 update1），使用**阻塞赋值**以保证两次更新之间的数据连贯性。

更新内容：
- **PHT**：根据实际结果对计数器加 1 或减 1（饱和）；
- **BTB**：若实际跳转，更新目标地址和 Tag；
- **RAS**：
  - `is_call=1`（rd=x1 的 JAL/JALR）→ 压栈 PC+4；
  - `is_return=1`（rs1=x1, rd=x0 的 JALR）→ 弹栈；
- **GHR**：移位并插入实际跳转结果 `{GHR[6:0], taken}`。

**注意**：flush 时**不清空**预测器状态，因为 flush 发生在分支解析后的下一周期，若此时清空，会丢失刚训练的分支信息，导致重复错误预测。

---

## 2.3 InstructionBuffer（InstructionBuffer.v）

### 职责

一个**环形 FIFO 队列**（深度 8），解耦 IF 和 PreDecode 的流量：

- 每周期可**入队最多 2 条**（来自 IF）；
- 每周期可**出队最多 2 条**（送给 PreDecode，当 PreDecode 准备好时）；
- 若队列空间不足，向 IF 发出 `stall_if` 背压信号。

### 容量管理

```verilog
in_cnt  = in_valid[0] + in_valid[1]  // 本周期入队数（0/1/2）
deq_cnt = out_ready ? min(count, 2) : 0  // 本周期出队数
free_slots = DEPTH - count
avail_next = free_slots + deq_cnt      // 同周期出队后的可用空间
can_enq  = (in_cnt <= avail_next)
stall_if = !can_enq
```

存储的每个 FIFO 槽包含：指令字、PC、预测跳转标志、预测目标、GHR 快照。

### 与 PreDecode 的握手

`out_ready` 由 PreDecode 侧的 `!stall` 驱动（rename 是否阻塞决定）。InstructionBuffer 的输出是**组合逻辑**（直接读 FIFO head），时序注册在 PreDecode 内部完成。

---

## 2.4 PreDecode（PreDecode.v）

### 职责

接收来自 InstructionBuffer 的最多 2 条指令，在**一个时钟周期内**（注册输出）完成**第一轮解码**：

- 识别**功能单元类型** `fu_type`（INT/MUL/MEM/FP）；
- 提取**架构寄存器地址** rs1/rs2/rd；
- 解码并符号/零扩展**立即数** imm；
- 标记是否**使用立即数**（`use_imm`）；
- 标记寄存器是否为**浮点寄存器**（`rs1_is_fp`/`rs2_is_fp`/`rd_is_fp`）。

### 功能单元类型识别（ALU_TYPE）

```verilog
// 仅通过 opcode[6:0] 确定大类，不细化操作码（由 PostDecode 完成）
OPCODE_OP      : M 扩展(funct7=0000001) → ALU_TYPE_MUL，否则 → ALU_TYPE_INT
OPCODE_OP_IMM,
OPCODE_LUI/AUIPC/JAL/JALR/BRANCH/SYSTEM → ALU_TYPE_INT
OPCODE_LOAD/STORE/MISC_MEM              → ALU_TYPE_MEM
OPCODE_LOAD_FP/STORE_FP/OP_FP/MADD/... → ALU_TYPE_FO
```

### 立即数解码

严格按 RISC-V 规范的 5 种编码格式：

| 格式 | 指令类型 | 示例 |
|------|----------|------|
| I-type | ADDI, LOAD, JALR, MISC_MEM | `{20{inst[31]}, inst[31:20]}` |
| S-type | STORE | `{20{inst[31]}, inst[31:25], inst[11:7]}` |
| B-type | BRANCH | `{19{inst[31]}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0}` |
| U-type | LUI, AUIPC | `{inst[31:12], 12'b0}` |
| J-type | JAL | `{11{inst[31]}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0}` |

**CSR 特殊处理**：CSR 立即数指令（funct3[2]=1）将 rs1 字段视为 5 位零扩展立即数（zimm），rs1 架构号返回 x0 以避免假依赖。

### 寄存器地址规则

| 指令类型 | rs1 | rs2 | rd |
|----------|-----|-----|----|
| LUI/AUIPC/JAL | x0（无 rs1） | 无 | inst[11:7] |
| BRANCH/STORE | inst[19:15] | inst[24:20] | x0（无写回） |
| CSR 立即数变体 | x0 | x0 | inst[11:7] |
| 其余 | inst[19:15] | inst[24:20]或x0 | inst[11:7] |

### 注意

PreDecode 是**流水线寄存器级**（registered outputs），flush 时清零所有输出，stall 时保持旧值不更新。这一对称模式在所有流水线级间寄存器中保持一致。
