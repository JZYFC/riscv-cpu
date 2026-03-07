# 09 已知问题与局限性

本文档记录设计中已确认的缺陷、未完成的实现（TODO）和需要注意的局限性。

---

## 9.1 功能缺陷（Bugs）

### Bug 1：FP_OP_CMP 与 FP_OP_DUMMY 编码冲突

**位置**：`riscv_define.v`

```verilog
`define FP_OP_CMP  4'd15
`define FP_OP_DUMMY 4'd15  // ← 两者相同！
```

**影响**：PostDecode 解码 FP 比较指令（FEQ/FLT/FLE → `FP_OP_CMP`）时，实际值为 15，与 `FP_OP_DUMMY` 完全相同。IssueBuffer 中调用 ibex_fpu 时，`fpu_op` 为 `FP_OP_DUMMY`，无法区分是"真正的 CMP 指令"还是"无效/占位操作"。实际上 FP 比较会通过 ibex_fpu 的 case 分支处理（FP_OP_FEQ/FLT 已定义并为有效操作码），但 FP_OP_CMP（值15）作为统一接口码会路由到 ibex_fpu 的 default 分支。

**建议修复**：将 `FP_OP_DUMMY` 改为不冲突的值（如 4'd15 保留给 DUMMY，将 FP_OP_CMP 改为实际未使用的值，或统一为 FEQ/FLT/FLE 分别定义为不同操作码）。

---

### Bug 2：DCache dirty bit 永不置位（WRITEBACK 状态死代码）

**位置**：`Cache.v`

DCache 使用写透（Write-Through）策略，标签中的 dirty 位（`tag_way[TAG_BITS+1]`）**永远为 0**。因此 `victim_dirty` 永远为 0，状态机永远不会进入 `WRITEBACK` 状态。WRITEBACK 的代码逻辑本身是正确的，但**永远不会被执行**。

**影响**：目前无功能影响（写透策略本身正确），但这意味着如果未来改为写回（Write-Back）策略，需要修复 dirty bit 的设置点（当前 store 命中时只将 `temp_tag[TAG_BITS+1] = 1'b0` 清零，从不设置为 1）。

---

### Bug 3：sw/ 测试 test_add / test_add_noref 失败（exit code 3）

**位置**：`sw/tests/test_add.c`，`sw/tests/test_add_noref.c`；触发路径为 `IssueBuffer.v` / `RegRename.v`

**现象**：

```
test_add.c       → FAIL code 3  (sub(10,3) 比较失败)
test_add_noref.c → FAIL code 3  (同样是 sub(10,3)，且比较的是字面量 7，不是 s1)
test_add_notest2.c → PASS       (移除 add(-1,1) 后 sub(10,3) 通过)
test_regkeep.c / test_regkeep2.c → PASS
```

**关键推断**：

| 观察 | 排除的假设 |
|------|-----------|
| `test_add_noref.c` 与字面量 7 比较仍失败 | 不是 callee-saved 寄存器（s1）被污染 |
| `test_add_notest2.c`（无 test2）通过 | bug 与执行 `add(-1, 1)=0` 之后的状态有关 |
| `test_sub_simple.c` 通过 | 单独的 sub 函数调用无问题 |
| `test_regkeep.c` 通过（同样的 phase1→phase2→phase3 结构） | 不是通用的 callee-saved 保持 bug |

**怀疑方向**：`add(-1, 1)` 中 `add(0xFFFFFFFF, 1)` 在某些情况下触发分支预测误判，恢复路径中出现物理寄存器状态未正确恢复的边缘情况。sub 函数返回正确结果但在写回或提交过程中被旧的 speculative 值覆盖（generation tag 问题或 checkpoint 恢复时机）。

**状态**：已确认、尚未修复。此 bug 仅在 sw/ 二进制加载测试台中出现，`Top_tb.v` 的 unit-test 框架未能覆盖此场景。

---

## 9.2 已完成项（Previously TODO）

### 整数除法（DIV/DIVU/REM/REMU）✓

**完成内容**：在 `FU.v` 中新增 `IntDivider` 模块，采用 **32 周期恢复式移位-减法算法**；`IssueBuffer.v` 实例化该模块，移除原有不可综合的 `$signed()/` 内联运算符和 `div_counter` 框架。

**关键实现细节**：
- `start` 脉冲触发后 32 个时钟周期内给出结果（特殊情况 1 周期内完成）；
- 符合 RISC-V 规范边界情况：`divisor==0` → quotient=0xFFFFFFFF / remainder=dividend；`DIV(INT_MIN,-1)` → quotient=INT_MIN / remainder=0；
- flush 安全：`div_busy==0` 时丢弃 IntDivider 的完成信号，不产生错误写回；
- 修复了一个多驱动 Bug（`div_dest_valid` 同时被组合块默认赋值为 0 和时序块 NBA 赋值，导致写回永远无效）。

---

## 9.3 未实现功能（TODO）

### TODO 1：TLB 缺失处理与 MMU

**位置**：`LSU.v`，`TLB.v`

**现状**：TLB 存在但未被填充（LSU 端 `we=1'b0`），所有访问 miss 后退化为裸金属模式（VA=PA）。

**建议**：实现 page table walker，在 TLB miss 时进行页表遍历并填充 TLB；同时需要扩展异常处理（Page Fault）。

### TODO 2：完整异常处理

**位置**：`Top.v`（Trap Vector = 0x1000 = 初始 PC）

**现状**：异常时 PC 跳转到 `TRAP_VECTOR=0x1000`，即重新从头开始执行，没有独立的异常处理程序入口。CSR 的 `mepc`（异常返回地址）、`mcause`（异常原因码）等未实现。

### TODO 3：浮点部分指令（FDIV/FSQRT/FCVT/FMV）

**位置**：`PostDecode.v`，`IssueBuffer.v`，`ibex_fpu.v`

解码已完成（路由到 FP_OP_DIV/SQRT/CVT/MV），但 ibex_fpu 对这些操作的支持有限或未验证。FP load/store 已解码为 LSU 路径（正确），但 FP 数据写入 float_prf 的路径需验证。

### TODO 4：FMA（融合乘加，3操作数浮点）

**位置**：`IssueBuffer.v`（ibex_fpu 接口只传了 A 和 B，C 固定为 0）

```verilog
ibex_fpu u_fpu0 (
    .A_i(fpu_a0), .B_i(fpu_b0),
    .C_i({`DATA_WIDTH{1'b0}}),  // ← 第三操作数永远为 0
    ...
);
```

FMADD/FMSUB 等指令需要 rs3，但当前 RS 中没有 rs3 字段，ibex_fpu 的 C 端口固定为 0。

---

## 9.4 设计局限性

### 局限 1：主存容量仅 4KB

`MainMemory.v` 使用 `ram[txn_addr[11:4]]` 进行索引，只用 8 位地址，对应 256 行 × 16 字节 = **4096 字节**。超过 4KB 的地址会发生环绕（Wrap-Around），无法运行较大程序。

### 局限 2：LSU 单 outstanding

同一时间只有一条访存指令可以 in-flight。不支持访存队列（LSQ）、Store Buffer、Load-to-Store 转发。对于访存密集型程序（如矩阵乘法），会成为显著瓶颈。

### 局限 3：Branch 必须是 ROB head 才能发射

IssueBuffer 中分支指令只有当 `rs_rob_idx == rob_head` 时才允许发射，意味着所有比分支更老的指令都必须先提交。这虽然消除了"精确分支"的部分复杂性，但大幅减少了乱序执行的并行度（分支后面的指令只能等待分支执行完毕才能发射）。

更好的设计应当：分支在操作数就绪时即发射，后续指令推测执行，分支错误时 flush。当前设计需要分支到头后才发射，实际上退化为了顺序分支执行。

### 局限 4：ROB GEN 仅 2 位

2 位代号提供 4 个不同的 generation 值。如果同一 ROB 槽在短时间内被分配超过 4 次（需要 ROB 完整循环 4 次），理论上会发生 gen 碰撞。在 32 项 ROB 的典型负载下，这不太可能发生，但在极端情况下（如异常后快速重分配）需要注意。

### 局限 5：检查点内存开销

RegRename 为每个 ROB 条目（32 个）保存一份完整的映射表快照（32 架构寄存器 × 6 位物理寄存器号），以及空闲列表状态。总开销：

```
整数：32 ROB × 32 arch_regs × 6 bits + 32 × (head+tail+count 等) ≈ 6144 bits
浮点：同上
合计：约 12 KB 的寄存器文件（在 FPGA 上使用 FF/LUT 实现）
```

实际 FPGA 综合时这是一个较大的面积开销，也是检查点机制相对于 ROB 回放（replay）方案的代价。

### 局限 6：分支预测器不区分条件分支和无条件跳转

BranchPredictor 对所有分支（条件 + JAL + JALR）统一用 Gshare PHT 预测。无条件跳转（JAL/JALR）本不需要 PHT（它们总是跳转），只需要 BTB 目标。当前实现中 JAL 也会更新 PHT，对 JAL 的 PHT 表项使用略显浪费，但功能正确。

### 局限 7：PreDecode 中 CSR 立即数变体的 rs1 处理

CSR 立即数变体（CSRRWI/CSRRSI/CSRRCI，funct3 bit2=1）将指令字的 rs1 字段（bits[19:15]）当作 5 位零扩展立即数（zimm），PreDecode 将 rs1 架构号设置为 x0 以避免假依赖。但 imm 字段实际存储的是 zimm（`{27'b0, inst[19:15]}`）。在 PostDecode 阶段，`use_imm=1` 的路径使 CSR 指令使用 rs1（其值=zimm 扩展后）而非寄存器读值，这一逻辑依赖 PreDecode 和 PostDecode 两端配合正确。
