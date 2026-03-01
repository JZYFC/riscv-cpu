# RISC-V 乱序执行 CPU —— 设计文档索引

## 项目简介

本项目是一个用 Verilog HDL 实现的 **RISC-V 32 位乱序执行（Out-of-Order, OoO）CPU**，运行于 Vivado 仿真环境。设计目标是实现一个具备现代高性能 CPU 核心特征的处理器，支持指令级并行（ILP），同时保持精确异常语义。

**ISA 支持：** RV32I（整数基本集）、RV32M（乘除法完整实现：乘法单周期，除法 32 周期恢复式迭代除法器，符合 RISC-V 规范边界情况）、部分 RV32F（浮点解码 + ibex_fpu 单元已接入）、基础 CSR 指令。

---

## 文档目录

| 文档 | 内容 |
|------|------|
| [01_architecture.md](01_architecture.md) | **整体架构**：系统参数、流水线纵览、模块关系图、数据流全貌 |
| [02_frontend.md](02_frontend.md) | **取指前端**：IF 模块、指令缓冲（InstructionBuffer）、分支预测器（BranchPredictor）、预解码（PreDecode） |
| [03_rename_rob.md](03_rename_rob.md) | **寄存器重命名与重排序缓冲**：RegRename 模块、ROB 模块、记分牌、检查点机制 |
| [04_postdecode_issue.md](04_postdecode_issue.md) | **后解码与发射执行**：PostDecode、IssueBuffer（保留站）、所有执行单元（INT ALU / MulDiv / Branch / System / FP） |
| [05_memory.md](05_memory.md) | **存储子系统**：LSU、数据缓存（Cache）、指令缓存（ICache）、TLB、内存仲裁器（MemArbiter）、主存（MainMemory） |
| [06_prf.md](06_prf.md) | **物理寄存器堆**：PhysicalRegFileShared，读写端口、旁路机制 |
| [07_control_flow.md](07_control_flow.md) | **全局控制与异常**：冲刷（flush）、重定向（redirect）、提交（commit）的完整信号链 |
| [08_parameters.md](08_parameters.md) | **参数速查表**：riscv_define.v 中所有宏定义的含义与取值 |
| [09_known_issues.md](09_known_issues.md) | **已知问题与局限性**：当前实现的缺陷、TODO 项、潜在陷阱 |

---

## 源文件对应关系

```
riscv_define.v          → 所有全局宏/参数定义（见 08_parameters.md）
Top.v                   → 顶层连线（贯穿全部文档）
IF.v                    → 取指级（见 02_frontend.md）
BranchPredictor.v       → Gshare + BTB + RAS（见 02_frontend.md）
InstructionBuffer.v     → IF→PreDecode 之间的 FIFO 缓冲（见 02_frontend.md）
PreDecode.v             → 第一遍解码（见 02_frontend.md）
RegRename.v             → 寄存器重命名 + ROB（见 03_rename_rob.md）
PostDecode.v            → 第二遍解码（见 04_postdecode_issue.md）
IssueBuffer.v           → 保留站 + 执行单元调度（见 04_postdecode_issue.md）
FU.v                    → IntALU / IntMulDivALU（见 04_postdecode_issue.md）
ibex_fpu.v              → 浮点 FPU（来自 ibex 开源项目）
PhysicalRegFileShared.v → 物理寄存器堆（见 06_prf.md）
LSU.v                   → 访存单元（见 05_memory.md）
Cache.v                 → 数据缓存（见 05_memory.md）
ICache.v                → 指令缓存（见 05_memory.md）
TLB.v                   → 地址转换表（见 05_memory.md）
MemArbiter.v            → I$/D$ 总线仲裁（见 05_memory.md）
MainMemory.v            → 仿真主存模型（见 05_memory.md）
Top_tb.v                → 仿真顶层 testbench
```

---

## 快速上手：一条指令的完整旅程

以一条 `ADD x3, x1, x2` 为例，追踪它从取指到提交的全过程：

1. **IF** 从 ICache 取回 128 位缓存行，提取对应 32 位指令字，调用分支预测器；
2. **InstructionBuffer** 暂存指令，解耦取指和解码速率；
3. **PreDecode** 识别 rs1/rs2/rd/imm/fu_type；
4. **RegRename** 将 x1→p_rs1、x2→p_rs2，为 x3 分配新物理寄存器 p_rd_new，旧映射 p_rd_old 记入 ROB；
5. **PostDecode** 将 `ADD` 翻译成 `FU_DEC_INT + ALU_OP_ADD`；
6. **IssueBuffer** 在保留站等待 p_rs1、p_rs2 均就绪，选出最老的就绪 INT 指令发射；
7. **IntALU** 计算 rs1+rs2；
8. **写回（WB）** 结果写入 PRF[p_rd_new]，ROB 条目标记 ready；
9. **提交（Commit）** ROB head 指向该指令，释放 p_rd_old，架构状态更新；
10. 若提交前发现异常则 flush 全流水线，PC 跳转至 TRAP_VECTOR。
