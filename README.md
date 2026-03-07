# RISC-V 乱序执行 CPU —— 设计文档索引

## 项目简介

本项目是一个用 Verilog HDL 实现的 **RISC-V 32 位乱序执行（Out-of-Order, OoO）CPU**，运行于 Vivado 仿真环境。设计目标是实现一个具备现代高性能 CPU 核心特征的处理器，支持指令级并行（ILP），同时保持精确异常语义。

**ISA 支持：** RV32I（整数基本集）、RV32M（乘除法完整实现：乘法单周期，除法 32 周期恢复式迭代除法器，符合 RISC-V 规范边界情况）、部分 RV32F（浮点解码 + ibex_fpu 单元已接入）、基础 CSR 指令。

## 项目Quick Start

仓库提供了 `scripts/create_project.tcl` 脚本，恢复 Vivado 工程结构。在项目根目录运行：

```bash
vivado -mode batch -source scripts/create_project.tcl
```

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
| [10_software_tests.md](10_software_tests.md) | **软件测试基础设施**：sw/ 目录工具链、Top_tb_bin.v 测试台、测试用例说明与结果 |

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
Top_tb.v                → 仿真顶层 testbench（内联 C 指令编码，unit-test 框架）
Top_tb_bin.v            → 二进制加载测试台（+BIN=<path> plusarg，运行编译好的程序）
```
