# 01 整体架构

## 1.1 设计目标与 ISA

本 CPU 实现 **RISC-V 32 位**指令集，采用**乱序超标量（Out-of-Order Superscalar）**微架构。核心设计目标：

- 每周期最多**取指 2 条、发射 2 条、提交 2 条**（2-wide 超标量）；
- 通过**寄存器重命名 + 重排序缓冲（ROB）**实现乱序执行但顺序提交，保证精确异常；
- 通过**保留站（Reservation Station, RS）**实现动态调度，避免 RAW 数据冒险阻塞流水线；
- 通过**分支预测器（Gshare + BTB + RAS）**减少控制流代价；
- **两级缓存**（ICache / DCache）+ 仿真主存，缓存行 128 位（16 字节），4 周期主存延迟。

**已实现的 ISA 子集：**
| 扩展 | 状态 |
|------|------|
| RV32I（整数基本集） | 完整实现 |
| RV32M 乘法（MUL/MULH/MULHSU/MULHU） | 完整实现（单周期） |
| RV32M 除法（DIV/DIVU/REM/REMU） | 完整实现（32 周期恢复式迭代除法器） |
| RV32F 浮点（基本算术） | 接入 ibex_fpu，部分指令支持 |

---

## 1.2 全局参数一览

| 参数 | 值 | 说明 |
|------|----|------|
| 数据位宽 | 32 位 | `DATA_WIDTH` |
| 地址位宽 | 32 位 | `INST_ADDR_WIDTH` |
| 取指批量 | 2 条/周期 | `IF_BATCH_SIZE` |
| ROB 容量 | 32 项 | `ROB_SIZE` |
| ROB 代号宽度 | 2 位 | `ROB_GEN_WIDTH`，用于过滤过期写回 |
| 整数物理寄存器数 | 64 | `INT_PREG_NUM` |
| 浮点物理寄存器数 | 64 | `FP_PREG_NUM` |
| 保留站深度 | 8 项 | `RS_DEPTH`（IssueBuffer 参数） |
| 指令缓冲深度 | 8 项 | InstructionBuffer 参数 |
| 缓存组数 | 64 | `CACHE_INDEX_BITS=6` |
| 缓存路数 | 2 路 | `CACHE_WAY_NUM` |
| 缓存行大小 | 128 位（16 字节） | `CACHE_LINE_SIZE` |
| 主存容量 | 4 KB | 256 缓存行 × 16 字节 |
| 主存延迟 | 4 周期 | `LATENCY=4` |
| 初始 PC | 0x0000_1000 | `INST_INIT` |
| 异常向量 | 0x0000_1000 | `TRAP_VECTOR`（同初始 PC） |
| 分支预测 GHR | 8 位 | `BP_GHR_BITS` |
| 分支预测 PHT | 1024 项 | `BP_IDX_BITS=10` |
| RAS 深度 | 8 项 | `BP_RAS_DEPTH` |

---

### 各阶段详细说明

| 阶段 | 模块 | 宽度 | 功能 |
|------|------|------|------|
| 取指 IF | `IF.v` + `BranchPredictor.v` | 2条/周期 | PC 管理、ICache 访问、分支预测 |
| 指令缓冲 | `InstructionBuffer.v` | FIFO 深度8 | 解耦取指与解码，提供背压 |
| 预解码 | `PreDecode.v` | 2条/周期（寄存器） | 识别 rs1/rs2/rd/imm/fu_type |
| 寄存器重命名 | `RegRename.v`（含ROB） | 2条/周期 | 逻辑→物理寄存器映射，ROB 分配 |
| 后解码 | `PostDecode.v` | 2条/周期（寄存器） | 生成完整微操作码（ALU_OP/BR_OP/MEM_OP等） |
| 发射/执行 | `IssueBuffer.v` + FU | 2条/周期 out-of-order | 保留站动态调度、多功能单元并行执行 |
| 写回 WB | 内嵌于 IssueBuffer | 3个WB端口（含LSU） | 结果写 PRF、通知 ROB ready |
| 提交 Commit | 内嵌于 ROB | 2条/周期 | 顺序释放 ROB，更新架构寄存器映射 |

---

## 1.4 模块依赖关系

```
Top.v
├── IF.v
│   └── BranchPredictor.v
├── InstructionBuffer.v
├── PreDecode.v
├── RegRename.v
│   └── ROB (在 RegRename.v 内)
├── PostDecode.v
├── IssueBuffer.v
│   ├── IntAlu (在 FU.v 内，×2)
│   ├── IntDivider (在 FU.v 内，×1，32 周期迭代除法器)
│   ├── ibex_fpu.v (×2)
│   └── LSU.v
│       ├── Cache.v (DCache)
│       └── TLB.v
├── ICache.v
├── MemArbiter.v
├── MainMemory.v
└── PhysicalRegFileShared.v
```

---

## 1.5 关键信号总线

### 全局冲刷/重定向

所有寄存器级流水线模块均接收 `flush` 信号。`flush` 由两种情况触发：

1. **分支预测错误（redirect）**：`IssueBuffer` 检测到分支结果与预测不符，产生 `redirect_valid` 和 `redirect_target`；
2. **精确异常（exception）**：ROB 提交时发现 `commit0_exception`，`flush_target` 跳转到 `TRAP_VECTOR`。

```verilog
// Top.v 中的全局冲刷逻辑
wire commit_exc    = commit0_exception;
wire global_flush  = redirect_valid | commit_exc;
wire flush_target  = redirect_valid ? redirect_target : TRAP_VECTOR;
```

### 写回总线（CDB，Common Data Bus）

IssueBuffer 提供 3 路写回端口：
- `wb0`：INT/Branch/System/FP 执行结果（slot 0）
- `wb1`：INT/Branch/System/FP 执行结果（slot 1）
- `wb2`：LSU 写回结果（通过 lsu_wb_* 信号聚合）

写回同时进入：
- **ROB**（标记对应条目 ready + 存储结果值）
- **PhysicalRegFileShared**（写入物理寄存器数据）
- **IssueBuffer 内部保留站**（通过 CDB 广播唤醒依赖指令）

---

## 1.6 乱序执行正确性保证

### 精确异常（Precise Exception）

所有指令在 ROB 中**按程序顺序分配条目**，但可以**乱序执行**并写回结果。ROB 的 head 指针只有在对应条目标记为 ready 后才前进并提交。异常只在提交时才对架构状态可见，未提交的推测执行结果不影响架构状态。

### WAW / WAR / RAW 消除

- **RAW（先写后读）**：通过物理寄存器重命名 + 保留站操作数就绪追踪解决；
- **WAW（写后写）**：两条指令写同一逻辑寄存器时，新指令分配新物理寄存器，旧映射通过 ROB 推迟释放；
- **WAR（读后写）**：重命名后逻辑寄存器与物理寄存器解耦，写新物理寄存器不影响读旧值。

### 推测执行与冲刷

分支指令进入 ROB 和保留站后，后续指令可继续推测执行。当分支在执行单元中解析出实际方向后：
- 若**预测正确**：正常流水；
- 若**预测错误**：`redirect_valid=1`，ROB 将分支之后的年轻条目全部无效化，`RegRename` 从该分支对应的**检查点快照**恢复寄存器映射表和空闲列表，IF 重定向到正确 PC。

---

## 1.7 存储层次结构

```
CPU (LSU/IFetch)
      │
      ▼
┌─────────────────────────────────────┐
│           MemArbiter                │
│   D 侧优先；I/D 不能并发访问主存    │
└─────────────────────────────────────┘
      │
      ├── ICache（只读，2路组相联，64组，128位行）
      │
      ├── DCache（读写，写透+写分配，2路组相联，64组，128位行）
      │     └── TLB（全相联，8项，轮询替换；未命中时使用虚地址=物理地址）
      │
      ▼
MainMemory（4KB，128位行，4周期固定延迟）
```
