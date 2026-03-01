# 03 寄存器重命名与重排序缓冲

`RegRename.v` 文件包含两个模块：**ROB**（Reorder Buffer，重排序缓冲）和 **RegRename**（寄存器重命名）。`RegRename` 实例化并包含 `ROB`，二者协同完成乱序执行所需的核心状态管理。

---

## 3.1 为什么需要寄存器重命名

RISC-V 有 32 个架构整数寄存器（x0~x31）和 32 个浮点寄存器（f0~f31）。当两条相互独立的指令恰好写同一个逻辑寄存器时，会产生**写后写（WAW）**伪相关；当后条指令读前条指令即将覆盖的寄存器时，产生**读后写（WAR）**伪相关。这两种伪相关不是真正的数据依赖，但在简单实现中会造成流水线停顿。

**寄存器重命名**的做法：为每条有写入目的寄存器的指令**分配一个新的物理寄存器**。物理寄存器数量（64 个）多于架构寄存器（32 个），空闲物理寄存器由**空闲列表**（free list）管理。逻辑寄存器→物理寄存器的当前映射由**映射表**（map table）记录。

---

## 3.2 ROB 模块

### 数据结构

ROB 是一个**循环队列**，容量 32 项，每项包含：

| 字段 | 宽度 | 说明 |
|------|------|------|
| `entry_valid` | 1 | 该槽是否被占用 |
| `entry_ready` | 1 | 执行完毕、可提交 |
| `entry_pc` | 32 | 指令 PC |
| `entry_has_dest` | 1 | 是否写目的寄存器 |
| `entry_is_float` | 1 | 目的寄存器是否为浮点 |
| `entry_arch_rd` | 5 | 目的架构寄存器号 |
| `entry_new_preg` | 6 | 新分配的物理寄存器 |
| `entry_old_preg` | 6 | 被替换的旧物理寄存器（用于异常恢复时释放） |
| `entry_value` | 32 | 执行结果（WB 写入） |
| `entry_exception` | 1 | 是否发生异常 |
| `entry_gen` | 2 | 代号（generation tag） |

队列用 `head`（提交指针）和 `tail`（分配指针）管理，`count` 跟踪有效项数。

### 代号（Generation Tag）

ROB 每个槽维护一个 2 位代号 `entry_gen`，每次在该槽分配新指令时递增。写回（WB）时携带 `wb_rob_gen`，只有当 `entry_gen[idx] == wb_rob_gen` 时才接受写回。这防止了**乱序清空后的旧写回**污染新分配指令的 ROB 条目（特别在 flush 后重分配同一 idx 的情况）。

### 分配（Alloc）

每周期最多分配 2 个条目（对应 2-wide fetch）：

```
alloc0_rob_idx = tail
alloc1_rob_idx = do_alloc0 ? ptr_inc(tail) : tail
alloc0_ready   = (free_slots >= 1)
alloc1_ready   = (free_slots >= 2)
```

### 写回（Writeback）

3 个写回端口（wb0/wb1/wb2），匹配 ROB idx 和 gen 后：
- 设置 `entry_ready = 1`；
- 存储 `entry_value` 和 `entry_exception`。

### 提交（Commit）

每周期检查 head 指向的最多 2 个连续条目（head_idx0, head_idx1）：
- 条件：`entry_valid && entry_ready`；
- 第二条额外条件：第一条**无异常**才能同时提交第二条；
- 提交后输出 `commit0/1_*` 信号供 RegRename 更新架构映射表，head 前进。

### Flush（冲刷）

两种 flush 情况：

1. **redirect（分支错误预测）**：`flush_is_redirect=1`，`flush_rob_idx` 为错误分支的 ROB 索引。保留 `[head .. flush_rob_idx]` 范围内的条目，清除其余年轻条目；`tail` 回退到 `flush_rob_idx+1`。

2. **异常提交**：`flush_is_redirect=0`，清空整个 ROB（head、tail 归零，所有条目 invalid）。

---

## 3.3 RegRename 模块

### 核心数据结构

```
// 映射表（逻辑 → 物理）
int_map[0:31]       : 整数架构寄存器当前映射到的物理寄存器
int_map_valid[0:31] : 映射是否有效
fp_map[0:31]        : 浮点寄存器当前映射
fp_map_valid[0:31]  : 映射是否有效

// 物理寄存器就绪位（记分牌，Scoreboard）
int_preg_ready[0:63]: 整数物理寄存器是否已有有效数据
fp_preg_ready[0:63] : 浮点物理寄存器是否已有有效数据

// 空闲列表（环形队列）
int_free[0:63]      : 空闲整数物理寄存器编号数组
fp_free[0:63]       : 空闲浮点物理寄存器编号数组
int_free_head/tail/count : 空闲队列头尾指针

// 检查点快照（每个 ROB 条目保存当时的完整映射状态）
int_map_cp[0:31][0:31]   : 整数映射表快照
fp_map_cp[0:31][0:31]    : 浮点映射表快照
int_head_cp[0:31] 等      : 空闲列表状态快照
```

### 初始化

启动时，x0~x31（整数）的初始映射分配物理寄存器 p0~p31，其中 p0 永远映射到 x0（RISC-V 零寄存器）。剩余 p32~p63 进入空闲列表。浮点寄存器同理。`int_preg_ready[0]` 初始就绪（x0=0）。

### 双指令重命名时序

当 `in_inst_valid = 2'b11`（两条指令同时进入），必须注意**指令间相关**：

**inst1 读取 inst0 的目的寄存器（RAW）**：

```verilog
// inst1 的源寄存器标签：若与 inst0 目的相同，直接使用 inst0 分配的新物理寄存器
rs1_tag1 = (has_dest0 && same_dest_type && (in_rs1_1 == in_rd_0))
           ? new_preg0 : int_map[in_rs1_1];
rs2_tag1 = (has_dest0 && same_dest_type && (in_rs2_1 == in_rd_0))
           ? new_preg0 : int_map[in_rs2_1];
```

**inst0 与 inst1 写同一逻辑寄存器（WAW）**：

```verilog
// inst1 的 old_preg 应为 inst0 分配的 new_preg（而非映射表中的旧值）
old_preg1 = (has_dest1 && has_dest0 && same_dest_type && same_dest_reg)
            ? new_preg0 : int_map[in_rd_1];
```

### 阻塞条件（rename_stall）

若以下任一条件不满足，重命名阶段阻塞：
- 整数空闲寄存器数量 ≥ 当前需求数（0/1/2）；
- 浮点空闲寄存器数量 ≥ 当前需求数；
- ROB 有足够空闲槽（`alloc0_ready`、`alloc1_ready`）。

### 物理寄存器就绪查询（记分牌接口）

IssueBuffer 在入队时需要知道操作数是否**当前已就绪**。RegRename 提供 4 个查询端口（sb_q0~sb_q3），输入物理寄存器号和类型（int/fp），输出当前就绪位：

```verilog
assign sb_q0_ready = sb_q0_is_fp ? fp_preg_ready[sb_q0_tag]
                                 : int_preg_ready[sb_q0_tag];
```

IssueBuffer 接收 `in_rs1_ready_now_0/1`、`in_rs2_ready_now_0/1` 作为入队时的就绪快照。

### 检查点机制（Checkpoint）

每次 rename_fire（成功提交一批指令到 ROB）时，将当前完整的映射表和空闲列表状态快照存储到以 **ROB 最后分配的条目 idx** 为下标的检查点数组。

```verilog
// 每次 rename_fire，分别存储 inst0 之后和 inst1 之后的映射快照
int_map_cp[alloc0_rob_idx] = int_map_after0   // inst0 分配完后的状态
int_map_cp[alloc1_rob_idx] = int_map_after1   // inst1 分配完后的状态
```

**flush 时恢复**：`RegRename` 在 flush 时，根据 `flush_rob_idx` 从对应检查点恢复映射表和空闲列表，一周期完成重命名状态回滚，无需重新 replay 所有指令。

### 提交时释放物理寄存器

ROB 提交输出 `commit0/1_old_preg`（被替换的旧物理寄存器），RegRename 在收到 commit 信号时将其放回空闲列表，并更新 `int_preg_ready`（写回时标记为就绪，提交时无需再改，就绪位由 WB 时设置）。

### 完整状态流

```
分配阶段：
  1. 从空闲列表取 new_preg（消耗 int_free_head）
  2. old_preg = int_map[rd]（旧映射）
  3. int_map[rd] = new_preg（更新映射表）
  4. int_preg_ready[new_preg] = 0（新寄存器尚未就绪）
  5. ROB 分配条目，存 new_preg/old_preg
  6. 保存检查点快照

执行/写回阶段：
  1. IssueBuffer 执行完成，写回 wb_valid + wb_value + wb_rob_idx
  2. RegRename/ROB 收到 wb：标记 ROB 条目 ready，更新 entry_value
  3. int_preg_ready[new_preg] = 1（物理寄存器就绪，CDB 广播）

提交阶段：
  1. ROB 提交：输出 commit0/1_old_preg
  2. RegRename 将 old_preg 放回空闲列表（int_free[tail++] = old_preg）
  3. int_preg_ready[old_preg] = 0（旧物理寄存器重新变为不就绪）

Flush 阶段：
  1. 从 int_map_cp[flush_rob_idx] 恢复 int_map[]
  2. 从 int_head_cp[flush_rob_idx] 恢复 int_free_head/tail/count
  3. ROB 清除年轻条目，tail 回退
```
