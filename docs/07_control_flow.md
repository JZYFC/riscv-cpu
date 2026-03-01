# 07 全局控制流：冲刷、重定向与提交

---

## 7.1 整体控制信号图

```
IssueBuffer
  │ redirect_valid
  │ redirect_target
  │ redirect_rob_idx
  └──────────────────────────────────────────────┐
                                                  ▼
ROB (in RegRename.v)                          Top.v
  │ commit0_exception                         global_flush = redirect_valid | commit_exc
  └──────────────────────────────────────────▶ flush_target = redirect_valid ? redirect_target
                                                                             : TRAP_VECTOR
                                                  │
                    ┌─────────────────────────────┼──────────────────────────────┐
                    ▼                             ▼                              ▼
                  IF.v                     InstructionBuffer             PreDecode.v
               (flush + redirect_pc)          (flush)                    (flush)
                    │
                    ▼
               RegRename.v                  PostDecode.v               IssueBuffer.v
               (flush + flush_is_redirect    (flush)                   (flush + flush_is_redirect
                + flush_rob_idx)                                        + flush_rob_idx)
```

---

## 7.2 分支错误预测重定向（Redirect）

### 触发

IssueBuffer 在分支执行时检测到 `br_mispredict`：

```verilog
br_mispredict = (br_taken != pred_taken) ||
                (br_taken && pred_taken && br_target != pred_target)
```

若 issue_idx0（或 issue_idx1）是 branch 且 mispredict，则：

```verilog
redirect_valid    <= 1;
redirect_target   <= br_target（实际跳转目标）;
redirect_rob_idx  <= rs_rob_idx[issue_idx0]（该分支在 ROB 中的位置）;
```

**注意**：`redirect_valid` 在**写回（registered）时发出**，即比分支执行晚一个时钟周期。这一周期延迟需要在 ROB 的 flush 逻辑中考虑（防止已提交的分支 idx 被错误使用）。

### 传播

`global_flush = redirect_valid | commit_exc`，该信号广播到所有流水级的 `flush` 输入：

| 模块 | flush 时行为 |
|------|-------------|
| **IF** | PC ← redirect_pc（正确路径目标），清空行缓存，`fetch_epoch++` |
| **InstructionBuffer** | head = tail = count = 0（清空队列） |
| **PreDecode** | `out_inst_valid ← 0`，所有输出清零 |
| **RegRename** | 从 `int_map_cp[flush_rob_idx]` 恢复映射表和空闲列表 |
| **PostDecode** | `out_inst_valid ← 0`，所有输出清零 |
| **IssueBuffer** | 将所有 ROB idx 不在 `[rob_head .. flush_rob_idx]` 范围内的 RS 槽置 invalid |
| **ROB** | 清除分支之后的所有年轻条目，tail 回退到 flush_rob_idx+1 |
| **LSU** | 放弃当前 in-flight 访存，回到 IDLE 状态 |

### ROB 部分冲刷

分支错误预测时，分支本身在 ROB 中的条目**保留**（它已经执行完毕，只是后续的推测执行需要撤销）。ROB 保留 `[head .. flush_rob_idx]` 的条目，清除之后的年轻条目：

```verilog
// 在 ROB 中
for (i = 0; i < ROB_SIZE; i++) begin
    if (entry_valid[i] && !in_range_inclusive(i, head, flush_rob_idx))
        entry_valid[i] <= 0; // 清除年轻的错误路径条目
end
tail <= ptr_inc(flush_rob_idx);
```

`in_range_inclusive` 处理环形缓冲区的折叠情况（head > flush_rob_idx 时）。

**边缘情况**：若分支在 redirect 到来的同一周期已经从 ROB head 提交，则 `rob_head == flush_rob_idx_next`（head 已经越过分支），此时 `redirect_keep_none=1`，将整个 ROB 清空（因为没有任何合法的旧指令还在 in-flight）。

---

## 7.3 提交（Commit）

### 触发条件

ROB 每周期检查 head 指向的最多 2 个连续条目：
- 条目必须 `entry_valid && entry_ready`（已执行完毕）；
- 第 2 条额外要求第 1 条**无异常**（异常必须是精确的，只提交一条）。

### 提交输出信号

```
commit0_valid     : 第一条指令提交有效
commit0_rob_idx   : ROB 索引
commit0_pc        : 指令 PC
commit0_has_dest  : 是否有目的寄存器
commit0_is_float  : 目的是否为浮点
commit0_arch_rd   : 架构目的寄存器号
commit0_new_preg  : 新物理寄存器（此时才成为架构物理寄存器）
commit0_old_preg  : 旧物理寄存器（此时释放回空闲列表）
commit0_value     : 提交值（目前 RegRename 不直接使用此值，架构值由 PRF 保存）
commit0_exception : 是否异常
（commit1_* 对称）
```

### RegRename 在提交时的动作

收到 `commit0_valid` 时（在 RegRename 的 always 块中）：

1. 若 `commit0_has_dest`：`old_preg` 放回空闲列表（`int_free[tail] = old_preg; tail++`）；
2. 重置 `int_preg_ready[old_preg] = 0`（旧寄存器重新变为"不就绪"）；
3. commit1 对称处理。

### 异常处理

若 `commit0_exception = 1`：
- `global_flush = 1`（`commit_exc=1`）；
- `flush_target = TRAP_VECTOR = 0x0000_1000`；
- `flush_is_redirect = 0`（不是 redirect，是完全清空）；
- ROB 全部清空，RegRename 从当前提交点恢复（检查点对应 flush_rob_idx 为 head 之前的条目，此时 `redirect_keep_none` 机制生效，整个 ROB 清空，RegRename 重建）。

当前实现中，精确异常后 CPU 跳转到 `TRAP_VECTOR=0x1000`，即**同一地址重新开始执行**（没有真正的异常处理程序）。这是仿真环境下的简化处理。

---

## 7.4 反压（Stall）信号链

流水线采用**从后向前**的反压模式：

```
IssueBuffer.stall_dispatch
    │（RS 满时）
    ▼
Top.v: dispatch_pipe_stall
    │
    ▼
dispatch_ready = !rn_stall && !dispatch_pipe_stall
    │
    ▼
PostDecode / RegRename stall 输入
    │
    ▼
InstructionBuffer out_ready（为 0 时 IB 停止出队）
    │
    ▼
PreDecode stall 输入
    │
    ▼
InstructionBuffer.stall_if（若 IB 即将满，停止 IF 入队）
    │
    ▼
IF.v stall 输入（停止取指）
```

另外：
- `ic_stall`（ICache miss）也会阻塞 IF；
- `rn_stall`（RegRename 物理寄存器或 ROB 资源不足）阻塞 dispatch 但不阻塞 IF/IB 层（它们可以继续缓冲）。

---

## 7.5 BranchPredictor 更新

分支执行完成后，IssueBuffer 输出 `bp_update0/1_*` 信号，在下一个时钟周期由 BranchPredictor 处理。这些信号经由 Top.v 的连线直接送到 IF.v 内部实例化的 BranchPredictor。

更新包含：PC、实际跳转结果（taken/not-taken）、实际目标地址、当时的 GHR 快照（`hist`）、是否为 call/return。BranchPredictor 用原始 GHR 快照（而非当前 GHR）重新计算 PHT/BTB 索引，确保更新对应到预测时的历史状态。
