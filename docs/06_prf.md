# 06 物理寄存器堆（PhysicalRegFileShared）

---

## 6.1 概述

`PhysicalRegFileShared.v` 实现了一个**整数和浮点共享的物理寄存器堆**，提供：
- **4 个读端口**（整数）：同时为两个执行槽各提供 rs1 和 rs2 数据；
- **2 个写端口**（整数）：同时接受两路写回；
- 浮点方向完全对称，也提供 4 读 2 写。

---

## 6.2 存储结构

```verilog
reg [31:0] int_prf   [0:63];  // 64 个整数物理寄存器
reg [31:0] float_prf [0:63];  // 64 个浮点物理寄存器
```

物理寄存器下标（6 位，`PREG_IDX_WIDTH=6`）在整数和浮点之间独立编号，各有 0~63 共 64 个槽。

---

## 6.3 读端口（组合逻辑 + 写旁路）

**关键设计**：读端口不直接读 `int_prf[addr]`，而是先检查是否有当周期的写操作覆盖了同一地址（**写旁路/写转发，Write Bypass**），若有则直接返回写端口的数据：

```verilog
assign i_rs1_data0 =
    (i_rd1_we && i_rd1_addr == i_rs1_addr0) ? i_rd1_data :  // 写口1 优先
    (i_rd0_we && i_rd0_addr == i_rs1_addr0) ? i_rd0_data :  // 写口0 次之
    int_prf[i_rs1_addr0];                                    // 否则读存储
```

**优先级**：写口 1 > 写口 0 > 存储内容。（写口 1 对应 issue_idx1，写口 0 对应 issue_idx0，两条指令如果恰好写同一物理寄存器，优先使用后一个写口的值——但这种情况在设计上不应发生，因为每条指令分配唯一的物理寄存器。）

所有 4 个读端口（i_rs1_data0/1, i_rs2_data0/1 和浮点对应的 4 个）均采用同样的旁路逻辑。

---

## 6.4 写端口（时序逻辑）

```verilog
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // 复位：所有寄存器清零
    end else begin
        if (i_rd0_we) int_prf[i_rd0_addr] <= i_rd0_data;
        if (i_rd1_we) int_prf[i_rd1_addr] <= i_rd1_data;
        if (f_rd0_we) float_prf[f_rd0_addr] <= f_rd0_data;
        if (f_rd1_we) float_prf[f_rd1_addr] <= f_rd1_data;
    end
end
```

写端口的数据在下一个时钟周期才真正存入数组，但旁路逻辑在同一周期即可将数据转发给读端口，确保当周期写的寄存器在当周期即可被读取（对于保留站 CDB 唤醒后立即读 PRF 的场景尤为重要）。

---

## 6.5 接口与 Top.v 的连接

`PhysicalRegFileShared` 在 `Top.v` 中实例化，读端口连接到 `IssueBuffer`（发射时读取操作数），写端口由 `IssueBuffer` 的写回输出（`i_wr_addr0/1`, `f_wr_addr0/1` 等）驱动。

```
                ┌──────────────────────┐
IssueBuffer     │ PhysicalRegFileShared│
  issue_idx0 ──▶│ i_rs1_addr0          │──▶ int_rs1_data0 ──▶ IssueBuffer
  issue_idx0 ──▶│ i_rs2_addr0          │──▶ int_rs2_data0
  issue_idx1 ──▶│ i_rs1_addr1          │──▶ int_rs1_data1
  issue_idx1 ──▶│ i_rs2_addr1          │──▶ int_rs2_data1
                │                      │
  wb0 result ──▶│ i_rd0_addr/data/we   │  (写回路径)
  wb1 result ──▶│ i_rd1_addr/data/we   │
                └──────────────────────┘
```

**注意**：PRF 读端口的地址（`i_rs1_addr0` 等）由 IssueBuffer 根据当前 `issue_idx0/1` 对应 RS 槽中存储的 `rs_rs1_tag/rs_rs2_tag` 来驱动（在 Top.v 中连线）。
