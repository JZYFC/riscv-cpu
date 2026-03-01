# 05 存储子系统

存储子系统从上到下分为：**LSU（访存单元）**、**DCache（数据缓存）** + **ICache（指令缓存）**、**TLB（地址转换）**、**MemArbiter（总线仲裁）**、**MainMemory（仿真主存）**。

---

## 5.1 存储系统整体结构

```
CPU (IssueBuffer/IF)
       │                    │
       ▼                    ▼
     LSU                  IF.v
       │                    │
       ▼                    ▼
    DCache               ICache
    (Cache.v)            (ICache.v)
       │                    │
       └────────┬───────────┘
                ▼
           MemArbiter
           (D侧优先)
                │
                ▼
           MainMemory
           (4KB, 4周期延迟)
```

缓存共同参数：
- **缓存行大小**：128 位（16 字节），即 4 条 32 位指令或 4 个 32 位数据；
- **组数**：64（`CACHE_INDEX_BITS=6`）；
- **路数**：2（2-way 组相联）；
- **Tag 位**：`32 - 6(index) - 4(offset) = 22 位`；
- **替换策略**：LRU（每组 1 位记录最近使用路）。

---

## 5.2 LSU（LSU.v）——访存单元

### 职责

接受来自 IssueBuffer 的单条访存请求，完成地址对齐检查、TLB 地址转换、DCache 访问，返回写回数据或异常信号。

**约束**：LSU 是**单 outstanding**设计——同一时间只处理一条访存请求。IssueBuffer 通过 `lsu_busy` 信号感知 LSU 是否可以接受新请求。

### 状态机

```
IDLE
  │ valid_in=1
  │ 若 misaligned → 立即报异常，回 IDLE
  │ 否则 → 锁存请求参数，进入 WAIT_CACHE
  ▼
WAIT_CACHE
  │（TLB 查询并行进行）
  │ DCache 命中 → cache_valid_out=1 → 提取数据，进入 IDLE，发 wb_valid
  │ DCache 未命中 → DCache 内部处理 refill，stall_cpu 期间 WAIT_CACHE 保持
  ▼
IDLE（完成）
```

### 地址转换（TLB）

LSU 在 `WAIT_CACHE` 状态期间，将 `addr_reg`（锁存的虚拟地址）发给 TLB：

```verilog
TLB u_tlb (
    .vaddr(addr_reg), .req(state == WAIT_CACHE),
    .paddr(tlb_paddr), .hit(tlb_hit), .miss(tlb_miss),
    .we(1'b0) ...  // LSU 侧只读，不负责填充 TLB
);
wire effective_paddr = tlb_hit ? tlb_paddr : addr_reg; // miss 时裸金属模式
```

**当前实现**：TLB 未命中时直接使用虚拟地址作为物理地址（裸金属/无 MMU 模式），适合仿真环境。

### 对齐检查

进入 WAIT_CACHE 前先检查对齐：
- LW/SW：地址 bits[1:0] 必须为 00；
- LH/LHU/SH：地址 bit[0] 必须为 0；
- LB/LBU/SB：无对齐要求。

若不对齐，立即写回异常（`wb_exception=1`），LSU 回到 IDLE。

### Store 字节使能（wstrb）

LSU 根据 `mem_op` 生成字节使能信号传给 DCache，用于 read-modify-write 中准确地更新缓存行内的目标字节：

```verilog
MEM_OP_SB: wstrb = 4'b0001 << addr[1:0];  // 1 字节
MEM_OP_SH: wstrb = 4'b0011 << addr[1:0];  // 2 字节（需已对齐）
MEM_OP_SW: wstrb = 4'b1111;                // 4 字节
```

同时对 store 数据进行**字节重复**对齐（replicate），使缓存 read-modify-write 逻辑更简单：

```verilog
MEM_OP_SB: aligned_wdata = {4{wdata[7:0]}};  // 4 字节均填充
MEM_OP_SH: aligned_wdata = {2{wdata[15:0]}}; // 高低 2 字节均填充
```

### Load 数据提取

DCache 返回整个 32 位对齐字，LSU 根据地址 bits[1:0] 和 bits[1] 提取所需字节/半字，并进行符号或零扩展：

```
LB / LBU : 按 addr[1:0] 从 cache_rdata 中取 8 位，符号或零扩展到 32 位
LH / LHU : 按 addr[1] 从 cache_rdata 中取 16 位
LW       : 直接返回 cache_rdata 全 32 位
```

---

## 5.3 DCache（Cache.v）——数据缓存

### 组织结构

```
64 组 × 2 路 × 128 位数据 + Tag（22+2位：dirty+valid+tag）+ LRU（1位）

地址分解：
  [31:10] → Tag（22位）
  [9:4]   → Index（6位，选 64 组之一）
  [3:0]   → Offset（16字节内偏移，最低2位用于字节选择）
```

### 写策略

**写分配（Write-Allocate）+ 写透（Write-Through）**：
- 写命中：更新缓存行内对应字节，同时向主存发出写透请求；
- 写未命中：先从主存 refill 整行，合并待写数据后写入缓存，再写透到主存；
- 不使用脏位（dirty bit 字段存在但始终清零）——写透意味着缓存与主存始终一致，无需在替换时写回。

### 状态机

```
IDLE
  ├─ 命中（hit）+ 读请求  → valid_out=1（直接命中，0延迟）
  ├─ 命中（hit）+ 写请求  → 更新缓存行，发写透请求 → WRITE_THROUGH
  └─ 未命中（miss）
         ├─ victim 是脏行（victim_dirty=1） → WRITEBACK（先回写旧行）
         └─ victim 不脏 → REFILL（直接取新行）

WRITEBACK：等 MainMemory 接受旧行，完成后 → REFILL

REFILL：向 MainMemory 请求新行，等待 mem_ready 且 mem_resp_addr 匹配；
        安装新行到 victim 路，更新 LRU；
        若是写未命中，合并 store 数据后进入 WRITE_THROUGH

WRITE_THROUGH：等 MainMemory 接受写透数据（mem_ready=1）→ valid_out=1 → IDLE
```

**注意**：由于当前只有写透策略，`victim_dirty` 始终为 0，WRITEBACK 状态实际是**死代码**（永远不会进入）。如果将来改为写回策略，WRITEBACK 路径才会激活。

### Hit 判断

```verilog
hit0 = valid0 && (saved_tag0 == tag)
hit1 = valid1 && (saved_tag1 == tag)
hit  = (hit0 || hit1) && req
stall_cpu = req && !hit   // 未命中时向 LSU 发出 stall
```

LRU 更新：命中 way0 → `lru[index]=1`（way1 为下次被替换的 LRU）；命中 way1 → `lru[index]=0`。

### Read-Modify-Write 操作

写命中和写未命中（refill 后）都需要对 128 位缓存行的特定字节进行 RMW：

```verilog
// 根据 offset[3:2] 确定写入哪个 32 位字
// 根据 wstrb[3:0] 确定写入哪些字节
// 示例（offset[3:2]=2'b01，即第二个32位字）：
if (wstrb[0]) temp_data[39:32] = wdata[7:0];
if (wstrb[1]) temp_data[47:40] = wdata[15:8];
...
```

### 仿真调试输出

代码中包含 `$display` 语句，在特定地址（`paddr[11:4] == 8'hEB`）的访问时打印缓存行内容，便于仿真调试。

---

## 5.4 ICache（ICache.v）——指令缓存

与 DCache 结构基本相同（64组×2路×128位），但有以下区别：

1. **只读**：ICache 没有 write 接口，`mem_we` 永远为 0；
2. **命中即返回整行**：输出整个 128 位 `rdata_line`，由 IF.v 内部按 `word_idx` 切取需要的 32 位字；
3. **无 WRITEBACK 状态**：只有 IDLE 和 REFILL 两个状态，结构简单；
4. **miss 元数据锁存**：在 REFILL 时锁存 miss_index、miss_tag、miss_victim_way、miss_line_addr，防止 PC 改变后 refill 地址出错；
5. **响应地址校验**：只有 `mem_resp_addr == miss_line_addr` 时才安装 refill 数据，防止仲裁器的乱序响应导致 ICache 行污染。

**ICache 请求方**：IF.v 在 `need_line`（当前行缓存未命中）且没有 stall 和 flush 时才发出 `ic_req`。

---

## 5.5 TLB（TLB.v）——地址转换旁路表

### 组织

**全相联结构**，8 个条目（`TLB_ENTRIES=8`），每条目含：

```
valid : 有效位
vpn   : 虚拟页号（vaddr[31:12]，20位）
ppn   : 物理页号（paddr[31:12]，20位）
```

物理地址 = `{ppn, vaddr[11:0]}`（页内偏移直接透传）。

### 查找（组合逻辑）

并行比较所有条目的 VPN 与输入 `vaddr[31:12]`，生成 `match_vec`，若有命中则输出对应 PPN 拼接页偏移，同时 `hit=1`；否则 `miss=1`。

### 填充（写接口）

`we=1` 时，将新 VPN→PPN 映射写入 `replace_ptr` 指向的条目，`replace_ptr` 循环自增（轮询替换）。**当前 LSU 不主动填充 TLB**（`we=1'b0`），故 TLB 初始时全为 invalid，所有访问都返回 `miss=1`，LSU 使用裸金属模式（虚拟地址=物理地址）。若要启用 MMU 功能，需要在 LSU 触发 TLB miss 时实现 page table walk 并调用 TLB 的写接口。

---

## 5.6 MemArbiter（MemArbiter.v）——内存仲裁器

### 职责

ICache 和 DCache 共享同一条主存总线。仲裁器在两者之间串行化请求，一次只服务一个。

### 优先级

**D 侧（DCache/LSU）优先于 I 侧（ICache）**：在 IDLE 状态时，若 D 侧和 I 侧同时发出请求，优先服务 D 侧。

### 状态机

```
IDLE
  ├─ d_req=1 → 锁存 D 请求，转 SERVE_D
  ├─ i_req=1 → 锁存 I 请求，转 SERVE_I
  └─ 无请求 → 保持 IDLE

SERVE_D：等待 mem_ready=1（主存完成），同时将 mem_rdata/mem_resp_addr 路由给 d_rdata/d_ready，完成后回 IDLE

SERVE_I：等待 mem_ready=1，路由给 i_rdata/i_ready，完成后回 IDLE
```

**关键细节**：请求被主存接受后，仲裁器立即停止驱动 `mem_req`（置 0），防止主存在完成当前请求后再次误接受一个重复请求（因为主存的 `txn_active` 在完成时才清零，若此时 `mem_req` 仍高，下一周期会重新接受）。

响应地址 `i_resp_addr`/`d_resp_addr` 透传给 ICache/DCache，使它们能校验 refill 数据是否为自己发出的请求的响应。

---

## 5.7 MainMemory（MainMemory.v）——仿真主存

### 参数

- 容量：**4 KB**（256 条 128 位缓存行）；
- 延迟：**4 个时钟周期**（固定延迟，`LATENCY=4`）；
- 地址索引：`mem_addr[11:4]`（8 位，选 256 行中的一行，4 位 offset 舍弃）。

### 单 outstanding 事务模型

MainMemory 内部有 `txn_active` 锁，同一时间只处理一个请求：

```
idle → mem_req=1 → 锁存 txn_addr, txn_we, txn_wdata, txn_active=1
txn_active: delay_cnt 计数，到 LATENCY 后：
  若 txn_we: ram[addr[11:4]] = txn_wdata
  否则: mem_rdata = ram[addr[11:4]]
  mem_ready = 1，mem_resp_addr = txn_addr，txn_active = 0
```

### 初始内容

```verilog
initial begin
    ram[8'h00] = 128'hDEAD_BEEF_...; // 地址 0x000（仿真调试用）
    ram[8'h01] = 128'h1111_2222_...; // 地址 0x010
end
```

程序的代码和数据需要通过 testbench 或 `$readmemh` 预加载到 `ram` 数组。

**重要限制**：4 KB 的地址空间意味着仅支持地址 `0x0000_0000 ~ 0x0000_0FFF`。超出该范围的访问会发生地址折叠（`addr[11:4]` 低 8 位重复），适用于小型仿真程序。
