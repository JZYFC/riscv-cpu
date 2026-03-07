# 10 软件测试基础设施

本章描述 `sw/` 目录下的 C 语言测试工程，以及配套的二进制加载测试台 `Top_tb_bin.v`，用于将编译好的 RV32I 程序直接注入仿真主存并运行。

---

## 10.1 整体架构

```
sw/
├── Makefile          编译驱动（riscv64-unknown-elf-gcc）
├── crt0.s            最小启动代码：初始化 SP、清零 BSS、调 main()、写 tohost
├── link.ld           链接脚本：地址空间布局
├── tests/            C 语言测试用例
│   ├── test_add.c
│   ├── test_add_123.c
│   ├── test_add_noref.c
│   ├── test_add_notest2.c
│   ├── test_regkeep.c
│   ├── test_regkeep2.c
│   └── test_sub_simple.c
```

仿真测试台：
```
cpu.srcs/sources_1/new/
└── Top_tb_bin.v      二进制加载测试台（独立于 Top_tb.v 的 unit-test 框架）
```

---

## 10.2 地址空间布局（link.ld）

MainMemory 总容量 4 KB（0x1000–0x1FFF）。链接脚本将地址空间划分如下：

| 地址范围 | 用途 |
|----------|------|
| `0x1000–0x1FEF` | 程序代码 + 只读数据 + 数据段 + BSS + 栈（向下增长） |
| `0x1FD0` | 栈顶（`_stack_top`，栈向低地址增长） |
| `0x1FE0` | `_putchar`：写低 8 位触发测试台打印字符 |
| `0x1FF0` | `_tohost`：写 0 = PASS，写非零 = FAIL（退出码） |

> **限制**：程序代码 + 数据 + 栈共享 4 KB，仅适合小型仿真程序。

---

## 10.3 启动代码（crt0.s）

`crt0.s` 提供 `_start` 符号，是程序的真实入口点：

```asm
_start:
    la   sp, _stack_top          # 初始化栈指针（0x1FD0）
    la   a0, _bss_start          # 清零 BSS 段
    la   a1, _bss_end
1:  bge  a0, a1, 2f
    sw   zero, 0(a0)
    addi a0, a0, 4
    j    1b
2:  call main                    # 调用 C 语言 main()
    la   t0, _tohost             # 将返回值写入 tohost（0x1FF0）
    sw   a0, 0(t0)
halt: j halt                     # 死循环等待仿真终止
```

测试台检测到 tohost 写入后立即终止仿真，无需 ECALL。

---

## 10.4 编译工具链（Makefile）

**工具链**：`riscv64-unknown-elf-gcc`（Ubuntu: `sudo apt-get install gcc-riscv64-unknown-elf`）

**编译选项**：
```
-march=rv32i -mabi=ilp32 -nostdlib -nostartfiles -O2 -g
```

**常用 make 目标**：

| 命令 | 说明 |
|------|------|
| `make` | 编译 `tests/test_add.c`，生成 `test.bin` |
| `make TEST=tests/test_mul.c` | 编译指定测试文件 |
| `make dump` | 反汇编前 100 行（调试用） |
| `make clean` | 删除 `test.elf` 和 `test.bin` |

**构建产物**：
- `test.elf`：ELF 格式，可用 `objdump` 反汇编
- `test.bin`：原始二进制，起始地址对应 0x1000，供测试台加载

---

## 10.5 测试台 Top_tb_bin.v

### 概述

`Top_tb_bin.v` 是独立于 `Top_tb.v` 的二进制加载测试台，通过 XSim plusarg 指定 `.bin` 文件路径，无需修改 Verilog 代码即可运行任意编译好的 RISC-V 程序。

### plusarg 接口

| plusarg | 是否必须 | 默认值 | 说明 |
|---------|---------|--------|------|
| `+BIN=<path>` | 必须 | — | `.bin` 文件绝对路径 |
| `+TIMEOUT=<cycles>` | 可选 | 200000 | 最大仿真周期数 |

### 终止条件

| 事件 | 行为 |
|------|------|
| MainMemory 写 `tohost`（0x1FF0），值=0 | `[PASS]` + `$finish(0)` |
| MainMemory 写 `tohost`（0x1FF0），值≠0 | `[FAIL]` + 打印失败上下文 + `$finish(1)` |
| 达到 `TIMEOUT` 周期仍无 tohost 写入 | `[TIMEOUT]` + 打印调试信息 + `$finish(1)` |

### tohost/putchar 检测机制

测试台通过 `always @(posedge clk)` 监视 `dut.u_mainmem` 内部信号：

```verilog
if (dut.u_mainmem.mem_ready && dut.u_mainmem.txn_we) begin
    // 已完成的写事务：检查地址
    if (txn_addr[11:4] == 8'hFF)  tohost_written <= 1;  // 0x1FF0
    if (txn_addr[11:4] == 8'hFE)  $write("%c", txn_wdata[7:0]);  // 0x1FE0
end
```

监视内部锁存寄存器（`txn_we`/`txn_addr`/`txn_wdata`）而非仲裁器端口，因为 MemArbiter 在 SERVE_D/SERVE_I 状态期间会将 `mem_req` 拉低。

### 失败时诊断输出（dump_failure_context）

超时或 tohost 非零时，测试台自动打印：
- 当前 PC、ROB 状态、RS 计数、LSU 忙碌标志
- 流水线各级 valid 信号、stall 链、flush/redirect 状态
- 架构寄存器（ra/sp/gp/s0/s1/a0~a5）
- tohost 和 putchar 内存字内容
- 栈帧（sp 附近 8 个字）
- DCache 行内容（tohost/putchar/sp 处）
- 最近 24 条提交记录（PC、寄存器、异常标志）

### 二进制加载（load_bin_file）

```
Byte 0 → ram[0][7:0]    地址 0x1000
Byte 1 → ram[0][15:8]   地址 0x1001
...                      小端序，16 字节打包为一个 128 位 MainMemory 行
```

使用 `$fread` 逐字节读取，规避 XSim `$feof` 的 off-by-one 滞后问题。

---

## 10.6 测试用例说明与结果

### 用例目的一览

| 文件 | 目的 |
|------|------|
| `test_add.c` | 综合测试：add/sub/循环/嵌套调用 |
| `test_add_123.c` | 最小复现：仅 test1+test2+test3 |
| `test_add_noref.c` | 诊断：test1 改为 add(4,4)=8，test3 与字面量 7 比较（而非 s1） |
| `test_add_notest2.c` | 诊断：移除 test2（add(-1,1)=0），观察 test3 是否通过 |
| `test_sub_simple.c` | 基准：纯 sub 调用，无复杂控制流 |
| `test_regkeep.c` | 诊断：callee-saved 寄存器跨调用保持 |
| `test_regkeep2.c` | 诊断：non-volatile 'expected' 强制放入 s0/s1，测试寄存器重命名恢复 |

### 最新运行结果

| 测试用例 | 状态 | 退出码 |
|----------|------|--------|
| `test_add.c` | **FAIL** | 3 |
| `test_add_123.c` | PASS | 0 |
| `test_add_noref.c` | **FAIL** | 3 |
| `test_add_notest2.c` | PASS | 0 |
| `test_sub_simple.c` | PASS | 0 |
| `test_regkeep.c` | PASS | 0 |
| `test_regkeep2.c` | PASS | 0 |

### 失败分析（Bug 3，见 09_known_issues.md）

**失败路径**：`sub(10, 3)` 在经历 `add(-1, 1)` 调用之后返回错误值。

**关键推断**：
- `test_add_noref.c` 中 test3 比较的是字面量 7（不是 s1），但同样失败 → **不是 callee-saved 寄存器污染，而是 sub 函数的返回值本身错误**
- `test_add_notest2.c` 移除 test2（add(-1,1)）后 test3 通过 → **bug 与执行 add(-1,1)=0 后的状态有关**
- `test_sub_simple.c` 和 `test_regkeep.c` 通过 → 单独的 sub 调用、单独的 callee-saved 测试均无问题
- 怀疑方向：特定指令序列触发的分支预测恢复路径中，寄存器重命名检查点/物理寄存器状态存在边缘情况

---

## 10.7 快速上手

```bash
# 1. 编译测试程序
cd /home/jzyfc/VivadoProjects/cpu/sw
make TEST=tests/test_add_123.c

# 2. 在 Vivado 仿真中加载（通过 Tcl + Top_tb_bin 仿真设置）
# 确保仿真顶层设置为 Top_tb_bin，并传入 plusarg：
#   +BIN=/home/jzyfc/VivadoProjects/cpu/sw/test.bin
#   +TIMEOUT=200000
```

