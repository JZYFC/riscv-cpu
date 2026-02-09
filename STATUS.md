# Out-of-Order CPU Status

This document summarizes the current two-wide OoO core as wired today (including I$/D$/LSU and FP blocks), and what is missing to reach a fully functional CPU.

## Quick Snapshot
- Pipeline: IF → InstructionBuffer → PreDecode → RegRename/ROB → PostDecode → Issue/Execute → Commit.
- Width: 2-wide fetch/rename/issue/commit; shared int/fp physical register file (64 int + 64 fp). Default queue depths: InstructionBuffer=8, Issue RS=8.
- Control: Global flush on branch mispredict (issue redirect) or `commit0_exception` to `TRAP_VECTOR`; commit1 is suppressed on commit0 exception; rename redirect restore uses ROB checkpoints for maps/head/count but free list contents are not checkpointed (known bug); predictor flush clears GHR/RAS only.
- Execute coverage: Integer ALU, branch/JAL/JALR (link), CSR array ops (4K CSR file), mul (combinational), div (fixed 8-count, single in-flight), basic FP add/sub/mul via `ibex_fpu` (combinational, 2-source only). LSU integrated with single outstanding/ordered issue (TLB + DCache).
- Fetch status: Predictor wired; IF fetches from ICache (128-bit line buffer), stalls on miss; if predicted taken, slot1 is suppressed; if slot1 crosses a line, slot1 is suppressed.
- Memory system: ICache + DCache connect through MemArbiter to MainMemory; DCache is 2-way with write-through on hits and writeback on dirty victim.

## Pipeline Walkthrough (by stage)
- **IF (IF.v)**: Tracks PC, queries Gshare predictor (GHR=8, IDX=10, BTB+RAS), chooses predicted target vs sequential. Handles stall/flush/redirect. Fetches from ICache using a 128-bit line buffer; stalls on miss; predicted-taken or cross-line fetch suppresses slot1.
- **InstructionBuffer (InstructionBuffer.v)**: 2-wide enqueue/dequeue FIFO (depth 8) with backpressure to IF. Preserves PC and prediction metadata, flushes on redirect.
- **PreDecode (PreDecode.v)**: One-cycle predecode to extract rs1/rs2/rd, immediate, FU class (int/mem/mul/fp), and int vs fp register type flags. Carries prediction metadata through.
- **RegRename + ROB (RegRename.v)**: Logical→physical mapping for int/fp with free lists; allocates ROB entries for all valid instructions; hands out new physical destinations and old mappings; frees old physicals on commit. ROB is 32 entries, dual-alloc, dual-commit, dual-writeback. Per-ROB snapshots are used for redirect recovery of maps/head/count, but free list arrays are not checkpointed (see bug below).
- **PostDecode (PostDecode.v)**: Full ISA-oriented decode after rename. Classifies FU selection (int, mul/div, branch, lsu, fp, system) and emits per-unit micro-ops (int ALU ops/sub flag/cmp signed, mul/div controls, branch op, mem op+sign/size, CSR op/addr, fp op, illegal).
- **Issue/Execute (IssueBuffer.v)**: Reservation-station scheduler (RS depth 8) with age tracking; attempts dual-issue per cycle. Captures operand readiness/values via PRF reads and CDB wakeups, includes simple bypass from writeback/divider. Executes int ALU, branch resolution, CSR array ops, mul (combinational), div (fixed 8-count, single in-flight), and basic FP add/sub/mul through `ibex_fpu` (C input tied to 0, rounding mode fixed to RNE). LSU requests are issued in-order with a single outstanding; stores only issue at ROB head. Produces up to two CDB results per cycle and PRF writes, writes ROB via Top, resolves branches (redirect + predictor training), asserts dispatch stall when RS lacks space.
- **LSU (LSU.v)**: Single-outstanding load/store engine with simple TLB + DCache. Detects misaligned access and returns a generic exception bit; no permission/page-fault modeling.
- **PhysicalRegFileShared (PhysicalRegFileShared.v)**: Shared int/fp PRF with two read pairs and two write ports per type; includes simple write bypass on reads.
- **Cache/Memory (ICache.v, Cache.v, MemArbiter.v, MainMemory.v)**: ICache provides 128-bit line fetches to IF; DCache is 2-way and services LSU requests via MemArbiter; MainMemory is the backing store.
- **Top (Top.v)**: Wires the whole pipeline, distributes flush/redirect, threads prediction metadata, instantiates shared PRF, LSU, caches, and exposes ROB commit signals. Flush on exception only consults `commit0_exception`.
- **Legacy helpers**: `FU.v` remains as a legacy/simple block; current pipeline uses IssueBuffer for execution.

## What Works Today
- Prediction pipeline (Gshare+BTB+RAS) with redirect and dual training ports; flush clears GHR/RAS (PHT/BTB persist).
- Rename supports int/fp maps, returns old physicals on commit; ROB enforces in-order retirement and exception gating for commit1.
- Shared PRF with bypass; IssueBuffer uses it for operand reads and writes.
- Issue/execute covers integer ALU, branches/JAL/JALR, CSR array ops, mul, fixed-latency div, basic FP add/sub/mul via `ibex_fpu`, and ordered LSU; CDB writes PRF and ROB.
- ICache + DCache + MemArbiter + MainMemory path works for basic fetch/load/store (byte/half/word) with sign/zero extension and misaligned detection.
- Directed `Top_tb.v` programs execute; some tests still fail (see Notes).

## Notes / Risks (current)
- CDB arbitration still only supports two producers; if LSU completes in the same cycle as two other producers, one result can be dropped.
- TASK[70] currently times out at 32/48 commits (log: `cpu.sim/sim_1/behav/xsim/simulate.log`). The immediate blocker is an IssueBuffer deadlock: ROB head is a store (`pc=0x00001014`, `rob=5`) whose RS entry keeps `rs1_ready=0` even though the same physical source register reports ready in rename (`int_preg_ready=1`). Since LSU picks the oldest memory op and stores only issue at ROB head, this stale ready bit prevents all forward progress.
- Rename free list recovery remains risky: redirect recovery rebuilds free-list contents from checkpoints/ROB visibility rather than restoring a true per-checkpoint queue image, so multi-redirect corner cases can still mis-allocate physical registers.
- FP compare/cvt and destination typing are inconsistent: OP-FP compare decodes to `FP_OP_CMP` (alias of DUMMY), and compare results are routed to FP regs instead of integer regs.
- FP div/sqrt/cvt/fma semantics are not implemented; FMA uses C=0 in the current wiring.

## Gaps to a Fully Functional OoO Core
- **Frontend / Fetch**: Cross-line slot1 fetch is still suppressed; no replay for split line fetches or fine-grained bubble management on redirect. No explicit `fence.i`/I$ invalidate handling.
- **Memory / LSU**: LSU is single-outstanding and in-order; stores only issue at ROB head. No LSQ/store buffer, load-store forwarding, or load replay; no AMO/atomic ops. TLB is bare-bones (no permissions/page faults), and D-cache has no coherence or invalidate support.
- **Floating Point**: Only 2-source FP is wired (C input tied to 0), so real FMA/rs3 path is missing. Rounding mode is fixed (RNE); `fflags/frm` are not implemented. FP compare/convert destination type is not handled correctly (OP-FP always marks `rd` as FP). `ibex_fpu` effectively only supports add/sub/mul; compare/convert/div/sqrt/fma semantics are not implemented despite decode support.
- **Rename / Recovery**: Implement true checkpoint/restore of `int_free[]` and `fp_free[]` contents (per ROB checkpoint) so redirect recovery restores the full free list state; current partial recovery still mis-allocates after multiple flushes.
- **Issue / Execute**: FU timing is overly idealized (most ops are combinational); div is single in-flight. CDB only carries two producers per cycle and drops the rest; there is no backpressure/holding when >2 results complete. No operand-forwarding policy for long-latency overlap beyond the simple CDB wakeup.
- **Commit / Exceptions**: Exceptions are a single bit; no cause/mtval/PC capture, CSR side effects, or trap sequencing. Only commit0 exception triggers flush; commit1 exceptions are suppressed. Memory exceptions are not modeled precisely.
- **Integration / Debug**: Add perf/debug visibility (pipeline valid bits, stall reasons, predictor hit/miss counters) for bring-up.
- **Verification**: Unit tests for predictor accuracy, rename rollback correctness, ROB commit ordering, issue selection and CDB arbitration, divider behavior, CSR access. Pipeline smoke tests: straight-line code, dependency chains, branch-heavy sequences, divider usage, redirect/flush correctness.
