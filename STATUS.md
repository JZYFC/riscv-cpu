# Out-of-Order CPU Status

This document summarizes the current two-wide OoO core as wired today (including I$/D$/LSU and FP blocks), and what is missing to reach a fully functional CPU.

## Quick Snapshot
- Pipeline: IF → InstructionBuffer → PreDecode → RegRename/ROB → PostDecode → DispatchQueue → Issue/Execute → Commit.
- Width: 2-wide fetch/rename/issue/commit; shared int/fp physical register file (64 int + 64 fp). Default queue depths: InstructionBuffer=8, Issue RS=8.
- Control: Global flush on branch mispredict (issue redirect) or `commit0_exception` to `TRAP_VECTOR`; commit1 is suppressed on commit0 exception; redirect recovery now guards the one-cycle "branch already committed" window so wrapped ROB ranges do not retain wrong-path uops; predictor state is not cleared by pipeline flush so mispredict training is preserved; `PreDecode`/`PostDecode` pipeline registers are also explicitly cleared on flush to prevent wrong-path reinjection.
- ROB integrity: WB acceptance is now guarded by `{rob_idx, rob_gen}` matching. Each ROB slot carries a generation tag that increments on re-allocation, so stale late writebacks from squashed/reused slots are ignored.
- Execute coverage: Integer ALU, branch/JAL/JALR (link), CSR array ops (4K CSR file), mul (combinational), div (fixed 8-count, single in-flight), basic FP add/sub/mul via `ibex_fpu` (combinational, 2-source only). LSU integrated with single outstanding/ordered issue (TLB + DCache).
- Fetch status: Predictor wired; IF fetches from ICache (128-bit line buffer), stalls on miss; if predicted taken, slot1 is suppressed; if slot1 crosses a line, slot1 is suppressed.
- Memory system: ICache + DCache connect through MemArbiter to MainMemory; DCache is 2-way with write-through on hits and writeback on dirty victim.

## Pipeline Walkthrough (by stage)
- **IF (IF.v)**: Tracks PC, queries Gshare predictor (GHR=8, IDX=10, BTB+RAS), chooses predicted target vs sequential. Handles stall/flush/redirect. Fetches from ICache using a 128-bit line buffer; stalls on miss; predicted-taken or cross-line fetch suppresses slot1.
- **InstructionBuffer (InstructionBuffer.v)**: 2-wide enqueue/dequeue FIFO (depth 8) with backpressure to IF. Preserves PC and prediction metadata, flushes on redirect.
- **PreDecode (PreDecode.v)**: One-cycle predecode to extract rs1/rs2/rd, immediate, FU class (int/mem/mul/fp), and int vs fp register type flags. Carries prediction metadata through.
- **RegRename + ROB (RegRename.v)**: Logical→physical mapping for int/fp with free lists; allocates ROB entries for all valid instructions; hands out new physical destinations and old mappings; frees old physicals on commit. ROB is 32 entries, dual-alloc, dual-commit, dual-writeback. Per-ROB snapshots are used for redirect recovery of maps/head/count, but free list arrays are not checkpointed (see bug below).
- **PostDecode (PostDecode.v)**: Full ISA-oriented decode after rename. Classifies FU selection (int, mul/div, branch, lsu, fp, system) and emits per-unit micro-ops (int ALU ops/sub flag/cmp signed, mul/div controls, branch op, mem op+sign/size, CSR op/addr, fp op, illegal).
- **Issue/Execute (IssueBuffer.v)**: Reservation-station scheduler (RS depth 8) with age tracking; attempts dual-issue per cycle. Enqueue source-ready now uses live rename scoreboard queries plus same-cycle CDB bypass (instead of rename-time ready bits), then keeps CDB wakeup for in-RS entries. Executes int ALU, branch resolution, CSR array ops, mul (combinational), div (fixed 8-count, single in-flight), and basic FP add/sub/mul through `ibex_fpu` (C input tied to 0, rounding mode fixed to RNE). LSU requests are issued in-order with a single outstanding; stores only issue at ROB head. Produces up to two CDB results per cycle and PRF writes, writes ROB via Top, resolves branches (redirect + predictor training), asserts dispatch stall when RS lacks space.
- **LSU (LSU.v)**: Single-outstanding load/store engine with simple TLB + DCache. Detects misaligned access and returns a generic exception bit; no permission/page-fault modeling.
- **PhysicalRegFileShared (PhysicalRegFileShared.v)**: Shared int/fp PRF with two read pairs and two write ports per type; includes simple write bypass on reads.
- **Cache/Memory (ICache.v, Cache.v, MemArbiter.v, MainMemory.v)**: ICache provides 128-bit line fetches to IF; DCache is 2-way and services LSU requests via MemArbiter; MainMemory is the backing store.
- **Top (Top.v)**: Wires the whole pipeline, distributes flush/redirect, threads prediction metadata, instantiates shared PRF/DispatchQueue/LSU/caches, and exposes ROB commit signals. Flush on exception only consults `commit0_exception`.
- **Legacy helpers**: `FU.v` remains as a legacy/simple block; current pipeline uses IssueBuffer for execution.

## What Works Today
- Prediction pipeline (Gshare+BTB+RAS) with redirect and dual training ports; branch updates are applied even on redirect flush cycles (prevents lost training and redirect loops).
- Rename supports int/fp maps, returns old physicals on commit; ROB enforces in-order retirement and exception gating for commit1.
- Shared PRF with bypass; IssueBuffer uses it for operand reads and writes.
- Issue/execute covers integer ALU, branches/JAL/JALR, CSR array ops, mul, fixed-latency div, basic FP add/sub/mul via `ibex_fpu`, and ordered LSU; DispatchQueue decouples post-decode from RS pressure; CDB writes PRF and ROB.
- ICache + DCache + MemArbiter + MainMemory path works for basic fetch/load/store (byte/half/word) with sign/zero extension and misaligned detection.
- Directed `Top_tb.v` programs execute; some tests still fail (see Notes).

## Notes / Risks (current)
- CDB arbitration still only supports two producers; if LSU completes in the same cycle as two other producers, one result can be dropped.
- Stale WB filtering is now implemented end-to-end (`RegRename`/`ROB`, `PostDecode`, `DispatchQueue`, `IssueBuffer`, `LSU`, `Top`) via ROB generation tags; regression simulation is still required to confirm the previous TASK[70] commit/memory mismatch is fully resolved.
- Redirect recovery now handles the delayed-flush edge case where `rob_head` advances past `flush_rob_idx` before flush is applied; `IssueBuffer` and `RegRename` treat that case as "no survivors" instead of applying wrapped in-range filters.
- Predictor flush/update ordering was fixed in `BranchPredictor.v`: pipeline flush no longer suppresses predictor updates, which addressed repeated mispredict loops and wrong-path exception commits in TASK[70].
- Decode-pipeline flush gap was fixed (`PreDecode.v`, `PostDecode.v`, `Top.v`): both decode register stages now consume `global_flush` and bubble outputs immediately on flush, preventing stale wrong-path uops with old ROB tags from being re-enqueued after recovery.
- TASK[70] is still blocked by fetch/data memory transaction instability:
  - `MainMemory.v` completes requests using live `mem_addr` (not latched accepted address).
  - `MemArbiter.v` serves `SERVE_D/SERVE_I` using live `d_*` / `i_*` signals (not latched in-flight fields).
  - `Cache.v` miss/writeback path still references live `paddr/we/wdata/offset` during `WRITEBACK/REFILL`.
  - Under mixed I/D traffic this can refill/return the wrong line, producing PC/instruction mismatches (observed in TASK[70] log) and early fall-through to `0x10c0`.
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
