# Out-of-Order CPU Implementation Status

## Overall Status
- Current testbench are all passing.
- The core is operating as a functional dual-issue OoO prototype with integrated I$/D$, ROB, rename, branch prediction, LSU, and basic FP pipeline support.

## Architecture Snapshot
- Pipeline: `IF -> InstructionBuffer -> PreDecode -> RegRename/ROB -> PostDecode -> DispatchQueue -> Issue/Execute -> Commit`.
- Width: 2-wide fetch/rename/issue/commit.
- Register files: shared physical files (`int=64`, `fp=64`) with rename + ROB retirement.
- ROB: 32 entries, dual allocate/dual commit, generation-tagged WB protection.
- Memory: shared MainMemory behind MemArbiter, with ICache + DCache frontends.

## Implemented and Working

### Frontend and control recovery
- Branch predictor, redirect path, and global flush are integrated.
- Decode pipeline registers (`PreDecode`, `PostDecode`) now explicitly clear on flush.
- Redirect edge case where branch commits before flush application is handled (`redirect_keep_none`).

### Rename/ROB correctness hardening
- ROB writeback now validates `{rob_idx, rob_gen}` to block stale late completions.
- Rename/Issue/LSU/Top path carries ROB generation metadata end-to-end.
- WB scoreboard declaration-order warning in `RegRename.v` is fixed.

### Memory path hardening
- Response-address tagging is implemented in shared memory path.
- ICache/DCache refill logic now guards installs by response-address match.
- This prevents cross-target refill acceptance under mixed I/D traffic.

### Task-level test infrastructure
- `Top_tb.v` task completion now uses retired PC completion (`pc == init_ra`) instead of fixed commit count.
- Commit/cycle counts are treated as safety limits.
- `DBG_*` task traces are fail-oriented and suppressed on passing runs.

## Current Known Limitations (Non-blocking)
- LSU model is conservative (single outstanding; no full LSQ/store buffer/load replay).
- CDB producer pressure is still limited by current arbitration width.
- FP support is partial relative to full ISA expectations (compare/cvt/div/sqrt/fma semantics and `fflags/frm` behavior are not fully modeled).
- Exception model remains simplified (coarse exception signaling versus full architectural trap metadata flow).

## Suggested Next Priorities (by gpt-5.3-codex)
1. Expand FP correctness to full architectural behavior (`fflags/frm`, compare/cvt/div/sqrt/fma semantics).
2. Upgrade memory subsystem toward LSQ/store-buffered OoO memory scheduling.
3. Add stress tests for high producer concurrency (CDB pressure), multi-redirect recovery, and memory ordering corner cases.
4. Add structured performance/debug counters for long-run analysis (stall reasons, predictor quality, load miss latency).

