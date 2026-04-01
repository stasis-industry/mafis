# MAFIS Roadmap

Known gaps, planned improvements, and non-goals.
Items are not prioritized within a section — promote to a GitHub Issue when ready to act.

---

## Solver: RHCR Improvements

### PBS — Focal Search (FOCAL_W heuristic)
**Status:** Not implemented. PBS currently uses pure-cost tie-breaking.  
**Impact:** Medium — PBS throughput is below reference on dense/warehouse maps. The reference C++ implementation uses a focal list (FOCAL_W = 1.0 by default) that finds conflict-minimal solutions faster.  
**What's needed:** Maintain a focal set of nodes within `[g_min, g_min * FOCAL_W]` from the open list; expand by conflict count (fewest conflicts first). Requires a separate heap sorted by `h_conflicts`, drained in parallel with the open list.  
**Reference:** `src/solver/pbs_planner.rs` — `PbsPlanner::plan_window()`; reference impl at `RHCR/src/PBS.cpp` (`focal_list`).

### Travel Penalties Wired Into Windowed Planners
**Status:** `WindowContext.travel_penalties` is populated but not yet consumed by any `WindowedPlanner`.  
**Impact:** Low — congestion-aware path selection is already tracked via `wait_counts` in `rhcr.rs`; the penalty layer is designed but inactive.  
**What's needed:** In `priority_astar_planner.rs` and `pbs_planner.rs`, add `travel_penalties[y*w+x]` as an additive g-cost bias in `spacetime_astar_fast`/`spacetime_astar_sequential`.  
**Reference:** `src/solver/windowed.rs:37` (`travel_penalties` field), `src/solver/priority_astar_planner.rs`.

### Distance Map Caching for Goal-Sequence Goals
**Status:** `fill_goal_sequences()` appends extra goals beyond the current immediate goal. The RHCR solver computes a `DistanceMap` for each agent's immediate goal but not for the extra sequence goals.  
**Impact:** Low — windowed planners currently use only the first distance map for sequence goals; heuristic quality degrades for agents pursuing a chained goal.  
**What's needed:** Cache `DistanceMap` per goal cell across replanning windows (evict when the goal is no longer in any agent's sequence). Pass aligned distance maps for each sequence goal into `WindowContext`.  
**Reference:** `src/solver/rhcr.rs` — `rebuild_distance_maps()` and `fill_goal_sequences()`.

---

## Analysis

### Formal MTBF / MTTR Confidence Intervals
**Status:** MTBF and MTTR are point estimates.  
**Impact:** Low for internal use, higher if published — reviewers will ask for CIs on these metrics.  
**What's needed:** Bootstrap CI (already done for throughput via `src/analysis/metrics.rs`) applied to MTBF/MTTR time series.

---

## Non-Goals

These are intentionally out of scope for MAFIS:

- **One-shot solver support** (CBS, LaCAM, PBS one-shot, LNS2) — MAFIS measures fault resilience under sustained operation; one-shot solvers are not lifelong-capable.
- **Optimal path planning** — MAFIS is a fault observatory, not a solver benchmark. Suboptimality is acceptable if throughput and resilience are measurable.
- **Real robot integration** — simulation-only by design.
