# Solver Fidelity Matrix

Audit date: 2026-03-28. Updated post-fix.

## Summary

| Solver | Paper | Status | Notes |
|--------|-------|--------|-------|
| PIBT | Okumura 2022 (IJCAI) | Not independently audited | Mature, 12 unit tests, collision-free verified |
| RHCR (PBS) | Li et al. 2021 (AAMAS) | Not independently audited | 10 unit tests, fallback tested |
| RHCR (PIBT-Window) | Li et al. 2021 (AAMAS) | Not independently audited | 1 unit test, collision-free verified |
| RHCR (Priority A*) | Li et al. 2021 (AAMAS) | Not independently audited | 2 unit tests, collision-free verified |
| Token Passing | Ma et al. 2017 (AAMAS) | Not independently audited | 4 unit tests, edge-swap verified |
| PIBT+APF | Pertzovsky et al. 2025 | Audited, 1 fix applied | Sequential APF, exponential decay, parameters match |
| TPTS | Ma et al. 2017 Alg. 2 | Audited, 1 fix applied | 4 documented deviations |
| RT-LaCAM | Liang et al. 2025 (SoCS) | Audited, 5 fixes applied | 2 documented deviations remain |

The original 5 solvers (PIBT, 3 RHCR, Token Passing) were not independently audited
against their papers in this session. They have mature test suites (collision-free,
determinism, metamorphic) but no line-by-line paper comparison was done.

## PIBT+APF (Pertzovsky et al., arXiv:2505.22753)

**Files:** `src/solver/apf_guidance.rs`, `src/solver/pibt_core.rs`

| Requirement | Status |
|-------------|--------|
| Sequential APF inside PIBT recursion | MATCH |
| Exponential decay w*gamma^(-dist) | MATCH |
| Goal cell returns bias 0 | MATCH |
| Parameters match Table 1 (w=0.1, gamma=3, d_max=2, t_max=2) | MATCH |
| NOT a GuidanceLayer wrapper | MATCH |
| Integration via cell_bias mechanism | MATCH |
| Idle agents contribute APF | MATCH (fixed) |

**Fix applied:** Idle agents now call `add_apf_for_agent` at their position.
Under fault conditions with many immobile agents, this repels tasked agents
from occupied cells.

**Remaining deviation:** APF projection starts from the agent's next position
(after PIBT commit), not current position. Minor impact with d_max=2. The
paper's Eq. 11 projects from the current configuration.

## TPTS (Ma et al., AAMAS 2017, Algorithm 2)

**Files:** `src/solver/tpts.rs`, `src/solver/token_common.rs`

| Requirement | Status |
|-------------|--------|
| Sequential token planning | MATCH |
| A* spacetime cost for swap | MATCH (fixed) |
| Snapshot before swap attempt | MATCH |
| Restore on swap failure | MATCH |

**Fix applied:** Both agents' paths are now removed from the constraint index
before the A* cost probes. Previously, each agent's own path inflated its
cost estimate, suppressing beneficial swaps.

**Documented deviations (4 total, in source header):**
1. No recursive GetTask (MAFIS TaskScheduler owns task assignment)
2. No Path2 endpoint parking (idle agents wait in place)
3. Swap cooldown to prevent oscillation (not in paper)
4. Bidirectional total-cost criterion (paper only checks one direction)

## RT-LaCAM (Liang et al., arXiv:2504.06091, SoCS 2025)

**Files:** `src/solver/rt_lacam.rs`

| Requirement | Status |
|-------------|--------|
| Lazy constraint DFS | MATCH |
| PIBT as config generator | MATCH (fixed: seed diversity) |
| Rerooting | PARTIAL |
| Persistent explored map | MATCH |
| Zobrist hashing | MATCH |
| Budget-bounded expansion | MATCH (fixed: early exit removed) |
| Memory cap + restart | MATCH |
| Horizon enforcement | MATCH (fixed: stale goal cleared) |

**Fixes applied (5):**
1. Removed MAX_DEPTH1_CANDIDATES early exit. Full budget now used for DFS.
2. PIBT seed now varies per node/depth (`zobrist_seed ^ node_id ^ lln.depth`).
3. max_horizon enforced in extract_next_config. Stale goal_node cleared after reroot.
4. Removed redundant O(n^2) collision check in generate_config.
5. Stopped re-pushing existing nodes to front of open (prevents cycling).

**Documented deviations (2, in source header):**
1. Rerooting only swaps the direct parent edge, not the full chain.
   BFS fallback handles deeper re-routing.
2. Rerooted node g is set to 0 (treated as new root).
