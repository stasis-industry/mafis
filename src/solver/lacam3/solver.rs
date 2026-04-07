//! LaCAM3LifelongSolver — wraps lacam3 for use as a MAFIS lifelong solver.
//!
//! REFERENCE: docs/papers_codes/lacam3/lacam3/src/pibt.cpp (configuration generator)
//! Lifelong adaptation is MAFIS-specific — lacam3 has no equivalent.
//!
//! ## Why this uses lacam3's PIBT directly instead of the full LaCAM* search
//!
//! lacam3's `Planner` (LaCAM* high-level configuration-space search) is
//! designed for **one-shot** MAPF: given (starts, goals), find a multi-step
//! plan where ALL agents reach their goals simultaneously. The search
//! terminates when `is_same_config(H->C, ins->goals)`.
//!
//! In MAFIS's lifelong/MAPD workload, agents constantly receive new goals
//! from the task scheduler as they complete old ones. Wrapping LaCAM* in a
//! naive replan loop produces two failure modes:
//!
//! 1. **Stale plans**: a multi-step plan computed at tick T becomes invalid
//!    by tick T+K when half the agents have new goals. The wrapper has to
//!    discard and replan often, losing the multi-step optimality benefit.
//! 2. **Search timeout**: as the workload progresses, the heterogeneity of
//!    agent goal distances grows (some agents are 5 cells from goal, others
//!    50). LaCAM*'s search budget gets exhausted finding configurations
//!    where ALL agents are at goals, and starts returning empty solutions.
//!    Empirically observed: lacam3+LaCAM* lifelong wrapper degrades to ~0.1
//!    tasks/tick after ~200 ticks on warehouse_large, vs PIBT's ~0.4.
//!
//! A proper lifelong LaCAM* adaptation requires **rerooting** and
//! **persistent search across ticks** (RT-LaCAM, Liang et al. SoCS 2025).
//! That work is out of scope for PAAMS 2026 — Liang's source isn't
//! publicly available, and implementing it from scratch reintroduces the
//! fidelity risk we cut RT-LaCAM for.
//!
//! ### What this wrapper actually uses
//!
//! Each tick, the wrapper calls `Pibt::set_new_config` directly to get a
//! single-step configuration update. This makes `lacam3_lifelong` =
//! "lacam3's specialized PIBT + swap technique called per-tick", which:
//!
//! - Uses the canonical lacam3/src/pibt.cpp implementation
//! - Includes the swap technique from the LaCAM* paper
//! - Differs from MAFIS's standalone PIBT (which uses pibt2/src/pibt.cpp,
//!   without the swap technique optimization from the LaCAM* paper)
//! - Is what lacam3 itself uses internally as its configuration generator
//!
//! The full LaCAM* search code remains in `planner.rs` (for one-shot tests
//! and future work). It is not invoked from this lifelong wrapper.
//!
//! ## Honest naming
//!
//! In experimental results, this solver should be presented as:
//!   "lacam3-PIBT-lifelong (lacam3's configuration generator wrapped per-tick)"
//! rather than "lacam3 lifelong" — to be transparent that we are not running
//! the full SOTA LaCAM* search in lifelong mode.

use bevy::prelude::*;

use crate::core::seed::SeededRng;
use crate::solver::lifelong::{
    AgentPlan, AgentState, LifelongSolver, SolverContext, StepResult,
};
use crate::solver::shared::heuristics::DistanceMapCache;
use crate::solver::shared::traits::{Optimality, Scalability, SolverInfo};

use super::dist_table::DistTable;
use super::instance::{Instance, id_to_pos, pos_to_id};
use super::pibt::Pibt;

pub struct LaCAM3LifelongSolver {
    /// Per-tick scratch buffer for action plans.
    plan_buffer: Vec<AgentPlan>,
    /// Tick counter for seeding lacam3's internal RNG (so each replan is
    /// reproducible but different across ticks).
    tick_counter: u64,
}

impl Default for LaCAM3LifelongSolver {
    fn default() -> Self {
        Self::new()
    }
}

impl LaCAM3LifelongSolver {
    pub fn new() -> Self {
        Self { plan_buffer: Vec::new(), tick_counter: 0 }
    }
}

impl LifelongSolver for LaCAM3LifelongSolver {
    fn name(&self) -> &'static str {
        "lacam3_lifelong"
    }

    fn info(&self) -> SolverInfo {
        SolverInfo {
            optimality: Optimality::Suboptimal,
            complexity: "O(N) per tick (single-step PIBT with swap)",
            scalability: Scalability::High,
            description: "LaCAM3-PIBT — lacam3's configuration generator (with swap technique from the LaCAM* paper) called per-tick. The full LaCAM* high-level search is not used in lifelong mode (see solver.rs docstring).",
            source: "Okumura, AAMAS 2024 (PIBT submodule only)",
            recommended_max_agents: Some(1000),
        }
    }

    fn reset(&mut self) {
        self.plan_buffer.clear();
        self.tick_counter = 0;
    }

    fn step<'a>(
        &'a mut self,
        ctx: &SolverContext,
        agents: &[AgentState],
        _distance_cache: &mut DistanceMapCache,
        _rng: &mut SeededRng,
    ) -> StepResult<'a> {
        let n = agents.len();
        self.plan_buffer.clear();
        if n == 0 {
            return StepResult::Replan(&self.plan_buffer);
        }

        // Build start and goal vectors. Idle agents get goal=start.
        let mut starts: Vec<IVec2> = Vec::with_capacity(n);
        let mut goals: Vec<IVec2> = Vec::with_capacity(n);
        for a in agents {
            starts.push(a.pos);
            goals.push(a.goal.unwrap_or(a.pos));
        }

        // Build the instance and lacam3 PIBT.
        let ins = Instance::new(ctx.grid, starts, goals);
        let dt = DistTable::new(ctx.grid, &ins);
        // Tick-based seed: deterministic per (sim seed, tick) but varies
        // each tick so PIBT tie-breakers don't get stuck on the same RNG.
        self.tick_counter = self.tick_counter.wrapping_add(1);
        let seed = ctx.tick.wrapping_mul(31).wrapping_add(self.tick_counter);
        let mut pibt = Pibt::new(&ins, &dt, seed, true /* flg_swap */, None);

        // Compute the next configuration via single-step PIBT.
        let q_from = ins.starts.clone();
        let mut q_to = vec![u32::MAX; n];
        let order: Vec<u32> = (0..n as u32).collect();
        let success = pibt.set_new_config(&q_from, &mut q_to, &order);

        // Emit one action per agent.
        let grid_width = ctx.grid.width;
        for (i, a) in agents.iter().enumerate() {
            let action = if success && q_to[i] != u32::MAX {
                let from = id_to_pos(q_from[i], grid_width);
                let to = id_to_pos(q_to[i], grid_width);
                crate::solver::shared::heuristics::delta_to_action(from, to)
            } else {
                // PIBT failed for this agent — emit Wait.
                crate::core::action::Action::Wait
            };
            self.plan_buffer.push((a.index, smallvec::smallvec![action]));
        }

        StepResult::Replan(&self.plan_buffer)
    }
}

// Suppress unused-import warning for `pos_to_id` (kept for symmetry with id_to_pos).
#[allow(dead_code)]
fn _force_pos_to_id_use() {
    let _ = pos_to_id(IVec2::ZERO, 1);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::grid::GridMap;
    use crate::core::task::TaskLeg;
    use crate::core::topology::ZoneMap;
    use std::collections::HashMap as StdHashMap;

    fn test_zones() -> ZoneMap {
        ZoneMap {
            pickup_cells: vec![IVec2::new(0, 0)],
            delivery_cells: vec![IVec2::new(4, 4)],
            corridor_cells: Vec::new(),
            recharging_cells: Vec::new(),
            zone_type: StdHashMap::new(),
            queue_lines: Vec::new(),
        }
    }

    #[test]
    fn lacam3_lifelong_empty_agents() {
        let grid = GridMap::new(5, 5);
        let zones = test_zones();
        let mut solver = LaCAM3LifelongSolver::new();
        let mut cache = DistanceMapCache::default();
        let mut rng = SeededRng::new(42);
        let ctx = SolverContext { grid: &grid, zones: &zones, tick: 0, num_agents: 0 };
        let result = solver.step(&ctx, &[], &mut cache, &mut rng);
        assert!(matches!(result, StepResult::Replan(plans) if plans.is_empty()));
    }

    #[test]
    fn lacam3_lifelong_single_agent_reaches_goal() {
        let grid = GridMap::new(5, 5);
        let zones = test_zones();
        let mut solver = LaCAM3LifelongSolver::new();
        let mut cache = DistanceMapCache::default();
        let mut rng = SeededRng::new(42);

        let mut pos = IVec2::ZERO;
        let goal = IVec2::new(4, 4);

        for tick in 0..30 {
            let agents = vec![AgentState {
                index: 0,
                pos,
                goal: Some(goal),
                has_plan: tick > 0,
                task_leg: TaskLeg::TravelEmpty(goal),
            }];
            let ctx = SolverContext { grid: &grid, zones: &zones, tick, num_agents: 1 };
            if let StepResult::Replan(plans) = solver.step(&ctx, &agents, &mut cache, &mut rng) {
                if let Some((_, actions)) = plans.first() {
                    if let Some(action) = actions.first() {
                        let new_pos = action.apply(pos);
                        assert!(grid.is_walkable(new_pos), "moved to obstacle at tick {tick}");
                        pos = new_pos;
                    }
                }
            }
            if pos == goal {
                return;
            }
        }
        assert_eq!(pos, goal, "agent should reach goal within 30 ticks");
    }

    #[test]
    fn lacam3_lifelong_two_agents_no_collision() {
        let grid = GridMap::new(7, 7);
        let zones = test_zones();
        let mut solver = LaCAM3LifelongSolver::new();
        let mut cache = DistanceMapCache::default();
        let mut rng = SeededRng::new(42);

        let mut positions = vec![IVec2::new(0, 3), IVec2::new(6, 3)];
        let goals = vec![IVec2::new(6, 3), IVec2::new(0, 3)];

        for tick in 0..40 {
            let agents: Vec<AgentState> = (0..2)
                .map(|i| AgentState {
                    index: i,
                    pos: positions[i],
                    goal: Some(goals[i]),
                    has_plan: tick > 0,
                    task_leg: TaskLeg::TravelEmpty(goals[i]),
                })
                .collect();
            let ctx = SolverContext { grid: &grid, zones: &zones, tick, num_agents: 2 };
            if let StepResult::Replan(plans) = solver.step(&ctx, &agents, &mut cache, &mut rng) {
                for (idx, actions) in plans {
                    if let Some(action) = actions.first() {
                        let new_pos = action.apply(positions[*idx]);
                        assert!(grid.is_walkable(new_pos));
                        positions[*idx] = new_pos;
                    }
                }
            }
            if positions[0] == positions[1] {
                panic!("vertex collision at tick {tick}: {:?}", positions);
            }
        }
    }

    /// End-to-end validation: lacam3-PIBT runs through the full MAFIS
    /// experiment runner on warehouse_large with 20 agents, 200 ticks, no faults.
    #[test]
    fn lacam3_lifelong_warehouse_large_baseline() {
        use crate::experiment::config::ExperimentConfig;
        use crate::experiment::runner::run_single_experiment;

        let config = ExperimentConfig {
            solver_name: "lacam3_lifelong".into(),
            topology_name: "warehouse_large".into(),
            scenario: None,
            scheduler_name: "random".into(),
            num_agents: 20,
            seed: 42,
            tick_count: 200,
            custom_map: None,
        };
        let result = run_single_experiment(&config);
        let tp = result.baseline_metrics.avg_throughput;
        let tasks = result.baseline_metrics.total_tasks;
        eprintln!(
            "lacam3_lifelong_warehouse_large_baseline: tp={tp:.4} tasks/tick, total_tasks={tasks}"
        );
        assert!(
            tp > 0.05,
            "lacam3_lifelong should produce non-trivial throughput (>0.05) on warehouse_large"
        );
    }
}
