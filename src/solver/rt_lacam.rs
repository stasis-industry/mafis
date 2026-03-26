//! RT-LaCAM — Real-Time LaCAM with persistent DFS state.
//!
//! Configuration-space DFS that runs for a bounded node budget per tick,
//! remembering search state between invocations. Natively lifelong.
//!
//! Reference: arXiv:2504.06091, SoCS 2025 — "Real-Time LaCAM"

use bevy::prelude::*;
use smallvec::smallvec;
use std::collections::HashSet;

use crate::core::action::{Action, Direction};
use crate::core::grid::GridMap;
use crate::core::seed::SeededRng;
use crate::core::task::TaskLeg;

use super::heuristics::{DistanceMap, DistanceMapCache, delta_to_action};
use super::lifelong::{AgentPlan, AgentState, LifelongSolver, SolverContext, StepResult};
use super::pibt_core::PibtCore;
use super::traits::{Optimality, Scalability, SolverInfo};

use crate::constants::{
    RT_LACAM_MAX_HORIZON, RT_LACAM_MAX_VISITED, RT_LACAM_MIN_HORIZON, RT_LACAM_NODE_BUDGET,
    RT_LACAM_ZOBRIST_SEED,
};

// ---------------------------------------------------------------------------
// Zobrist hashing — formula-based, zero allocation
// ---------------------------------------------------------------------------

#[inline]
fn zobrist_hash(agent: usize, cell: usize, seed: u64) -> u64 {
    let mut x = seed
        ^ (agent as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15)
        ^ (cell as u64).wrapping_mul(0x517C_C1B7_2722_0A95);
    x = (x ^ (x >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    x ^ (x >> 31)
}

fn hash_config(positions: &[IVec2], width: i32, seed: u64) -> u64 {
    let mut h: u64 = 0;
    for (i, &pos) in positions.iter().enumerate() {
        let cell = (pos.y * width + pos.x) as usize;
        h ^= zobrist_hash(i, cell, seed);
    }
    h
}

// ---------------------------------------------------------------------------
// Configuration — joint state of all agents
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct Configuration {
    positions: Vec<IVec2>,
    hash: u64,
    depth: usize,
}

// ---------------------------------------------------------------------------
// RT-LaCAM Solver
// ---------------------------------------------------------------------------

pub struct RtLaCAMSolver {
    // Config
    node_budget: usize,
    max_horizon: usize,

    // Persistent search state
    dfs_stack: Vec<Configuration>,
    visited: HashSet<u64>,
    search_active: bool,

    // Metadata
    grid_width: i32,
    last_num_agents: usize,
    zobrist_seed: u64,

    // Output
    plan_buffer: Vec<AgentPlan>,

    // Fallback
    pibt_fallback: PibtCore,

    // Scratch buffers
    agent_pairs_buf: Vec<(IVec2, IVec2)>,
    positions_buf: Vec<IVec2>,
    goals_buf: Vec<IVec2>,
    has_task_buf: Vec<bool>,
}

impl RtLaCAMSolver {
    pub fn new(grid_area: usize, _num_agents: usize) -> Self {
        let horizon = ((grid_area as f64).sqrt() as usize)
            .clamp(RT_LACAM_MIN_HORIZON, RT_LACAM_MAX_HORIZON);

        Self {
            node_budget: RT_LACAM_NODE_BUDGET,
            max_horizon: horizon,
            dfs_stack: Vec::new(),
            visited: HashSet::new(),
            search_active: false,
            grid_width: 0,
            last_num_agents: 0,
            zobrist_seed: RT_LACAM_ZOBRIST_SEED,
            plan_buffer: Vec::new(),
            pibt_fallback: PibtCore::new(),
            agent_pairs_buf: Vec::new(),
            positions_buf: Vec::new(),
            goals_buf: Vec::new(),
            has_task_buf: Vec::new(),
        }
    }

    fn restart_search(&mut self) {
        self.dfs_stack.clear();
        self.visited.clear();
        self.search_active = false;
    }

    /// Generate candidate next positions for one agent, sorted by distance to goal.
    fn agent_candidates(pos: IVec2, grid: &GridMap, dist_map: &DistanceMap) -> Vec<IVec2> {
        let mut cands = Vec::with_capacity(5);
        for dir in Direction::ALL {
            let next = pos + dir.offset();
            if grid.is_walkable(next) {
                cands.push(next);
            }
        }
        cands.push(pos); // Wait
        cands.sort_unstable_by_key(|&c| dist_map.get(c));
        cands
    }

    /// Run bounded DFS expansion. Returns number of nodes expanded.
    fn expand_dfs(
        &mut self,
        grid: &GridMap,
        goals: &[IVec2],
        dist_maps: &[&DistanceMap],
    ) -> usize {
        let n = goals.len();
        let width = self.grid_width;
        let seed = self.zobrist_seed;
        let mut expanded = 0;

        while expanded < self.node_budget && !self.dfs_stack.is_empty() {
            // Cap visited set
            if self.visited.len() > RT_LACAM_MAX_VISITED {
                self.restart_search();
                break;
            }

            let config = self.dfs_stack.pop().unwrap();
            expanded += 1;

            if config.depth >= self.max_horizon {
                continue;
            }

            // Check all at goals
            if config
                .positions
                .iter()
                .zip(goals.iter())
                .all(|(p, g)| p == g)
            {
                continue;
            }

            // Greedily assign each agent to best candidate (recompute, no storage)
            let mut new_positions = config.positions.clone();
            let mut decided = vec![false; n];

            for agent in 0..n {
                let cands =
                    Self::agent_candidates(config.positions[agent], grid, dist_maps[agent]);
                let mut found = false;

                for &cand in &cands {
                    // Check vertex collision
                    let vertex_ok = (0..n)
                        .all(|j| j == agent || !decided[j] || new_positions[j] != cand);

                    // Check edge collision (swap)
                    let edge_ok = (0..n).all(|j| {
                        j == agent
                            || !decided[j]
                            || !(new_positions[j] == config.positions[agent]
                                && cand == config.positions[j])
                    });

                    if vertex_ok && edge_ok {
                        new_positions[agent] = cand;
                        decided[agent] = true;
                        found = true;
                        break;
                    }
                }
                if !found {
                    new_positions[agent] = config.positions[agent];
                    decided[agent] = true;
                }
            }

            let new_hash = hash_config(&new_positions, width, seed);

            if !self.visited.contains(&new_hash) {
                self.visited.insert(new_hash);
                self.dfs_stack.push(Configuration {
                    positions: new_positions,
                    hash: new_hash,
                    depth: config.depth + 1,
                });
            }
        }

        expanded
    }

    /// Use PIBT as fallback when DFS hasn't found a usable plan.
    fn pibt_fallback_step<'a>(
        &'a mut self,
        ctx: &SolverContext,
        agents: &[AgentState],
        distance_cache: &mut DistanceMapCache,
    ) -> StepResult<'a> {
        self.pibt_fallback.set_shuffle_seed(ctx.tick);

        self.positions_buf.clear();
        self.positions_buf.extend(agents.iter().map(|a| a.pos));

        self.goals_buf.clear();
        self.goals_buf
            .extend(agents.iter().map(|a| a.goal.unwrap_or(a.pos)));

        self.agent_pairs_buf.clear();
        self.agent_pairs_buf
            .extend(agents.iter().map(|a| (a.pos, a.goal.unwrap_or(a.pos))));

        let dist_maps = distance_cache.get_or_compute(ctx.grid, &self.agent_pairs_buf);

        self.has_task_buf.clear();
        self.has_task_buf.extend(agents.iter().map(|a| {
            let goal = a.goal.unwrap_or(a.pos);
            goal != a.pos
        }));

        let actions = self.pibt_fallback.one_step_with_tasks(
            &self.positions_buf,
            &self.goals_buf,
            ctx.grid,
            &dist_maps,
            &self.has_task_buf,
        );

        self.plan_buffer.clear();
        for (i, &action) in actions.iter().enumerate() {
            self.plan_buffer.push((agents[i].index, smallvec![action]));
        }

        StepResult::Replan(&self.plan_buffer)
    }
}

impl LifelongSolver for RtLaCAMSolver {
    fn name(&self) -> &'static str {
        "rt_lacam"
    }

    fn info(&self) -> SolverInfo {
        SolverInfo {
            optimality: Optimality::Suboptimal,
            complexity: "O(node_budget) per tick, amortized config-space DFS",
            scalability: Scalability::High,
            description:
                "RT-LaCAM — real-time configuration-space DFS with persistent search state.",
            recommended_max_agents: None,
        }
    }

    fn reset(&mut self) {
        self.restart_search();
        self.pibt_fallback.reset();
        self.plan_buffer.clear();
    }

    fn step<'a>(
        &'a mut self,
        ctx: &SolverContext,
        agents: &[AgentState],
        distance_cache: &mut DistanceMapCache,
        _rng: &mut SeededRng,
    ) -> StepResult<'a> {
        if agents.is_empty() {
            self.plan_buffer.clear();
            return StepResult::Replan(&self.plan_buffer);
        }

        let n = agents.len();

        // Detect agent/grid changes → restart
        if n != self.last_num_agents || ctx.grid.width != self.grid_width {
            self.grid_width = ctx.grid.width;
            self.last_num_agents = n;
            self.restart_search();
        }

        // Build distance maps
        self.agent_pairs_buf.clear();
        self.agent_pairs_buf
            .extend(agents.iter().map(|a| (a.pos, a.goal.unwrap_or(a.pos))));
        let dist_maps = distance_cache.get_or_compute(ctx.grid, &self.agent_pairs_buf);

        // Initialize search if needed
        if !self.search_active {
            let positions: Vec<IVec2> = agents.iter().map(|a| a.pos).collect();
            let hash = hash_config(&positions, self.grid_width, self.zobrist_seed);

            self.visited.clear();
            self.visited.insert(hash);
            self.dfs_stack.clear();
            self.dfs_stack.push(Configuration {
                positions,
                hash,
                depth: 0,
            });
            self.search_active = true;
        }

        self.goals_buf.clear();
        self.goals_buf
            .extend(agents.iter().map(|a| a.goal.unwrap_or(a.pos)));

        // Clone goals to avoid borrow conflict with &mut self in expand_dfs.
        let goals_snapshot = self.goals_buf.clone();

        // Run bounded DFS
        self.expand_dfs(ctx.grid, &goals_snapshot, &dist_maps);

        // Check if DFS produced a depth-1 child we can commit
        // The stack may contain configs at various depths. Find one at depth 1.
        let commit_idx = self
            .dfs_stack
            .iter()
            .rposition(|c| c.depth == 1);

        if let Some(idx) = commit_idx {
            let next_config = &self.dfs_stack[idx];
            // Validate walkability (audit fix #6)
            let all_walkable = next_config
                .positions
                .iter()
                .all(|&p| ctx.grid.is_walkable(p));

            if all_walkable {
                self.plan_buffer.clear();
                for (i, a) in agents.iter().enumerate() {
                    if i < next_config.positions.len() {
                        let action = delta_to_action(a.pos, next_config.positions[i]);
                        self.plan_buffer.push((a.index, smallvec![action]));
                    }
                }
                self.restart_search();
                return StepResult::Replan(&self.plan_buffer);
            } else {
                self.restart_search();
            }
        }

        // No usable plan — PIBT fallback
        self.pibt_fallback_step(ctx, agents, distance_cache)
    }

    fn save_priorities(&self) -> Vec<f32> {
        self.pibt_fallback.priorities().to_vec()
    }

    fn restore_priorities(&mut self, priorities: &[f32]) {
        self.pibt_fallback.set_priorities(priorities);
        self.restart_search();
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::grid::GridMap;
    use crate::core::seed::SeededRng;
    use crate::core::topology::ZoneMap;
    use crate::solver::heuristics::DistanceMapCache;
    use std::collections::HashMap;

    fn test_zones() -> ZoneMap {
        ZoneMap {
            pickup_cells: vec![IVec2::new(0, 0)],
            delivery_cells: vec![IVec2::new(4, 4)],
            corridor_cells: Vec::new(),
            recharging_cells: Vec::new(),
            zone_type: HashMap::new(),
            queue_lines: Vec::new(),
        }
    }

    #[test]
    fn rt_lacam_empty_agents() {
        let grid = GridMap::new(5, 5);
        let zones = test_zones();
        let mut solver = RtLaCAMSolver::new(25, 0);
        let mut cache = DistanceMapCache::default();
        let mut rng = SeededRng::new(42);
        let ctx = SolverContext {
            grid: &grid,
            zones: &zones,
            tick: 0,
            num_agents: 0,
        };
        let result = solver.step(&ctx, &[], &mut cache, &mut rng);
        assert!(matches!(result, StepResult::Replan(plans) if plans.is_empty()));
    }

    #[test]
    fn rt_lacam_single_agent_reaches_goal() {
        let grid = GridMap::new(5, 5);
        let zones = test_zones();
        let mut solver = RtLaCAMSolver::new(25, 1);
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
            let ctx = SolverContext {
                grid: &grid,
                zones: &zones,
                tick,
                num_agents: 1,
            };
            if let StepResult::Replan(plans) = solver.step(&ctx, &agents, &mut cache, &mut rng) {
                if let Some((_, actions)) = plans.first() {
                    if let Some(action) = actions.first() {
                        pos = action.apply(pos);
                    }
                }
            }
            if pos == goal {
                return;
            }
        }
        assert_eq!(pos, goal);
    }

    #[test]
    fn rt_lacam_reset_clears_state() {
        let mut solver = RtLaCAMSolver::new(25, 5);
        solver.reset();
        assert!(solver.dfs_stack.is_empty());
        assert!(solver.visited.is_empty());
        assert!(!solver.search_active);
    }

    #[test]
    fn rt_lacam_deterministic() {
        let grid = GridMap::new(5, 5);
        let zones = test_zones();
        let goal = IVec2::new(3, 3);
        let mut results = Vec::new();

        for _ in 0..2 {
            let mut solver = RtLaCAMSolver::new(25, 1);
            let mut cache = DistanceMapCache::default();
            let mut rng = SeededRng::new(42);
            let mut pos = IVec2::ZERO;
            let mut positions = Vec::new();

            for tick in 0..15 {
                let agents = vec![AgentState {
                    index: 0,
                    pos,
                    goal: Some(goal),
                    has_plan: tick > 0,
                    task_leg: TaskLeg::TravelEmpty(goal),
                }];
                let ctx = SolverContext {
                    grid: &grid,
                    zones: &zones,
                    tick,
                    num_agents: 1,
                };
                if let StepResult::Replan(plans) =
                    solver.step(&ctx, &agents, &mut cache, &mut rng)
                {
                    if let Some((_, actions)) = plans.first() {
                        if let Some(action) = actions.first() {
                            pos = action.apply(pos);
                        }
                    }
                }
                positions.push(pos);
            }
            results.push(positions);
        }
        assert_eq!(results[0], results[1]);
    }

    #[test]
    fn zobrist_hash_different_configs() {
        let h1 = hash_config(
            &[IVec2::new(0, 0), IVec2::new(1, 0)],
            5,
            RT_LACAM_ZOBRIST_SEED,
        );
        let h2 = hash_config(
            &[IVec2::new(1, 0), IVec2::new(0, 0)],
            5,
            RT_LACAM_ZOBRIST_SEED,
        );
        assert_ne!(h1, h2);
    }

    #[test]
    fn zobrist_hash_is_deterministic() {
        let positions = vec![IVec2::new(2, 3), IVec2::new(4, 1)];
        let h1 = hash_config(&positions, 5, RT_LACAM_ZOBRIST_SEED);
        let h2 = hash_config(&positions, 5, RT_LACAM_ZOBRIST_SEED);
        assert_eq!(h1, h2);
    }
}
