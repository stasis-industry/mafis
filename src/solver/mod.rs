pub mod lifelong;
pub mod pibt;
// RHCR is compute-intensive (PBS tree search with O(N × cascade) plan_agent
// calls per window) and requires rayon parallelism on the root-node build to
// stay interactive at realistic agent counts. WASM is single-threaded and
// cannot run rayon, so RHCR is **native-only**. The web build gets PIBT,
// Token Passing — both of which hit 60 FPS at 200+ agents in WASM.
// RHCR users should run the desktop build.
#[cfg(not(target_arch = "wasm32"))]
pub mod rhcr;
pub mod shared;
pub mod token;

// ---------------------------------------------------------------------------
// Backward-compatible re-exports — external callers keep working
// ---------------------------------------------------------------------------

// `crate::solver::heuristics::*` still works
pub use shared::heuristics;
// `crate::solver::traits::*` still works
pub use shared::traits;
// `crate::solver::pibt::PibtLifelongSolver` etc. still work (via pibt/mod.rs re-exports)

use bevy::prelude::*;

use self::lifelong::LifelongSolver;
use self::pibt::{PibtLifelongSolver, default_active_solver};
#[cfg(not(target_arch = "wasm32"))]
use self::rhcr::{RhcrConfig, RhcrSolver};
use self::token::TokenPassingSolver;

// ---------------------------------------------------------------------------
// Solver registry
// ---------------------------------------------------------------------------

/// All available solver names with human-readable labels.
///
/// Fidelity discipline: every solver in this registry has a faithful Rust
/// implementation traceable to a public reference source under `docs/papers_codes/`.
///
/// **Web (WASM) excludes RHCR** — it would need rayon for interactive
/// performance at realistic agent counts, which WASM cannot provide.
#[cfg(not(target_arch = "wasm32"))]
pub const SOLVER_NAMES: &[(&str, &str)] = &[
    ("pibt", "PIBT — Priority Inheritance with Backtracking"),
    ("rhcr_pbs", "RHCR (PBS) — Rolling-Horizon with Priority-Based Search"),
    ("token_passing", "Token Passing — Decentralized Sequential Planning"),
];

#[cfg(target_arch = "wasm32")]
pub const SOLVER_NAMES: &[(&str, &str)] = &[
    ("pibt", "PIBT — Priority Inheritance with Backtracking"),
    ("token_passing", "Token Passing — Decentralized Sequential Planning"),
];

/// Create a LifelongSolver by name with auto-computed defaults.
/// `grid_area` and `num_agents` are used for RHCR auto-config.
///
/// **For production WASM callers, prefer [`lifelong_solver_from_name_sized`]**
/// which threads concrete grid dimensions through and pre-sizes RHCR's PBS
/// scratch buffers — eliminating the first-tick allocation stall (~3 MB) that
/// users perceive as "slow simulation start".
///
/// This legacy form is kept so the experiment runner and ~40 test call sites
/// don't need touching; for tests the first-tick stall is invisible.
pub fn lifelong_solver_from_name(
    name: &str,
    #[cfg_attr(target_arch = "wasm32", allow(unused_variables))] grid_area: usize,
    #[cfg_attr(target_arch = "wasm32", allow(unused_variables))] num_agents: usize,
) -> Option<Box<dyn LifelongSolver>> {
    match name {
        "pibt" => Some(Box::new(PibtLifelongSolver::new())),
        #[cfg(not(target_arch = "wasm32"))]
        "rhcr_pbs" => {
            let cfg = RhcrConfig::auto(grid_area, num_agents);
            Some(Box::new(RhcrSolver::new(cfg)))
        }
        "token_passing" => Some(Box::new(TokenPassingSolver::new())),
        _ => None,
    }
}

/// Create a LifelongSolver by name with **pre-sized scratch buffers** for
/// solvers that benefit from knowing the grid dimensions at construction.
///
/// For RHCR specifically, this pre-allocates the `FlatConstraintIndex` /
/// `SeqGoalGrid` / `FlatCAT` slabs (~3 MB at 1000 cells × 20 horizon) so the
/// first `plan_window` call doesn't trigger an allocation spike on the WASM
/// main thread. Other solvers ignore the grid dimensions (they don't
/// pre-allocate from grid dims).
///
/// Production callers (`SetSolver` bridge command, `begin_loading` system)
/// should use this entry point. Tests and the headless experiment runner can
/// keep using the legacy [`lifelong_solver_from_name`].
pub fn lifelong_solver_from_name_sized(
    name: &str,
    #[cfg_attr(target_arch = "wasm32", allow(unused_variables))] grid_w: usize,
    #[cfg_attr(target_arch = "wasm32", allow(unused_variables))] grid_h: usize,
    #[cfg_attr(target_arch = "wasm32", allow(unused_variables))] num_agents: usize,
) -> Option<Box<dyn LifelongSolver>> {
    match name {
        "pibt" => Some(Box::new(PibtLifelongSolver::new())),
        #[cfg(not(target_arch = "wasm32"))]
        "rhcr_pbs" => {
            let cfg = RhcrConfig::auto(grid_w * grid_h, num_agents);
            Some(Box::new(RhcrSolver::with_grid(cfg, grid_w, grid_h)))
        }
        "token_passing" => Some(Box::new(TokenPassingSolver::new())),
        _ => None,
    }
}

// Re-export for convenience
pub use self::lifelong::ActiveSolver;

// ---------------------------------------------------------------------------
// Plugin
// ---------------------------------------------------------------------------

pub struct SolverPlugin;

impl Plugin for SolverPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(default_active_solver());
    }
}

// ---------------------------------------------------------------------------
// Factory integration tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod factory_tests {
    use super::*;

    #[test]
    fn factory_unknown_returns_none() {
        assert!(lifelong_solver_from_name("unknown", 100, 10).is_none());
    }

    #[test]
    fn factory_existing_solvers_still_work() {
        for &(name, _) in SOLVER_NAMES.iter() {
            assert!(
                lifelong_solver_from_name(name, 100, 10).is_some(),
                "factory should create '{name}'"
            );
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn solver_names_has_three_entries() {
        // Native: pibt, rhcr_pbs, token_passing
        assert_eq!(SOLVER_NAMES.len(), 3);
    }

    #[cfg(target_arch = "wasm32")]
    #[test]
    fn solver_names_has_two_entries_on_wasm() {
        // Web: RHCR excluded (needs rayon for interactive perf)
        assert_eq!(SOLVER_NAMES.len(), 2);
    }
}
