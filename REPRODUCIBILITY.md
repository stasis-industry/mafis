# Reproducing MAFIS Results

This document describes how to reproduce the validation results from the paper
"MAFIS: A Fault Resilience Observatory for Lifelong Multi-Agent Path Finding."

## Requirements

- **Rust** 1.75+ (tested on 1.93.0)
- **Python** 3.8+ (tested on 3.14.3), stdlib only, no pip packages needed
- ~4 GB RAM for full experiment matrix
- ~2 GB disk for compiled binaries + results

## Quick Verification (~3 min)

```bash
cargo check                  # Type check (~5s)
cargo test                   # 473 tests (~3 min)
```

This runs all unit tests (417), integration tests (26), metamorphic tests (6),
calibration tests (6), benchmark tests (3), desktop tests (14), and a paper
smoke test (1). All should pass with 0 failures.

## Reproduce Property Verification (~4 min)

The calibration tests verify published algorithmic properties:

```bash
cargo test --release --test calibration -- --nocapture
```

This runs 6 property tests:
1. **Throughput saturation** -- per-agent throughput decreases at high density (Okumura 2022)
2. **PIBT completeness** -- no deadlock on any topology
3. **Liveness at high density** -- PIBT produces tasks even at n=70 on dense grids
4. **All solvers functional** -- all 8 solvers produce positive throughput
5. **Topology sensitivity** -- different maps produce different throughput profiles
6. **Differential measurement** -- deterministic baselines + faults have measurable effect

## Reproduce Full Experiment Matrix (~1-2 hours)

```bash
cargo test --release full_paper_matrix -- --ignored --nocapture
```

This runs 300+ paired experiments across multiple sub-matrices.
The experiment matrix currently covers 5 of the 8 solvers (the original set).

Results are written to `results/`:
- `*_runs.csv` -- per-run metrics (one row per seed)
- `*_summary.csv` -- aggregated statistics
- `*.json` -- structured output

Individual sub-matrices can be run separately:
```bash
cargo test --release solver_resilience -- --ignored --nocapture
cargo test --release scale_sensitivity -- --ignored --nocapture
cargo test --release scheduler_effect -- --ignored --nocapture
```

## Additional Analysis (not in Paper 1)

The Braess analysis script computes fault-induced congestion relief statistics.
These results are reserved for a follow-up study (Paper 2).

```bash
# Generate Braess experiment data first:
cargo test --release braess_resilience -- --ignored --nocapture
# Then run analysis:
python3 analysis/braess_analysis.py
```

The analysis script uses only Python standard library (no numpy/scipy/matplotlib).

## One-Command Reproduction

```bash
sh scripts/reproduce.sh
```

Runs the full pipeline: tests, calibration, experiments, analysis.

## Expected Runtimes

| Step | Time | Hardware |
|------|------|----------|
| `cargo check` | ~5s | Any |
| `cargo test` | ~3 min | Any |
| Calibration tests | ~4 min | Any |
| Full paper matrix | ~1-2 hours | 8+ cores recommended (Rayon parallelism) |
| Python analysis | ~1s | Any |

## Determinism

All simulations are deterministic: same seed + same config = identical output.
The paired experimental design (baseline vs. faulted) uses the same seed for both
runs, so metric differences are causally attributable to the fault.

## Solver Fidelity

See `docs/solver_fidelity.md` for a per-solver audit of paper accuracy,
documented deviations, and test coverage.
