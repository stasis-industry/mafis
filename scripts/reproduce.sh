#!/bin/sh
# reproduce.sh — Full reproduction pipeline for MAFIS paper results.
# Usage: sh scripts/reproduce.sh
#
# Runs: type check → tests → calibration → experiments → analysis
# Expected total time: ~1-2 hours on 8+ cores.

set -e

echo "═══════════════════════════════════════════════════════════"
echo " MAFIS Reproduction Pipeline"
echo "═══════════════════════════════════════════════════════════"
echo ""

# Step 1: Type check
echo "── Step 1/5: Type check ──"
cargo check
echo "✓ Type check passed"
echo ""

# Step 2: Full test suite
echo "── Step 2/5: Test suite ──"
cargo test --release 2>&1 | tail -20
echo "✓ Tests passed"
echo ""

# Step 3: Calibration tests
echo "── Step 3/5: Calibration (property verification) ──"
cargo test --release --test calibration -- --nocapture
echo "✓ Calibration passed"
echo ""

# Step 4: Paper experiments
echo "── Step 4/5: Paper experiments (this takes ~1-2 hours) ──"
cargo test --release full_paper_matrix -- --ignored --nocapture
echo "✓ Experiments complete"
echo ""

# Step 5: Analysis
echo "── Step 5/5: Statistical analysis ──"
if [ -f results/braess_resilience_runs.csv ]; then
    python3 analysis/braess_analysis.py
    echo "✓ Analysis complete"
else
    echo "⚠ Braess experiment data not found — run braess_resilience separately:"
    echo "  cargo test --release braess_resilience -- --ignored --nocapture"
fi
echo ""

echo "═══════════════════════════════════════════════════════════"
echo " Done. Results in results/"
echo "═══════════════════════════════════════════════════════════"
ls -la results/*.csv results/*.json 2>/dev/null | head -20
