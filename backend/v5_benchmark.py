"""
V5 (CCH-IK) paper benchmark — diagnostics, ablation, and sensitivity.

This is a *V5-only* companion to master_benchmark.py. Where master_benchmark
produces the cross-solver comparison table, this script produces the three
tables a CCH-IK paper needs and that master_benchmark does NOT cover:

  1. DIAGNOSTICS  — full V5 vs its critical baselines (fixed-λ homotopy + V4),
                    reporting the V5 diagnostic outputs that master_benchmark
                    drops: conflict_index, lambda_final, difficulty_score, plus
                    solution-quality columns (clearance, joint-limit violations)
                    and 95% Wilson confidence intervals on success.

  2. ABLATION     — all 8 combinations of Components A/B/C on a *shared* target
                    set, so the contribution of each component is isolated
                    apples-to-apples (same targets, same per-trial RNG).

  3. SENSITIVITY  — sweep of CONFLICT_THRESHOLD and LAMBDA_BETA, to show the
                    result is not a knife-edge of two hand-tuned constants.

Design notes
------------
- Targets are generated ONCE per (robot, scenario, seed) and reused across every
  variant/baseline, and every trial uses a fixed per-trial RNG seed independent
  of the variant. So no variant is ever compared on an easier draw.
- Components A/B/C and the hyperparameters are module-level globals in the V5
  solver, read at runtime. We snapshot them, patch them per variant, and restore
  them in a finally block. Nothing is left mutated on exit.
- difficulty_score / conflict_index accumulate every iteration regardless of
  which components are on, so they remain valid diagnostics even in ablation
  rows where Component A is disabled.
- Wall-clock carries OS noise (mean/p95/p99); success / collision / error /
  diagnostic columns are deterministic given the seed.

Usage
-----
    python v5_benchmark.py                                  # diagnostics + ablation, UR5, n=300
    python v5_benchmark.py --trials 60 --seeds 1            # quick preview
    python v5_benchmark.py --modes diagnostics ablation sensitivity
    python v5_benchmark.py --modes sensitivity --sens-scenario near_singular
    python v5_benchmark.py --robots ur5 franka_panda
    python v5_benchmark.py --out results/v5               # -> results/v5.csv + .md
"""

from __future__ import annotations

import argparse
import csv
import itertools
import math
import os
import sys

import numpy as np

from app.core.kinematics import get_robot_spec
from app.api.scenarios import generate_target
from app.solvers.registry import run_solver
import app.solvers.protein_homotopy.solver as v5

ALL_ROBOTS = ["planar3dof", "ur5", "franka_panda"]
ALL_SCENARIOS = ["open_space", "near_singular", "cluttered"]

# External baselines for the diagnostics table (the two comparisons that matter
# for the paper: the fixed-schedule ablation control, and the success-rate /
# speed reference V4). These run via the registry, not the V5 globals.
BASELINES = ["fixed_lambda_ik", "protein_fast"]

# The five V5 globals we patch. Snapshotted and restored around every run.
V5_GLOBALS = ["COMPONENT_A", "COMPONENT_B", "COMPONENT_C",
              "CONFLICT_THRESHOLD", "LAMBDA_BETA"]

# Sensitivity sweep grid (defaults; override via CLI if needed).
SENS_THRESHOLDS = [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
SENS_BETAS = [2.0, 3.0, 3.84, 5.0, 6.0, 8.0]


# ---------------------------------------------------------------------------
# Stats helpers
# ---------------------------------------------------------------------------

def wilson_halfwidth_pct(successes: int, n: int, z: float = 1.96) -> float:
    """95% Wilson score interval half-width for a binomial proportion, in
    percentage points. Honest error bar for success-rate claims at small n."""
    if n == 0:
        return float("nan")
    p = successes / n
    denom = 1.0 + z * z / n
    margin = z * math.sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n)) / denom
    return margin * 100.0


def _pctile(a: np.ndarray, q: float) -> float:
    return float(np.percentile(a, q)) if a.size else float("nan")


# ---------------------------------------------------------------------------
# Target sets — built once per (robot, scenario), shared across all variants
# ---------------------------------------------------------------------------

def build_targets(spec, scenario: str, seeds: list[int], n_trials: int):
    """Return {seed: [(q0, T_target), ...]} — identical draw reused by every
    variant so comparisons are apples-to-apples."""
    out = {}
    for seed in seeds:
        gen = np.random.default_rng(seed)
        out[seed] = [generate_target(spec, gen, scenario) for _ in range(n_trials)]
    return out


# ---------------------------------------------------------------------------
# Run one variant (a solve_fn) over the shared target set
# ---------------------------------------------------------------------------

def run_variant(solve_fn, spec, targets_by_seed: dict, warmup: int) -> list:
    """Run solve_fn over every shared target, with an untimed warm-up first.
    Per-trial RNG depends only on (seed, i), never on the variant — so timing
    and stochastic restarts are reproducible and identical across variants."""
    seeds = sorted(targets_by_seed)
    wseed = seeds[0]
    wtargets = targets_by_seed[wseed]
    for w in range(warmup):
        q0, T = wtargets[w % len(wtargets)]
        solve_fn(spec, q0, T, np.random.default_rng(10_000 + w))

    recs = []
    for seed in seeds:
        for i, (q0, T) in enumerate(targets_by_seed[seed]):
            rng = np.random.default_rng(seed * 1_000_003 + i)
            recs.append(solve_fn(spec, q0, T, rng))
    return recs


def aggregate(recs: list) -> dict:
    """Aggregate a list of SolveResult into the full metric + diagnostic row."""
    n = len(recs)
    succ = sum(int(r.success) for r in recs)
    tms = np.array([r.wall_time_ms for r in recs])
    clear = np.array([r.min_self_distance for r in recs])
    return {
        "n": n,
        "success_pct": 100.0 * succ / n,
        "ci95_pp": wilson_halfwidth_pct(succ, n),
        "mean_ms": float(tms.mean()),
        "p50_ms": _pctile(tms, 50),
        "p95_ms": _pctile(tms, 95),
        "p99_ms": _pctile(tms, 99),
        "mean_iters": float(np.mean([r.iterations for r in recs])),
        "collision_pct": 100.0 * float(np.mean(clear < 0)),
        "mean_clearance_m": float(clear.mean()),
        "mean_pos_err_mm": float(np.mean([r.pos_error * 1000.0 for r in recs])),
        "mean_orient_err_mrad": float(np.mean([r.orient_error * 1000.0 for r in recs])),
        "mean_jlv": float(np.mean([r.joint_limit_violations for r in recs])),
        "mean_restarts": float(np.mean([r.restarts for r in recs])),
        # --- V5 diagnostics (meaningful only for V5 variants) ---
        "mean_conflict_index": float(np.mean([r.conflict_index for r in recs])),
        "mean_lambda_final": float(np.mean([r.lambda_final for r in recs])),
        "lambda_lt08_pct": 100.0 * float(np.mean([r.lambda_final < 0.8 for r in recs])),
        "mean_difficulty": float(np.mean([r.difficulty_score for r in recs])),
    }


# ---------------------------------------------------------------------------
# V5 variant factory — patches the globals, runs, restores
# ---------------------------------------------------------------------------

def with_v5_config(a=None, b=None, c=None, threshold=None, beta=None):
    """Return a solve_fn that runs CCH-IK with the given config patched in.
    Only non-None fields are overridden; the rest keep their module defaults.
    Patching happens per-call and is restored immediately, so concurrent
    snapshots never collide and nothing leaks between variants."""
    overrides = {
        "COMPONENT_A": a, "COMPONENT_B": b, "COMPONENT_C": c,
        "CONFLICT_THRESHOLD": threshold, "LAMBDA_BETA": beta,
    }
    overrides = {k: val for k, val in overrides.items() if val is not None}

    def solve_fn(spec, q0, T, rng):
        snapshot = {k: getattr(v5, k) for k in overrides}
        try:
            for k, val in overrides.items():
                setattr(v5, k, val)
            return v5.solve_protein_homotopy(spec, q0, T, rng)
        finally:
            for k, val in snapshot.items():
                setattr(v5, k, val)

    return solve_fn


def baseline_fn(name: str):
    return lambda spec, q0, T, rng: run_solver(name, spec, q0, T, rng)


# ---------------------------------------------------------------------------
# Modes
# ---------------------------------------------------------------------------

def mode_diagnostics(spec, robot, scenario, targets, warmup) -> list:
    """Full V5 + the two paper baselines, with diagnostic columns."""
    rows = []
    variants = [("CCH-IK (full V5)", with_v5_config(), True)]
    variants += [(name, baseline_fn(name), False) for name in BASELINES]
    for label, fn, is_v5 in variants:
        agg = aggregate(run_variant(fn, spec, targets, warmup))
        rows.append({"mode": "diagnostics", "robot": robot, "scenario": scenario,
                     "label": label, "is_v5": is_v5, **agg})
    return rows


def mode_ablation(spec, robot, scenario, targets, warmup) -> list:
    """All 8 A/B/C combinations on the shared target set."""
    rows = []
    for a, b, c in itertools.product([True, False], repeat=3):
        label = f"A{int(a)} B{int(b)} C{int(c)}"
        fn = with_v5_config(a=a, b=b, c=c)
        agg = aggregate(run_variant(fn, spec, targets, warmup))
        rows.append({"mode": "ablation", "robot": robot, "scenario": scenario,
                     "label": label, "is_v5": True,
                     "compA": int(a), "compB": int(b), "compC": int(c), **agg})
    return rows


def mode_sensitivity(spec, robot, scenario, targets, warmup,
                     thresholds, betas) -> list:
    """Sweep CONFLICT_THRESHOLD × LAMBDA_BETA with all components on."""
    rows = []
    for thr, beta in itertools.product(thresholds, betas):
        label = f"thr={thr:.2f} beta={beta:.2f}"
        fn = with_v5_config(a=True, b=True, c=True, threshold=thr, beta=beta)
        agg = aggregate(run_variant(fn, spec, targets, warmup))
        rows.append({"mode": "sensitivity", "robot": robot, "scenario": scenario,
                     "label": label, "is_v5": True,
                     "c_threshold": thr, "lambda_beta": beta, **agg})
    return rows


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

CSV_COLS = [
    "mode", "robot", "scenario", "label", "is_v5",
    "compA", "compB", "compC", "c_threshold", "lambda_beta",
    "n", "success_pct", "ci95_pp", "mean_ms", "p50_ms", "p95_ms", "p99_ms",
    "mean_iters", "collision_pct", "mean_clearance_m",
    "mean_pos_err_mm", "mean_orient_err_mrad", "mean_jlv", "mean_restarts",
    "mean_conflict_index", "mean_lambda_final", "lambda_lt08_pct", "mean_difficulty",
]


def write_csv(rows: list, path: str) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLS, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)


def _group(rows, mode):
    by = {}
    for r in rows:
        if r["mode"] == mode:
            by.setdefault((r["robot"], r["scenario"]), []).append(r)
    return by


def write_markdown(rows: list, path: str, meta: dict) -> None:
    L = [
        "# V5 (CCH-IK) Benchmark", "",
        f"- Trials/seed: **{meta['trials']}**  |  Seeds: **{meta['seeds']}**  "
        f"(n={meta['trials'] * len(meta['seeds'])} per cell)",
        f"- Warm-up: {meta['warmup']} untimed solves per variant",
        f"- Robots: {', '.join(meta['robots'])}  |  Scenarios: {', '.join(meta['scenarios'])}",
        "- Success error bars are 95% Wilson half-widths (±pp). Timing is "
        "wall-clock; success/collision/error/diagnostic columns are deterministic.",
        "",
    ]

    # ---- DIAGNOSTICS ----
    diag = _group(rows, "diagnostics")
    if diag:
        L += ["## 1. Diagnostics — CCH-IK vs baselines", ""]
        for (robot, scenario), cell in diag.items():
            L += [f"### {robot} — {scenario}", "",
                  "| Variant | Succ% (±95%) | Mean ms | p95 | Collide% | Clear m | "
                  "JLV | Conflict C | λ_final | λ<0.8% | Difficulty |",
                  "|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|"]
            for r in sorted(cell, key=lambda x: (not x["is_v5"], x["mean_ms"])):
                diag_cols = (
                    f"{r['mean_conflict_index']:.3f} | {r['mean_lambda_final']:.3f} | "
                    f"{r['lambda_lt08_pct']:.1f} | {r['mean_difficulty']:.4f}"
                    if r["is_v5"] else "— | — | — | —"
                )
                L.append(
                    f"| {r['label']} | {r['success_pct']:.1f} ±{r['ci95_pp']:.1f} | "
                    f"{r['mean_ms']:.1f} | {r['p95_ms']:.1f} | {r['collision_pct']:.1f} | "
                    f"{r['mean_clearance_m']:.4f} | {r['mean_jlv']:.2f} | {diag_cols} |")
            L.append("")

    # ---- ABLATION ----
    abl = _group(rows, "ablation")
    if abl:
        L += ["## 2. Ablation — Components A (λ-control) / B (surgery) / C (seed)", "",
              "A1B1C1 = full V5. A0B0C0 = fixed linear-λ baseline. "
              "Same targets across all 8 rows.", ""]
        for (robot, scenario), cell in abl.items():
            L += [f"### {robot} — {scenario}", "",
                  "| A | B | C | Succ% (±95%) | Mean ms | Collide% | Clear m | "
                  "JLV | Difficulty |",
                  "|:-:|:-:|:-:|--:|--:|--:|--:|--:|--:|"]
            # order: full V5 first, then by descending success
            for r in sorted(cell, key=lambda x: (-(x["compA"] & x["compB"] & x["compC"]),
                                                  -x["success_pct"])):
                L.append(
                    f"| {r['compA']} | {r['compB']} | {r['compC']} | "
                    f"{r['success_pct']:.1f} ±{r['ci95_pp']:.1f} | {r['mean_ms']:.1f} | "
                    f"{r['collision_pct']:.1f} | {r['mean_clearance_m']:.4f} | "
                    f"{r['mean_jlv']:.2f} | {r['mean_difficulty']:.4f} |")
            L.append("")

    # ---- SENSITIVITY ----
    sens = _group(rows, "sensitivity")
    if sens:
        L += ["## 3. Sensitivity — CONFLICT_THRESHOLD × LAMBDA_BETA", "",
              "All components on. Defaults are threshold=0.60, beta=3.84.", ""]
        for (robot, scenario), cell in sens.items():
            L += [f"### {robot} — {scenario}", "",
                  "| threshold | beta | Succ% (±95%) | Mean ms | Difficulty |",
                  "|--:|--:|--:|--:|--:|"]
            for r in sorted(cell, key=lambda x: (x["c_threshold"], x["lambda_beta"])):
                L.append(
                    f"| {r['c_threshold']:.2f} | {r['lambda_beta']:.2f} | "
                    f"{r['success_pct']:.1f} ±{r['ci95_pp']:.1f} | "
                    f"{r['mean_ms']:.1f} | {r['mean_difficulty']:.4f} |")
            L.append("")

    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="V5 (CCH-IK) diagnostics / ablation / sensitivity benchmark.")
    ap.add_argument("--trials", type=int, default=100, help="trials per seed per cell")
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3], help="RNG seeds (noise-averaged)")
    ap.add_argument("--warmup", type=int, default=8, help="untimed warm-up solves per variant")
    ap.add_argument("--robots", nargs="+", default=["ur5"], choices=ALL_ROBOTS,
                    help="default ur5 (V5's primary benchmark arm)")
    ap.add_argument("--scenarios", nargs="+", default=ALL_SCENARIOS, choices=ALL_SCENARIOS)
    ap.add_argument("--modes", nargs="+", default=["diagnostics", "ablation"],
                    choices=["diagnostics", "ablation", "sensitivity"])
    ap.add_argument("--sens-scenario", default="near_singular", choices=ALL_SCENARIOS,
                    help="sensitivity sweep runs on this scenario only (it is heavy)")
    ap.add_argument("--sens-thresholds", type=float, nargs="+", default=SENS_THRESHOLDS)
    ap.add_argument("--sens-betas", type=float, nargs="+", default=SENS_BETAS)
    ap.add_argument("--out", default="v5_benchmark_results", help="output stem (.csv/.md appended)")
    args = ap.parse_args(argv)

    # Hard guard: never exit with the V5 globals mutated, even on crash.
    global_snapshot = {k: getattr(v5, k) for k in V5_GLOBALS}

    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    rows: list = []
    try:
        for robot in args.robots:
            spec = get_robot_spec(robot)

            for scenario in args.scenarios:
                # one shared target set per (robot, scenario), reused by every mode
                targets = build_targets(spec, scenario, args.seeds, args.trials)

                if "diagnostics" in args.modes:
                    rows += mode_diagnostics(spec, robot, scenario, targets, args.warmup)
                    print(f"[diagnostics  {robot:<12} {scenario:<13}] done", flush=True)

                if "ablation" in args.modes:
                    rows += mode_ablation(spec, robot, scenario, targets, args.warmup)
                    print(f"[ablation     {robot:<12} {scenario:<13}] 8 combos done", flush=True)

            if "sensitivity" in args.modes:
                scenario = args.sens_scenario
                targets = build_targets(spec, scenario, args.seeds, args.trials)
                rows += mode_sensitivity(spec, robot, scenario, targets, args.warmup,
                                         args.sens_thresholds, args.sens_betas)
                n_cfg = len(args.sens_thresholds) * len(args.sens_betas)
                print(f"[sensitivity  {robot:<12} {scenario:<13}] {n_cfg} configs done", flush=True)
    finally:
        for k, val in global_snapshot.items():
            setattr(v5, k, val)

    csv_path, md_path = args.out + ".csv", args.out + ".md"
    meta = dict(trials=args.trials, seeds=args.seeds, warmup=args.warmup,
                robots=args.robots, scenarios=args.scenarios)
    write_csv(rows, csv_path)
    write_markdown(rows, md_path, meta)
    print(f"\nWrote {csv_path} and {md_path}  ({len(rows)} rows).", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
