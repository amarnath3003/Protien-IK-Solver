"""
Master benchmark — paper-grade, end-to-end.

Runs every registered solver across all robots, all scenarios, and every metric,
multi-seed and noise-averaged, then emits both a machine-readable CSV and
human-readable markdown tables.

This is the single reproducible artifact behind the results tables: same robots,
same scenario distributions, same target set per (scenario, seed) shared across
all solvers (so no solver is compared on an easier draw), same warm-up policy.

Usage
-----
    python master_benchmark.py                         # full run, defaults
    python master_benchmark.py --trials 60 --seeds 1   # quick preview
    python master_benchmark.py --skip-slow             # drop the ~1s homotopy pair
    python master_benchmark.py --robots ur5 --scenarios cluttered
    python master_benchmark.py --solvers protein_fast trac_ik_style
    python master_benchmark.py --out results/master    # -> results/master.csv + .md

Notes
-----
- Solvers are pulled from the live registry, so whatever `protein_fast` currently
  IS (e.g. the barrierless-first "ProteinIK Fast") is what gets benchmarked --
  nothing here hardcodes a version.
- Timing is wall-clock per solve (each solver's own internal stopwatch), with a
  short untimed warm-up per (robot, scenario, solver) to remove first-call /
  allocation transients. Wall-clock carries OS noise; success / collision / error
  metrics are deterministic given the seed.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

from app.core.kinematics import get_robot_spec
from app.api.scenarios import generate_target
from app.solvers.registry import (
    SOLVER_REGISTRY, SOLVER_DISPLAY_NAMES, get_solvers_for_robot, run_solver,
)

ALL_ROBOTS = ["planar3dof", "ur5", "franka_panda"]
ALL_SCENARIOS = ["open_space", "near_singular", "cluttered"]
# The two homotopy solvers run ~1s/solve; --skip-slow drops them for fast runs.
SLOW_SOLVERS = {"protein_homotopy", "fixed_lambda_ik"}

# Metric columns emitted per (robot, scenario, solver) cell.
METRIC_FIELDS = [
    "success_pct", "mean_ms", "p50_ms", "p95_ms", "p99_ms",
    "mean_iters", "collision_pct", "mean_clearance_m",
    "mean_pos_err_mm", "mean_orient_err_mrad",
    "mean_joint_limit_violations", "mean_restarts",
]


def _pct(x: float) -> float:
    return float(x) * 100.0


def bench_cell(robot: str, scenario: str, solver: str, spec,
               n_trials: int, seeds: list[int], warmup: int) -> dict:
    """Run one (robot, scenario, solver) cell across all seeds; return aggregated
    metrics. Targets are regenerated per seed but shared across solvers by the
    caller's identical seeding, so the comparison is apples-to-apples."""
    tms, iters, succ, clash = [], [], 0, 0
    clearance, pos_mm, orient_mr, jlv, restarts = [], [], [], [], []
    total = 0

    for seed in seeds:
        gen = np.random.default_rng(seed)
        targets = [generate_target(spec, gen, scenario) for _ in range(n_trials)]

        # untimed warm-up (allocation / first-call transients)
        for w in range(warmup):
            q0, T = targets[w % n_trials]
            run_solver(solver, spec, q0, T, np.random.default_rng(10_000 + w))

        for i, (q0, T) in enumerate(targets):
            rng = np.random.default_rng(seed * 1_000_003 + i)
            r = run_solver(solver, spec, q0, T, rng)
            total += 1
            tms.append(r.wall_time_ms)
            iters.append(r.iterations)
            succ += int(r.success)
            clash += int(r.min_self_distance < 0)
            clearance.append(r.min_self_distance)
            pos_mm.append(r.pos_error * 1000.0)
            orient_mr.append(r.orient_error * 1000.0)
            jlv.append(r.joint_limit_violations)
            restarts.append(r.restarts)

    tms = np.array(tms)
    return {
        "robot": robot, "scenario": scenario, "solver": solver,
        "display_name": SOLVER_DISPLAY_NAMES.get(solver, solver),
        "n": total,
        "success_pct": _pct(succ / total),
        "mean_ms": float(tms.mean()),
        "p50_ms": float(np.percentile(tms, 50)),
        "p95_ms": float(np.percentile(tms, 95)),
        "p99_ms": float(np.percentile(tms, 99)),
        "mean_iters": float(np.mean(iters)),
        "collision_pct": _pct(clash / total),
        "mean_clearance_m": float(np.mean(clearance)),
        "mean_pos_err_mm": float(np.mean(pos_mm)),
        "mean_orient_err_mrad": float(np.mean(orient_mr)),
        "mean_joint_limit_violations": float(np.mean(jlv)),
        "mean_restarts": float(np.mean(restarts)),
    }


def write_csv(rows: list[dict], path: str) -> None:
    cols = ["robot", "scenario", "solver", "display_name", "n"] + METRIC_FIELDS
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)


def write_markdown(rows: list[dict], path: str, meta: dict) -> None:
    by_cell: dict[tuple, list[dict]] = {}
    for r in rows:
        by_cell.setdefault((r["robot"], r["scenario"]), []).append(r)

    lines = [
        "# Master Benchmark", "",
        f"- Trials/seed: **{meta['trials']}**  |  Seeds: **{meta['seeds']}**  "
        f"(n={meta['trials'] * len(meta['seeds'])} per cell)",
        f"- Warm-up: {meta['warmup']} untimed solves per cell",
        f"- Robots: {', '.join(meta['robots'])}",
        f"- Scenarios: {', '.join(meta['scenarios'])}",
        "",
        "Timing is wall-clock (OS noise applies to mean/p95/p99); success, "
        "collision, and error columns are deterministic given the seed.", "",
    ]
    for robot in meta["robots"]:
        for scenario in meta["scenarios"]:
            cell = by_cell.get((robot, scenario))
            if not cell:
                continue
            lines += [f"## {robot} — {scenario}", "",
                      "| Solver | Succ% | Mean ms | p50 | p95 | p99 | Iters | "
                      "Collide% | Clear m | Pos mm | Orient mrad | JLV | Restarts |",
                      "|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|"]
            for r in sorted(cell, key=lambda x: x["mean_ms"]):
                lines.append(
                    f"| {r['display_name']} | {r['success_pct']:.1f} | "
                    f"{r['mean_ms']:.1f} | {r['p50_ms']:.1f} | {r['p95_ms']:.1f} | "
                    f"{r['p99_ms']:.1f} | {r['mean_iters']:.0f} | "
                    f"{r['collision_pct']:.1f} | {r['mean_clearance_m']:.4f} | "
                    f"{r['mean_pos_err_mm']:.3f} | {r['mean_orient_err_mrad']:.3f} | "
                    f"{r['mean_joint_limit_violations']:.2f} | {r['mean_restarts']:.2f} |")
            lines.append("")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def main(argv=None) -> int:
    # Solver display names contain non-latin-1 glyphs (e.g. the λ in "Fixed-λ
    # Homotopy"). On Windows the default console/redirect encoding is cp1252, which
    # can't encode them, so a progress print() would crash mid-run and lose the
    # entire sweep (results are only written at the end). Force UTF-8 stdout.
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description="Master benchmark for the ProteinIK solver suite.")
    ap.add_argument("--trials", type=int, default=100, help="trials per seed per cell")
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3], help="RNG seeds (noise-averaged)")
    ap.add_argument("--warmup", type=int, default=8, help="untimed warm-up solves per cell")
    ap.add_argument("--robots", nargs="+", default=ALL_ROBOTS, choices=ALL_ROBOTS)
    ap.add_argument("--scenarios", nargs="+", default=ALL_SCENARIOS, choices=ALL_SCENARIOS)
    ap.add_argument("--solvers", nargs="+", default=None, help="subset of solver ids (default: all valid)")
    ap.add_argument("--skip-slow", action="store_true", help=f"drop slow solvers: {sorted(SLOW_SOLVERS)}")
    ap.add_argument("--out", default="master_benchmark_results", help="output path stem (.csv/.md appended)")
    args = ap.parse_args(argv)

    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    rows: list[dict] = []
    for robot in args.robots:
        spec = get_robot_spec(robot)
        valid = get_solvers_for_robot(robot)
        solvers = [s for s in valid if (args.solvers is None or s in args.solvers)]
        if args.skip_slow:
            solvers = [s for s in solvers if s not in SLOW_SOLVERS]
        for scenario in args.scenarios:
            for solver in solvers:
                row = bench_cell(robot, scenario, solver, spec,
                                 args.trials, args.seeds, args.warmup)
                rows.append(row)
                print(f"[{robot:<12} {scenario:<13} {row['display_name']:<28}] "
                      f"succ {row['success_pct']:5.1f}%  mean {row['mean_ms']:7.1f}ms  "
                      f"p95 {row['p95_ms']:7.1f}  collide {row['collision_pct']:5.1f}%",
                      flush=True)

    csv_path, md_path = args.out + ".csv", args.out + ".md"
    meta = dict(trials=args.trials, seeds=args.seeds, warmup=args.warmup,
                robots=args.robots, scenarios=args.scenarios)
    write_csv(rows, csv_path)
    write_markdown(rows, md_path, meta)
    print(f"\nWrote {csv_path} and {md_path}  ({len(rows)} cells).", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
