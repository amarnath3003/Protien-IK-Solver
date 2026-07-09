"""
Phase 2 (sim_migration_plan.md §5): the sim-oracle benchmark runner.

Mirrors ``master_benchmark.py`` -- same robots, same scenario target
distributions (via ``scenarios.generate_target``, so cells are directly
comparable) -- but adds the honest second opinion the whole migration exists for:

  * runs each solver on the fast ``RobotSpec`` core exactly as production does, then
  * **re-scores its ``q_final`` in PyBullet**: the sim's real EE pose (``sim_succ``)
    and the sim's real mesh self-collision (``sim_collide``, from
    ``getClosestPoints``), and
  * adds **PyBullet's native IK** as a free, widely-trusted baseline column.

The headline columns to read side by side:
  * ``our_succ`` vs ``sim_succ``  -- does a solve we call successful hold up under an
    independent simulator's FK? (If Phase-1 parity is clean, these should agree; a
    gap means a model-parity leak.)
  * ``our_col`` vs ``sim_col``    -- is our capsule self-collision proxy telling the
    truth vs real mesh collision? (This is the Phase-3 headline previewed here.)

Runs headless (PyBullet DIRECT). Must be executed in ``.venv-sim`` (the only env
with PyBullet); the solvers need only numpy, which that env has. Example:

    PYTHONPATH=. .venv-sim/Scripts/python -m bench.sim_benchmark \
        --robots ur5 --trials 50 --seeds 1 2 3 --out sim_oracle_ur5

    PYTHONPATH=. .venv-sim/Scripts/python -m bench.sim_benchmark --skip-slow
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
    SOLVER_DISPLAY_NAMES, get_solvers_for_robot, run_solver,
)
from app.sim.pybullet_backend import PyBulletBackend

# Only the two arms with a canonical URDF. planar3dof has no standard URDF and is
# validated analytically (plan §7, risk #5), so it has no sim oracle.
ALL_ROBOTS = ["ur5", "franka_panda"]
ALL_SCENARIOS = ["open_space", "near_singular", "cluttered"]
SLOW_SOLVERS = {"protein_homotopy", "fixed_lambda_ik"}

NATIVE = "pybullet_native_ik"  # synthetic "solver" id for the baseline column


def _pct(x: float) -> float:
    return 100.0 * float(x)


def bench_cell(bk: PyBulletBackend, robot: str, scenario: str, solver: str, spec,
               n_trials: int, seeds: list[int]) -> dict:
    """Run one (robot, scenario, solver) cell across seeds, re-scoring every
    ``q_final`` in the sim. ``solver == NATIVE`` runs PyBullet's own IK instead of
    one of ours (same targets), so the baseline sees the identical draw."""
    n = 0
    our_succ = our_col = sim_succ = sim_col = agree_succ = 0
    tms, our_pos_mm, sim_pos_mm, our_clear, sim_clear = [], [], [], [], []

    for seed in seeds:
        gen = np.random.default_rng(seed)
        targets = [generate_target(spec, gen, scenario) for _ in range(n_trials)]
        for i, (q0, T_dh) in enumerate(targets):
            n += 1
            if solver == NATIVE:
                import time
                t0 = time.perf_counter()
                q_final = bk.native_ik(bk.dh_to_sim(T_dh), q0)
                wall_ms = (time.perf_counter() - t0) * 1000.0
                # native IK has no "our-FK" notion; leave our_* neutral.
                our_ok = False
                our_min_d = float("nan")
                our_pos = float("nan")
            else:
                rng = np.random.default_rng(seed * 1_000_003 + i)
                r = run_solver(solver, spec, q0, T_dh, rng)
                q_final = np.asarray(r.q_final, dtype=float)
                wall_ms = r.wall_time_ms
                our_ok = bool(r.success)
                our_min_d = float(r.min_self_distance)
                our_pos = r.pos_error * 1000.0

            sc = bk.score(q_final, T_dh)

            tms.append(wall_ms)
            our_succ += int(our_ok)
            sim_succ += int(sc.sim_success)
            agree_succ += int(our_ok == sc.sim_success)
            if not np.isnan(our_min_d):
                our_col += int(our_min_d < 0)
                our_clear.append(our_min_d)
                our_pos_mm.append(our_pos)
            sim_col += int(sc.sim_in_collision)
            sim_clear.append(sc.sim_min_self_distance)
            sim_pos_mm.append(sc.sim_pos_error * 1000.0)

    tms = np.array(tms)
    have_our = len(our_clear) > 0
    return {
        "robot": robot, "scenario": scenario, "solver": solver,
        "display_name": SOLVER_DISPLAY_NAMES.get(solver, "PyBullet native IK"),
        "n": n,
        "our_success_pct": _pct(our_succ / n) if solver != NATIVE else float("nan"),
        "sim_success_pct": _pct(sim_succ / n),
        "success_agree_pct": _pct(agree_succ / n) if solver != NATIVE else float("nan"),
        "our_collision_pct": _pct(our_col / n) if have_our else float("nan"),
        "sim_collision_pct": _pct(sim_col / n),
        "our_mean_clearance_m": float(np.mean(our_clear)) if have_our else float("nan"),
        "sim_mean_clearance_m": float(np.mean(sim_clear)),
        "our_mean_pos_mm": float(np.mean(our_pos_mm)) if have_our else float("nan"),
        "sim_mean_pos_mm": float(np.mean(sim_pos_mm)),
        "mean_ms": float(tms.mean()),
        "p95_ms": float(np.percentile(tms, 95)),
    }


def _fmt(x, nd=1):
    return "  -  " if (isinstance(x, float) and np.isnan(x)) else f"{x:.{nd}f}"


def write_csv(rows: list[dict], path: str) -> None:
    cols = ["robot", "scenario", "solver", "display_name", "n",
            "our_success_pct", "sim_success_pct", "success_agree_pct",
            "our_collision_pct", "sim_collision_pct",
            "our_mean_clearance_m", "sim_mean_clearance_m",
            "our_mean_pos_mm", "sim_mean_pos_mm", "mean_ms", "p95_ms"]
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
        "# Sim-Oracle Benchmark (PyBullet)", "",
        f"- Trials/seed: **{meta['trials']}**  |  Seeds: **{meta['seeds']}**  "
        f"(n={meta['trials'] * len(meta['seeds'])} per cell)",
        f"- Robots: {', '.join(meta['robots'])}  |  Scenarios: {', '.join(meta['scenarios'])}",
        "",
        "Each solver runs on our DH `RobotSpec` core; `q_final` is then re-scored in "
        "PyBullet (real FK + real mesh self-collision). `PyBullet native IK` is the "
        "sim's own solver on the identical targets.",
        "",
        "**How to read it:** `our_succ` vs `sim_succ` tests whether a solve we call "
        "good survives an independent simulator's FK (Phase-1 parity, end to end). "
        "`our_col` vs `sim_col` tests whether our capsule collision proxy matches "
        "real mesh collision (the Phase-3 headline, previewed).",
        "",
    ]
    for robot in meta["robots"]:
        for scenario in meta["scenarios"]:
            cell = by_cell.get((robot, scenario))
            if not cell:
                continue
            lines += [f"## {robot} — {scenario}", "",
                      "| Solver | our_succ% | sim_succ% | agree% | our_col% | "
                      "sim_col% | our_clear m | sim_clear m | our_pos mm | "
                      "sim_pos mm | Mean ms | p95 ms |",
                      "|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|"]
            # solvers first (by sim success desc), native baseline last
            body = [r for r in cell if r["solver"] != NATIVE]
            body.sort(key=lambda x: -x["sim_success_pct"])
            body += [r for r in cell if r["solver"] == NATIVE]
            for r in body:
                lines.append(
                    f"| {r['display_name']} | {_fmt(r['our_success_pct'])} | "
                    f"{_fmt(r['sim_success_pct'])} | {_fmt(r['success_agree_pct'])} | "
                    f"{_fmt(r['our_collision_pct'])} | {_fmt(r['sim_collision_pct'])} | "
                    f"{_fmt(r['our_mean_clearance_m'], 4)} | {_fmt(r['sim_mean_clearance_m'], 4)} | "
                    f"{_fmt(r['our_mean_pos_mm'], 3)} | {_fmt(r['sim_mean_pos_mm'], 3)} | "
                    f"{_fmt(r['mean_ms'])} | {_fmt(r['p95_ms'])} |")
            lines.append("")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def main(argv=None) -> int:
    # Force UTF-8 stdout: solver display names contain non-latin-1 glyphs (e.g. the
    # λ in "Fixed-λ Homotopy") that crash cp1252 console/redirect encoding on
    # Windows, which would abort the sweep before results are written.
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description="PyBullet sim-oracle benchmark for the ProteinIK suite.")
    ap.add_argument("--trials", type=int, default=50, help="trials per seed per cell")
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--robots", nargs="+", default=ALL_ROBOTS, choices=ALL_ROBOTS)
    ap.add_argument("--scenarios", nargs="+", default=ALL_SCENARIOS, choices=ALL_SCENARIOS)
    ap.add_argument("--solvers", nargs="+", default=None, help="subset of solver ids")
    ap.add_argument("--skip-slow", action="store_true", help=f"drop {sorted(SLOW_SOLVERS)}")
    ap.add_argument("--no-native", action="store_true", help="skip the PyBullet native-IK baseline")
    ap.add_argument("--out", default="sim_oracle_results", help="output stem (.csv/.md)")
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
        if not args.no_native:
            solvers = solvers + [NATIVE]

        with PyBulletBackend(robot) as bk:
            print(f"[{robot}] oracle ready: ee={bk.ee_link} offset={bk.offset_side} "
                  f"residual={bk.offset_residual:.2e}", flush=True)
            for scenario in args.scenarios:
                for solver in solvers:
                    row = bench_cell(bk, robot, scenario, solver, spec,
                                     args.trials, args.seeds)
                    rows.append(row)
                    print(f"[{robot:<12} {scenario:<13} {row['display_name']:<28}] "
                          f"our_succ {_fmt(row['our_success_pct']):>5}  "
                          f"sim_succ {row['sim_success_pct']:5.1f}  "
                          f"our_col {_fmt(row['our_collision_pct']):>5}  "
                          f"sim_col {row['sim_collision_pct']:5.1f}  "
                          f"mean {row['mean_ms']:7.1f}ms", flush=True)

    csv_path, md_path = args.out + ".csv", args.out + ".md"
    meta = dict(trials=args.trials, seeds=args.seeds,
                robots=args.robots, scenarios=args.scenarios)
    write_csv(rows, csv_path)
    write_markdown(rows, md_path, meta)
    print(f"\nWrote {csv_path} and {md_path}  ({len(rows)} cells).", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
