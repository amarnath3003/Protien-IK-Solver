"""
Quick high-DOF ("snake" arm) benchmark — V1 / V4 / V6 + classical baselines.

The core kinematics + every solver are DOF-agnostic (they consume a RobotSpec and
nothing else), and the core/proxy benchmark needs only numpy — no URDF, no
PyBullet/MuJoCo. A "snake" arm is therefore just a RobotSpec with many revolute
joints in series. This runner builds a few of them (8 / 12 / 16 / 24 DOF),
generates reachable targets (open_space = FK of a random config), and runs the
requested solver set once per (arm, solver) cell.

This is a *quick* smoke, not the paper artifact: single seed, modest trial count,
open_space only by default. Point is to see how V1/V4/V6 and the baselines behave
as redundancy grows well past the 7-DOF Franka ceiling.

Run from backend/:
    PYTHONPATH=. python bench/snake_quick_bench.py
    PYTHONPATH=. python bench/snake_quick_bench.py --trials 60 --dofs 8 16 32
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from app.core.kinematics import RobotSpec
from app.api.scenarios import generate_target
from app.solvers.registry import SOLVER_DISPLAY_NAMES, run_solver


def snake_spec(n: int, link_len: float = 0.12) -> RobotSpec:
    """An n-DOF serial revolute "snake": equal-length links with alternating
    +/- 90 deg twists so the chain genuinely explores 3D (not a planar arm),
    giving a meaningful 6-DOF pose task with n-6 redundant DOF.

    Standard DH: a = link_len each link, d = 0, alpha alternates ±pi/2 so
    consecutive joint axes are orthogonal (a real 3D wrist-less snake).
    Generous ±170 deg joint limits; thin uniform link radius for the
    self-collision proxy.
    """
    a = np.full(n, link_len)
    d = np.zeros(n)
    alpha = np.where(np.arange(n) % 2 == 0, np.pi / 2, -np.pi / 2)
    theta_offset = np.zeros(n)
    lim = np.deg2rad(170.0)
    joint_limits = np.stack([np.full(n, -lim), np.full(n, lim)], axis=1)
    link_radius = np.full(n, 0.02)
    return RobotSpec(
        name=f"snake{n}",
        a=a, d=d, alpha=alpha, theta_offset=theta_offset,
        joint_limits=joint_limits, link_radius=link_radius,
        dh_convention="standard",
    )


# V1 / V4 / V6 headliners plus the classical baselines. multi_start / homotopy
# omitted by default (slow, not the point of a quick DOF-scaling smoke).
DEFAULT_SOLVERS = [
    "protein_ik",       # V1
    "protein_fast",     # V4
    "protein_raw",      # V6
    "jacobian_dls",
    "ccd",
    "fabrik",
    "trac_ik_style",
]


def bench_cell(spec: RobotSpec, solver: str, n_trials: int, seed: int,
               scenario: str, warmup: int) -> dict:
    gen = np.random.default_rng(seed)
    targets = [generate_target(spec, gen, scenario) for _ in range(n_trials)]

    for w in range(warmup):
        q0, T = targets[w % n_trials]
        run_solver(solver, spec, q0, T, np.random.default_rng(90_000 + w))

    tms, iters, succ, clash = [], [], 0, 0
    pos_mm, orient_mr, clearance = [], [], []
    for i, (q0, T) in enumerate(targets):
        rng = np.random.default_rng(seed * 1_000_003 + i)
        r = run_solver(solver, spec, q0, T, rng)
        tms.append(r.wall_time_ms)
        iters.append(r.iterations)
        succ += int(r.success)
        clash += int(r.min_self_distance < 0)
        pos_mm.append(r.pos_error * 1000.0)
        orient_mr.append(r.orient_error * 1000.0)
        clearance.append(r.min_self_distance)

    tms = np.array(tms)
    return {
        "solver": solver,
        "display_name": SOLVER_DISPLAY_NAMES.get(solver, solver),
        "n": n_trials,
        "success_pct": 100.0 * succ / n_trials,
        "mean_ms": float(tms.mean()),
        "p95_ms": float(np.percentile(tms, 95)),
        "mean_iters": float(np.mean(iters)),
        "collision_pct": 100.0 * clash / n_trials,
        "mean_pos_mm": float(np.mean(pos_mm)),
        "mean_orient_mrad": float(np.mean(orient_mr)),
        "mean_clearance_m": float(np.mean(clearance)),
    }


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

    ap = argparse.ArgumentParser(description="Quick high-DOF snake-arm IK benchmark.")
    ap.add_argument("--dofs", type=int, nargs="+", default=[8, 12, 16, 24],
                    help="snake DOF counts to test")
    ap.add_argument("--trials", type=int, default=40, help="targets per (arm, solver)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--warmup", type=int, default=4)
    ap.add_argument("--scenario", default="open_space",
                    choices=["open_space", "near_singular", "cluttered"])
    ap.add_argument("--solvers", nargs="+", default=DEFAULT_SOLVERS)
    args = ap.parse_args(argv)

    print(f"=== Snake-arm quick benchmark ===", flush=True)
    print(f"  dofs={args.dofs}  trials={args.trials}  seed={args.seed}  "
          f"scenario={args.scenario}  solvers={args.solvers}", flush=True)

    all_rows: dict[int, list[dict]] = {}
    t0 = time.perf_counter()
    for n in args.dofs:
        spec = snake_spec(n)
        print(f"\n-- snake{n} ({n}-DOF, reach ~{n * 0.12:.2f}m) "
              f"{'-' * 30}", flush=True)
        print(f"{'Solver':<28} {'Succ%':>6} {'Mean ms':>8} {'p95':>7} "
              f"{'Iters':>6} {'Col%':>6} {'Pos mm':>8} {'Orient mr':>10}", flush=True)
        rows = []
        for solver in args.solvers:
            try:
                r = bench_cell(spec, solver, args.trials, args.seed,
                               args.scenario, args.warmup)
            except Exception as e:
                print(f"{SOLVER_DISPLAY_NAMES.get(solver, solver):<28}  ERROR: {e!r}",
                      flush=True)
                continue
            rows.append(r)
            print(f"{r['display_name']:<28} {r['success_pct']:>6.1f} "
                  f"{r['mean_ms']:>8.1f} {r['p95_ms']:>7.1f} {r['mean_iters']:>6.0f} "
                  f"{r['collision_pct']:>6.1f} {r['mean_pos_mm']:>8.2f} "
                  f"{r['mean_orient_mrad']:>10.2f}", flush=True)
        all_rows[n] = rows

    print(f"\nDone in {time.perf_counter() - t0:.1f}s.", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
