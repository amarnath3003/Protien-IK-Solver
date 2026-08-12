"""Quantify run-to-run noise in the DOF-scaling sweep.

Real TRAC-IK (tracikpy) is wall-clock budgeted -- ``TracIKSolver(..., timeout=0.005,
solve_type="Speed")`` -- so its result depends on how many restarts fit in 5 ms on
the day. KineticFold's native port is seeded and deterministic. That asymmetry
lands directly on paper Table 5's ratio column, so we measure it rather than
quote a single run.

Run (WSL Ubuntu-2204, from backend/):
    PYTHONPATH=. python3 scrap/dof_tracik_noise.py --repeats 5
"""
from __future__ import annotations

import argparse
import statistics as st

import numpy as np

import native_bench.run_native_usecase  # noqa: F401  (env + native registry swap)
import usecase_experiments as U
from app.core.kinematics import self_collision_min_distance

DOFS = [4, 6, 8, 12, 16]
SOLVERS = ["protein_fast", "trac_ik_style"]


def clean_pct(spec, solver: str) -> float:
    """exp_E's cell, proxy-scored: identical target/RNG streams."""
    cln = tot = 0
    for seed in (1, 2):
        g = np.random.default_rng(1000 + seed)
        tg = [U.generate_target(spec, g, "cluttered") for _ in range(60)]
        for i, (q0, T) in enumerate(tg):
            rng = np.random.default_rng(seed * 1_000_003 + i)
            r = U.run_solver(solver, spec, q0, T, rng)
            cln += int(U.solved(r) and self_collision_min_distance(spec, np.asarray(r.q_final)) >= 0.0)
            tot += 1
    return 100.0 * cln / tot


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=5)
    args = ap.parse_args()

    print(f"clean-solve %% across {args.repeats} repeats (n=120/cell)\n")
    print(f"{'DOF':>4} {'solver':<15} {'values':<34} {'min':>5} {'max':>5} {'spread':>7}")
    table = {}
    for dof in DOFS:
        spec = U.planar_ndof_spec(dof)
        for s in SOLVERS:
            vals = [clean_pct(spec, s) for _ in range(args.repeats)]
            table[(dof, s)] = vals
            print(f"{dof:>4} {s:<15} {str([round(v,1) for v in vals]):<34} "
                  f"{min(vals):>5.1f} {max(vals):>5.1f} {max(vals)-min(vals):>7.1f}")
        kf, tr = table[(dof, "protein_fast")], table[(dof, "trac_ik_style")]
        rats = [k / t for k, t in zip(kf, tr) if t > 0]
        if rats:
            print(f"{'':>4} {'-> ratio':<15} {str([round(r,2) for r in rats]):<34} "
                  f"{min(rats):>5.2f} {max(rats):>5.2f} {'median':>7} {st.median(rats):.2f}")
        else:
            print(f"{'':>4} {'-> ratio':<15} KineticFold only (TRAC-IK at 0.0 in all repeats)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
