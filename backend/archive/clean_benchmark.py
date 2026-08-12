"""
Phase 3 deliverable: does `solve_clean` (K-candidate + PyBullet-certified selection)
actually bring the real self-collision rate down, and at what cost?

Reports, per (robot, scenario, solver): single-shot vs clean-solve real-collision %,
success %, mean clearance, and wall-cost (K× solves + K collision queries). Same
scenario target distributions as the other benchmarks, so numbers are comparable.

    PYTHONPATH=. .venv-sim/Scripts/python -m bench.clean_benchmark --robots ur5 \
        --solvers protein_fast --K 16 --trials 60 --out clean_ur5
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from app.core.kinematics import get_robot_spec
from app.api.scenarios import generate_target
from app.solvers.registry import SOLVER_DISPLAY_NAMES
from app.sim.pybullet_backend import PyBulletBackend
from app.sim.clean_solve import solve_clean

ALL_SCENARIOS = ["open_space", "near_singular", "cluttered"]


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", nargs="+", default=["ur5"])
    ap.add_argument("--scenarios", nargs="+", default=ALL_SCENARIOS)
    ap.add_argument("--solvers", nargs="+", default=["protein_fast"])
    ap.add_argument("--K", type=int, default=16)
    ap.add_argument("--trials", type=int, default=60)
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2])
    ap.add_argument("--out", default="clean_benchmark")
    args = ap.parse_args(argv)

    lines = ["# Clean-Solve Benchmark (PyBullet-certified collision selection)", "",
             f"- K candidates: **{args.K}**  |  trials/seed {args.trials} × seeds {args.seeds}",
             "- `single`: honest one-shot (K=1). `clean`: best of K by real PyBullet clearance.",
             ""]
    for robot in args.robots:
        spec = get_robot_spec(robot)
        with PyBulletBackend(robot) as bk:
            print(f"[{robot}] oracle ready ({bk.ee_link}, {bk.offset_side})", flush=True)
            lines += [f"## {robot}", "",
                      "| Solver | Scenario | succ% | single col% | **clean col%** | "
                      "single clear | clean clear | mean cand | ms/solve (clean) |",
                      "|:--|:--|--:|--:|--:|--:|--:|--:|--:|"]
            for solver in args.solvers:
                for scen in args.scenarios:
                    n = 0
                    single_col = clean_col = 0
                    n_single = 0
                    s_clear, c_clear, ncand = [], [], []
                    t_clean = []
                    for seed in args.seeds:
                        gen = np.random.default_rng(seed)
                        targets = [generate_target(spec, gen, scen) for _ in range(args.trials)]
                        for ti, (q0, T) in enumerate(targets):
                            t0 = time.perf_counter()
                            cs = solve_clean(bk, solver, spec, q0, T, K=args.K,
                                             seed=seed * 10_000 + ti)
                            t_clean.append((time.perf_counter() - t0) * 1000.0)
                            if cs.result is None:
                                continue
                            n += 1
                            clean_col += int(cs.sim_in_collision)
                            c_clear.append(cs.sim_min_self_distance)
                            ncand.append(cs.n_candidates)
                            if cs.single_success:      # candidate 0 = honest single-shot
                                n_single += 1
                                single_col += int(cs.single_in_collision)
                                s_clear.append(cs.single_sim_min_self_distance)
                    if n == 0:
                        continue
                    succ = n
                    single_pct = 100 * single_col / max(n_single, 1)
                    row = (f"| {SOLVER_DISPLAY_NAMES.get(solver, solver)} | {scen} | "
                           f"{100*succ/n:.1f} | {single_pct:.1f} | "
                           f"**{100*clean_col/n:.1f}** | {np.mean(s_clear):+.4f} | "
                           f"{np.mean(c_clear):+.4f} | {np.mean(ncand):.1f} | "
                           f"{np.mean(t_clean):.0f} |")
                    lines.append(row)
                    print(f"  [{solver:13s} {scen:13s}] "
                          f"single_col {single_pct:5.1f}  "
                          f"clean_col {100*clean_col/n:5.1f}  "
                          f"mean_cand {np.mean(ncand):4.1f}  "
                          f"ms/solve {np.mean(t_clean):6.0f}", flush=True)
            lines.append("")

    with open(args.out + ".md", "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print(f"\nWrote {args.out}.md", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
