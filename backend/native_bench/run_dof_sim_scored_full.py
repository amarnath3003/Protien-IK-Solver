"""Engine cross-check for the *powered* DOF sweep (Table 5 at n=1000, three solvers).

``run_dof_sim_scored.py`` re-scores the DOF sweep in PyBullet and MuJoCo, but its
constants are pinned to the superseded n=120 pilot: five DOF points, two solvers,
seeds [1,2] x 60. That left §5.4's engine-scored ratios describing a sweep the
paper no longer reports.

This runner drives the same module at the sweep's real configuration -- seven DOF
points, three solvers (Multi-start included), seeds 1..10 x 100 -- by overriding
the module constants rather than editing it, so the committed pilot artifact
(`dof_scaling_sim_scored.json`) stays exactly reproducible.

Two geometries, unchanged in meaning from the original:
  capsule  -- equals the proxy by construction (a capsule's surface gap IS
              segment-distance-minus-radii), so agreement validates the collision
              IMPLEMENTATION, not the geometry.
  cylinder -- flat end caps: a genuinely different idealisation of the same arm,
              and the column that tests whether the solver ORDERING is an artifact
              of the capsule caps.

Run (inside WSL Ubuntu-2204, from backend/):
    export ROBOT_DESCRIPTIONS_CACHE="$HOME/.cache/robot_descriptions"
    PYTHONPATH=. python3 native_bench/run_dof_sim_scored_full.py \
        --out results/dof_scaling/dof_scaling_sim_scored_full.json
"""
from __future__ import annotations

import argparse
import sys

import native_bench.run_dof_sim_scored as M

DOFS = [4, 6, 8, 10, 12, 14, 16]
SOLVERS = ["protein_fast", "trac_ik_style", "multi_start"]
SEEDS = list(range(1, 11))
N_PER_SEED = 100


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out",
                    default="results/dof_scaling/dof_scaling_sim_scored_full.json")
    ap.add_argument("--geoms", default="capsule,cylinder")
    ap.add_argument("--dofs", default=",".join(str(d) for d in DOFS))
    ap.add_argument("--solvers", default=",".join(SOLVERS))
    ap.add_argument("--seeds", default=",".join(str(s) for s in SEEDS))
    ap.add_argument("--n", type=int, default=N_PER_SEED)
    args = ap.parse_args(argv)

    # Override the pilot's pinned constants. run_cell() reads these at call time.
    M.SOLVERS = [s.strip() for s in args.solvers.split(",") if s.strip()]
    M.SEEDS = [int(s) for s in args.seeds.split(",") if s.strip()]
    M.N_PER_SEED = args.n

    print(f"[sim-scored-full] dofs={args.dofs} solvers={M.SOLVERS} "
          f"seeds={M.SEEDS[0]}..{M.SEEDS[-1]} n={len(M.SEEDS) * M.N_PER_SEED}/cell "
          f"geoms={args.geoms}", flush=True)

    return M.main(["--out", args.out, "--geoms", args.geoms, "--dofs", args.dofs])


if __name__ == "__main__":
    sys.exit(main())
