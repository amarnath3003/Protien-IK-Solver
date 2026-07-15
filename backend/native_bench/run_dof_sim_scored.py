"""DOF-scaling sweep (paper Table 5 / usecase EXP E), scored three ways.

Why
---
Every other headline number in the paper is scored by two independent physics
engines ("solve once, score three ways"). The DOF-scaling result was the sole
exception: its planar arms are synthetic, had no URDF, and so were scored by
the capsule proxy alone (``app/sim/models.py`` excluded them outright). This
runner closes that gap using ``app.sim.planar_model``, which emits a URDF whose
collision solids are exactly the proxy's capsules.

It reproduces ``usecase_experiments.exp_E``'s loop verbatim -- identical target
stream (``default_rng(1000 + seed)``) and identical per-solve RNG
(``default_rng(seed * 1_000_003 + i)``) -- so the proxy column here must equal
the committed ``usecase_results.json``. Any drift means the replication is wrong,
and the runner asserts that rather than quietly reporting different numbers.

Two geometries, answering two different questions
------------------------------------------------
``capsule``  -- the solid the proxy models. A capsule's surface gap *is*
    segment-distance-minus-radii, so the engines recompute the proxy's own
    arithmetic with independent narrow-phase code. Agreement (measured at
    ~1e-16) validates our collision IMPLEMENTATION. It is not independent
    evidence about the geometry, and must not be reported as such.

``cylinder`` -- flat end caps instead of hemispherical: a genuinely different
    idealisation of the same arm. This tests whether the KineticFold/TRAC-IK
    clean-solve RATIO is an artifact of the capsule caps. This is the column
    that answers "would this shrink like the UR5 claim did (Section 5.6)?".

Run (WSL Ubuntu-2204, from backend/):
    PYTHONPATH=. python3 native_bench/run_dof_sim_scored.py \
        --out results/native/dof_scaling_sim_scored.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

# Side effects, in order: point robot_descriptions at the cache (_env.apply),
# then swap SOLVER_REGISTRY to genuine TRAC-IK + native C++ KineticFold. Importing
# this module is how run_native_usecase guarantees an identical solver setup.
import native_bench.run_native_usecase  # noqa: F401

import usecase_experiments as U
from app.core.kinematics import ROBOT_REGISTRY, self_collision_min_distance
from app.sim import planar_model

DOFS = [4, 6, 8, 12, 16]
SOLVERS = ["protein_fast", "trac_ik_style"]
N_PER_SEED = 60
SEEDS = [1, 2]
GEOMS = ["capsule", "cylinder"]


def _backends(robot: str):
    """Build both oracles for ``robot``; each self-verifies FK parity on construction."""
    from app.sim.mujoco_backend import MuJoCoBackend
    from app.sim.pybullet_backend import PyBulletBackend
    return {"pb": PyBulletBackend(robot), "mj": MuJoCoBackend(robot)}


def run_cell(spec, solver: str, backends: dict) -> dict:
    """One (dof, solver) cell: exp_E's exact loop, scored by proxy + both engines."""
    tot = 0
    solved_n = 0
    clean = {"proxy": 0, "pb": 0, "mj": 0}
    # verdict disagreements vs the proxy, and worst distance gap, for diagnostics
    disagree = {"pb": 0, "mj": 0}
    max_gap = {"pb": 0.0, "mj": 0.0}

    for seed in SEEDS:
        g = np.random.default_rng(1000 + seed)
        tg = [U.generate_target(spec, g, "cluttered") for _ in range(N_PER_SEED)]
        for i, (q0, T) in enumerate(tg):
            rng = np.random.default_rng(seed * 1_000_003 + i)
            r = U.run_solver(solver, spec, q0, T, rng)
            tot += 1

            ok = U.solved(r)          # DH-frame reach test, exactly as exp_E
            solved_n += int(ok)
            q = np.asarray(r.q_final)

            d_proxy = self_collision_min_distance(spec, q)
            clean["proxy"] += int(ok and d_proxy >= 0.0)

            for key, be in backends.items():
                in_col, d_eng = be.self_collision(q)
                clean[key] += int(ok and not in_col)
                disagree[key] += int((d_proxy < 0.0) != in_col)
                if d_eng < 0.49:  # unsaturated: engine actually measured this pair
                    max_gap[key] = max(max_gap[key], abs(d_proxy - d_eng))

    pct = lambda x: round(100.0 * x / tot, 4)  # noqa: E731
    return dict(
        n=tot,
        solved_pct=pct(solved_n),
        clean_pct=pct(clean["proxy"]),          # name matches usecase_results.json
        clean_pct_pybullet=pct(clean["pb"]),
        clean_pct_mujoco=pct(clean["mj"]),
        verdict_disagree_pb_pct=pct(disagree["pb"]),
        verdict_disagree_mj_pct=pct(disagree["mj"]),
        max_dist_gap_pb=max_gap["pb"],
        max_dist_gap_mj=max_gap["mj"],
    )


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="results/native/dof_scaling_sim_scored.json")
    ap.add_argument("--geoms", default=",".join(GEOMS))
    ap.add_argument("--dofs", default=",".join(str(d) for d in DOFS))
    args = ap.parse_args(argv)

    geoms = [g.strip() for g in args.geoms.split(",") if g.strip()]
    dofs = [int(d) for d in args.dofs.split(",") if d.strip()]

    out: dict[str, list] = {}
    t0 = time.perf_counter()

    for geom in geoms:
        rows = []
        print(f"\n{'='*80}\n### geometry = {geom}\n{'='*80}", flush=True)
        for dof in dofs:
            robot = f"planar{dof}dof"
            # rebuild the arm under this geometry (register_planar_arm allows
            # re-registration only for arms it generated itself)
            planar_model.GENERATED_URDFS.pop(robot, None)
            ROBOT_REGISTRY.pop(robot, None)
            planar_model.register_planar_arm(dof, geom_type=geom)

            spec = planar_model.planar_ndof_spec(dof)
            ref = U.planar_ndof_spec(dof)
            # The sim layer must model the SAME arm the experiment solves on.
            for field in ("a", "d", "alpha", "theta_offset", "link_radius", "joint_limits"):
                assert np.allclose(getattr(spec, field), getattr(ref, field)), \
                    f"planar{dof}dof spec drift in '{field}' vs usecase_experiments"

            backends = _backends(robot)
            for solver in SOLVERS:
                row = dict(exp="E", dof=dof, solver=solver, geom=geom,
                           **run_cell(ref, solver, backends))
                rows.append(row)
                print(f"  [planar {dof:>2}-DOF {solver:<16}] solved {row['solved_pct']:5.1f}%  "
                      f"CLEAN proxy {row['clean_pct']:5.1f}%  "
                      f"pb {row['clean_pct_pybullet']:5.1f}%  mj {row['clean_pct_mujoco']:5.1f}%  "
                      f"| disagree pb/mj {row['verdict_disagree_pb_pct']:.1f}/"
                      f"{row['verdict_disagree_mj_pct']:.1f}%  "
                      f"maxgap {max(row['max_dist_gap_pb'], row['max_dist_gap_mj']):.2e}",
                      flush=True)
            for be in backends.values():
                be.close()
        out[geom] = rows

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\n[done] {time.perf_counter() - t0:.1f}s -> {args.out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
