"""
Tri-simulator mini-benchmark: V4 vs baselines under ALL THREE collision models on
the same solves — our in-house capsule proxy, PyBullet real mesh, and MuJoCo real
mesh. The paper reports PyBullet + MuJoCo (the two independent real-mesh oracles);
"our sim" is shown alongside to make the proxy's optimism visible in the same table.

For each (robot, scenario, solver) we run the solver ONCE on our fast ``RobotSpec``
core, then read its self-collision three ways for the identical ``q_final``:

  * **our**  — the capsule proxy (``SolveResult.min_self_distance``), i.e. what the
    solver itself optimizes against.
  * **PB**   — PyBullet ``getClosestPoints`` (real mesh).
  * **MJ**   — MuJoCo ``mj_geomDistance`` (real mesh), on the *identical* URDF and the
    *identical* non-adjacent link pairs PyBullet uses (fairness; see mujoco_backend).

Success is our-FK success (already validated to equal sim FK in Phase 1). The
question this answers cleanly: **is V4 the lowest-collision fast/high-success solver
— and does that hold under real meshes, not just our proxy?**

Runs headless; needs pybullet + mujoco, so execute in ``.venv-sim``:

    PYTHONPATH=. .venv-sim/Scripts/python -m bench.tri_sim_benchmark
    PYTHONPATH=. .venv-sim/Scripts/python -m bench.tri_sim_benchmark --robots ur5 --trials 60
"""

from __future__ import annotations

import argparse
import csv
import sys

import numpy as np

from app.core.kinematics import get_robot_spec
from app.api.scenarios import generate_target
from app.solvers.registry import SOLVER_DISPLAY_NAMES, get_solvers_for_robot, run_solver
from app.sim.pybullet_backend import PyBulletBackend
from app.sim.mujoco_backend import MuJoCoBackend

ALL_ROBOTS = ["ur5", "franka_panda"]
ALL_SCENARIOS = ["open_space", "near_singular", "cluttered"]
# V4 + the practical baselines a reader compares against (no slow homotopy variants).
DEFAULT_SOLVERS = ["protein_fast", "trac_ik_style", "multi_start", "protein_ik",
                   "jacobian_dls", "ccd", "fabrik"]
V4 = "protein_fast"
HIGH_SUCCESS = 90.0  # a solver must clear this to be a fair collision comparison


def cell(pb, mj, spec, robot, scenario, solver, n_trials, seeds) -> dict:
    n = 0
    succ = our_col = pb_col = mj_col = 0
    our_clear, pb_clear, mj_clear = [], [], []
    for seed in seeds:
        gen = np.random.default_rng(seed)
        targets = [generate_target(spec, gen, scenario) for _ in range(n_trials)]
        for i, (q0, T_dh) in enumerate(targets):
            n += 1
            rng = np.random.default_rng(seed * 1_000_003 + i)
            r = run_solver(solver, spec, q0, T_dh, rng)
            q_final = np.asarray(r.q_final, dtype=float)
            succ += int(bool(r.success))
            our_col += int(r.min_self_distance < 0)
            our_clear.append(float(r.min_self_distance))
            sp = pb.score(q_final, T_dh)
            sm = mj.score(q_final, T_dh)
            pb_col += int(sp.sim_in_collision)
            mj_col += int(sm.sim_in_collision)
            pb_clear.append(sp.sim_min_self_distance)
            mj_clear.append(sm.sim_min_self_distance)
    return {
        "robot": robot, "scenario": scenario, "solver": solver,
        "display_name": SOLVER_DISPLAY_NAMES.get(solver, solver), "n": n,
        "succ_pct": 100 * succ / n,
        "our_col_pct": 100 * our_col / n,
        "pb_col_pct": 100 * pb_col / n,
        "mj_col_pct": 100 * mj_col / n,
        "our_clear_m": float(np.mean(our_clear)),
        "pb_clear_m": float(np.mean(pb_clear)),
        "mj_clear_m": float(np.mean(mj_clear)),
    }


def _winner(cells: list[dict], key: str) -> str:
    """Lowest-collision solver under `key`, among the high-success ones."""
    pool = [c for c in cells if c["succ_pct"] >= HIGH_SUCCESS]
    if not pool:
        return "—"
    best = min(pool, key=lambda c: c[key])
    return best["solver"]


def write_markdown(path: str, meta: dict, rows: list[dict]) -> None:
    by = {}
    for r in rows:
        by.setdefault((r["robot"], r["scenario"]), []).append(r)

    L = ["# Tri-Simulator Mini-Benchmark — V4 vs baselines under 3 collision models", "",
         f"- Trials/seed **{meta['trials']}** × seeds **{meta['seeds']}** "
         f"(n={meta['trials'] * len(meta['seeds'])}/cell)  |  same `q_final` scored 3 ways",
         "- **our** = capsule proxy (what the solver optimizes) · **PB** = PyBullet real "
         "mesh · **MJ** = MuJoCo real mesh (identical URDF & link pairs)",
         f"- Collision **winner** = lowest-collision solver among those ≥{HIGH_SUCCESS:.0f}% success. "
         "Paper uses **PB + MJ**; `our` shown to expose the proxy's optimism.", ""]

    # verdict matrix first — the headline
    L += ["## Verdict — which solver collides least (high-success solvers only)", "",
          "| Robot | Scenario | our sim | **PyBullet** | **MuJoCo** |",
          "|:--|:--|:--|:--|:--|"]
    for robot in meta["robots"]:
        for scenario in meta["scenarios"]:
            c = by.get((robot, scenario))
            if not c:
                continue
            def tag(k):
                w = _winner(c, k)
                name = SOLVER_DISPLAY_NAMES.get(w, w)
                return f"**{name}**" if w == V4 else name
            L.append(f"| {robot} | {scenario} | {tag('our_col_pct')} | "
                     f"{tag('pb_col_pct')} | {tag('mj_col_pct')} |")
    L += ["", "A **bold** cell = V4 wins that (arm, scenario, simulator).", ""]

    # per-cell detail tables
    for robot in meta["robots"]:
        for scenario in meta["scenarios"]:
            c = by.get((robot, scenario))
            if not c:
                continue
            c = sorted(c, key=lambda x: x["pb_col_pct"])
            L += [f"## {robot} — {scenario}", "",
                  "| Solver | succ% | our col% | **PB col%** | **MJ col%** | "
                  "our clear | PB clear | MJ clear |",
                  "|:--|--:|--:|--:|--:|--:|--:|--:|"]
            for r in c:
                mark = " ⟵ V4" if r["solver"] == V4 else ""
                L.append(f"| {r['display_name']}{mark} | {r['succ_pct']:.0f} | "
                         f"{r['our_col_pct']:.0f} | {r['pb_col_pct']:.0f} | {r['mj_col_pct']:.0f} | "
                         f"{r['our_clear_m']:+.4f} | {r['pb_clear_m']:+.4f} | {r['mj_clear_m']:+.4f} |")
            L.append("")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description="Tri-sim mini-benchmark (our/PyBullet/MuJoCo).")
    ap.add_argument("--robots", nargs="+", default=ALL_ROBOTS, choices=ALL_ROBOTS)
    ap.add_argument("--scenarios", nargs="+", default=ALL_SCENARIOS, choices=ALL_SCENARIOS)
    ap.add_argument("--solvers", nargs="+", default=DEFAULT_SOLVERS)
    ap.add_argument("--trials", type=int, default=50)
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2])
    ap.add_argument("--out", default="tri_sim_benchmark")
    args = ap.parse_args(argv)

    rows: list[dict] = []
    for robot in args.robots:
        spec = get_robot_spec(robot)
        valid = set(get_solvers_for_robot(robot))
        solvers = [s for s in args.solvers if s in valid]
        with PyBulletBackend(robot) as pb, \
                MuJoCoBackend(robot, collision_link_names=pb.collision_link_names) as mj:
            print(f"[{robot}] backends ready (PB resid={pb.offset_residual:.1e}, "
                  f"MJ resid={mj.offset_residual:.1e})", flush=True)
            for scenario in args.scenarios:
                for solver in solvers:
                    r = cell(pb, mj, spec, robot, scenario, solver, args.trials, args.seeds)
                    rows.append(r)
                    print(f"  [{scenario:<13} {r['display_name']:<20}] succ {r['succ_pct']:5.1f}  "
                          f"our_col {r['our_col_pct']:5.1f}  PB {r['pb_col_pct']:5.1f}  "
                          f"MJ {r['mj_col_pct']:5.1f}", flush=True)

    meta = dict(robots=args.robots, scenarios=args.scenarios,
                trials=args.trials, seeds=args.seeds)
    write_markdown(args.out + ".md", meta, rows)
    with open(args.out + ".csv", "w", newline="", encoding="utf-8") as f:
        cols = ["robot", "scenario", "solver", "display_name", "n", "succ_pct",
                "our_col_pct", "pb_col_pct", "mj_col_pct",
                "our_clear_m", "pb_clear_m", "mj_clear_m"]
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"\nWrote {args.out}.md and {args.out}.csv", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
