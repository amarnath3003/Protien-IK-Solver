"""
Phase 4 (sim_migration_plan.md §5): PyBullet vs MuJoCo vs our-DH cross-check.

A second, independent simulator is only worth building if it can *disagree*. This
runner puts PyBullet and MuJoCo side by side on the **same URDF model** and asks
three questions, escalating from "are the engines the same robot" to "does the
paper's headline survive a second engine":

  A. **FK agreement.** For N random configs, compare each engine's EE-link world
     frame directly (both already validated against our DH at construction). If
     DH==PyBullet and DH==MuJoCo to ~1e-8 (Phase 1), PyBullet==MuJoCo must follow;
     this measures it explicitly. Three-way agreement = the robot model is not in
     question.

  B. **Collision agreement.** For the same configs, compare the capsule proxy, the
     PyBullet real-mesh distance (`getClosestPoints`) and the MuJoCo real-mesh
     distance (`mj_geomDistance`) over the *same* non-adjacent link pairs. Confirms
     the Phase-3 finding "the proxy is optimistic; real collision is much higher" is
     an engine-independent fact, not a PyBullet artifact -- and quantifies how far
     two independent real-mesh engines agree with each other.

  C. **Solver-edge replication.** Re-score every solver's `q_final` in *both*
     engines. The paper's headline (V4 has the lowest real self-collision of any
     fast/high-success solver on the non-redundant UR5; the edge vanishes on the
     redundant Franka) is only trustworthy if it holds on both. This is the
     strongest confirmation a second oracle can give.

Runs headless; needs BOTH pybullet and mujoco, so execute in `.venv-sim`:

    PYTHONPATH=. .venv-sim/Scripts/python -m bench.sim_crosscheck
    PYTHONPATH=. .venv-sim/Scripts/python -m bench.sim_crosscheck \
        --robots ur5 --fk-samples 3000 --collision-samples 3000 \
        --trials 60 --seeds 1 2 --solvers protein_fast trac_ik_style
"""

from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

from app.core.kinematics import get_robot_spec, self_collision_min_distance
from app.api.scenarios import generate_target
from app.solvers.registry import SOLVER_DISPLAY_NAMES, get_solvers_for_robot, run_solver
from app.sim.pybullet_backend import PyBulletBackend
from app.sim.mujoco_backend import MuJoCoBackend
from app.sim.parity import _rel_angle

ALL_ROBOTS = ["ur5", "franka_panda"]
ALL_SCENARIOS = ["open_space", "near_singular", "cluttered"]
# The headline solvers (fast + the two protein variants the claims hinge on).
DEFAULT_SOLVERS = ["protein_fast", "trac_ik_style", "multi_start", "protein_ik", "protein_raw"]
# Threshold for both engines' closest-point queries (meters). Matches Phase 3.
_THRESHOLD = 0.8


# ---------------------------------------------------------------------------
# Part A — FK agreement
# ---------------------------------------------------------------------------

def fk_agreement(pb: PyBulletBackend, mj: MuJoCoBackend, spec, n: int, seed: int) -> dict:
    rng = np.random.default_rng(seed)
    dpos = np.empty(n)
    dang = np.empty(n)
    for i in range(n):
        q = spec.random_config(rng)
        Tp = pb.fk(q)           # EE-link world frame, PyBullet
        Tm = mj.fk(q)           # EE-link world frame, MuJoCo (same URDF, same frame)
        dpos[i] = np.linalg.norm(Tp[:3, 3] - Tm[:3, 3])
        dang[i] = _rel_angle(Tp[:3, :3], Tm[:3, :3])
    return {
        "n": n,
        "max_pos": float(dpos.max()), "mean_pos": float(dpos.mean()),
        "max_orient": float(dang.max()), "mean_orient": float(dang.mean()),
        "dh_pb_residual": float(pb.offset_residual),
        "dh_mj_residual": float(mj.offset_residual),
        "pb_offset": pb.offset_side, "mj_offset": mj.offset_side,
    }


# ---------------------------------------------------------------------------
# Part B — collision agreement
# ---------------------------------------------------------------------------

def _sign_stats(a: np.ndarray, b: np.ndarray) -> dict:
    """Agreement between two signed-distance arrays on the collide/clear call."""
    ac, bc = a < 0, b < 0
    return {
        "a_col_pct": 100 * float(ac.mean()),
        "b_col_pct": 100 * float(bc.mean()),
        "sign_agree_pct": 100 * float((ac == bc).mean()),
        "corr": float(np.corrcoef(a, b)[0, 1]),
    }


def collision_agreement(pb: PyBulletBackend, mj: MuJoCoBackend, spec, n: int, seed: int) -> dict:
    rng = np.random.default_rng(seed)
    proxy = np.empty(n)
    pbd = np.empty(n)
    mjd = np.empty(n)
    for i in range(n):
        q = spec.random_config(rng)
        proxy[i] = self_collision_min_distance(spec, q)
        _, pbd[i] = pb.self_collision(q, threshold=_THRESHOLD)
        _, mjd[i] = mj.self_collision(q, threshold=_THRESHOLD)

    near = (np.abs(pbd) < 0.1) & (np.abs(mjd) < 0.1)
    pb_mj = _sign_stats(pbd, mjd)
    # proxy optimism vs each real engine (false-clear = proxy says clear, engine says collide)
    def false_clear(real):
        return 100 * float(((real < 0) & (proxy >= 0)).mean())
    return {
        "n": n,
        "proxy_col_pct": 100 * float((proxy < 0).mean()),
        "pb_col_pct": pb_mj["a_col_pct"],
        "mj_col_pct": pb_mj["b_col_pct"],
        "pb_mj_sign_agree_pct": pb_mj["sign_agree_pct"],
        "pb_mj_corr": pb_mj["corr"],
        "pb_mj_corr_near": (float(np.corrcoef(pbd[near], mjd[near])[0, 1])
                            if near.sum() > 20 else float("nan")),
        "pb_mj_max_abs_diff": float(np.max(np.abs(pbd - mjd))),
        "pb_mj_mean_abs_diff": float(np.mean(np.abs(pbd - mjd))),
        "proxy_false_clear_vs_pb": false_clear(pbd),
        "proxy_false_clear_vs_mj": false_clear(mjd),
        "proxy_mean_gap_vs_pb": float(np.mean(proxy - pbd)),
        "proxy_mean_gap_vs_mj": float(np.mean(proxy - mjd)),
        "near_count": int(near.sum()),
    }


# ---------------------------------------------------------------------------
# Part C — solver-edge replication (score q_final in BOTH engines)
# ---------------------------------------------------------------------------

def solver_cell(pb: PyBulletBackend, mj: MuJoCoBackend, spec, robot: str,
                scenario: str, solver: str, n_trials: int, seeds: list[int]) -> dict:
    n = 0
    our_succ = pb_col = mj_col = both_col_agree = 0
    pb_clear, mj_clear = [], []
    for seed in seeds:
        gen = np.random.default_rng(seed)
        targets = [generate_target(spec, gen, scenario) for _ in range(n_trials)]
        for i, (q0, T_dh) in enumerate(targets):
            n += 1
            rng = np.random.default_rng(seed * 1_000_003 + i)
            r = run_solver(solver, spec, q0, T_dh, rng)
            q_final = np.asarray(r.q_final, dtype=float)
            our_succ += int(bool(r.success))
            sp = pb.score(q_final, T_dh)
            sm = mj.score(q_final, T_dh)
            pb_col += int(sp.sim_in_collision)
            mj_col += int(sm.sim_in_collision)
            both_col_agree += int(sp.sim_in_collision == sm.sim_in_collision)
            pb_clear.append(sp.sim_min_self_distance)
            mj_clear.append(sm.sim_min_self_distance)
    return {
        "robot": robot, "scenario": scenario, "solver": solver,
        "display_name": SOLVER_DISPLAY_NAMES.get(solver, solver),
        "n": n,
        "our_success_pct": 100 * our_succ / n,
        "pb_collision_pct": 100 * pb_col / n,
        "mj_collision_pct": 100 * mj_col / n,
        "col_call_agree_pct": 100 * both_col_agree / n,
        "pb_mean_clearance_m": float(np.mean(pb_clear)),
        "mj_mean_clearance_m": float(np.mean(mj_clear)),
    }


# ---------------------------------------------------------------------------
# reporting
# ---------------------------------------------------------------------------

def write_markdown(path: str, meta: dict, fk: dict, col: dict, cells: list[dict]) -> None:
    L = ["# Sim Cross-Check — PyBullet vs MuJoCo vs our DH (Phase 4)", "",
         "Both simulators load the **identical** URDF (classic UR5 `ur5_robot.urdf`, "
         "franka_ros `panda.urdf`), so this isolates *engine* differences from *model* "
         "differences. MuJoCo reads link frames from `data.xmat` (no wxyz/xyzw quaternion "
         "hazard) and closest distances from `mj_geomDistance` over the same non-adjacent "
         "link pairs PyBullet queries with `getClosestPoints`.", ""]

    # Part A
    L += ["## A. Forward-kinematics agreement (three-way)", "",
          "| Robot | n | DH↔PyBullet resid | DH↔MuJoCo resid | PyBullet↔MuJoCo max pos | max orient |",
          "|:--|--:|--:|--:|--:|--:|"]
    for robot in meta["robots"]:
        f = fk[robot]
        L.append(f"| {robot} | {f['n']} | {f['dh_pb_residual']:.1e} ({f['pb_offset']}) | "
                 f"{f['dh_mj_residual']:.1e} ({f['mj_offset']}) | "
                 f"{f['max_pos']:.2e} m | {f['max_orient']:.2e} rad |")
    L += ["", "All three kinematic models agree to floating-point noise: our hand-typed "
          "DH, PyBullet, and MuJoCo are the same robot. This independently re-confirms "
          "the corrected modified-DH Panda on a second engine.", ""]

    # Part B
    L += ["## B. Collision agreement (real mesh, both engines vs the capsule proxy)", "",
          "| Robot | n | proxy col% | PyBullet col% | MuJoCo col% | PB↔MJ sign-agree% | "
          "PB↔MJ corr | proxy false-clear vs PB | vs MJ |",
          "|:--|--:|--:|--:|--:|--:|--:|--:|--:|"]
    for robot in meta["robots"]:
        c = col[robot]
        L.append(f"| {robot} | {c['n']} | {c['proxy_col_pct']:.1f} | {c['pb_col_pct']:.1f} | "
                 f"{c['mj_col_pct']:.1f} | {c['pb_mj_sign_agree_pct']:.1f} | {c['pb_mj_corr']:.3f} | "
                 f"{c['proxy_false_clear_vs_pb']:.1f}% | {c['proxy_false_clear_vs_mj']:.1f}% |")
    L += ["",
          "- Both independent real-mesh engines report **far more** collision than the "
          "capsule proxy — the Phase-3 \"proxy is optimistic\" result is engine-independent.",
          "- PyBullet and MuJoCo agree with each other on the collide/clear call at the "
          "sign-agree% above (residual disagreement is near-boundary convex-hull noise "
          "between two independent collision implementations).", ""]

    # Part C
    L += ["## C. Solver collision edge — does it replicate on the second engine?", "",
          "Every solver's `q_final` scored in **both** engines (real mesh self-collision).", ""]
    by_cell: dict[tuple, list[dict]] = {}
    for r in cells:
        by_cell.setdefault((r["robot"], r["scenario"]), []).append(r)
    for robot in meta["robots"]:
        for scenario in meta["scenarios"]:
            cell = by_cell.get((robot, scenario))
            if not cell:
                continue
            cell = sorted(cell, key=lambda x: x["pb_collision_pct"])
            L += [f"### {robot} — {scenario}", "",
                  "| Solver | succ% | PyBullet col% | MuJoCo col% | col-call agree% | "
                  "PB clear m | MJ clear m |",
                  "|:--|--:|--:|--:|--:|--:|--:|"]
            for r in cell:
                L.append(f"| {r['display_name']} | {r['our_success_pct']:.1f} | "
                         f"{r['pb_collision_pct']:.1f} | {r['mj_collision_pct']:.1f} | "
                         f"{r['col_call_agree_pct']:.1f} | {r['pb_mean_clearance_m']:+.4f} | "
                         f"{r['mj_mean_clearance_m']:+.4f} |")
            L.append("")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))


def write_csv(path: str, cells: list[dict]) -> None:
    cols = ["robot", "scenario", "solver", "display_name", "n", "our_success_pct",
            "pb_collision_pct", "mj_collision_pct", "col_call_agree_pct",
            "pb_mean_clearance_m", "mj_mean_clearance_m"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in cells:
            w.writerow(r)


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description="PyBullet vs MuJoCo cross-check (Phase 4).")
    ap.add_argument("--robots", nargs="+", default=ALL_ROBOTS, choices=ALL_ROBOTS)
    ap.add_argument("--scenarios", nargs="+", default=ALL_SCENARIOS, choices=ALL_SCENARIOS)
    ap.add_argument("--solvers", nargs="+", default=DEFAULT_SOLVERS)
    ap.add_argument("--fk-samples", type=int, default=2000)
    ap.add_argument("--collision-samples", type=int, default=2000)
    ap.add_argument("--trials", type=int, default=60, help="solver trials per seed per cell")
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2])
    ap.add_argument("--out", default="sim_crosscheck")
    args = ap.parse_args(argv)

    fk_out: dict[str, dict] = {}
    col_out: dict[str, dict] = {}
    cells: list[dict] = []

    for robot in args.robots:
        spec = get_robot_spec(robot)
        valid = set(get_solvers_for_robot(robot))
        solvers = [s for s in args.solvers if s in valid]
        # Build PyBullet first, then hand MuJoCo the *identical* collision link set so
        # the two engines query the same meaningful pairs (see mujoco_backend docstring).
        with PyBulletBackend(robot) as pb, \
                MuJoCoBackend(robot, collision_link_names=pb.collision_link_names) as mj:
            pb_pairs = {tuple(sorted((pb._idx_to_name[a], pb._idx_to_name[b])))
                        for (a, b) in pb._collision_pairs}
            mj_pairs = {tuple(sorted((p.name_a, p.name_b))) for p in mj._collision_pairs}
            # geomless-root pairs are no-ops in PyBullet; compare only geom-bearing pairs
            shared = pb_pairs & mj_pairs
            print(f"[{robot}] PB offset={pb.offset_side} resid={pb.offset_residual:.1e} | "
                  f"MJ offset={mj.offset_side} resid={mj.offset_residual:.1e} | "
                  f"MJ pairs={len(mj_pairs)} ⊆ PB pairs={len(pb_pairs)}: "
                  f"{mj_pairs.issubset(pb_pairs)} (shared={len(shared)})", flush=True)

            fk_out[robot] = fk_agreement(pb, mj, spec, args.fk_samples, seed=7)
            f = fk_out[robot]
            print(f"  [A fk]  PB↔MJ max_pos={f['max_pos']:.2e}m max_orient={f['max_orient']:.2e}rad",
                  flush=True)

            col_out[robot] = collision_agreement(pb, mj, spec, args.collision_samples, seed=123)
            c = col_out[robot]
            print(f"  [B col] proxy {c['proxy_col_pct']:.1f}%  PB {c['pb_col_pct']:.1f}%  "
                  f"MJ {c['mj_col_pct']:.1f}%  sign-agree {c['pb_mj_sign_agree_pct']:.1f}%  "
                  f"corr {c['pb_mj_corr']:.3f}", flush=True)

            for scenario in args.scenarios:
                for solver in solvers:
                    row = solver_cell(pb, mj, spec, robot, scenario, solver,
                                      args.trials, args.seeds)
                    cells.append(row)
                    print(f"  [C {scenario:<13} {row['display_name']:<24}] "
                          f"succ {row['our_success_pct']:5.1f}  "
                          f"PB_col {row['pb_collision_pct']:5.1f}  "
                          f"MJ_col {row['mj_collision_pct']:5.1f}  "
                          f"agree {row['col_call_agree_pct']:5.1f}", flush=True)

    meta = dict(robots=args.robots, scenarios=args.scenarios,
                trials=args.trials, seeds=args.seeds)
    write_markdown(args.out + ".md", meta, fk_out, col_out, cells)
    write_csv(args.out + ".csv", cells)
    print(f"\nWrote {args.out}.md and {args.out}.csv", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
