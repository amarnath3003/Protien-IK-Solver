"""
Use-case experiments: WHERE does ProteinIK (V4) actually excel?

The master benchmark answers "which solver is best per robot/scenario on
success/latency/collision". This script asks a different, deployment-oriented
question: IK solvers get chosen by the ROLE they play in a system (real-time
servo loop, planning-time goal sampler, offline batch generator, reliability
fallback, hyper-redundant folding). Each role rewards a DIFFERENT metric. We
run one experiment per role and see whether V4 wins or loses that role's metric.

Everything reuses the shared app kinematics/solvers so it's apples-to-apples
with the paper's benchmark. Self-collision is the capsule proxy (no environment
obstacles) — stated honestly; for the high-DOF folding experiment self-collision
IS the right metric (it is the protein-folding analog).

Run:
    python usecase_experiments.py --smoke      # tiny, ~1 min, validates plumbing
    python usecase_experiments.py              # full run
    python usecase_experiments.py --only E     # single experiment
"""
from __future__ import annotations

import argparse
import json
import sys
import time

import numpy as np

from app.core.kinematics import (
    RobotSpec, get_robot_spec, end_effector_pose, self_collision_min_distance,
)
from app.api.scenarios import generate_target
from app.solvers.registry import run_solver, SOLVER_DISPLAY_NAMES

# ----- tolerances defining a "good" solve (match solver defaults) -----
POS_TOL = 1e-3      # 1 mm
ORIENT_TOL = 1e-2   # 10 mrad


def planar_ndof_spec(n: int, total_reach: float = 1.0) -> RobotSpec:
    """An n-link planar (RRR...) arm, links summing to total_reach.
    FK/Jacobian/collision are all DOF-agnostic, so this is a clean testbed
    for the hyper-redundant regime the built-in arms (<=7 DOF) never reach."""
    a = np.full(n, total_reach / n)
    d = np.zeros(n)
    alpha = np.zeros(n)
    theta_offset = np.zeros(n)
    joint_limits = np.array([[-np.pi, np.pi]] * n)
    # thin uniform links; radius scaled so a folded chain genuinely self-clashes
    link_radius = np.full(n, 0.02)
    return RobotSpec(name=f"planar{n}dof", a=a, d=d, alpha=alpha,
                     theta_offset=theta_offset, joint_limits=joint_limits,
                     link_radius=link_radius)


def solved(r) -> bool:
    return bool(r.success) and r.pos_error < POS_TOL and r.orient_error < ORIENT_TOL


def clean(r) -> bool:
    """A config you'd actually deploy: reaches target AND is self-collision-free."""
    return solved(r) and r.min_self_distance >= 0.0


def wrap(dq: np.ndarray) -> np.ndarray:
    return (dq + np.pi) % (2 * np.pi) - np.pi


def pct(x) -> float:
    return 100.0 * float(x)


# ---------------------------------------------------------------------------
# EXP A — Real-time servo loop: WARM seed, small target move. Role reward =
# bounded, deterministic per-call latency. (Deployment: control loop @ >=100Hz.)
# ---------------------------------------------------------------------------
def exp_A(robots, solvers, n_steps, seeds):
    print("\n=== EXP A: real-time servo (warm seed, small move) — latency is king ===", flush=True)
    rows = []
    for robot in robots:
        spec = get_robot_spec(robot)
        for solver in solvers:
            lat, succ = [], 0
            tot = 0
            for seed in seeds:
                rng = np.random.default_rng(seed)
                q = spec.random_config(rng)
                for _ in range(n_steps):
                    # target = small joint move from current pose (servo tracking)
                    dq = rng.normal(0, 0.05, spec.n_joints)
                    T = end_effector_pose(spec, spec.clip(q + dq))
                    r = run_solver(solver, spec, q.copy(), T, rng)
                    lat.append(r.wall_time_ms)
                    succ += int(solved(r))
                    tot += 1
                    if solved(r):
                        q = np.asarray(r.q_final)  # warm-start next step from result
            lat = np.array(lat)
            rows.append(dict(exp="A", robot=robot, solver=solver, n=tot,
                             success=pct(succ / tot),
                             p50_ms=float(np.percentile(lat, 50)),
                             p99_ms=float(np.percentile(lat, 99)),
                             max_ms=float(lat.max()),
                             over_10ms=pct(np.mean(lat > 10)),
                             over_50ms=pct(np.mean(lat > 50))))
            print(f"  [{robot:<12} {SOLVER_DISPLAY_NAMES.get(solver,solver):<22}] "
                  f"succ {rows[-1]['success']:5.1f}%  p50 {rows[-1]['p50_ms']:6.2f}  "
                  f"p99 {rows[-1]['p99_ms']:7.1f}  max {rows[-1]['max_ms']:7.1f}  "
                  f">10ms {rows[-1]['over_10ms']:5.1f}%", flush=True)
    return rows


# ---------------------------------------------------------------------------
# EXP B — Planning-time goal sampler: K tries per target. Role reward =
# produce >=1 collision-free goal, and DIVERSE goals (planners want variety).
# ---------------------------------------------------------------------------
def exp_B(robot, solvers, n_targets, K, seed):
    print("\n=== EXP B: goal-config sampler for a motion planner (usable rate + diversity) ===", flush=True)
    spec = get_robot_spec(robot)
    tgen = np.random.default_rng(seed)
    targets = [generate_target(spec, tgen, "cluttered") for _ in range(n_targets)]
    rows = []
    for solver in solvers:
        atleast1, div_vals = 0, []
        usable_frac = []
        for ti, (q0, T) in enumerate(targets):
            configs = []
            usable_here = 0
            for k in range(K):
                rng = np.random.default_rng(seed * 7919 + ti * 101 + k)
                r = run_solver(solver, spec, spec.random_config(rng), T, rng)
                if clean(r):
                    configs.append(np.asarray(r.q_final))
                    usable_here += 1
            atleast1 += int(usable_here >= 1)
            usable_frac.append(usable_here / K)
            if len(configs) >= 2:
                C = np.array(configs)
                dists = [np.linalg.norm(wrap(C[i] - C[j]))
                         for i in range(len(C)) for j in range(i + 1, len(C))]
                div_vals.append(float(np.mean(dists)))
        rows.append(dict(exp="B", robot=robot, solver=solver, n_targets=n_targets, K=K,
                         atleast1_pct=pct(atleast1 / n_targets),
                         mean_usable_frac=pct(np.mean(usable_frac)),
                         diversity_rad=float(np.mean(div_vals)) if div_vals else 0.0))
        print(f"  [{SOLVER_DISPLAY_NAMES.get(solver,solver):<22}] "
              f">=1 usable goal {rows[-1]['atleast1_pct']:5.1f}%  "
              f"usable/attempt {rows[-1]['mean_usable_frac']:5.1f}%  "
              f"diversity {rows[-1]['diversity_rad']:.3f} rad", flush=True)
    return rows


# ---------------------------------------------------------------------------
# EXP C — Offline batch generation: 1 attempt/target. Role reward =
# CLEAN-solve rate (accurate AND collision-free), latency irrelevant.
# ---------------------------------------------------------------------------
def exp_C(robots, solvers, scenarios, n, seeds):
    print("\n=== EXP C: offline batch generation — clean-solve rate (quality per solve) ===", flush=True)
    rows = []
    for robot in robots:
        spec = get_robot_spec(robot)
        for scenario in scenarios:
            for solver in solvers:
                cln, sv, tot = 0, 0, 0
                for seed in seeds:
                    g = np.random.default_rng(seed)
                    tg = [generate_target(spec, g, scenario) for _ in range(n)]
                    for i, (q0, T) in enumerate(tg):
                        rng = np.random.default_rng(seed * 1_000_003 + i)
                        r = run_solver(solver, spec, q0, T, rng)
                        cln += int(clean(r)); sv += int(solved(r)); tot += 1
                rows.append(dict(exp="C", robot=robot, scenario=scenario, solver=solver,
                                 n=tot, solved_pct=pct(sv / tot), clean_pct=pct(cln / tot)))
                print(f"  [{robot:<12} {scenario:<13} {SOLVER_DISPLAY_NAMES.get(solver,solver):<22}] "
                      f"solved {rows[-1]['solved_pct']:5.1f}%  CLEAN {rows[-1]['clean_pct']:5.1f}%", flush=True)
    return rows


# ---------------------------------------------------------------------------
# EXP D — Reliability fallback tier: of the targets TRAC-IK punts (fails or
# returns a colliding config), what fraction does V4 rescue with a clean solve?
# ---------------------------------------------------------------------------
def exp_D(robots, primary, fallback, scenarios, n, seeds):
    print("\n=== EXP D: fallback tier — rescue rate on targets the fast solver punts ===", flush=True)
    rows = []
    for robot in robots:
        spec = get_robot_spec(robot)
        for scenario in scenarios:
            punts = 0
            rescued_fb = 0
            rescued_primary_self = 0  # sanity: does primary itself clean-solve them (should be ~0)
            tot = 0
            for seed in seeds:
                g = np.random.default_rng(seed)
                tg = [generate_target(spec, g, scenario) for _ in range(n)]
                for i, (q0, T) in enumerate(tg):
                    tot += 1
                    rp = run_solver(primary, spec, q0, T, np.random.default_rng(seed * 13 + i))
                    if not clean(rp):  # primary punted (failed OR colliding)
                        punts += 1
                        rf = run_solver(fallback, spec, q0, T, np.random.default_rng(seed * 17 + i))
                        rescued_fb += int(clean(rf))
            rate = pct(rescued_fb / punts) if punts else 0.0
            rows.append(dict(exp="D", robot=robot, scenario=scenario,
                             primary=primary, fallback=fallback, n=tot,
                             punt_pct=pct(punts / tot), rescue_pct=rate))
            print(f"  [{robot:<12} {scenario:<13}] {primary} punted {rows[-1]['punt_pct']:5.1f}% of targets "
                  f"-> {SOLVER_DISPLAY_NAMES.get(fallback,fallback)} rescued {rate:5.1f}% of those", flush=True)
    return rows


# ---------------------------------------------------------------------------
# EXP E — Hyper-redundant folding in clutter (the mechanism niche). Planar
# arms at growing DOF. Does V4's collision-free edge over TRAC-IK GROW with DOF?
# ---------------------------------------------------------------------------
def exp_E(dofs, solvers, n, seeds):
    print("\n=== EXP E: hyper-redundant folding — clean-solve vs DOF (the protein regime) ===", flush=True)
    rows = []
    for dof in dofs:
        spec = planar_ndof_spec(dof)
        for solver in solvers:
            cln, sv, tot = 0, 0, 0
            for seed in seeds:
                g = np.random.default_rng(1000 + seed)
                tg = [generate_target(spec, g, "cluttered") for _ in range(n)]
                for i, (q0, T) in enumerate(tg):
                    rng = np.random.default_rng(seed * 1_000_003 + i)
                    r = run_solver(solver, spec, q0, T, rng)
                    cln += int(clean(r)); sv += int(solved(r)); tot += 1
            rows.append(dict(exp="E", dof=dof, solver=solver, n=tot,
                             solved_pct=pct(sv / tot), clean_pct=pct(cln / tot)))
            print(f"  [planar {dof:>2}-DOF  {SOLVER_DISPLAY_NAMES.get(solver,solver):<22}] "
                  f"solved {rows[-1]['solved_pct']:5.1f}%  CLEAN {rows[-1]['clean_pct']:5.1f}%", flush=True)
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--smoke", action="store_true", help="tiny run to validate plumbing")
    ap.add_argument("--only", default=None, help="run a single experiment: A/B/C/D/E")
    ap.add_argument("--out", default="usecase_results.json")
    args = ap.parse_args(argv)

    KEY = ["protein_fast", "trac_ik_style", "jacobian_dls"]
    t0 = time.perf_counter()
    results = {}

    if args.smoke:
        cfg = dict(
            A=dict(robots=["ur5"], solvers=KEY, n_steps=8, seeds=[1]),
            B=dict(robot="ur5", solvers=["protein_fast", "trac_ik_style", "multi_start"], n_targets=5, K=5, seed=1),
            C=dict(robots=["ur5"], solvers=KEY, scenarios=["cluttered"], n=15, seeds=[1]),
            D=dict(robots=["ur5"], primary="trac_ik_style", fallback="protein_fast", scenarios=["cluttered"], n=20, seeds=[1]),
            E=dict(dofs=[4, 8, 12], solvers=["protein_fast", "trac_ik_style"], n=12, seeds=[1]),
        )
    else:
        cfg = dict(
            A=dict(robots=["ur5", "franka_panda"], solvers=KEY, n_steps=120, seeds=[1, 2, 3]),
            B=dict(robot="ur5", solvers=["protein_fast", "trac_ik_style", "multi_start"], n_targets=40, K=8, seed=1),
            C=dict(robots=["ur5", "franka_panda", "planar3dof"], solvers=KEY + ["multi_start"],
                   scenarios=["open_space", "cluttered"], n=100, seeds=[1, 2]),
            D=dict(robots=["ur5", "franka_panda"], primary="trac_ik_style", fallback="protein_fast",
                   scenarios=["cluttered", "near_singular"], n=100, seeds=[1, 2]),
            E=dict(dofs=[4, 6, 8, 12, 16], solvers=["protein_fast", "trac_ik_style"], n=60, seeds=[1, 2]),
        )

    want = args.only.upper() if args.only else None
    if want in (None, "A"): results["A"] = exp_A(**cfg["A"])
    if want in (None, "B"): results["B"] = exp_B(**cfg["B"])
    if want in (None, "C"): results["C"] = exp_C(**cfg["C"])
    if want in (None, "D"): results["D"] = exp_D(**cfg["D"])
    if want in (None, "E"): results["E"] = exp_E(**cfg["E"])

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
    print(f"\nDONE in {time.perf_counter()-t0:.1f}s -> {args.out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
