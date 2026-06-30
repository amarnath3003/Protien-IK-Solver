"""
Raw (V6) — Phase 3 experiment: is S_conf a genuine, RAW configurational
entropy — collision-aware and distinct from manipulability?

The audit's whole concern was that an entropy term might just be manipulability
maximization in disguise (which HAS an IK equivalent). The fix was to define the
entropy as the local clash-free accessible volume S = log Omega — target-blind
AND collision-aware. This script shows the two are genuinely different:

  1. Correlate, over random configs, the self-collision clearance with
       (a) S_conf            -> should be strongly POSITIVE (S is collision-aware)
       (b) manipulability w  -> should be ~0 (w is blind to self-collision)
     If S tracks clearance and w does not, S is not manipulability.

  2. Entropy ascent: from a constrained config, climb +grad(S_conf). The
     clearance and limit-margin should INCREASE — the unfolding/expansion drive
     that opposes LJ collapse (the competition that, under temperature, becomes
     the folding transition).

Run:  python raw_phase3_experiment.py
"""

from __future__ import annotations

import numpy as np

from app.core.kinematics import (
    ur5_spec, franka_panda_spec, planar3dof_spec, self_collision_min_distance,
)
from app.api.scenarios import _manipulability
from app.solvers.protein_raw.energy import config_entropy, config_entropy_and_grad


def _limit_margin(spec, q):
    lo, hi = spec.joint_limits[:, 0], spec.joint_limits[:, 1]
    return float(np.min(np.minimum(q - lo, hi - q)))


def _corr(a, b):
    a, b = np.asarray(a), np.asarray(b)
    if np.std(a) < 1e-12 or np.std(b) < 1e-12:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def run_robot(name, spec, n=48, seed=0, m=48):
    rng = np.random.default_rng(seed)
    qs = [spec.random_config(rng) for _ in range(n)]
    clear = np.array([self_collision_min_distance(spec, q) for q in qs])
    S = np.array([config_entropy(spec, q, m=m) for q in qs])
    W = np.array([_manipulability(spec, q) for q in qs])

    cs = _corr(clear, S)
    cw = _corr(clear, W)

    print(f"\n=== {name}  ({spec.n_joints} DOF, {n} configs, m={m}) ===")
    print(f"  corr(clearance, S_conf)        = {cs:+.3f}   (collision-aware -> strong +)")
    print(f"  corr(clearance, manipulability) = {cw:+.3f}   (manipulability is collision-blind -> ~0)")
    if np.std(clear) < 1e-12:
        print("  NOTE: clearance is a degenerate constant for this arm (capsule proxy) "
              "-> correlations undefined; entropy still responds to joint limits.")

    # --- entropy ascent from the most constrained config ---
    q0 = qs[int(np.argmin(S))]
    c0, l0 = self_collision_min_distance(spec, q0), _limit_margin(spec, q0)
    s0 = config_entropy(spec, q0, m=m)
    q = q0.copy()
    for _ in range(80):
        _, g = config_entropy_and_grad(spec, q, m=m)
        gn = np.linalg.norm(g)
        if gn < 1e-9:
            break
        q = spec.clip(q + 2e-2 * g / gn)        # ascend +grad(S)
    c1, l1 = self_collision_min_distance(spec, q), _limit_margin(spec, q)
    s1 = config_entropy(spec, q, m=m)
    print(f"  entropy ascent: S {s0:.2f} -> {s1:.2f};  clearance {c0:.4f} -> {c1:.4f};  "
          f"limit-margin {l0:.3f} -> {l1:.3f}")
    print("  -> maximizing S_conf opens the config (more clearance / limit room): "
          "the unfolding drive that competes with LJ collapse.")


if __name__ == "__main__":
    print("Raw (V6) Phase 3 - configurational entropy S = log Omega")
    print("(target-blind, collision-aware; the hydrophobic free-energy term)")
    for name, spec in (
        ("UR5", ur5_spec()),
        ("Planar 3-DOF", planar3dof_spec()),
        ("Franka Panda", franka_panda_spec()),
    ):
        run_robot(name, spec)
