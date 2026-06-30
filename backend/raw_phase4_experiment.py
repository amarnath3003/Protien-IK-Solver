"""
Raw (V6) — Phase 4 experiment: does the Σ ratio predict IK difficulty from the
landscape BEFORE solving?

The claim: Σ = σ_E/ΔE, measured by sampling the energy landscape for a target,
rank-orders scenario difficulty the way V5's (during-solve) difficulty_score
does — open_space (easy) < cluttered < near_singular (hard) — but computed
before any solve attempt. Higher Σ = ruggeder/flatter funnel = harder.

Also reports the glass temperature T_glass (the solver's cooling target).

Run:  python raw_phase4_experiment.py
"""

from __future__ import annotations

import numpy as np

from app.core.kinematics import ur5_spec, franka_panda_spec, planar3dof_spec
from app.api.scenarios import generate_target
from app.solvers.protein_raw.landscape import (
    RawParams, sigma_ratio, configurational_entropy_scale, glass_temperature,
)

SCENARIOS = ["open_space", "cluttered", "near_singular"]


def run_robot(name, spec, n_targets=10, seed=0):
    rng = np.random.default_rng(seed)
    p = RawParams.calibrate(spec, rng)
    s0 = configurational_entropy_scale(spec, p.rho)

    print(f"\n=== {name}  ({spec.n_joints} DOF, sigma_scale={p.sigma_scale:.2f}, S0={s0:.1f}) ===")
    print(f"{'scenario':<16}{'mean Sigma':>12}{'mean sigma_E':>14}{'mean dE':>10}{'mean T_glass':>14}")
    means = {}
    for sc in SCENARIOS:
        sigmas, sigEs, dEs, tgs = [], [], [], []
        for _ in range(n_targets):
            _, T_target = generate_target(spec, rng, sc)
            r = sigma_ratio(spec, T_target, p, rng, n_samples=300)
            sigmas.append(r["sigma"])
            sigEs.append(r["sigma_E"])
            dEs.append(r["delta_E"])
            tgs.append(glass_temperature(r["sigma_E"], s0))
        means[sc] = float(np.mean(sigmas))
        print(f"{sc:<16}{np.mean(sigmas):>12.3f}{np.mean(sigEs):>14.3f}"
              f"{np.mean(dEs):>10.3f}{np.mean(tgs):>14.3f}")

    order = sorted(means, key=means.get)
    ok = means["open_space"] <= means["cluttered"] <= means["near_singular"]
    print(f"  ordering by Sigma (easy->hard): {' < '.join(order)}")
    print(f"  -> {'MATCHES' if ok else 'does NOT match'} expected difficulty "
          f"(open < cluttered < near_singular).")


if __name__ == "__main__":
    print("Raw (V6) Phase 4 - Sigma ratio landscape-difficulty predictor")
    for name, spec in (
        ("UR5", ur5_spec()),
        ("Franka Panda", franka_panda_spec()),
        ("Planar 3-DOF", planar3dof_spec()),
    ):
        run_robot(name, spec)
