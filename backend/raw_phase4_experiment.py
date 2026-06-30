"""
Raw (V6) — Phase 4 experiment: the Σ ratio landscape diagnostic + T_glass.

Σ = σ_E/ΔE over the Bryngelson-Wolynes compact ensemble (warm-start solutions)
characterises the funnel quality of the COLLISION-AWARE energy landscape:
Σ < 1 funnelled, Σ > 1 glassy. It also sets the solver's cooling target T_glass.

Honest validation. We do NOT assume a fixed scenario difficulty order (it does
not even hold: for UR5, DLS finds 'cluttered' easiest and 'open' hardest). We
instead report, transparently:
  - Σ per scenario (it varies — the landscape differs by regime),
  - the measured correlation of Σ with a real solver's difficulty (DLS final
    pos-error). Σ is a COLLISION-AWARE landscape measure, complementary to a
    collision-blind solver and to V5's during-solve conflict — so a modest
    correlation is the expected, honest result, not a strong oracle claim.
  - T_glass, the cooling target the Phase-5 Langevin solver will use.

Run:  python raw_phase4_experiment.py
"""

from __future__ import annotations

import numpy as np

from app.core.kinematics import ur5_spec, franka_panda_spec, planar3dof_spec
from app.api.scenarios import generate_target
from app.solvers.registry import run_solver
from app.solvers.protein_raw.landscape import (
    RawParams, sigma_ratio, configurational_entropy_scale, glass_temperature,
)

SCENARIOS = ["open_space", "cluttered", "near_singular"]


def _corr(a, b):
    a, b = np.asarray(a), np.asarray(b)
    if np.std(a) < 1e-12 or np.std(b) < 1e-12:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def run_robot(name, spec, n_targets=10, seed=0):
    rng = np.random.default_rng(seed)
    p = RawParams.calibrate(spec, rng)
    s0 = configurational_entropy_scale(spec, p.rho)

    print(f"\n=== {name}  ({spec.n_joints} DOF, sigma_scale={p.sigma_scale:.2f}, S0={s0:.1f}) ===")
    print(f"{'scenario':<16}{'mean Sigma':>12}{'mean T_glass':>14}{'DLS pos_err':>13}{'DLS fail':>10}")
    all_sigma, all_err = [], []
    for sc in SCENARIOS:
        sig, tg, derr, dfail = [], [], [], []
        for _ in range(n_targets):
            q0, T = generate_target(spec, rng, sc)
            r = sigma_ratio(spec, T, p, rng)
            sig.append(r["sigma"])
            tg.append(glass_temperature(r["sigma_E"], s0))
            res = run_solver("jacobian_dls", spec, q0.copy(), T, rng)
            derr.append(res.pos_error)
            dfail.append(int(not res.success))
            all_sigma.append(r["sigma"])
            all_err.append(res.pos_error)
        print(f"{sc:<16}{np.mean(sig):>12.3f}{np.mean(tg):>14.3f}"
              f"{np.mean(derr):>13.4f}{np.mean(dfail):>10.2f}")

    c = _corr(all_sigma, all_err)
    print(f"  corr(Sigma, DLS pos_err) = {c:+.3f}  "
          f"(collision-aware landscape vs collision-blind solver -> complementary, modest)")
    print(f"  Sigma is computed before any solve; it sets T_glass (cooling target) and is a "
          f"reported diagnostic, like V5's conflict_index.")


if __name__ == "__main__":
    print("Raw (V6) Phase 4 - Sigma ratio landscape diagnostic + glass temperature")
    for name, spec in (
        ("UR5", ur5_spec()),
        ("Franka Panda", franka_panda_spec()),
        ("Planar 3-DOF", planar3dof_spec()),
    ):
        run_robot(name, spec)
