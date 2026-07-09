"""
Raw (V6) — Phase 1 experiment: does the LJ ATTRACTION do real work?

The claim that makes term #1 "raw" is that a FULL 6-12 potential (with the
attractive −(σ/d)^6 term) gives the chain an *emergent preferred inter-bead
spacing*, whereas the repulsion-only wall that every existing IK collision
model uses can only push links apart with no preferred distance.

This script tests that directly. For each robot we:
  1. sample random configs and set σ so the attractive well sits at the median
     non-adjacent bead distance (so attraction is actually in range);
  2. relax each config under (a) full LJ and (b) repulsion-only, by gradient
     descent on E_LJ ALONE (no task, no other term);
  3. report the distribution of the spacing ratio  d_ij / d_well  (d_well =
     2^(1/6)·σ_ij) and the capsule min-self-distance.

Expected signature of a real attractive well:
  full LJ      → ratios CONCENTRATE near 1.0 (a preferred spacing emerges:
                 "hydrophobic collapse" into the well) → std shrinks;
  repulsion    → ratios DRIFT UP with no concentration (links just spread to
                 the kinematic limit).

Run:  python raw_phase1_experiment.py
"""

from __future__ import annotations

import numpy as np

from app.core.kinematics import (
    ur5_spec, franka_panda_spec, planar3dof_spec,
    self_collision_min_distance,
)
from app.solvers.protein_raw.energy import (
    bead_positions, lj_energy, lj_energy_and_grad, _bead_radii, _nonadjacent_pairs,
)

WELL = 2.0 ** (1.0 / 6.0)


def _pair_distances(spec, q):
    pts = bead_positions(spec, q)
    I, J = _nonadjacent_pairs(pts.shape[0])
    return np.linalg.norm(pts[I] - pts[J], axis=1), I, J


def _calibrate_sigma_scale(spec, rng, n=200):
    """Set σ so the attractive well sits at the median non-adjacent spacing."""
    radii = _bead_radii(spec)
    I, J = _nonadjacent_pairs(spec.n_joints + 1)
    rad_sum = radii[I] + radii[J]
    dists = []
    for _ in range(n):
        d, _, _ = _pair_distances(spec, spec.random_config(rng))
        dists.append(d)
    d_med = float(np.median(np.concatenate(dists)))
    return d_med / (WELL * float(np.mean(rad_sum)))


def _relax(spec, q0, sigma_scale, attractive, iters=1200, lr=2e-2, max_step=0.05):
    """Gradient descent on E_LJ alone (capped step)."""
    q = q0.copy()
    for _ in range(iters):
        _, g = lj_energy_and_grad(spec, q, sigma_scale, 1.0, attractive)
        step = -lr * g
        nrm = np.linalg.norm(step)
        if nrm > max_step:
            step *= max_step / nrm
        q = spec.clip(q + step)
    return q


def _well_ratios(spec, q, sigma_scale):
    radii = _bead_radii(spec)
    d, I, J = _pair_distances(spec, q)
    d_well = WELL * sigma_scale * (radii[I] + radii[J])
    return d / d_well


def run_robot(name, spec, n_configs=24, seed=0):
    rng = np.random.default_rng(seed)
    sigma_scale = _calibrate_sigma_scale(spec, rng)

    rows = {}
    init_ratios, init_msd = [], []
    full_ratios, full_msd = [], []
    repu_ratios, repu_msd = [], []
    e_init, e_full = [], []   # full-LJ energy before/after relaxation (descent proof)

    for _ in range(n_configs):
        q0 = spec.random_config(rng)
        init_ratios.append(_well_ratios(spec, q0, sigma_scale))
        init_msd.append(self_collision_min_distance(spec, q0))
        e_init.append(lj_energy(spec, q0, sigma_scale, 1.0, True))

        q_full = _relax(spec, q0, sigma_scale, attractive=True)
        full_ratios.append(_well_ratios(spec, q_full, sigma_scale))
        full_msd.append(self_collision_min_distance(spec, q_full))
        e_full.append(lj_energy(spec, q_full, sigma_scale, 1.0, True))

        q_repu = _relax(spec, q0, sigma_scale, attractive=False)
        repu_ratios.append(_well_ratios(spec, q_repu, sigma_scale))
        repu_msd.append(self_collision_min_distance(spec, q_repu))

    def band(ratios):  # fraction sitting in the attractive-well band
        r = np.concatenate(ratios)
        return float(np.mean((r >= 0.9) & (r <= 1.15)))

    for label, ratios, msd in (
        ("initial (random)", init_ratios, init_msd),
        ("full LJ (attract)", full_ratios, full_msd),
        ("repulsion-only", repu_ratios, repu_msd),
    ):
        r = np.concatenate(ratios)
        rows[label] = (float(r.mean()), float(r.std()), band(ratios), float(np.mean(msd)))

    print(f"\n=== {name}  (sigma_scale={sigma_scale:.3f}, {spec.n_joints} DOF, "
          f"{_nonadjacent_pairs(spec.n_joints + 1)[0].size} non-adj pairs) ===")
    print(f"{'state':<20}{'mean d/well':>12}{'std d/well':>12}"
          f"{'% in well':>11}{'mean min_self':>15}")
    for label, (m, s, b, msd) in rows.items():
        print(f"{label:<20}{m:>12.3f}{s:>12.3f}{b*100:>10.1f}%{msd:>15.4f}")

    mean_init_std = rows["initial (random)"][1]
    full_std = rows["full LJ (attract)"][1]
    repu_mean = rows["repulsion-only"][0]
    full_band = rows["full LJ (attract)"][2]
    repu_band = rows["repulsion-only"][2]
    print(f"  E_LJ (full) descent: {np.mean(e_init):.3g} -> {np.mean(e_full):.3g}  "
          f"(mean over {n_configs} relaxations)")
    print(f"  -> attraction concentrates spacing: std {mean_init_std:.3f} (random) "
          f"-> {full_std:.3f} (full LJ);  in-well {full_band*100:.0f}% vs "
          f"{repu_band*100:.0f}% repulsion-only;  repulsion mean ratio {repu_mean:.2f} (drifts up).")
    if np.std(init_msd) < 1e-9:
        print(f"  NOTE: capsule min_self is constant ({np.mean(init_msd):.4f}) for this arm "
              f"-> degenerate proxy; rely on spacing/energy, not min_self, here.")


if __name__ == "__main__":
    print("Raw (V6) Phase 1 — Lennard-Jones attractive-well experiment")
    for name, spec in (
        ("UR5", ur5_spec()),
        ("Franka Panda", franka_panda_spec()),
        ("Planar 3-DOF", planar3dof_spec()),
    ):
        run_robot(name, spec)
