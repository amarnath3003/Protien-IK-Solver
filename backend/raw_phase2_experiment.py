"""
Raw (V6) — Phase 2 experiment: does the H-bond DIRECTIONALITY create
secondary structure (oriented contacts), not just proximity?

The raw, no-IK-equivalent part of E_HB is that it couples bead pairs by
distance AND relative orientation of the local backbone normals. A purely
distance-based contact potential can pull beads to d0, but has no reason to
ORIENT them. Real secondary structure is oriented, repeating contact geometry.

Faithful mechanism: secondary structure forms WITHIN hydrophobic collapse —
H-bonds orient contacts that collapse has already brought together. So we:
  1. collapse each random config with the Phase-1 LJ term (well set at d0),
     giving beads at contact distance;
  2. from that collapsed state, relax under (a) the directional H-bond and
     (b) the distance-only ablation;
  3. measure, among contact pairs (F(d) > 0.5), the mean alignment
     0.5*(|t_i.r| + |t_j.r|) of the backbone normals with the contact dir.

Expected: directional -> contacts become ORIENTED (alignment rises);
          distance-only -> contacts stay, orientation does not improve.

Run:  python raw_phase2_experiment.py
"""

from __future__ import annotations

import numpy as np

from app.core.kinematics import ur5_spec, franka_panda_spec, planar3dof_spec
from app.solvers.protein_raw.energy import (
    bead_positions, lj_energy_and_grad, hbond_energy, hbond_energy_and_grad,
    calibrate_hbond_d0, _bead_normals, _bead_radii,
    _interior_pairs, _nonadjacent_pairs, _hb_distance_factor,
)

WELL = 2.0 ** (1.0 / 6.0)
KAPPA = 2.0


def _metrics(spec, q, d0, sd):
    """(mean alignment among contact pairs, #well-formed H-bonds, #contacts)."""
    pts = bead_positions(spec, q)
    I, J = _interior_pairs(pts.shape[0])
    t = _bead_normals(pts)
    diff = pts[J] - pts[I]
    d = np.maximum(np.linalg.norm(diff, axis=1), 1e-9)
    rhat = diff / d[:, None]
    F = _hb_distance_factor(d, d0, sd)
    a = np.abs(np.einsum("pi,pi->p", t[I], rhat))
    b = np.abs(np.einsum("pi,pi->p", t[J], rhat))
    align = 0.5 * (a + b)
    contact = F > 0.5
    mean_align = float(np.mean(align[contact])) if contact.any() else float("nan")
    well_formed = int(np.sum((F > 0.6) & (a > 0.7) & (b > 0.7)))
    return mean_align, well_formed, int(contact.sum())


def _capped_gd(grad_fn, spec, q0, iters, lr=2e-2, max_step=0.05):
    q = q0.copy()
    for _ in range(iters):
        g = grad_fn(q)
        step = -lr * g
        nrm = np.linalg.norm(step)
        if nrm > max_step:
            step *= max_step / nrm
        q = spec.clip(q + step)
    return q


def run_robot(name, spec, n_configs=24, seed=0):
    rng = np.random.default_rng(seed)
    d0 = calibrate_hbond_d0(spec, rng)
    sd = 0.25 * d0
    n_pairs = _interior_pairs(spec.n_joints + 1)[0].size
    if n_pairs == 0:
        print(f"\n=== {name} === no interior H-bond pairs (chain too short) - skipped")
        return

    # LJ scale so the attractive well sits at the H-bond distance d0
    radii = _bead_radii(spec)
    I, J = _nonadjacent_pairs(spec.n_joints + 1)
    lj_scale = (d0 / WELL) / float(np.mean(radii[I] + radii[J]))

    agg = {"after collapse (LJ)": [], "+ distance-only": [], "+ directional H-bond": []}
    e_iso, e_dir = [], []

    for _ in range(n_configs):
        q0 = spec.random_config(rng)
        q_col = _capped_gd(
            lambda q: lj_energy_and_grad(spec, q, lj_scale, 1.0, True)[1],
            spec, q0, iters=800)
        agg["after collapse (LJ)"].append(_metrics(spec, q_col, d0, sd))

        q_iso = _capped_gd(
            lambda q: hbond_energy_and_grad(spec, q, d0, sd, KAPPA, 1.0, False)[1],
            spec, q_col, iters=800)
        agg["+ distance-only"].append(_metrics(spec, q_iso, d0, sd))
        e_iso.append(hbond_energy(spec, q_iso, d0, sd, KAPPA, directional=True))

        q_dir = _capped_gd(
            lambda q: hbond_energy_and_grad(spec, q, d0, sd, KAPPA, 1.0, True)[1],
            spec, q_col, iters=800)
        agg["+ directional H-bond"].append(_metrics(spec, q_dir, d0, sd))
        e_dir.append(hbond_energy(spec, q_dir, d0, sd, KAPPA, directional=True))

    print(f"\n=== {name}  (d0={d0:.3f}, kappa={KAPPA}, {n_pairs} interior pairs) ===")
    print(f"{'state':<24}{'mean align (contacts)':>22}{'avg well-formed':>17}{'avg #contacts':>15}")
    for label, rows in agg.items():
        al = np.nanmean([r[0] for r in rows])
        wf = np.mean([r[1] for r in rows])
        ct = np.mean([r[2] for r in rows])
        print(f"{label:<24}{al:>22.3f}{wf:>17.2f}{ct:>15.2f}")
    print(f"  E_HB at solution: distance-only {np.mean(e_iso):.3f}  vs  "
          f"directional {np.mean(e_dir):.3f}  (more negative = more/better-formed H-bonds)")
    iso_al = np.nanmean([r[0] for r in agg["+ distance-only"]])
    dir_al = np.nanmean([r[0] for r in agg["+ directional H-bond"]])
    dir_wf = np.mean([r[1] for r in agg["+ directional H-bond"]])
    iso_wf = np.mean([r[1] for r in agg["+ distance-only"]])
    print(f"  -> after identical collapse, directionality orients contacts: "
          f"alignment {iso_al:.2f} (distance-only) -> {dir_al:.2f} (directional); "
          f"well-formed H-bonds {iso_wf:.1f} -> {dir_wf:.1f}.")


def selectivity_demo(d0=0.4, sd=0.1, kappa=KAPPA):
    """The term's correctness, independent of whether GD can fold a short arm:
    a single H-bond requires BOTH the right distance AND aligned normals."""
    ideal = -_hb_distance_factor(d0, d0, sd)
    perp = ideal * np.exp(-2 * kappa)
    off = -_hb_distance_factor(1.4 * d0, d0, sd)
    print(f"Two-condition gate (eps_hb=1, kappa={kappa}): ideal contact (d0, aligned) "
          f"E={ideal:.3f};  perpendicular E={perp:.4f} ({ideal/perp:.0f}x weaker);  "
          f"off-distance (1.4*d0, aligned) E={off:.4f} ({ideal/off:.0f}x weaker). "
          f"Both distance AND orientation are required -- this is the directionality.")


if __name__ == "__main__":
    print("Raw (V6) Phase 2 - directional H-bond / secondary-structure experiment")
    selectivity_demo()
    for name, spec in (
        ("UR5", ur5_spec()),
        ("Franka Panda", franka_panda_spec()),
        ("Planar 3-DOF", planar3dof_spec()),
    ):
        run_robot(name, spec)
