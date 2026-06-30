"""
Raw (V6) Phase 2 tests — directional hydrogen-bond energy.

Run from backend/:  pytest tests/test_raw_hbond.py -v

Covers:
  - distance gate F(d) peaks at d0; angular gate peaks at aligned normals
  - backbone normal = triplet-plane normal (right-angle and collinear cases)
  - chains too short (planar 3-DOF) have no interior H-bond pairs → E_HB = 0
  - directionality only ever reduces |E| (angular gates in [0,1])
  - E_HB <= 0 and the FD gradient is a genuine descent direction
"""

from __future__ import annotations

import numpy as np
import pytest

from app.core.kinematics import ur5_spec, franka_panda_spec, planar3dof_spec
from app.solvers.protein_raw.energy import (
    hbond_energy, hbond_energy_and_grad, calibrate_hbond_d0,
    _bead_normals, _interior_pairs, _hb_distance_factor, _hb_angle_factor,
)

SPECS = {
    "ur5": ur5_spec(),
    "franka_panda": franka_panda_spec(),
    "planar3dof": planar3dof_spec(),
}


# ─── gate shapes ───────────────────────────────────────────────────────────

def test_distance_factor_peaks_at_d0():
    d0, sd = 0.3, 0.1
    assert _hb_distance_factor(d0, d0, sd) == pytest.approx(1.0)
    assert _hb_distance_factor(d0 + 0.2, d0, sd) < 1.0
    assert _hb_distance_factor(d0 - 0.2, d0, sd) < 1.0


def test_angle_factor_peaks_at_alignment():
    k = 3.0
    assert _hb_angle_factor(1.0, k) == pytest.approx(1.0)
    assert _hb_angle_factor(-1.0, k) == pytest.approx(1.0)   # |x| → both orientations
    assert _hb_angle_factor(0.0, k) == pytest.approx(np.exp(-k))
    assert _hb_angle_factor(0.5, k) < _hb_angle_factor(0.9, k)


# ─── backbone normal = triplet-plane normal ────────────────────────────────

def test_bead_normal_right_angle():
    pts = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [2, 1, 0]], float)
    t = _bead_normals(pts)
    # bead 1 triplet (p0,p1,p2): v1=(1,0,0), v2=(0,1,0) → normal ±z
    assert abs(t[1][2]) == pytest.approx(1.0)
    assert np.allclose(t[1][:2], 0.0)
    # endpoints have no normal
    assert np.allclose(t[0], 0.0) and np.allclose(t[3], 0.0)


def test_bead_normal_collinear_is_zero():
    pts = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]], float)
    t = _bead_normals(pts)
    assert np.allclose(t, 0.0)


# ─── short chain has no H-bonds ────────────────────────────────────────────

def test_planar_has_no_hbond_pairs():
    spec = planar3dof_spec()
    I, J = _interior_pairs(spec.n_joints + 1)
    assert I.size == 0
    rng = np.random.default_rng(0)
    assert hbond_energy(spec, spec.random_config(rng), d0=0.2, sigma_d=0.05) == 0.0


# ─── physical properties on real arms ──────────────────────────────────────

@pytest.mark.parametrize("robot", ["ur5", "franka_panda"])
def test_energy_nonpositive_and_directionality_reduces_magnitude(robot):
    spec = SPECS[robot]
    rng = np.random.default_rng(2)
    d0 = calibrate_hbond_d0(spec, rng)
    sd = 0.25 * d0
    for _ in range(8):
        q = spec.random_config(rng)
        e_dir = hbond_energy(spec, q, d0, sd, directional=True)
        e_iso = hbond_energy(spec, q, d0, sd, directional=False)
        assert e_dir <= 1e-12                       # E_HB <= 0
        assert e_iso <= 1e-12
        # angular gates ∈ [0,1] → directional energy is never MORE negative
        assert e_dir >= e_iso - 1e-9


@pytest.mark.parametrize("robot", ["ur5", "franka_panda"])
def test_fd_gradient_is_descent_direction(robot):
    spec = SPECS[robot]
    rng = np.random.default_rng(5)
    d0 = calibrate_hbond_d0(spec, rng)
    sd = 0.25 * d0
    checked = 0
    for _ in range(10):
        q = spec.random_config(rng)
        E, g = hbond_energy_and_grad(spec, q, d0, sd)
        gn = np.linalg.norm(g)
        if gn < 1e-7:
            continue
        q2 = spec.clip(q - 1e-4 * g / gn)           # tiny step along −grad
        E2 = hbond_energy(spec, q2, d0, sd)
        assert E2 <= E + 1e-7
        checked += 1
    assert checked > 0
