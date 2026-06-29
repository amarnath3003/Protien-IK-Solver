"""
Raw (V6) Phase 1 tests — Lennard-Jones energy + analytic force.

Run from the backend/ directory:  pytest tests/test_raw_energy.py -v

Covers:
  - LJ pair potential shape: zero at σ, minimum −ε at 2^(1/6)·σ, attractive well
  - repulsion-only mode is non-negative and monotonically decreasing
  - analytic field gradient matches central finite differences (all 3 robots)
  - attraction actually changes the energy vs. repulsion-only
"""

from __future__ import annotations

import numpy as np
import pytest

from app.core.kinematics import ur5_spec, franka_panda_spec, planar3dof_spec
from app.solvers.protein_raw.energy import (
    lj_pair, lj_energy, lj_energy_and_grad,
)

SPECS = {
    "ur5": ur5_spec(),
    "franka_panda": franka_panda_spec(),
    "planar3dof": planar3dof_spec(),
}

WELL = 2.0 ** (1.0 / 6.0)   # d/σ at the LJ minimum


# ─── pair potential shape ─────────────────────────────────────────────────

def test_lj_pair_zero_at_sigma():
    assert lj_pair(1.0, sigma=1.0, epsilon=1.0) == pytest.approx(0.0, abs=1e-12)


def test_lj_pair_minimum_depth_and_location():
    sigma, eps = 0.3, 2.0
    d_min = WELL * sigma
    # depth is exactly −ε at the well
    assert lj_pair(d_min, sigma, eps) == pytest.approx(-eps, rel=1e-9)
    # it is a genuine minimum: lower than either side
    assert lj_pair(d_min, sigma, eps) < lj_pair(d_min * 0.9, sigma, eps)
    assert lj_pair(d_min, sigma, eps) < lj_pair(d_min * 1.2, sigma, eps)


def test_repulsion_only_nonnegative_and_decreasing():
    sigma = 0.5
    ds = np.linspace(0.4, 3.0, 50)
    vals = np.array([lj_pair(d, sigma, attractive=False) for d in ds])
    assert np.all(vals >= 0.0)
    assert np.all(np.diff(vals) <= 1e-12)   # monotonically non-increasing


def test_attraction_lowers_energy_beyond_sigma():
    # beyond σ the −(σ/d)^6 term is negative, so the full potential sits
    # strictly below the repulsion-only wall
    sigma = 0.5
    for d in (0.6, 0.9, 1.5):
        assert lj_pair(d, sigma, attractive=True) < lj_pair(d, sigma, attractive=False)


# ─── analytic gradient vs finite difference ───────────────────────────────

@pytest.mark.parametrize("robot", list(SPECS))
@pytest.mark.parametrize("attractive", [True, False])
def test_analytic_grad_matches_fd(robot, attractive):
    spec = SPECS[robot]
    rng = np.random.default_rng(7)
    sigma_scale, eps = 1.0, 1.0
    max_rel = 0.0
    for _ in range(5):
        q = spec.random_config(rng)
        _, g = lj_energy_and_grad(spec, q, sigma_scale, eps, attractive)
        # central finite difference of the scalar energy
        fd = np.zeros_like(g)
        h = 1e-6
        for i in range(spec.n_joints):
            qp, qm = q.copy(), q.copy()
            qp[i] += h
            qm[i] -= h
            ep = lj_energy(spec, qp, sigma_scale, eps, attractive)
            em = lj_energy(spec, qm, sigma_scale, eps, attractive)
            fd[i] = (ep - em) / (2 * h)
        denom = np.maximum(np.linalg.norm(g), 1e-6)
        max_rel = max(max_rel, np.linalg.norm(g - fd) / denom)
    assert max_rel < 1e-4, f"{robot} ({attractive=}): analytic≠FD, rel={max_rel:.2e}"


# ─── consistency between scalar and grad-returning energy ─────────────────

@pytest.mark.parametrize("robot", list(SPECS))
def test_energy_consistency(robot):
    spec = SPECS[robot]
    rng = np.random.default_rng(3)
    for _ in range(5):
        q = spec.random_config(rng)
        e_only = lj_energy(spec, q)
        e_grad, _ = lj_energy_and_grad(spec, q)
        assert e_only == pytest.approx(e_grad, rel=1e-12, abs=1e-12)
