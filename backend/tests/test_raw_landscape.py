"""
Raw (V6) Phase 4 tests — Σ ratio + glass temperature.

Run from backend/:  pytest tests/test_raw_landscape.py -v

Covers:
  - the Σ measure itself: funnelled energy ensemble -> small Σ; glassy -> large Σ (>1 achievable)
  - sigma_ratio returns a positive, finite Σ on a reachable target
  - glass_temperature = sigma_E/sqrt(2 S0), positive and monotonic in sigma_E
  - configurational_entropy_scale > 0 for every robot
  - warm_start reduces task error (it is a usable native-energy proxy)
"""

from __future__ import annotations

import numpy as np
import pytest

from app.core.kinematics import (
    ur5_spec, franka_panda_spec, planar3dof_spec,
    end_effector_pose, pose_error,
)
from app.solvers.protein_raw.landscape import (
    RawParams, sigma_ratio, _sigma_from_energies, warm_start,
    configurational_entropy_scale, glass_temperature,
)

SPECS = {"ur5": ur5_spec(), "franka_panda": franka_panda_spec(),
         "planar3dof": planar3dof_spec()}


# ─── the Σ measure: funnel vs glass ────────────────────────────────────────

def test_sigma_funnel_smaller_than_glass():
    funnel = np.concatenate([[0.0], np.full(30, 10.0)])   # one deep minimum
    glass = np.linspace(0.0, 1.0, 31)                     # no gap above the min
    assert _sigma_from_energies(funnel)["sigma"] < _sigma_from_energies(glass)["sigma"]


def test_sigma_glassy_exceeds_one():
    glassy = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 5.0])     # min not isolated, high spread
    assert _sigma_from_energies(glassy)["sigma"] > 1.0


def test_sigma_funnelled_below_one():
    funnel = np.concatenate([[0.0], np.full(50, 8.0)])
    assert _sigma_from_energies(funnel)["sigma"] < 1.0


# ─── sigma_ratio on a real target ──────────────────────────────────────────

@pytest.mark.parametrize("robot", list(SPECS))
def test_sigma_ratio_positive_finite(robot):
    spec = SPECS[robot]
    rng = np.random.default_rng(0)
    p = RawParams.calibrate(spec, rng)
    T = end_effector_pose(spec, spec.random_config(rng))
    out = sigma_ratio(spec, T, p, rng, n_seeds=12)
    assert np.isfinite(out["sigma"]) and out["sigma"] > 0
    assert out["sigma_E"] >= 0 and out["delta_E"] >= -1e-9


# ─── glass temperature ─────────────────────────────────────────────────────

def test_glass_temperature_formula_and_monotonic():
    s0 = 20.0
    assert glass_temperature(0.4, s0) == pytest.approx(0.4 / np.sqrt(2 * s0))
    assert glass_temperature(0.8, s0) > glass_temperature(0.4, s0)
    assert glass_temperature(0.4, s0) > 0


@pytest.mark.parametrize("robot", list(SPECS))
def test_config_entropy_scale_positive(robot):
    assert configurational_entropy_scale(SPECS[robot]) > 0


# ─── warm-start native proxy ───────────────────────────────────────────────

@pytest.mark.parametrize("robot", list(SPECS))
def test_warm_start_reduces_task_error(robot):
    spec = SPECS[robot]
    rng = np.random.default_rng(2)
    improved = 0
    for _ in range(5):
        T = end_effector_pose(spec, spec.random_config(rng))
        q0 = spec.random_config(rng)
        e0 = np.linalg.norm(pose_error(end_effector_pose(spec, q0), T)[:3])
        q1 = warm_start(spec, q0, T)
        e1 = np.linalg.norm(pose_error(end_effector_pose(spec, q1), T)[:3])
        improved += int(e1 <= e0 + 1e-9)
    assert improved >= 4    # warm-start should help on almost all targets
