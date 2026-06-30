"""
Raw (V6) Phase 3 tests — configurational entropy S = log Omega.

Run from backend/:  pytest tests/test_raw_entropy.py -v

Covers:
  - S <= 0 and is deterministic under a fixed stencil (common random numbers)
  - S is lower near a joint limit (less local accessible volume)
  - S is COLLISION-AWARE: lower for a near-self-collision config than an open one
    (this is what distinguishes it from manipulability, which ignores collision)
  - the FD gradient is a genuine ASCENT direction for S
  - target-blind: the API takes no target at all
"""

from __future__ import annotations

import inspect
import numpy as np
import pytest

from app.core.kinematics import (
    ur5_spec, planar3dof_spec, self_collision_min_distance,
)
from app.solvers.protein_raw.energy import (
    config_entropy, config_entropy_and_grad,
)


def test_entropy_nonpositive():
    spec = ur5_spec()
    rng = np.random.default_rng(0)
    for _ in range(5):
        q = spec.random_config(rng)
        assert config_entropy(spec, q, m=32) <= 1e-9


def test_entropy_deterministic_with_fixed_seed():
    spec = ur5_spec()
    q = spec.random_config(np.random.default_rng(1))
    a = config_entropy(spec, q, m=32, seed=7)
    b = config_entropy(spec, q, m=32, seed=7)
    assert a == b


def test_entropy_target_blind_signature():
    # the entropy must not depend on any target — its signature has no T_target
    params = inspect.signature(config_entropy).parameters
    assert not any("target" in p.lower() for p in params)


def test_entropy_lower_near_joint_limit_planar():
    spec = planar3dof_spec()
    lo, hi = spec.joint_limits[:, 0], spec.joint_limits[:, 1]
    mid = 0.5 * (lo + hi)            # center of the limit box — lots of freedom
    near = hi - 0.02                 # all joints pinned near the upper limit
    s_mid = config_entropy(spec, mid, m=64, rho=0.2)
    s_near = config_entropy(spec, near, m=64, rho=0.2)
    assert s_near < s_mid


def test_entropy_is_collision_aware_ur5():
    # the open config has much higher clearance; S must rank it above the clash
    spec = ur5_spec()
    rng = np.random.default_rng(3)
    qs = [spec.random_config(rng) for _ in range(200)]
    ds = [self_collision_min_distance(spec, q) for q in qs]
    q_open = qs[int(np.argmax(ds))]
    q_clash = qs[int(np.argmin(ds))]
    s_open = config_entropy(spec, q_open, m=64)
    s_clash = config_entropy(spec, q_clash, m=64)
    assert s_clash < s_open


def test_fd_gradient_is_ascent_direction_ur5():
    spec = ur5_spec()
    rng = np.random.default_rng(5)
    # start from a fairly constrained config so the gradient is non-trivial
    cand = [spec.random_config(rng) for _ in range(60)]
    q = min(cand, key=lambda c: self_collision_min_distance(spec, c))
    S, g = config_entropy_and_grad(spec, q, m=48)
    gn = np.linalg.norm(g)
    if gn < 1e-9:
        pytest.skip("flat region")
    q2 = spec.clip(q + 3e-3 * g / gn)              # step along +grad (ascent)
    S2 = config_entropy(spec, q2, m=48, seed=12345)
    assert S2 >= S - 1e-6
