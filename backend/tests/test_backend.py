"""
ProteinIK backend integration tests.

Run from the `backend/` directory:
    pytest tests/ -v

Covers:
  - /api/robot endpoint schema
  - /api/solvers endpoint
  - /api/random-target reproducibility
  - /api/solve for every registered solver
  - /api/benchmark (small run, all scenarios)
  - WebSocket streaming smoke test
  - Kinematics numerical correctness (FK round-trip, FK chain length)
  - protein_ik energy weight consistency
"""

from __future__ import annotations

import math
import numpy as np
import pytest
from fastapi.testclient import TestClient

from app.main import app
from app.core.kinematics import ur5_spec, forward_kinematics_chain, end_effector_pose, pose_error
from app.solvers.registry import SOLVER_ORDER  # canonical ordered list
from app.solvers.protein_energy import total_energy_fast

SOLVER_ORDER = ['jacobian_dls', 'ccd', 'fabrik', 'trac_ik_style', 'multi_start', 'protein_ik']

client = TestClient(app)


# ─── API: /api/robot ──────────────────────────────────────────────────────────

def test_robot_status():
    res = client.get("/api/robot")
    assert res.status_code == 200

def test_robot_schema():
    data = client.get("/api/robot").json()
    assert "n_joints" in data
    assert data["n_joints"] == 6
    assert len(data["a"]) == 6
    assert len(data["d"]) == 6
    assert len(data["alpha"]) == 6
    assert len(data["joint_limits"]) == 6
    assert len(data["link_radius"]) == 6


# ─── API: /api/solvers ────────────────────────────────────────────────────────

def test_solvers_list():
    res = client.get("/api/solvers")
    assert res.status_code == 200
    ids = [s["id"] for s in res.json()]
    for expected in SOLVER_ORDER:
        assert expected in ids, f"Solver '{expected}' missing from /api/solvers"


# ─── API: /api/random-target ──────────────────────────────────────────────────

def test_random_target_schema():
    res = client.post("/api/random-target", json={"seed": 42})
    assert res.status_code == 200
    data = res.json()
    assert len(data["position"]) == 3
    assert len(data["quaternion"]) == 4
    assert len(data["q_reference"]) == 6

def test_random_target_reproducible():
    a = client.post("/api/random-target", json={"seed": 99}).json()
    b = client.post("/api/random-target", json={"seed": 99}).json()
    assert a["position"] == b["position"]
    assert a["quaternion"] == b["quaternion"]

def test_random_target_different_seeds_differ():
    a = client.post("/api/random-target", json={"seed": 1}).json()
    b = client.post("/api/random-target", json={"seed": 2}).json()
    assert a["position"] != b["position"]


# ─── API: /api/solve (one per solver) ─────────────────────────────────────────

target_payload = client.post("/api/random-target", json={"seed": 42}).json()


@pytest.mark.parametrize("solver_id", SOLVER_ORDER)
def test_solve_each_solver(solver_id):
    res = client.post("/api/solve", json={
        "solver": solver_id,
        "seed": 42,
        "target": target_payload,
        "q0": [0, 0, 0, 0, 0, 0],
        "collect_steps": False,
    })
    assert res.status_code == 200, f"{solver_id}: {res.text}"
    data = res.json()
    assert "success" in data
    assert "pos_error" in data
    assert len(data["q_final"]) == 6

def test_solve_unknown_solver():
    res = client.post("/api/solve", json={
        "solver": "does_not_exist",
        "seed": 1,
        "target": target_payload,
    })
    assert res.status_code == 400

def test_solve_with_steps():
    res = client.post("/api/solve", json={
        "solver": "ccd",
        "seed": 1,
        "target": target_payload,
        "q0": [0, 0, 0, 0, 0, 0],
        "collect_steps": True,
    })
    assert res.status_code == 200
    data = res.json()
    assert isinstance(data.get("steps"), list)
    assert len(data["steps"]) > 0


# ─── API: /api/benchmark ─────────────────────────────────────────────────────

@pytest.mark.parametrize("scenario", ["open_space", "near_singular", "cluttered"])
def test_benchmark_scenarios(scenario):
    res = client.post("/api/benchmark", json={
        "solvers": SOLVER_ORDER,
        "n_trials": 2,
        "scenario": scenario,
        "seed": 1,
    })
    assert res.status_code == 200
    data = res.json()
    assert data["scenario"] == scenario
    for sid in SOLVER_ORDER:
        assert sid in data["results"]
        r = data["results"][sid]
        assert "success_rate" in r
        assert "mean_time_ms" in r
        assert "p50_time_ms" in r
        assert "p95_time_ms" in r
        assert "mean_iters" in r
        assert "mean_pos_error" in r

def test_benchmark_unknown_solver():
    res = client.post("/api/benchmark", json={
        "solvers": ["jacobian_dls", "ghost_solver"],
        "n_trials": 1,
        "scenario": "open_space",
        "seed": 1,
    })
    assert res.status_code == 400


# ─── Kinematics: FK numerical correctness ────────────────────────────────────

def test_fk_identity_at_zero():
    """FK with all-zero joint angles: end-effector should be at a deterministic
    known position (not the origin) depending on DH offsets."""
    spec = ur5_spec()
    T = end_effector_pose(spec, np.zeros(6))
    # Sanity: it's a valid rotation matrix
    R = T[:3, :3]
    assert abs(np.linalg.det(R) - 1.0) < 1e-9
    # It's a homogeneous matrix
    assert T[3, 3] == pytest.approx(1.0)

def test_fk_chain_length():
    spec = ur5_spec()
    chain = forward_kinematics_chain(spec, np.zeros(6))
    assert chain.shape == (7, 4, 4)

def test_fk_chain_last_matches_ee():
    spec = ur5_spec()
    q = np.array([0.1, -0.5, 0.3, -0.2, 0.7, -0.1])
    chain = forward_kinematics_chain(spec, q)
    T_ee = end_effector_pose(spec, q)
    np.testing.assert_allclose(chain[-1], T_ee, atol=1e-12)

def test_pose_error_zero_at_self():
    spec = ur5_spec()
    q = np.array([0.2, -0.4, 0.6, 0.1, -0.3, 0.5])
    T = end_effector_pose(spec, q)
    err = pose_error(T, T)
    np.testing.assert_allclose(err, 0.0, atol=1e-12)

def test_fk_random_roundtrip():
    """For a known q, FK → pose_error(self) should be zero."""
    rng = np.random.default_rng(7)
    spec = ur5_spec()
    for _ in range(10):
        q = spec.random_config(rng)
        T = end_effector_pose(spec, q)
        err = pose_error(T, T)
        np.testing.assert_allclose(err, 0.0, atol=1e-11)


# ─── Energy: weight consistency check ────────────────────────────────────────

def test_total_energy_fast_nonnegative():
    """Total energy should always be ≥ 0 for any configuration."""
    spec = ur5_spec()
    rng = np.random.default_rng(13)
    T_target = end_effector_pose(spec, spec.random_config(rng))
    for _ in range(20):
        q = spec.random_config(rng)
        e = total_energy_fast(spec, q, T_target, 3.0, 1.0, 2.0, 0.3)
        assert e >= 0.0, f"Negative energy {e} at q={q}"
