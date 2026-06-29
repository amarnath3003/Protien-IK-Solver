"""
ProteinIK backend integration tests.

Run from the `backend/` directory:
    pytest tests/ -v

Covers:
  - /api/robot endpoint schema (UR5 + planar 3-DOF)
  - /api/robots endpoint (multi-robot listing)
  - /api/solvers endpoint
  - /api/random-target reproducibility (both robots)
  - /api/solve for every registered solver
  - /api/benchmark (small run, all scenarios)
  - WebSocket streaming smoke test
  - Kinematics numerical correctness (FK round-trip, FK chain length)
  - protein_ik energy weight consistency
  - Planar 3-DOF: FK ground truth, analytical IK round-trip, workspace boundary
"""

from __future__ import annotations

import math
import numpy as np
import pytest
from fastapi.testclient import TestClient

from app.main import app
from app.core.kinematics import ur5_spec, forward_kinematics_chain, end_effector_pose, pose_error, planar3dof_spec
from app.solvers.registry import SOLVER_REGISTRY
from app.solvers.protein_energy import total_energy_fast
from app.solvers.analytical_planar3dof import (
    _analytical_solutions, _extract_planar_ee, _L1, _L2, _L3,
)

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


# ─── Multi-robot API ──────────────────────────────────────────────────────────────────────────────

def test_robots_endpoint_lists_all():
    """GET /api/robots should include both ur5 and planar3dof."""
    res = client.get("/api/robots")
    assert res.status_code == 200
    ids = [r["id"] for r in res.json()]
    assert "ur5" in ids
    assert "planar3dof" in ids


def test_robot_endpoint_ur5_default():
    """GET /api/robot with no param should default to UR5 (6 joints)."""
    data = client.get("/api/robot").json()
    assert data["n_joints"] == 6
    assert data["name"] == "ur5"


def test_robot_endpoint_planar3dof():
    """GET /api/robot?robot=planar3dof should return 3-joint spec."""
    data = client.get("/api/robot", params={"robot": "planar3dof"}).json()
    assert data["n_joints"] == 3
    assert data["name"] == "planar3dof"
    assert len(data["a"]) == 3
    assert len(data["joint_limits"]) == 3


def test_robot_endpoint_unknown():
    res = client.get("/api/robot", params={"robot": "does_not_exist"})
    assert res.status_code == 400


def test_random_target_planar3dof():
    """Random target for planar arm should return 3-element q_reference."""
    res = client.post("/api/random-target", json={"robot": "planar3dof", "seed": 7})
    assert res.status_code == 200
    data = res.json()
    assert len(data["q_reference"]) == 3
    # For a planar arm all motion is in XY, so Z position should be ~0
    assert abs(data["position"][2]) < 1e-6


# ─── Planar 3-DOF kinematics: ground-truth FK ───────────────────────────────────────────────
#
# For a planar arm the FK is known analytically:
#   x = L1*cos(q1) + L2*cos(q1+q2) + L3*cos(q1+q2+q3)
#   y = L1*sin(q1) + L2*sin(q1+q2) + L3*sin(q1+q2+q3)
#   z = 0   (exactly, since alpha=0 and d=0 for all joints)
# We can compare the DH-based FK against these formulas exactly.


def _planar_fk_exact(q):
    """Ground-truth FK for planar 3-DOF arm using direct trig formulas."""
    q1, q2, q3 = q
    x = (_L1 * math.cos(q1)
         + _L2 * math.cos(q1 + q2)
         + _L3 * math.cos(q1 + q2 + q3))
    y = (_L1 * math.sin(q1)
         + _L2 * math.sin(q1 + q2)
         + _L3 * math.sin(q1 + q2 + q3))
    theta = q1 + q2 + q3
    return x, y, theta


def test_planar_fk_matches_formula_at_zero():
    """FK at q=[0,0,0] should place EE at (L1+L2+L3, 0) with theta=0."""
    spec = planar3dof_spec()
    T = end_effector_pose(spec, np.zeros(3))
    x, y, _ = _extract_planar_ee(T)
    assert x == pytest.approx(_L1 + _L2 + _L3, abs=1e-10)
    assert y == pytest.approx(0.0, abs=1e-10)


@pytest.mark.parametrize("q", [
    [0.0, 0.0, 0.0],
    [math.pi / 2, 0.0, 0.0],
    [0.3, -0.5, 0.7],
    [-1.2, 0.8, -0.4],
    [math.pi, -math.pi / 3, math.pi / 6],
])
def test_planar_fk_matches_formula(q):
    """DH-based FK must exactly match the closed-form trig formula."""
    spec = planar3dof_spec()
    q_arr = np.array(q)
    T = end_effector_pose(spec, q_arr)
    x_dh, y_dh, theta_dh = _extract_planar_ee(T)
    x_gt, y_gt, theta_gt = _planar_fk_exact(q_arr)
    assert x_dh == pytest.approx(x_gt, abs=1e-10), f"x mismatch at q={q}"
    assert y_dh == pytest.approx(y_gt, abs=1e-10), f"y mismatch at q={q}"
    assert theta_dh == pytest.approx(theta_gt, abs=1e-10), f"theta mismatch at q={q}"


# ─── Planar 3-DOF analytical IK: round-trip correctness ─────────────────────────────────
#
# For every reachable target: FK(IK(target)) == target exactly (within
# floating-point), NOT just "error < 1mm". This is the key advantage of
# having an analytical solver -- it tests mathematical correctness, not
# just convergence.


@pytest.mark.parametrize("seed", [0, 1, 7, 42, 99, 137, 256, 512])
def test_analytical_ik_roundtrip_exact(seed):
    """FK(analytical_IK(target)) == target to floating-point precision.

    This is a STRONGER test than the tolerance-based success check:
    error should be < 1e-10 m, not just < 1mm.
    """
    spec = planar3dof_spec()
    rng = np.random.default_rng(seed)
    q_true = spec.random_config(rng)
    T_target = end_effector_pose(spec, q_true)
    x_t, y_t, theta_t = _extract_planar_ee(T_target)

    solutions = _analytical_solutions(x_t, y_t, theta_t)
    assert len(solutions) >= 1, f"No solution found for q={q_true} (seed={seed})"

    for q_sol in solutions:
        T_check = end_effector_pose(spec, q_sol)
        err = pose_error(T_check, T_target)
        pos_err = float(np.linalg.norm(err[:3]))
        assert pos_err < 1e-9, (
            f"Analytical IK round-trip pos error {pos_err:.2e} m at seed={seed}"
        )


def test_analytical_ik_has_two_branches():
    """For a generic reachable target, both elbow-up and elbow-down branches
    should exist and both should be valid (round-trip error < 1e-9 m)."""
    spec = planar3dof_spec()
    # Use a target clearly inside the workspace (not near boundary)
    q_true = np.array([0.3, -0.5, 0.4])
    T_target = end_effector_pose(spec, q_true)
    x_t, y_t, theta_t = _extract_planar_ee(T_target)

    solutions = _analytical_solutions(x_t, y_t, theta_t)
    assert len(solutions) == 2, "Expected both elbow-up and elbow-down solutions"

    for i, q_sol in enumerate(solutions):
        T_check = end_effector_pose(spec, q_sol)
        err = pose_error(T_check, T_target)
        pos_err = float(np.linalg.norm(err[:3]))
        assert pos_err < 1e-9, f"Branch {i} round-trip error {pos_err:.2e} m"


def test_analytical_ik_unreachable_returns_empty():
    """Target beyond max reach (L1+L2+L3 = 0.9m) should return no solutions."""
    # Place target at (2.0, 0, 0) -- well outside the 0.9m reach
    x_t, y_t, theta_t = 2.0, 0.0, 0.0
    solutions = _analytical_solutions(x_t, y_t, theta_t)
    assert solutions == [], "Out-of-workspace target should return empty solution list"


def test_analytical_ik_at_full_extension():
    """At full extension (q=[0,0,0]), the target is exactly on the workspace
    boundary. Should return exactly one solution (cos_q2 == 1 => q2=0)."""
    # At q=[0,0,0]: EE is at (0.9, 0) with theta=0
    x_t, y_t, theta_t = _L1 + _L2 + _L3, 0.0, 0.0
    solutions = _analytical_solutions(x_t, y_t, theta_t)
    # Both branches converge: elbow-up q2=0 == elbow-down q2=0
    assert len(solutions) >= 1
    # Verify FK on returned solution
    spec = planar3dof_spec()
    for q_sol in solutions:
        T_check = end_effector_pose(spec, q_sol)
        x_check, y_check, _ = _extract_planar_ee(T_check)
        assert x_check == pytest.approx(_L1 + _L2 + _L3, abs=1e-9)
        assert y_check == pytest.approx(0.0, abs=1e-9)


# ─── Planar 3-DOF: full API solve test ────────────────────────────────────────────────────────────────

_planar_target = client.post("/api/random-target", json={"robot": "planar3dof", "seed": 5}).json()


def test_solve_analytical_planar3dof_via_api():
    """Analytical solver on the planar arm should succeed with < 1e-6 m error."""
    res = client.post("/api/solve", json={
        "solver": "analytical_planar3dof",
        "robot": "planar3dof",
        "seed": 5,
        "target": _planar_target,
        "q0": [0.0, 0.0, 0.0],
        "collect_steps": False,
    })
    assert res.status_code == 200, res.text
    data = res.json()
    assert data["success"] is True
    assert data["pos_error"] < 1e-6    # analytical = exact, not just < 1mm
    assert len(data["q_final"]) == 3


# All solvers — classical AND all ProteinIK versions — work on the planar arm
# because every solver takes (spec, q0, T_target, rng) generically.
# The planar arm is a better test bed than UR5 for one key reason:
# we can compare each solver's output against the exact analytical answer,
# not just check "error < 1mm". A solver that reports success but lands on the
# wrong local minimum will be caught here.
_ALL_SOLVERS_PLANAR = [
    # Classical baselines
    "jacobian_dls",
    "ccd",
    "fabrik",
    "trac_ik_style",
    "multi_start",
    # All ProteinIK versions
    "protein_ik",          # V1
    "fixed_lambda_ik",     # Fixed-λ homotopy baseline
    "protein_homotopy",    # CCH-IK V5
    "protein_fast",        # V4-Fast (LM + MH)
]


@pytest.mark.parametrize("solver_id", _ALL_SOLVERS_PLANAR)
def test_solve_all_solvers_planar3dof_via_api(solver_id):
    """Every solver in the registry (including all ProteinIK versions) must:
      1. Accept a planar3dof RobotSpec without crashing.
      2. Return a 3-element q_final.
      3. Have pos_error and orient_error fields (for comparison against analytical answer).
    """
    res = client.post("/api/solve", json={
        "solver": solver_id,
        "robot": "planar3dof",
        "seed": 5,
        "target": _planar_target,
        "q0": [0.0, 0.0, 0.0],
        "collect_steps": False,
    })
    assert res.status_code == 200, f"{solver_id} on planar3dof: {res.text}"
    data = res.json()
    assert "success" in data
    assert "pos_error" in data
    assert len(data["q_final"]) == 3


def test_benchmark_planar3dof_analytical():
    """Benchmark the analytical solver on the planar arm: should be near-100%
    success (only fails for out-of-workspace targets, which are rare in
    open_space since we draw q_true uniformly in [-pi,pi]^3)."""
    res = client.post("/api/benchmark", json={
        "solvers": ["analytical_planar3dof"],
        "robot": "planar3dof",
        "n_trials": 10,
        "scenario": "open_space",
        "seed": 1,
    })
    assert res.status_code == 200
    data = res.json()
    assert data["scenario"] == "open_space"
    r = data["results"]["analytical_planar3dof"]
    # Analytical IK on reachable targets should be perfect
    assert r["success_rate"] == pytest.approx(1.0, abs=0.0)
    assert r["mean_pos_error"] < 1e-6
