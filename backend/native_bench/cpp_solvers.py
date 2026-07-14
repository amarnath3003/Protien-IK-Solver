"""Adapters that make the NATIVE C++ ProteinIK solvers (pik_native, built from
backend/cpp/) callable with the repo's (spec, q0, T_target, rng) -> SolveResult
contract — so the master benchmark runs the C++ solvers exactly as it ran the
Python ones. Timing is measured in Python around the single C++ call, so the
Mean/p95/p99 ms columns reflect true native latency (apples-to-apples with the
compiled TRAC-IK / KDL / RTB baselines)."""
from __future__ import annotations

import sys
import time
import numpy as np

sys.path.insert(0, "/mnt/c/Coding Projects/Protien IK/backend/cpp")
import pik_native as pn  # noqa: E402

from app.core.kinematics import RobotSpec  # noqa: E402
from app.core.types import SolveResult  # noqa: E402

# Real-mesh-calibrated capsule radii (from solver_calib.py), keyed by DOF.
_CALIB_RADIUS = {
    6: [0.07, 0.06, 0.055, 0.05, 0.05, 0.045],
    7: [0.064, 0.054, 0.039, 0.039, 0.039, 0.034, 0.029],
}

_ROBOT_CACHE: dict[str, object] = {}
_CALIB_ROBOT_CACHE: dict[str, object] = {}


def _cpp_robot(spec: RobotSpec, radius=None):
    r = list(spec.link_radius) if radius is None else list(radius)
    return pn.make_robot(
        list(map(float, spec.a)), list(map(float, spec.d)), list(map(float, spec.alpha)),
        list(map(float, spec.theta_offset)),
        list(map(float, spec.joint_limits[:, 0])), list(map(float, spec.joint_limits[:, 1])),
        r, spec.dh_convention == "modified")


def _robot(spec: RobotSpec):
    if spec.name not in _ROBOT_CACHE:
        _ROBOT_CACHE[spec.name] = _cpp_robot(spec)
    return _ROBOT_CACHE[spec.name]


def _calib_robot(spec: RobotSpec):
    if spec.name not in _CALIB_ROBOT_CACHE:
        rad = _CALIB_RADIUS.get(spec.n_joints)  # planar (3) keeps default radii
        _CALIB_ROBOT_CACHE[spec.name] = _cpp_robot(spec, rad)
    return _CALIB_ROBOT_CACHE[spec.name]


def _seed(rng) -> int:
    return int(rng.integers(0, 2**63 - 1))


def _finalize(spec: RobotSpec, r: dict, wall_ms: float, name: str) -> SolveResult:
    q = np.asarray(r["q"], dtype=float)
    jlv = int(np.sum((q <= spec.joint_limits[:, 0] + 1e-9) |
                     (q >= spec.joint_limits[:, 1] - 1e-9)))
    return SolveResult(
        solver_name=name,
        success=bool(r["success"]),
        q_final=q.tolist(),
        pos_error=float(r["pos_error"]),
        orient_error=float(r["orient_error"]),
        iterations=int(r["iterations"]),
        wall_time_ms=float(wall_ms),
        min_self_distance=float(r["min_self_distance"]),
        joint_limit_violations=jlv,
        restarts=int(r["restarts"]),
        conflict_index=float(r.get("conflict_index", 0.0)),
        lambda_final=float(r.get("lambda_final", 1.0) or 1.0),
    )


def solve_v4_cpp(spec, q0, T_target, rng, collect_steps=False) -> SolveResult:
    R = _robot(spec); sd = _seed(rng)
    t0 = time.perf_counter()
    r = pn.solve_v4(R, np.asarray(q0, float), np.asarray(T_target, float), sd, False)
    return _finalize(spec, r, (time.perf_counter() - t0) * 1000.0, "protein_fast")


def solve_o2_cpp(spec, q0, T_target, rng, collect_steps=False) -> SolveResult:
    R = _robot(spec); sd = _seed(rng)
    t0 = time.perf_counter()
    r = pn.solve_v4(R, np.asarray(q0, float), np.asarray(T_target, float), sd, True)
    return _finalize(spec, r, (time.perf_counter() - t0) * 1000.0, "protein_fast_o2")


def solve_calib_cpp(spec, q0, T_target, rng, collect_steps=False) -> SolveResult:
    R = _calib_robot(spec); sd = _seed(rng)
    t0 = time.perf_counter()
    r = pn.solve_v4(R, np.asarray(q0, float), np.asarray(T_target, float), sd, False)
    return _finalize(spec, r, (time.perf_counter() - t0) * 1000.0, "protein_fast_calib")


def solve_v1_cpp(spec, q0, T_target, rng, collect_steps=False) -> SolveResult:
    R = _robot(spec); sd = _seed(rng)
    t0 = time.perf_counter()
    r = pn.solve_v1(R, np.asarray(q0, float), np.asarray(T_target, float), sd)
    return _finalize(spec, r, (time.perf_counter() - t0) * 1000.0, "protein_ik")


def solve_raw_cpp(spec, q0, T_target, rng, collect_steps=False) -> SolveResult:
    R = _robot(spec); sd = _seed(rng)
    t0 = time.perf_counter()
    r = pn.solve_raw(R, np.asarray(q0, float), np.asarray(T_target, float), sd)
    return _finalize(spec, r, (time.perf_counter() - t0) * 1000.0, "protein_raw")


# CCD / FABRIK are deterministic (no RNG); the seed arg is accepted but ignored
# by the C++ binding. Same in-repo algorithm as the Python, compiled — so the
# quality columns match the Python to float tolerance, only the timing is native.
def solve_ccd_cpp(spec, q0, T_target, rng, collect_steps=False) -> SolveResult:
    R = _robot(spec)
    t0 = time.perf_counter()
    r = pn.solve_ccd(R, np.asarray(q0, float), np.asarray(T_target, float), 0)
    return _finalize(spec, r, (time.perf_counter() - t0) * 1000.0, "ccd")


def solve_fabrik_cpp(spec, q0, T_target, rng, collect_steps=False) -> SolveResult:
    R = _robot(spec)
    t0 = time.perf_counter()
    r = pn.solve_fabrik(R, np.asarray(q0, float), np.asarray(T_target, float), 0)
    return _finalize(spec, r, (time.perf_counter() - t0) * 1000.0, "fabrik")
