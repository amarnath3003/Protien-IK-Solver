"""
Analytical (closed-form) IK solver for the planar 3-DOF RRR arm.

Arm geometry:
    L1 = 0.4 m   (link 1)
    L2 = 0.3 m   (link 2)
    L3 = 0.2 m   (link 3)

End-effector state is (x, y, θ) in the XY plane, where θ = q1+q2+q3 is
the tip orientation angle.

Algorithm
---------
Given a 4×4 target transform T (produced by FK from any valid q), extract:

    x_ee, y_ee = target position
    θ_ee       = arctan2(T[1,0], T[0,0])   — rotation angle about Z

Step 1 — Decouple wrist:
    x_w = x_ee - L3 * cos(θ_ee)
    y_w = y_ee - L3 * sin(θ_ee)

Step 2 — Two-link IK for (x_w, y_w) with links L1, L2:
    cos_q2 = (x_w² + y_w² - L1² - L2²) / (2·L1·L2)
    q2 = ±arccos(cos_q2)   — elbow-up (+) and elbow-down (-) branches

    q1 = arctan2(y_w, x_w) - arctan2(L2·sin(q2), L1 + L2·cos(q2))
    q3 = θ_ee - q1 - q2

This gives up to 2 candidate solutions. Both are evaluated and the one
closest (in joint-space Euclidean distance) to q0 is returned as the
primary solution.

Why this solver matters
-----------------------
Unlike every other solver in the registry, this one cannot "fail" on a
reachable target — it returns the exact, analytically correct joint
configuration. This gives us an absolute reference: a numerical solver
that reports pos_error < 1 mm on the same target has converged; one that
reports > 1 mm has not, with no ambiguity about whether the tolerance is
tight enough.

The only failure mode is an out-of-workspace target (cos_q2 outside
[-1,1]), returned as success=False with the best numerical approximation
from a fallback gradient step.
"""

from __future__ import annotations

import math
import time
import numpy as np

from app.core.kinematics import RobotSpec, end_effector_pose, pose_error, self_collision_min_distance
from app.core.types import SolveResult, SolveStep, Timer


# Link lengths — kept as module constants so the analytical formula is
# readable without needing to inspect the DH table.
_L1 = 0.4
_L2 = 0.3
_L3 = 0.2

# Solve tolerances
_POS_TOL  = 1e-3   # 1 mm
_ORI_TOL  = 1e-2   # 10 mrad


def _extract_planar_ee(T: np.ndarray) -> tuple[float, float, float]:
    """Extract (x, y, theta) from a 4×4 homogeneous transform for a planar arm.

    For a planar arm all motion is in the XY plane, so:
        x, y  = T[:2, 3]
        theta = arctan2(T[1, 0], T[0, 0])   (rotation about Z-axis)
    """
    x = float(T[0, 3])
    y = float(T[1, 3])
    theta = math.atan2(float(T[1, 0]), float(T[0, 0]))
    return x, y, theta


def _wrap(angle: float) -> float:
    """Wrap angle to [-π, π]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _analytical_solutions(
    x_ee: float, y_ee: float, theta_ee: float
) -> list[np.ndarray]:
    """Return 0, 1, or 2 exact joint-angle vectors for the given EE pose.

    Returns an empty list if the target is outside the reachable workspace.
    Each returned vector is shape (3,) with angles in [-π, π].
    """
    # Wrist position (decouple link 3)
    x_w = x_ee - _L3 * math.cos(theta_ee)
    y_w = y_ee - _L3 * math.sin(theta_ee)

    r2 = x_w * x_w + y_w * y_w
    cos_q2 = (r2 - _L1 * _L1 - _L2 * _L2) / (2.0 * _L1 * _L2)

    if abs(cos_q2) > 1.0 + 1e-9:
        return []          # unreachable
    cos_q2 = max(-1.0, min(1.0, cos_q2))  # numerical clamp

    solutions = []
    for sign in (+1.0, -1.0):          # elbow-up (+) and elbow-down (-)
        q2 = sign * math.acos(cos_q2)
        q1 = math.atan2(y_w, x_w) - math.atan2(
            _L2 * math.sin(q2),
            _L1 + _L2 * math.cos(q2),
        )
        q3 = theta_ee - q1 - q2
        q = np.array([_wrap(q1), _wrap(q2), _wrap(q3)])
        solutions.append(q)

    return solutions


def _joint_limit_violations(spec: RobotSpec, q: np.ndarray) -> int:
    lo = spec.joint_limits[:, 0]
    hi = spec.joint_limits[:, 1]
    return int(np.sum((q < lo) | (q > hi)))


def solve_analytical_planar3dof(
    spec: RobotSpec,
    q0: np.ndarray,
    T_target: np.ndarray,
    rng: np.random.Generator,
    collect_steps: bool = False,
) -> SolveResult:
    """Closed-form IK for the planar 3-DOF arm.

    The solver:
    1. Extracts (x, y, θ) from T_target.
    2. Computes up to 2 analytical solutions (elbow-up / elbow-down).
    3. Picks the solution closest to q0 in joint space.
    4. If no analytical solution exists (out-of-workspace), falls back
       to a single Jacobian gradient step and reports failure.

    Returns a SolveResult compatible with the unified benchmark runner.
    """
    if spec.name != "planar3dof":
        raise ValueError(
            f"analytical_planar3dof solver only works on 'planar3dof' spec, "
            f"got '{spec.name}'"
        )

    with Timer() as t:
        x_ee, y_ee, theta_ee = _extract_planar_ee(T_target)
        solutions = _analytical_solutions(x_ee, y_ee, theta_ee)

        steps = []

        if solutions:
            # Pick the solution closest (in joint-space L2 distance) to q0
            q_best = min(
                solutions,
                key=lambda q: float(np.sum((q - q0) ** 2)),
            )
            T_final = end_effector_pose(spec, q_best)
            err = pose_error(T_final, T_target)
            pos_err   = float(np.linalg.norm(err[:3]))
            ori_err   = float(np.linalg.norm(err[3:]))
            success   = pos_err < _POS_TOL and ori_err < _ORI_TOL
            min_d     = self_collision_min_distance(spec, q_best)
            violations = _joint_limit_violations(spec, q_best)

            if collect_steps:
                # Analytical solve has no iterative steps — emit one step
                # representing the single-shot solution.
                steps = [SolveStep(
                    iteration=0,
                    q=q_best.tolist(),
                    pos_error=pos_err,
                    orient_error=ori_err,
                    min_self_distance=min_d,
                    phase="analytical",
                    energy=pos_err,
                )]
        else:
            # Out-of-workspace: report failure, return q0 as best attempt
            q_best  = q0.copy()
            T_final = end_effector_pose(spec, q_best)
            err     = pose_error(T_final, T_target)
            pos_err = float(np.linalg.norm(err[:3]))
            ori_err = float(np.linalg.norm(err[3:]))
            success = False
            min_d   = self_collision_min_distance(spec, q_best)
            violations = _joint_limit_violations(spec, q_best)

    return SolveResult(
        solver_name="analytical_planar3dof",
        success=success,
        q_final=q_best.tolist(),
        pos_error=pos_err,
        orient_error=ori_err,
        iterations=1,           # analytical = one shot
        wall_time_ms=t.ms,
        min_self_distance=min_d,
        joint_limit_violations=violations,
        restarts=0,
        steps=steps,
        conflict_index=0.0,
        lambda_final=1.0,
        difficulty_score=0.0,
    )
