"""
Jacobian-based Damped Least Squares (DLS) IK solver.

The standard numerical baseline used across the IK literature. Handles
near-singularities better than plain pseudoinverse via the damping term,
but remains a single-trajectory, locally-linearized method: it commits to
one basin from its initial guess and has no mechanism to recover from
joint-limit lockup or local minima on its own (this is exactly the
"standard numerical solvers rely on local linearization and are
inherently sensitive" limitation noted in the literature).
"""

from __future__ import annotations

import numpy as np
import time

from app.core.kinematics import (
    RobotSpec, end_effector_pose, geometric_jacobian, pose_error,
    self_collision_min_distance,
)
from app.core.types import SolveResult, SolveStep


def solve_dls(
    spec: RobotSpec,
    q0: np.ndarray,
    T_target: np.ndarray,
    max_iters: int = 200,
    pos_tol: float = 1e-3,
    orient_tol: float = 1e-2,
    damping: float = 0.05,
    step_scale: float = 1.0,
    collect_steps: bool = False,
) -> SolveResult:
    q = q0.copy()
    steps = []
    t0 = time.perf_counter()
    success = False
    it = 0

    for it in range(1, max_iters + 1):
        T_cur = end_effector_pose(spec, q)
        err = pose_error(T_cur, T_target)
        pos_e = float(np.linalg.norm(err[:3]))
        orient_e = float(np.linalg.norm(err[3:]))

        if collect_steps:
            steps.append(SolveStep(
                iteration=it, q=q.tolist(), pos_error=pos_e, orient_error=orient_e,
                min_self_distance=self_collision_min_distance(spec, q), phase="dls",
            ))

        if pos_e < pos_tol and orient_e < orient_tol:
            success = True
            break

        J = geometric_jacobian(spec, q)
        # damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 * err
        JJt = J @ J.T
        lam2 = damping ** 2
        dq = J.T @ np.linalg.solve(JJt + lam2 * np.eye(6), err)
        q = spec.clip(q + step_scale * dq)

    T_final = end_effector_pose(spec, q)
    err_final = pose_error(T_final, T_target)
    wall_ms = (time.perf_counter() - t0) * 1000.0
    violations = int(np.sum((q <= spec.joint_limits[:, 0] + 1e-9) |
                             (q >= spec.joint_limits[:, 1] - 1e-9)))

    return SolveResult(
        solver_name="jacobian_dls",
        success=success,
        q_final=q.tolist(),
        pos_error=float(np.linalg.norm(err_final[:3])),
        orient_error=float(np.linalg.norm(err_final[3:])),
        iterations=it,
        wall_time_ms=wall_ms,
        min_self_distance=self_collision_min_distance(spec, q),
        joint_limit_violations=violations,
        steps=steps,
    )
