"""
Cyclic Coordinate Descent (CCD) IK solver.

Classical baseline: sweeps joints from end-effector back to base (or base
to tip, configurable), rotating each joint individually to reduce the
position error as much as possible given all other joints fixed, then
moves to the next joint. Greedy and local by construction -- this is the
"local, joint-by-joint" classical method that the protein-IK solver's
local-relaxation phase will be compared against directly, since on the
surface they sound similar (both touch one joint relative to neighbors)
but CCD has no notion of neighbor-only "settling" before target-awareness,
no collision term, and no escape mechanism for getting stuck.

Note: textbook CCD aligns position only (each joint rotates to point the
end-effector at the target point). It has no mechanism to also satisfy a
target *orientation*, since a single joint rotation can't independently
control both. To make this a fair, usable 6-DOF baseline (position +
orientation), this implementation alternates: most sweeps optimize
position, and on the final joints of the chain (closest to the wrist,
which on a 6-DOF arm largely set orientation) we additionally blend in an
orientation-reduction term. This is the standard practical extension used
in animation/robotics CCD implementations, not a departure from the
classical method's spirit.
"""

from __future__ import annotations

import numpy as np
import time

from app.core.kinematics import (
    RobotSpec, forward_kinematics_chain, pose_error, end_effector_pose,
    self_collision_min_distance,
)
from app.core.types import SolveResult, SolveStep


def solve_ccd(
    spec: RobotSpec,
    q0: np.ndarray,
    T_target: np.ndarray,
    max_iters: int = 300,
    pos_tol: float = 1e-3,
    orient_tol: float = 1e-2,
    collect_steps: bool = False,
) -> SolveResult:
    q = q0.copy()
    n = spec.n_joints
    target_pos = T_target[:3, 3]
    target_rot = T_target[:3, :3]
    # Last joints also get an orientation-correction blend (standard CCD extension).
    # Reserve 3 for 6+ DOF, 1 for short arms — on a 3-DOF arm the old formula
    # set all 3 as wrist joints, making orientation fight position on every joint.
    n_wrist = 3 if n >= 6 else (1 if n >= 2 else 0)
    wrist_joints = set(range(n - n_wrist, n))
    steps = []
    t0 = time.perf_counter()
    success = False
    it = 0

    for it in range(1, max_iters + 1):
        # one full sweep = base -> tip, one joint rotation update each
        for i in range(n):
            chain = forward_kinematics_chain(spec, q)  # re-evaluate after each joint update
            p_i = chain[i, :3, 3]
            z_i = chain[i, :3, 2]
            p_end = chain[n, :3, 3]
            R_end = chain[n, :3, :3]

            # --- position term: vector from joint i to end-effector and
            # to target, projected onto the plane perpendicular to z_i ---
            v_end = p_end - p_i
            v_target = target_pos - p_i
            v_end_proj = v_end - np.dot(v_end, z_i) * z_i
            v_target_proj = v_target - np.dot(v_target, z_i) * z_i

            n_end = np.linalg.norm(v_end_proj)
            n_target = np.linalg.norm(v_target_proj)
            pos_angle = 0.0
            if n_end > 1e-9 and n_target > 1e-9:
                v_end_u = v_end_proj / n_end
                v_target_u = v_target_proj / n_target
                cos_a = np.clip(np.dot(v_end_u, v_target_u), -1.0, 1.0)
                sin_a = np.dot(np.cross(v_end_u, v_target_u), z_i)
                pos_angle = np.arctan2(sin_a, cos_a)

            angle = pos_angle

            # --- orientation term, wrist joints only: rotate to reduce
            # the relative-rotation error's component along z_i ---
            if i in wrist_joints:
                R_err = target_rot @ R_end.T
                cos_t = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
                theta = np.arccos(cos_t)
                if theta > 1e-8:
                    axis = np.array([
                        R_err[2, 1] - R_err[1, 2],
                        R_err[0, 2] - R_err[2, 0],
                        R_err[1, 0] - R_err[0, 1],
                    ]) / (2.0 * np.sin(theta))
                    orient_angle = np.dot(axis, z_i) * theta
                    # blend: position dominant far from convergence,
                    # orientation contributes a fraction each step
                    angle = pos_angle + 0.5 * orient_angle

            q[i] = q[i] + angle
            q[i] = np.clip(q[i], spec.joint_limits[i, 0], spec.joint_limits[i, 1])

        T_cur = end_effector_pose(spec, q)
        err = pose_error(T_cur, T_target)
        pos_e = float(np.linalg.norm(err[:3]))
        orient_e = float(np.linalg.norm(err[3:]))

        if collect_steps:
            steps.append(SolveStep(
                iteration=it, q=q.tolist(), pos_error=pos_e, orient_error=orient_e,
                min_self_distance=self_collision_min_distance(spec, q), phase="ccd_sweep",
            ))

        if pos_e < pos_tol and orient_e < orient_tol:
            success = True
            break

    T_final = end_effector_pose(spec, q)
    err_final = pose_error(T_final, T_target)
    wall_ms = (time.perf_counter() - t0) * 1000.0
    violations = int(np.sum((q <= spec.joint_limits[:, 0] + 1e-9) |
                             (q >= spec.joint_limits[:, 1] - 1e-9)))

    return SolveResult(
        solver_name="ccd",
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
