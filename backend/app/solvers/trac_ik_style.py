"""
TRAC-IK-style solver: Damped Least Squares augmented with stuck-detection
and random restart.

This mirrors the real-world production approach used by TRAC-IK and
similar libraries: run a local numerical solver (DLS here), monitor for
lack of progress (a "stuck" condition -- usually caused by a joint-limit
wall or local minimum), and when stuck, restart from a fresh random seed
rather than continuing to iterate uselessly. Whichever restart attempt
converges first (or the best partial result if none converge within the
attempt budget) is returned.

This is the most important classical baseline for the protein-IK solver
to beat, since it already implements one of the protein-folding-style
mechanisms we considered (chaperone-style rescue-on-stuck), just in its
simplest global form (full random restart) rather than a scoped/local
form. If the protein-IK solver can't outperform this, the "locally scoped
rescue" idea doesn't have legs.
"""

from __future__ import annotations

import numpy as np
import time

from app.core.kinematics import (
    RobotSpec, end_effector_pose, geometric_jacobian, pose_error,
    self_collision_min_distance,
)
from app.core.types import SolveResult, SolveStep


def solve_trac_ik(
    spec: RobotSpec,
    q0: np.ndarray,
    T_target: np.ndarray,
    rng: np.random.Generator,
    max_total_iters: int = 300,
    iters_per_attempt: int = 50,
    stuck_window: int = 8,
    stuck_eps: float = 1e-5,
    pos_tol: float = 1e-3,
    orient_tol: float = 1e-2,
    damping: float = 0.05,
    collect_steps: bool = False,
) -> SolveResult:
    steps = []
    t0 = time.perf_counter()
    success = False
    total_it = 0
    restarts = 0

    q = q0.copy()
    best_q = q.copy()
    best_err = np.inf

    while total_it < max_total_iters:
        recent_errs = []
        attempt_converged = False

        for local_it in range(1, iters_per_attempt + 1):
            total_it += 1
            T_cur = end_effector_pose(spec, q)
            err = pose_error(T_cur, T_target)
            pos_e = float(np.linalg.norm(err[:3]))
            orient_e = float(np.linalg.norm(err[3:]))
            combined = pos_e + 0.3 * orient_e  # single scalar for stuck-detection & best-tracking

            if combined < best_err:
                best_err = combined
                best_q = q.copy()

            if collect_steps:
                steps.append(SolveStep(
                    iteration=total_it, q=q.tolist(), pos_error=pos_e, orient_error=orient_e,
                    min_self_distance=self_collision_min_distance(spec, q), phase="dls_attempt",
                ))

            if pos_e < pos_tol and orient_e < orient_tol:
                success = True
                attempt_converged = True
                best_q = q.copy()
                break

            if total_it >= max_total_iters:
                break

            # --- stuck detection: track recent combined-error progress ---
            recent_errs.append(combined)
            if len(recent_errs) >= stuck_window:
                window = recent_errs[-stuck_window:]
                progress = window[0] - window[-1]
                if progress < stuck_eps:
                    # stuck: abandon this attempt, trigger random restart
                    break

            J = geometric_jacobian(spec, q)
            JJt = J @ J.T
            lam2 = damping ** 2
            dq = J.T @ np.linalg.solve(JJt + lam2 * np.eye(6), err)
            q = spec.clip(q + dq)

        if attempt_converged or total_it >= max_total_iters:
            break

        # --- random restart (global, full-chain re-seed) ---
        restarts += 1
        q = spec.random_config(rng)

    q_final = best_q
    T_final = end_effector_pose(spec, q_final)
    err_final = pose_error(T_final, T_target)
    wall_ms = (time.perf_counter() - t0) * 1000.0
    violations = int(np.sum((q_final <= spec.joint_limits[:, 0] + 1e-9) |
                             (q_final >= spec.joint_limits[:, 1] - 1e-9)))

    return SolveResult(
        solver_name="trac_ik_style",
        success=success,
        q_final=q_final.tolist(),
        pos_error=float(np.linalg.norm(err_final[:3])),
        orient_error=float(np.linalg.norm(err_final[3:])),
        iterations=total_it,
        wall_time_ms=wall_ms,
        min_self_distance=self_collision_min_distance(spec, q_final),
        joint_limit_violations=violations,
        restarts=restarts,
        steps=steps,
    )
