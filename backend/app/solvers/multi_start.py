"""
Multi-start / population-based IK solver.

Generates N diverse initial seeds (population), runs DLS from each
independently and simultaneously (in the sense that all are tracked
together, not sequentially-on-failure like TRAC-IK's restart), and
returns the best converged result. This mirrors the multi-start /
generative-candidate family in the literature (e.g. IKFlow's best-of-N
warm-starting, or classic multi-start nonlinear optimization for
redundant-arm IK), which the protein-IK solver's "kinetic partitioning"
inspired candidate-pruning design should be compared against directly,
since they're the closest existing cousin to that idea.

Unlike TRAC-IK's sequential restart-on-stuck, this commits a fixed
population budget up front and lets every branch run to its own
convergence or iteration budget, then picks the best -- no early pruning
based on intermediate signals. That's the key structural difference from
the protein-IK solver's planned candidate-pruning mechanism.
"""

from __future__ import annotations

import numpy as np
import time

from app.core.kinematics import (
    RobotSpec, end_effector_pose, geometric_jacobian, pose_error,
    self_collision_min_distance,
)
from app.core.types import SolveResult, SolveStep


def _dls_run(spec, q0, T_target, max_iters, pos_tol, orient_tol, damping):
    q = q0.copy()
    for it in range(1, max_iters + 1):
        T_cur = end_effector_pose(spec, q)
        err = pose_error(T_cur, T_target)
        pos_e = float(np.linalg.norm(err[:3]))
        orient_e = float(np.linalg.norm(err[3:]))
        if pos_e < pos_tol and orient_e < orient_tol:
            return q, it, True, pos_e, orient_e
        J = geometric_jacobian(spec, q)
        JJt = J @ J.T
        lam2 = damping ** 2
        dq = J.T @ np.linalg.solve(JJt + lam2 * np.eye(6), err)
        q = spec.clip(q + dq)
    T_cur = end_effector_pose(spec, q)
    err = pose_error(T_cur, T_target)
    return q, max_iters, False, float(np.linalg.norm(err[:3])), float(np.linalg.norm(err[3:]))


def solve_multi_start(
    spec: RobotSpec,
    q0: np.ndarray,
    T_target: np.ndarray,
    rng: np.random.Generator,
    population_size: int = 8,
    max_iters_per_member: int = 60,
    pos_tol: float = 1e-3,
    orient_tol: float = 1e-2,
    damping: float = 0.05,
    collect_steps: bool = False,
) -> SolveResult:
    t0 = time.perf_counter()
    steps = []

    # population: the provided seed plus (population_size - 1) random seeds,
    # spread across the joint space for diversity
    seeds = [q0.copy()] + [spec.random_config(rng) for _ in range(population_size - 1)]

    results = []
    total_iters = 0
    for member_idx, seed in enumerate(seeds):
        q_final, it, converged, pos_e, orient_e = _dls_run(
            spec, seed, T_target, max_iters_per_member, pos_tol, orient_tol, damping
        )
        total_iters += it
        combined = pos_e + 0.3 * orient_e
        results.append((combined, q_final, converged, pos_e, orient_e))
        if collect_steps:
            steps.append(SolveStep(
                iteration=total_iters, q=q_final.tolist(), pos_error=pos_e, orient_error=orient_e,
                min_self_distance=self_collision_min_distance(spec, q_final),
                phase=f"population_member_{member_idx}",
            ))

    # pick best by combined error; prefer converged members if any exist
    converged_results = [r for r in results if r[2]]
    pool = converged_results if converged_results else results
    best = min(pool, key=lambda r: r[0])
    _, q_final, success, pos_e, orient_e = best

    wall_ms = (time.perf_counter() - t0) * 1000.0
    violations = int(np.sum((q_final <= spec.joint_limits[:, 0] + 1e-9) |
                             (q_final >= spec.joint_limits[:, 1] - 1e-9)))

    return SolveResult(
        solver_name="multi_start",
        success=success,
        q_final=q_final.tolist(),
        pos_error=pos_e,
        orient_error=orient_e,
        iterations=total_iters,
        wall_time_ms=wall_ms,
        min_self_distance=self_collision_min_distance(spec, q_final),
        joint_limit_violations=violations,
        restarts=population_size - 1,
        steps=steps,
    )
