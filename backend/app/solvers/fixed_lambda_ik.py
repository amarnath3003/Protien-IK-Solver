"""
Fixed-lambda homotopy IK — Baseline 4 for V5 ablation.

Same E(q,λ) and gradient as CCH-IK. λ = iter/max_iters (fixed).
No conflict detection. Isolates the contribution of Component A.
"""

from __future__ import annotations

import numpy as np
import time

from app.core.kinematics import (
    RobotSpec, end_effector_pose, pose_error, self_collision_min_distance,
)
from app.core.types import SolveResult, SolveStep
from app.solvers.protein_homotopy.core import (
    compute_conflict,
    fd_constraint_gradient,
    backtracking_line_search,
    geometric_seed,
    e_target,
    e_constraints,
)

# ---------------------------------------------------------------------------
# Ablation switches (toggle for ablation study)
# ---------------------------------------------------------------------------
COMPONENT_A = False  # conflict-controlled λ advancement (DISABLED)
COMPONENT_C = True   # geometric warm-start seed

# ---------------------------------------------------------------------------
# Hyperparameters (tuned on Puma560 6-DOF)
# ---------------------------------------------------------------------------
CONFLICT_THRESHOLD = 0.2   # C above this → hold λ
DELTA_LAMBDA       = 0.04  # λ step per iteration when conflict is low
ORIENT_WEIGHT      = 0.3   # orientation weight in combined error
MAX_ITERS          = 250   # per-trajectory iteration budget
N_RESTARTS         = 2     # random restarts after primary attempt
FORCE_ADVANCE_AFTER = 30   # force λ advance if stuck for this many iters


# ---------------------------------------------------------------------------
# Single-trajectory solver
# ---------------------------------------------------------------------------

def _solve_single(
    spec: RobotSpec,
    q0: np.ndarray,
    T_target: np.ndarray,
    max_iters: int,
    pos_tol: float,
    ori_tol: float,
    collect_steps: bool,
    it_offset: int,
) -> tuple[np.ndarray, float, bool, int, float, float, list]:
    """
    One Fixed-λ Homotopy IK trajectory from seed q0.
    """
    q        = q0.copy()
    lambda_  = 0.0
    best_q   = q.copy()
    best_err = np.inf
    C_final  = 0.0
    steps_out: list[SolveStep] = [] if collect_steps else []

    iters_at_same_lambda = 0

    for it in range(max_iters):

        # ---- task gradient (analytical via Jacobian transpose) -------------
        T_cur  = end_effector_pose(spec, q)
        err_vec = pose_error(T_cur, T_target)       # ∈ ℝ⁶ task space
        pos_e  = float(np.linalg.norm(err_vec[:3]))
        ori_e  = float(np.linalg.norm(err_vec[3:]))

        # Numerical Jacobian for g_target (6×n → n-vector in joint space)
        n = spec.n_joints
        J = np.zeros((6, n))
        eps_fd = 1e-5
        for i in range(n):
            qp = q.copy(); qp[i] += eps_fd
            T_p = end_effector_pose(spec, qp)
            J[:, i] = (pose_error(T_p, T_target) - err_vec) / eps_fd
        g_target = J.T @ err_vec                    # ∈ ℝⁿ joint space

        # ---- constraint gradient (finite differences) ----------------------
        g_constr = fd_constraint_gradient(spec, q)  # ∈ ℝⁿ

        # ---- full-vector conflict (Component A) ----------------------------
        C = compute_conflict(g_target, g_constr)
        C_final = C

        # ---- λ advancement -------------------------------------------------
        prev_lambda = lambda_
        if COMPONENT_A:
            if C < CONFLICT_THRESHOLD and lambda_ < 1.0:
                lambda_ = min(1.0, lambda_ + DELTA_LAMBDA)
                iters_at_same_lambda = 0
            else:
                iters_at_same_lambda += 1
                if iters_at_same_lambda >= FORCE_ADVANCE_AFTER and lambda_ < 1.0:
                    lambda_ = min(1.0, lambda_ + 0.5 * DELTA_LAMBDA)
                    iters_at_same_lambda = 0
        else:
            # Fixed linear schedule for ablation (Component A off)
            lambda_ = min(1.0, (it + 1) / max_iters)

        # ---- combined gradient (no surgery) --------------------------------
        g = g_target + lambda_ * g_constr

        # ---- Armijo backtracking step size ---------------------------------
        alpha = backtracking_line_search(spec, q, g, T_target, lambda_)
        q = spec.clip(q - alpha * g)

        # ---- track best solution -------------------------------------------
        err_scalar = pos_e + ORIENT_WEIGHT * ori_e
        if err_scalar < best_err:
            best_err = err_scalar
            best_q   = q.copy()

        # ---- step recording for visualisation ------------------------------
        if collect_steps:
            phase = ("cch_lambda_advance"
                     if lambda_ > prev_lambda
                     else "cch_lambda_hold")
            steps_out.append(SolveStep(
                iteration=it_offset + it,
                q=q.tolist(),
                pos_error=pos_e,
                orient_error=ori_e,
                min_self_distance=self_collision_min_distance(spec, q),
                phase=phase,
                energy=float(lambda_),   # energy slot holds λ for visualisation
            ))

        # ---- convergence check (only valid once constraints fully active) --
        if lambda_ >= 1.0 and pos_e < pos_tol and ori_e < ori_tol:
            return best_q, best_err, True, it + 1, C_final, lambda_, steps_out

    return best_q, best_err, False, max_iters, C_final, lambda_, steps_out


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

def solve_fixed_lambda_ik(
    spec: RobotSpec,
    q0: np.ndarray,
    T_target: np.ndarray,
    rng: np.random.Generator,
    max_iters: int = MAX_ITERS,
    pos_tol: float = 1e-3,
    orient_tol: float = 1e-2,
    collect_steps: bool = False,
) -> SolveResult:
    """
    Fixed-lambda IK multi-start driver.
    """
    t0 = time.perf_counter()

    # Component C: geometric warm-start
    q_seed = geometric_seed(spec, q0, T_target) if COMPONENT_C else q0.copy()

    # Seeds: improved seed + N_RESTARTS random restarts
    seeds = [q_seed] + [spec.random_config(rng) for _ in range(N_RESTARTS)]

    global_best_q   = q_seed.copy()
    global_best_err = np.inf
    global_C        = 0.0
    global_lambda   = 0.0
    success         = False
    total_iters     = 0
    all_steps: list[SolveStep] = []

    for i, seed in enumerate(seeds):
        best_q, best_err, conv, iters, C_f, lam_f, steps_i = _solve_single(
            spec, seed, T_target,
            max_iters, pos_tol, orient_tol,
            collect_steps=(collect_steps and i == 0),
            it_offset=total_iters,
        )
        total_iters += iters
        if steps_i:
            all_steps.extend(steps_i)

        if best_err < global_best_err:
            global_best_err = best_err
            global_best_q   = best_q.copy()
            global_C        = C_f
            global_lambda   = lam_f

        if conv:
            success = True
            break   # stop at first converged solution

    q = global_best_q
    T_fin   = end_effector_pose(spec, q)
    err_fin = pose_error(T_fin, T_target)
    violations = int(np.sum(
        (q <= spec.joint_limits[:, 0] + 1e-9) |
        (q >= spec.joint_limits[:, 1] - 1e-9)
    ))

    return SolveResult(
        solver_name="fixed_lambda_ik",
        success=success,
        q_final=q.tolist(),
        pos_error=float(np.linalg.norm(err_fin[:3])),
        orient_error=float(np.linalg.norm(err_fin[3:])),
        iterations=total_iters,
        wall_time_ms=(time.perf_counter() - t0) * 1000.0,
        min_self_distance=self_collision_min_distance(spec, q),
        joint_limit_violations=violations,
        restarts=len(seeds) - 1,
        steps=all_steps,
        conflict_index=global_C,
        lambda_final=global_lambda,
    )
