"""
ProteinIK V5 — Conflict-Controlled Homotopy IK (CCH-IK)

Core contribution
-----------------
Homotopy path-following for constrained serial-chain IK where the
continuation parameter λ advances adaptively based on the full-vector
cosine conflict between task and constraint gradients in joint space.

Theoretical basis (honest)
--------------------------
Homotopy path existence: the Implicit Function Theorem (Allgower &
Georg 1990) guarantees a locally smooth solution path q(λ) exists when
∂²E/∂q² is non-singular. This is NOT the classical penalty convergence
theorem (which requires c → ∞); λ is bounded ∈ [0, 1] and we make no
claim of global convergence. The path breaks at kinematic singularities.

Conflict-controlled advancement empirically attempts to stay on the
regular (non-singular) portion of the path by detecting when constraint
introduction creates gradient incompatibility.

Biological motivation (honest)
-------------------------------
Minimal frustration principle: proteins fold fast because their energy
landscapes are shaped so that gradient conflicts are minimised — native
interactions cooperate rather than compete (Bryngelson & Wolynes 1987).
This is the design intuition only. All algorithmic choices are justified
by the optimisation theory above.

Two toggleable components (ablation-ready)
------------------------------------------
  COMPONENT_A = True   conflict-controlled λ schedule  [the contribution]
  COMPONENT_C = True   geometric warm-start seed       [standard practice]

Novel diagnostic outputs
------------------------
  conflict_index  C ∈ [-1, 1]  full-vector cosine at solution
                  Interpretation:
                    C < 0  cooperative — task and constraints agree
                    C ≈ 0  orthogonal  — independent objectives
                    C > 0  conflicted  — constraints oppose task
  lambda_final    λ ∈ [0, 1]  how far constraints were introduced
                  λ < 0.8 → solver could not fully introduce constraints
"""

from __future__ import annotations

import numpy as np
import time

from app.core.kinematics import (
    RobotSpec, end_effector_pose, pose_error, self_collision_min_distance,
    geometric_jacobian,
)
from app.core.types import SolveResult, SolveStep
from app.solvers.protein_homotopy.core import (
    compute_conflict,
    fd_constraint_gradient,
    backtracking_line_search,
    geometric_seed,
    e_target,
    e_constraints,
    pcgrad_project,
)

# ---------------------------------------------------------------------------
# Ablation switches (toggle for ablation study)
# ---------------------------------------------------------------------------
COMPONENT_A = True   # conflict-controlled λ advancement          [the contribution]
COMPONENT_B = True   # gradient surgery (PCGrad projection)       [reduces interference]
COMPONENT_C = True   # geometric warm-start seed                  [standard practice]
COMPONENT_D = True   # null-space constraint-aware endgame        [constraints reach the OUTPUT]
COMPONENT_E = True   # monotonic predictor-corrector continuation [λ never retreats → path completes]

# ---------------------------------------------------------------------------
# Hyperparameters
# ---------------------------------------------------------------------------
# Conflict threshold on [0, 2] scale: 0=aligned, 1=ortho, 2=opposed.
CONFLICT_THRESHOLD  = 0.6   # C above this → hold λ

# Exponential λ progression (Upgrade A):
# delta(λ) = LAMBDA_MAX_STEP * exp(-LAMBDA_BETA * C)
# At C=0 (full alignment): step = LAMBDA_MAX_STEP  (aggressive)
# At C=THRESHOLD (0.6):   step ≈ LAMBDA_MAX_STEP * 0.09  (nearly stopped)
# beta = ln(10) / 0.6 ≈ 3.84 gives a 10× range across [0, threshold]
LAMBDA_MAX_STEP     = 0.10  # max λ step per iteration (when perfectly aligned)
LAMBDA_BETA         = 3.84  # exponential decay rate for λ step vs. conflict

# Conflict retreat (Upgrade B):
# When stuck for CONFLICT_RETREAT_AFTER iters, take a deterministic constraint-
# descent step on the most conflicted joints, then retract λ slightly.
CONFLICT_RETREAT_AFTER  = 20    # iters before triggering retreat / progress-force
RETREAT_ALPHA           = 0.15  # step size for the constraint retreat move
LAMBDA_RETRACT_FACTOR   = 0.90  # λ *= this on retreat (homotopy back-off)
MIN_LAMBDA_PROGRESS     = 0.05  # Component E: forced monotonic λ step when stuck

ORIENT_WEIGHT       = 0.3   # orientation weight in combined error
MAX_ITERS           = 400   # per-trajectory iteration budget
N_RESTARTS          = 3     # random restarts after primary attempt
LM_ENDGAME_THRESH   = 0.05  # switch to LM polish when pos_err < this (metres)


# ---------------------------------------------------------------------------
# Fast fused FK+Jacobian (one chain pass, no np.cross overhead)
# ---------------------------------------------------------------------------

def _fast_pose_jac(spec: RobotSpec, q: np.ndarray):
    """Returns (end_effector_4x4, geometric_Jacobian_6xN) from one FK pass."""
    from app.core.kinematics import forward_kinematics_chain, joint_axis_frames
    chain = forward_kinematics_chain(spec, q)
    n = spec.n_joints
    pose = chain[n]
    z, p = joint_axis_frames(spec, chain)   # rotation axes + points (DH-aware)
    d = chain[n, :3, 3] - p       # lever arms to end-effector
    J = np.empty((6, n))
    J[0, :] = z[:, 1] * d[:, 2] - z[:, 2] * d[:, 1]
    J[1, :] = z[:, 2] * d[:, 0] - z[:, 0] * d[:, 2]
    J[2, :] = z[:, 0] * d[:, 1] - z[:, 1] * d[:, 0]
    J[3:, :] = z.T
    return pose, J


def _null_space_declash(spec: RobotSpec, q: np.ndarray, T_target: np.ndarray,
                        pos_tol: float, ori_tol: float,
                        steps: int = 8, alpha: float = 0.1) -> np.ndarray:
    """Component D — redundancy-resolution refinement of the converged solution.

    Descends the constraint energy (self-collision + joint-limit) in the task
    NULL SPACE, so the end-effector stays on target while the configuration
    moves away from clashes. This is what carries the homotopy's constraint
    handling into the OUTPUT instead of letting the task-only endgame wash it
    out. On a non-redundant arm the null-space projector is ~0 → safe no-op; on
    a redundant arm (e.g. Franka) it actively declashes the returned pose.
    """
    n = spec.n_joints
    eye6 = np.eye(6)
    eyen = np.eye(n)
    d_best = self_collision_min_distance(spec, q)
    for _ in range(steps):
        _, J = _fast_pose_jac(spec, q)
        J_pinv = J.T @ np.linalg.inv(J @ J.T + 1e-6 * eye6)   # damped pseudoinverse
        N = eyen - J_pinv @ J                                 # null-space projector
        g_c = fd_constraint_gradient(spec, q)                 # uphill grad of e_constraints
        q_try = spec.clip(q - alpha * (N @ g_c))
        err = pose_error(end_effector_pose(spec, q_try), T_target)
        if np.linalg.norm(err[:3]) < pos_tol and np.linalg.norm(err[3:]) < ori_tol:
            d_new = self_collision_min_distance(spec, q_try)
            if d_new > d_best:
                q, d_best = q_try, d_new
            else:
                break                     # no further constraint improvement
        else:
            alpha *= 0.5                  # step left the target basin — shrink
            if alpha < 1e-3:
                break
    return q


def _lm_polish(spec: RobotSpec, q0: np.ndarray, T_target: np.ndarray,
               pos_tol: float, ori_tol: float,
               max_steps: int = 20, lam0: float = 0.05):
    """Levenberg-Marquardt endgame: fast convergence once we're close.
    Identical in structure to V4's LM polish; runs after homotopy places
    us in the basin of attraction.  Returns (q, converged).

    With COMPONENT_D, once the target tolerance is met the solution is refined
    in the task null space (see `_null_space_declash`) so the constraint
    handling reaches the returned configuration, not just the trajectory.
    """
    q = q0.copy()
    pose, J = _fast_pose_jac(spec, q)
    err = pose_error(pose, T_target)
    lam = lam0
    for _ in range(max_steps):
        pos_e = float(np.linalg.norm(err[:3]))
        ori_e = float(np.linalg.norm(err[3:]))
        if pos_e < pos_tol and ori_e < ori_tol:
            break
        dq = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * np.eye(6), err)
        q_try = spec.clip(q + dq)
        pose_t, J_t = _fast_pose_jac(spec, q_try)
        err_t = pose_error(pose_t, T_target)
        if (np.linalg.norm(err_t[:3]) + 0.3 * np.linalg.norm(err_t[3:])
                < np.linalg.norm(err[:3]) + 0.3 * np.linalg.norm(err[3:])):
            q, J, err, lam = q_try, J_t, err_t, max(lam * 0.5, 1e-4)
        else:
            lam = min(lam * 2.5, 2.0)
            if lam >= 2.0:
                break
    pos_e = float(np.linalg.norm(err[:3]))
    ori_e = float(np.linalg.norm(err[3:]))
    converged = pos_e < pos_tol and ori_e < ori_tol
    if COMPONENT_D and converged:
        q = _null_space_declash(spec, q, T_target, pos_tol, ori_tol)
    return q, converged


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
) -> tuple[np.ndarray, float, bool, int, float, float, float, list]:
    """
    One CCH-IK trajectory from seed q0.

    Returns
    -------
    best_q, best_err, converged, iters_used, C_final, lambda_final,
    difficulty_score, steps
    """
    q        = q0.copy()
    lambda_  = 0.0
    best_q   = q.copy()
    best_err = np.inf
    C_final  = 0.0
    steps_out: list[SolveStep] = [] if collect_steps else []

    iters_at_same_lambda = 0   # consecutive iters without λ advance
    conflict_integral    = 0.0 # running sum of C — for difficulty_score

    for it in range(max_iters):

        # ---- task gradient (analytical Jacobian — one FK pass) ---------------
        T_cur   = end_effector_pose(spec, q)
        err_vec = pose_error(T_cur, T_target)       # ∈ ℝ⁶  (p_target - p_cur, orient)
        pos_e   = float(np.linalg.norm(err_vec[:3]))
        ori_e   = float(np.linalg.norm(err_vec[3:]))

        # Analytical geometric Jacobian (replaces 6-FK FD loop).
        # TRUE gradient of E_target w.r.t. q:
        #   ∂E_target/∂q = -Jᵀ @ err_vec
        # (err_vec points TOWARD target; gradient of squared-error points AWAY)
        J        = geometric_jacobian(spec, q)
        g_target = -J.T @ err_vec          # ∈ ℝⁿ  true gradient (uphill for E_target)

        # ---- constraint gradient (finite differences) ----------------------
        g_constr = fd_constraint_gradient(spec, q)  # ∈ ℝⁿ  true gradient (uphill for E_constr)

        # ---- full-vector conflict ----------------------------------------
        # Both TRUE gradients. C ∈ [0,2]: 0=aligned, 1=ortho, 2=opposed.
        C = compute_conflict(g_target, g_constr)
        C_final = C
        conflict_integral += C  # accumulate for difficulty_score (Upgrade C)

        # ---- λ advancement (Component A) — exponential step ----------------
        # delta(λ) = LAMBDA_MAX_STEP * exp(-LAMBDA_BETA * C)
        # Justification: when objectives strongly agree (C ≈ 0), the combined
        # landscape is well-funneled — we can introduce constraints aggressively.
        # When C approaches threshold, the step shrinks continuously toward 0.
        # This replaces the binary if/else with a smooth, parameter-free schedule.
        prev_lambda = lambda_
        if COMPONENT_A:
            if C < CONFLICT_THRESHOLD and lambda_ < 1.0:
                delta = LAMBDA_MAX_STEP * np.exp(-LAMBDA_BETA * C)
                lambda_ = min(1.0, lambda_ + delta)
                iters_at_same_lambda = 0
            else:
                iters_at_same_lambda += 1

                # ---- Conflict retreat (Upgrade B) ---------------------------
                # When stuck: instead of nudging λ forward (a hack), take a
                # deterministic step to reduce constraint violation specifically
                # in the joints where task and constraint forces most oppose.
                # Then retract λ slightly so the homotopy can re-thread from
                # this improved position.
                # Filter: joints where per-element product < 0 means those
                # two gradients directly oppose each other in that dimension.
                if iters_at_same_lambda >= CONFLICT_RETREAT_AFTER and lambda_ > 0.0:
                    # Corrector step (both branches): reduce constraint violation
                    # on the joints where task and constraint forces most oppose.
                    per_joint_conflict = g_target * g_constr  # ∈ ℝⁿ, elementwise
                    conflict_mask = per_joint_conflict < 0    # dimensions where forces oppose
                    if conflict_mask.any():
                        # g_constr is the true (uphill) gradient of E_constraints,
                        # so q -= alpha * g_constr is constraint descent.
                        q_retreat = q.copy()
                        q_retreat[conflict_mask] -= RETREAT_ALPHA * g_constr[conflict_mask]
                        q = spec.clip(q_retreat)  # clip full array after update
                    if COMPONENT_E:
                        # Predictor-corrector: NEVER retreat — retracting λ stalls
                        # the continuation so it never reaches λ≈1 and the LM
                        # endgame never fires (the cluttered-regime failure mode).
                        # Force a small MONOTONIC forward step: the path always
                        # completes, the corrector above having reduced the local
                        # conflict first.
                        lambda_ = min(1.0, lambda_ + MIN_LAMBDA_PROGRESS)
                    else:
                        # Original conflict-retreat: retract λ (homotopy back-off).
                        lambda_ = max(0.0, lambda_ * LAMBDA_RETRACT_FACTOR)
                    iters_at_same_lambda = 0
        else:
            # Component A off: fixed linear schedule (ablation baseline)
            lambda_ = min(1.0, (it + 1) / max_iters)

        # ---- gradient surgery (Component B) --------------------------------
        # On the [0,2] scale: C >= 0.6 means conflict is significant enough
        # that constraint gradient is meaningfully opposing the task.
        g_constr_used = (
            pcgrad_project(g_constr, g_target)
            if COMPONENT_B and C >= CONFLICT_THRESHOLD
            else g_constr
        )

        # ---- combined TRUE gradient → descend via q -= α * g ---------------
        g = g_target + lambda_ * g_constr_used

        # ---- Armijo backtracking step size ---------------------------------
        alpha = backtracking_line_search(spec, q, g, T_target, lambda_)
        q = spec.clip(q - alpha * g)

        # ---- LM endgame: switch to fast LM once we're in the basin ----------
        if lambda_ >= 0.8 and pos_e < LM_ENDGAME_THRESH:
            q_lm, lm_conv = _lm_polish(spec, q, T_target, pos_tol, ori_tol)
            T_lm = end_effector_pose(spec, q_lm)
            err_lm = pose_error(T_lm, T_target)
            pe_lm = float(np.linalg.norm(err_lm[:3]))
            oe_lm = float(np.linalg.norm(err_lm[3:]))
            sc_lm = pe_lm + ORIENT_WEIGHT * oe_lm
            if sc_lm < best_err:
                best_err = sc_lm
                best_q   = q_lm.copy()
            if lm_conv:
                if collect_steps:
                    steps_out.append(SolveStep(
                        iteration=it_offset + it, q=q_lm.tolist(),
                        pos_error=pe_lm, orient_error=oe_lm,
                        min_self_distance=self_collision_min_distance(spec, q_lm),
                        phase="cch_lm_endgame", energy=float(lambda_),
                    ))
                difficulty = conflict_integral / max(1, it + 1)
                return best_q, best_err, True, it + 1, C_final, lambda_, difficulty, steps_out
            q = q_lm  # continue from LM result

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

        # ---- convergence check -----------------------------------------
        if pos_e < pos_tol and ori_e < ori_tol:
            difficulty = conflict_integral / max(1, it + 1)
            return best_q, best_err, True, it + 1, C_final, lambda_, difficulty, steps_out

        # ---- final LM sweep on best_q before giving up (end of budget) -----
        if it == max_iters - 1:
            q_lm, lm_conv = _lm_polish(spec, best_q, T_target, pos_tol, ori_tol, max_steps=30)
            if lm_conv:
                difficulty = conflict_integral / max(1, max_iters)
                return q_lm, best_err, True, max_iters, C_final, lambda_, difficulty, steps_out

    difficulty = conflict_integral / max(1, max_iters)
    return best_q, best_err, False, max_iters, C_final, lambda_, difficulty, steps_out


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

def solve_protein_homotopy(
    spec: RobotSpec,
    q0: np.ndarray,
    T_target: np.ndarray,
    rng: np.random.Generator,
    max_iters: int = 0,  # 0 → auto-scale by DOF
    pos_tol: float = 1e-3,
    orient_tol: float = 1e-2,
    collect_steps: bool = False,
) -> SolveResult:
    """
    CCH-IK multi-start driver.

    Runs Component C (geometric seed), then up to 1 + N_RESTARTS
    trajectories. Stops at the first converged solution.
    Returns the best-error solution overall.
    """
    # Scale iter budget with DOF: redundant arms need more landscape exploration
    if max_iters == 0:
        max_iters = MAX_ITERS + max(0, spec.n_joints - 6) * 150

    t0 = time.perf_counter()

    # Component C: geometric warm-start
    q_seed = geometric_seed(spec, q0, T_target) if COMPONENT_C else q0.copy()

    # Seeds: improved seed + N_RESTARTS random restarts
    seeds = [q_seed] + [spec.random_config(rng) for _ in range(N_RESTARTS)]

    global_best_q   = q_seed.copy()
    global_best_err = np.inf
    global_C        = 0.0
    global_lambda   = 0.0
    global_difficulty = 0.0
    success         = False
    total_iters     = 0
    all_steps: list[SolveStep] = []
    difficulty_sum  = 0.0
    difficulty_n    = 0

    for i, seed in enumerate(seeds):
        best_q, best_err, conv, iters, C_f, lam_f, diff_f, steps_i = _solve_single(
            spec, seed, T_target,
            max_iters, pos_tol, orient_tol,
            collect_steps=(collect_steps and i == 0),
            it_offset=total_iters,
        )
        total_iters  += iters
        difficulty_sum += diff_f
        difficulty_n  += 1
        if steps_i:
            all_steps.extend(steps_i)

        if best_err < global_best_err:
            global_best_err  = best_err
            global_best_q    = best_q.copy()
            global_C         = C_f
            global_lambda    = lam_f
            global_difficulty = diff_f

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
        solver_name="protein_homotopy",
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
        difficulty_score=difficulty_sum / max(1, difficulty_n),  # mean across restarts
    )
