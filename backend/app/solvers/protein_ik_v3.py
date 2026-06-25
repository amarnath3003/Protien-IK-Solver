"""
ProteinIK V3 -- staged folding solver with the three upgrades derived from
the deep folding analysis (see docs/protein_folding_deep_dive.md):

  (A) BEST-SO-FAR + COLLISION-AWARE SELECTION (native-state realism)
      Folding returns the lowest-free-energy state actually *visited*, not
      wherever the last thermal kick left the chain. Among candidates that
      reach the target, the sterically cleanest is preferred -- the native
      state is well-packed, not merely on-target. V2 returned its final q
      (which Metropolis can have walked uphill from a good basin) and
      selected on pose error alone.

  (B) BARRIERLESS QUADRATIC ENDGAME (downhill / speed-limit folding)
      The bottom of a folding funnel is locally quadratic; the fastest
      proteins fold barrierlessly, just diffusing downhill. The optimizer
      that matches that regime is adaptive Levenberg-Marquardt: heavy
      damping when far (robust, like coarse collapse), damping -> 0 as the
      basin is approached (Newton-fast quadratic convergence). Each step is
      accepted only if it lowers the error, so descent is monotone -- no
      overshoot. This replaces V2's fixed-damping, unguarded DLS step and
      is the main attack on the *speed* gap to TRAC-IK.

  (C) ADAPTIVE ENSEMBLE / REPLICA FOLDING (the ensemble folds in parallel)
      A test tube folds ~1e15 copies at once; replica-exchange is the
      canonical computational realization. A single chain (V2) cannot match
      the basin diversity that gives TRAC-IK / multi-start their ~99%. V3
      folds an ENSEMBLE -- but adaptively: it runs one trajectory, and only
      spawns additional diverse replicas if that trajectory fails to reach
      the native state. Easy targets stay single-trajectory (fast); hard
      targets get parallel-search robustness. This is the main attack on the
      *robustness* (success-rate) gap.

The staged-folding identity (target-blind relaxation -> coarse collapse ->
funnel narrowing with Metropolis -> chaperone rescue) is preserved as the
per-replica engine, so V3 keeps ProteinIK's structural, collision-aware
character rather than collapsing into plain multi-start DLS.
"""

from __future__ import annotations

import numpy as np
import time

from app.core.kinematics import (
    RobotSpec, forward_kinematics_chain, end_effector_pose, pose_error,
    self_collision_min_distance, geometric_jacobian,
)
from app.core.types import SolveResult, SolveStep
from app.solvers.protein_energy import (
    joint_limit_energy, neighbor_smoothness_energy, neutral_pose_energy,
    total_energy_fast, frustration_index,
)

# Stage-3 energy weights (target, joint-limit, collision, smoothness).
_W = (3.0, 1.0, 2.0, 0.3)


def _combined_err(spec: RobotSpec, q: np.ndarray, T_target: np.ndarray) -> tuple[float, float, float]:
    """Returns (pos_err, orient_err, combined). `combined` is the single
    scalar used everywhere for best-tracking / acceptance, matching the
    convention the baselines use (pos + 0.3 * orient)."""
    T_cur = end_effector_pose(spec, q)
    err = pose_error(T_cur, T_target)
    pos_e = float(np.linalg.norm(err[:3]))
    orient_e = float(np.linalg.norm(err[3:]))
    return pos_e, orient_e, pos_e + 0.3 * orient_e


def _lm_polish(spec: RobotSpec, q0: np.ndarray, T_target: np.ndarray,
               pos_tol: float, orient_tol: float, max_steps: int,
               lam0: float = 0.08) -> tuple[np.ndarray, float, bool, int]:
    """(B) Adaptive Levenberg-Marquardt descent -- the barrierless quadratic
    endgame. Damping `lam` shrinks on every accepted (error-reducing) step
    (toward Newton-fast convergence in the quadratic basin) and grows when a
    step would increase error (back off, stay robust). Monotone by
    construction. Returns (q, combined_err, converged, steps_used).
    """
    q = q0.copy()
    pos_e, orient_e, e_cur = _combined_err(spec, q, T_target)
    lam = lam0
    steps_used = 0
    for _ in range(max_steps):
        if pos_e < pos_tol and orient_e < orient_tol:
            return q, e_cur, True, steps_used
        steps_used += 1
        T_cur = end_effector_pose(spec, q)
        err = pose_error(T_cur, T_target)
        J = geometric_jacobian(spec, q)
        JJt = J @ J.T
        dq = J.T @ np.linalg.solve(JJt + (lam ** 2) * np.eye(6), err)
        q_try = spec.clip(q + dq)
        p_t, o_t, e_try = _combined_err(spec, q_try, T_target)
        if e_try < e_cur:
            q, pos_e, orient_e, e_cur = q_try, p_t, o_t, e_try
            lam = max(lam * 0.5, 1e-4)   # trust more -> Newton-fast
        else:
            lam = min(lam * 2.5, 2.0)    # overshoot -> damp harder
            if lam >= 2.0:
                break  # damped to a standstill; this basin is exhausted
    pos_e, orient_e, e_cur = _combined_err(spec, q, T_target)
    return q, e_cur, (pos_e < pos_tol and orient_e < orient_tol), steps_used


def _fold_once(
    spec: RobotSpec, q0: np.ndarray, T_target: np.ndarray, rng: np.random.Generator,
    max_iters: int, pos_tol: float, orient_tol: float,
    stage1_iters: int, stage2_iters: int, stuck_window: int, stuck_eps: float,
    do_stage1: bool, steps: list | None, it_offset: int,
) -> tuple[np.ndarray, float, bool, int, int]:
    """One folding trajectory. Returns (best_q, best_combined, success,
    iters_used, rescues). Tracks best-so-far throughout (A) and uses the
    adaptive LM endgame (B) for refinement and on convergence approach."""
    n = spec.n_joints
    q = q0.copy()
    q_neutral = np.zeros(n)
    it = 0

    best_q = q.copy()
    _, _, best_combined = _combined_err(spec, q, T_target)
    rescues = 0

    def record(phase, energy=None):
        if steps is None:
            return
        pos_e, orient_e, _ = _combined_err(spec, q, T_target)
        steps.append(SolveStep(
            iteration=it_offset + it, q=q.tolist(),
            pos_error=pos_e, orient_error=orient_e,
            min_self_distance=self_collision_min_distance(spec, q),
            phase=phase, energy=energy,
        ))

    def consider(cand_q):
        """Update best-so-far (A)."""
        nonlocal best_q, best_combined
        _, _, c = _combined_err(spec, cand_q, T_target)
        if c < best_combined:
            best_combined = c
            best_q = cand_q.copy()

    # ---------- STAGE 1: target-blind local relaxation (secondary structure)
    if do_stage1:
        for _ in range(stage1_iters):
            it += 1
            for i in range(n):
                base = (neutral_pose_energy(q, q_neutral)
                        + neighbor_smoothness_energy(q) + joint_limit_energy(spec, q))
                best_q_i, best_e = q[i], base
                for cand in (q[i] - 0.3, q[i] + 0.3):
                    cand = np.clip(cand, spec.joint_limits[i, 0], spec.joint_limits[i, 1])
                    q_try = q.copy(); q_try[i] = cand
                    e_try = (neutral_pose_energy(q_try, q_neutral)
                             + neighbor_smoothness_energy(q_try) + joint_limit_energy(spec, q_try))
                    if e_try < best_e:
                        best_e, best_q_i = e_try, cand
                q[i] = best_q_i
            record("local_blind_relax")
        consider(q)

    # ---------- STAGE 2: coarse collapse (hydrophobic collapse) ----------
    for _ in range(stage2_iters):
        it += 1
        T_cur = end_effector_pose(spec, q)
        err = pose_error(T_cur, T_target)
        J = geometric_jacobian(spec, q)
        JJt = J @ J.T
        dq = J.T @ np.linalg.solve(JJt + (0.15 ** 2) * np.eye(6), err)
        q = spec.clip(q + 0.4 * dq)
        record("coarse_collapse")
    consider(q)

    # ---------- STAGE 3 + 4: funnel narrowing (Metropolis) + LM + rescue ----
    recent = []
    search_radius = 0.5
    radius_decay = 0.985
    cur_energy = total_energy_fast(spec, q, T_target, *_W)

    while it < max_iters:
        it += 1
        # cooling schedule for Metropolis acceptance (thermal trap escape)
        T0, Tf = 0.3, 0.01
        frac = min(it / max_iters, 1.0)
        temp = T0 * (Tf / T0) ** frac

        # narrowing stochastic local search every other iteration
        if it % 2 == 0:
            for i in range(n):
                cand = np.clip(q[i] + rng.uniform(-search_radius, search_radius),
                               spec.joint_limits[i, 0], spec.joint_limits[i, 1])
                q_try = q.copy(); q_try[i] = cand
                e_try = total_energy_fast(spec, q_try, T_target, *_W)
                if e_try < cur_energy or rng.uniform() < np.exp(-(e_try - cur_energy) / max(temp, 1e-6)):
                    q, cur_energy = q_try, e_try

        # (B) adaptive LM step toward target -- the funnel's downhill direction
        T_cur = end_effector_pose(spec, q)
        err = pose_error(T_cur, T_target)
        pos_e, orient_e = float(np.linalg.norm(err[:3])), float(np.linalg.norm(err[3:]))

        # once inside the basin, hand off entirely to the LM endgame: a few
        # Newton-fast monotone steps drive it to tolerance quickly.
        if pos_e < 0.05 and orient_e < 0.2:
            q, cur_combined, conv, _ = _lm_polish(
                spec, q, T_target, pos_tol, orient_tol, max_steps=12)
            cur_energy = total_energy_fast(spec, q, T_target, *_W)
            record("funnel_lm_endgame", energy=cur_energy)
            consider(q)
            if conv:
                return best_q, best_combined, True, it, rescues
        else:
            J = geometric_jacobian(spec, q)
            JJt = J @ J.T
            dq = J.T @ np.linalg.solve(JJt + (0.05 ** 2) * np.eye(6), err)
            q = spec.clip(q + dq)
            cur_energy = total_energy_fast(spec, q, T_target, *_W)
            record("funnel_narrowing", energy=cur_energy)
            consider(q)

        search_radius *= radius_decay

        pos_e, orient_e, _ = _combined_err(spec, q, T_target)
        if pos_e < pos_tol and orient_e < orient_tol:
            return best_q, best_combined, True, it, rescues

        # ---------- STAGE 4: chaperone rescue (iterative annealing) ----------
        recent.append(cur_energy)
        if len(recent) >= stuck_window:
            if recent[0] - recent[-1] < stuck_eps:   # stalled
                rescues += 1
                scope_sizes = [1, 3, 5, n]
                scope = scope_sizes[min(rescues - 1, len(scope_sizes) - 1)]
                if scope >= n:
                    q = spec.random_config(rng)
                else:
                    contributions = frustration_index(spec, q, T_target)
                    worst = int(np.argmax(contributions))
                    half = scope // 2
                    lo = max(0, worst - half)
                    hi = min(n, lo + scope)
                    lo = max(0, hi - scope)
                    fresh = spec.random_config(rng)
                    for j in range(lo, hi):
                        q[j] = fresh[j]
                search_radius = 0.5
                recent = []
                cur_energy = total_energy_fast(spec, q, T_target, *_W)
                record(f"chaperone_rescue_{scope}", energy=cur_energy)
                consider(q)

    return best_q, best_combined, False, it, rescues


def solve_protein_ik_v3(
    spec: RobotSpec,
    q0: np.ndarray,
    T_target: np.ndarray,
    rng: np.random.Generator,
    max_iters: int = 150,
    pos_tol: float = 1e-3,
    orient_tol: float = 1e-2,
    collect_steps: bool = False,
    stage1_iters: int = 6,
    stage2_iters: int = 10,
    stuck_window: int = 10,
    stuck_eps: float = 2e-4,
    max_replicas: int = 6,
) -> SolveResult:
    """(C) Adaptive ensemble driver. Folds trajectory 0 from the given seed;
    if it does not reach the native state, spawns diverse replicas (each a
    full staged fold + LM endgame) until one converges or the replica budget
    is exhausted. Returns the globally best state visited across all
    replicas, preferring the sterically cleanest among converged candidates.
    """
    n = spec.n_joints
    t0 = time.perf_counter()
    steps: list | None = [] if collect_steps else None

    # Contact-order-inspired difficulty scaling of the per-replica budget.
    reach_needed = float(np.linalg.norm(T_target[:3, 3]))
    max_reach = float(np.sum(np.abs(spec.a) + np.abs(spec.d)))
    reach_ratio = min(reach_needed / max_reach, 1.0) if max_reach > 0 else 0.0
    s = np.linalg.svd(geometric_jacobian(spec, q0), compute_uv=False)
    cond = s[0] / (s[-1] + 1e-6)
    difficulty = 1.0 + min(reach_ratio, 1.0) + min(cond / 100.0, 1.0)
    s1 = int(stage1_iters * difficulty)
    s2 = int(stage2_iters * difficulty)

    global_best_q = None
    global_best_combined = np.inf
    total_iters = 0
    total_rescues = 0
    success = False
    # candidates that actually reached the target -- collision-aware tie-break (A)
    converged_candidates: list[tuple[float, np.ndarray]] = []

    # ---------- FAST PATH: barrierless downhill from the seed (B) ----------
    # Most targets fold fast: a pure LM descent from q0 reaches the basin in a
    # handful of Newton-fast steps (this is the "downhill / speed-limit"
    # regime). Accept it immediately ONLY if it is also sterically clean --
    # otherwise fall through to the collision-aware staged ensemble, which is
    # exactly where ProteinIK's structural advantage lives. This keeps the
    # easy majority near DLS/TRAC speed without surrendering the collision win
    # on the hard, clash-prone cases.
    fp_q, fp_combined, fp_conv, fp_steps = _lm_polish(
        spec, q0, T_target, pos_tol, orient_tol, max_steps=30)
    total_iters += fp_steps
    global_best_combined = fp_combined
    global_best_q = fp_q.copy()
    if fp_conv and self_collision_min_distance(spec, fp_q) >= 0.0:
        success = True
        converged_candidates.append((self_collision_min_distance(spec, fp_q), fp_q.copy()))

    # Stop spawning replicas once we hold a collision-free converged candidate,
    # or after collecting a few converged-but-clashing ones (diminishing
    # returns when the target itself can only be reached in a strained pose).
    def _have_clean() -> bool:
        return any(d >= 0.0 for d, _ in converged_candidates)

    for replica in range(max_replicas):
        if success and (_have_clean() or len(converged_candidates) >= 2):
            break
        # replica 0 reuses the seed; later replicas use diverse random seeds
        # (the ensemble's value is basin diversity). Stage-1 target-blind
        # relaxation is skipped -- the LM endgame does the actual convergence,
        # so paying for coordinate-descent settling is not worth the time.
        seed = q0.copy() if replica == 0 else spec.random_config(rng)
        best_q, best_combined, conv, iters, rescues = _fold_once(
            spec, seed, T_target, rng, max_iters, pos_tol, orient_tol,
            s1, s2, stuck_window, stuck_eps,
            do_stage1=False, steps=steps, it_offset=total_iters,
        )
        total_iters += iters
        total_rescues += rescues
        if best_combined < global_best_combined:
            global_best_combined = best_combined
            global_best_q = best_q.copy()
        if conv:
            success = True
            # collision-aware native-state selection (A): a converged but
            # clashing fold is not yet "native" -- keep folding for a cleaner
            # one until clean or the small extra-attempt budget is spent.
            d = self_collision_min_distance(spec, best_q)
            converged_candidates.append((d, best_q.copy()))
            if d >= 0.0:
                break

    # (A) among converged candidates, prefer the sterically cleanest (largest
    # min self-distance) -- the well-packed "native state".
    if converged_candidates:
        d_best, q_best = max(converged_candidates, key=lambda c: c[0])
        global_best_q = q_best

    q = global_best_q if global_best_q is not None else q0.copy()

    # ---------- Stability-checked termination (native kinetic stability) ----
    if success:
        _, _, base_combined = _combined_err(spec, q, T_target)
        jitter_failures = 0
        n_jit = 5
        for _ in range(n_jit):
            qj = spec.clip(q + rng.normal(0, 0.001, size=n))
            _, _, cj = _combined_err(spec, qj, T_target)
            if cj > base_combined + 10 * (pos_tol + 0.3 * orient_tol):
                jitter_failures += 1
        if jitter_failures >= n_jit - 1:
            success = False

    T_final = end_effector_pose(spec, q)
    err_final = pose_error(T_final, T_target)
    wall_ms = (time.perf_counter() - t0) * 1000.0
    violations = int(np.sum((q <= spec.joint_limits[:, 0] + 1e-9) |
                            (q >= spec.joint_limits[:, 1] - 1e-9)))

    return SolveResult(
        solver_name="protein_ik_v3",
        success=success,
        q_final=q.tolist(),
        pos_error=float(np.linalg.norm(err_final[:3])),
        orient_error=float(np.linalg.norm(err_final[3:])),
        iterations=total_iters,
        wall_time_ms=wall_ms,
        min_self_distance=self_collision_min_distance(spec, q),
        joint_limit_violations=violations,
        restarts=total_rescues,
        steps=steps if steps is not None else [],
    )
