"""
ProteinIK V4.1 -- making "ProteinIK Fast" actually fast.

GOAL (project motto): ProteinIK Fast should be the fastest of the protein
lineup and competitive with TRAC-IK, using the protein-folding architecture
plus optimization -- not a pivot away from it.

DIAGNOSIS. V4's *median* already ties TRAC-IK (~11 ms vs ~10 ms on UR5).
What makes V4 look slow is its *tail*: on the ~10% of targets where the
barrierless fast-path misses, V4 fires a full ensemble of stochastic
Metropolis folds, and those solves consume ~57% of total wall time. So the
lever is the tail, and a per-step micro-opt cannot move it. V4.1 attacks the
tail on two fronts, both staying inside the folding domain:

1. BARRIERLESS-FIRST ENSEMBLE (the tail killer). The folding-funnel /
   minimal-frustration principle: a smooth, unfrustrated energy landscape
   folds by downhill diffusion alone -- no stochastic search required. So
   each replica FIRST attempts a cheap barrierless (Levenberg-Marquardt)
   fold from its seed; only a seed whose landscape is *frustrated* (LM fails
   to reach the native state) escalates to the full stochastic Metropolis
   funnel + chaperone rescue. The cheap downhill path resolves the bulk of
   targets in ~TRAC-IK time; the expensive protein machinery fires only
   where frustration actually demands it -- exactly where V4's success and
   self-collision wins come from. Collision-aware native-state selection is
   unchanged: a converged-but-clashing barrierless fold is kept only as a
   fallback and the search continues (cheaply, then via the full fold) for a
   sterically clean basin, so V4's lowest-collision-rate property is
   preserved.

2. ALLOCATION-LIGHT FK PRIMITIVES (the per-step floor). Profiling showed the
   hot loop dominated by forward_kinematics_chain building a fresh 4x4
   np.array literal per joint (~130k allocs / 40 solves) and every DLS/LM
   step allocating a new np.eye(6). V4.1 replaces these with:

     _fast_chain(spec, q): vectorized DH-local build into a preallocated
       buffer + np.matmul(out=); returns the chain AND per-joint locals L.
     _incremental_chain(spec, chain, L, i, cand): when only joint i changes,
       copy frames 0..i and recompute the suffix reusing cached L[k].
     _I6: the constant 6x6 identity, shared by every solve.

   Both FK primitives are verified BIT-IDENTICAL to
   core.forward_kinematics_chain (0.0 max diff over 9000 configs across
   ur5 / franka / planar), so the folds they drive are numerically the same
   fold V4 runs -- only cheaper.

HONESTY. Unlike the V4->V4 micro-pass, point 1 *changes behavior*: V4.1 no
longer reproduces V4's exact trajectory. It is validated by the metrics that
matter -- success rate and self-collision rate held at or above V4's, mean
and tail latency cut -- not by bit-for-bit identity. See the benchmark in
docs for measured numbers.
"""

from __future__ import annotations

import numpy as np
import time

from app.core.kinematics import (
    RobotSpec, end_effector_pose, pose_error, self_collision_min_distance,
    self_collision_min_distance_from_chain,
)
from app.core.types import SolveResult, SolveStep
from app.solvers.protein_energy import (
    total_energy_fast, frustration_index,
    joint_limit_energy, neighbor_smoothness_energy, _collision_energy_from_distance,
)

# Stage-3 energy weights (target, joint-limit, collision, smoothness).
_W = (3.0, 1.0, 2.0, 0.3)

# Constant 6x6 identity reused by every DLS / LM solve (V4 rebuilt np.eye(6)
# on every step -- ~28k allocations / 40 solves). Never mutated.
_I6 = np.eye(6)


def _combined_err(spec: RobotSpec, q: np.ndarray, T_target: np.ndarray) -> tuple[float, float, float]:
    """Returns (pos_err, orient_err, combined). `combined` is the single
    scalar used everywhere for best-tracking / acceptance, matching the
    convention the baselines use (pos + 0.3 * orient)."""
    T_cur = end_effector_pose(spec, q)
    err = pose_error(T_cur, T_target)
    pos_e = float(np.linalg.norm(err[:3]))
    orient_e = float(np.linalg.norm(err[3:]))
    return pos_e, orient_e, pos_e + 0.3 * orient_e


def _fast_chain(spec: RobotSpec, q: np.ndarray,
                ca: np.ndarray, sa: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Allocation-light forward kinematics. Builds all n per-joint DH local
    transforms with vectorized assignment into a preallocated (n,4,4) buffer
    (no per-joint np.array literal) and chains them with np.matmul(out=).

    Returns (chain (n+1,4,4), L (n,4,4)) where L[i] is joint i's local DH
    transform -- cached so _incremental_chain can reuse the unchanged ones.
    `ca`/`sa` are cos/sin of spec.alpha, precomputed once by the caller.
    Bit-identical to core.forward_kinematics_chain."""
    n = spec.n_joints
    thetas = q + spec.theta_offset
    ct = np.cos(thetas); st = np.sin(thetas)
    a = spec.a; d = spec.d
    L = np.zeros((n, 4, 4))
    L[:, 0, 0] = ct;  L[:, 0, 1] = -st * ca; L[:, 0, 2] = st * sa;  L[:, 0, 3] = a * ct
    L[:, 1, 0] = st;  L[:, 1, 1] = ct * ca;  L[:, 1, 2] = -ct * sa; L[:, 1, 3] = a * st
    L[:, 2, 1] = sa;  L[:, 2, 2] = ca;       L[:, 2, 3] = d
    L[:, 3, 3] = 1.0
    T = np.empty((n + 1, 4, 4))
    T[0] = np.eye(4)
    for i in range(n):
        np.matmul(T[i], L[i], out=T[i + 1])
    return T, L


def _incremental_chain(spec: RobotSpec, chain: np.ndarray, L: np.ndarray,
                       i: int, cand: float,
                       ca: np.ndarray, sa: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Chain for a config that differs from the one that produced `chain`
    only at joint i (set to `cand`). Frames 0..i are unchanged; rebuild the
    one changed local transform and propagate from frame i+1, reusing the
    cached L[k] for k>i. Returns (new_chain, Li). Bit-identical to a full
    _fast_chain / core.forward_kinematics_chain recompute."""
    n = spec.n_joints
    theta_i = cand + spec.theta_offset[i]
    ct = np.cos(theta_i); st = np.sin(theta_i)
    Li = np.zeros((4, 4))
    Li[0, 0] = ct; Li[0, 1] = -st * ca[i]; Li[0, 2] = st * sa[i]; Li[0, 3] = spec.a[i] * ct
    Li[1, 0] = st; Li[1, 1] = ct * ca[i];  Li[1, 2] = -ct * sa[i]; Li[1, 3] = spec.a[i] * st
    Li[2, 1] = sa[i]; Li[2, 2] = ca[i]; Li[2, 3] = spec.d[i]
    Li[3, 3] = 1.0
    T = np.empty_like(chain)
    T[:i + 1] = chain[:i + 1]
    np.matmul(T[i], Li, out=T[i + 1])
    for k in range(i + 1, n):
        np.matmul(T[k], L[k], out=T[k + 1])
    return T, Li


def _energy_from_chain(spec: RobotSpec, chain: np.ndarray, q: np.ndarray,
                       T_target: np.ndarray, w_target: float, w_limit: float,
                       w_collision: float, w_smooth: float) -> float:
    """total_energy_fast, but consuming an already-computed FK chain instead
    of recomputing it. Bit-identical to total_energy_fast(spec, q, ...) for
    the same q (chain must be the FK chain of q)."""
    e = 0.0
    if w_target > 0:
        err = pose_error(chain[-1], T_target)
        e += w_target * float(np.linalg.norm(err[:3]) + 0.3 * np.linalg.norm(err[3:]))
    if w_limit > 0:
        e += w_limit * joint_limit_energy(spec, q)
    if w_collision > 0:
        d_min = self_collision_min_distance_from_chain(spec, chain)
        e += w_collision * _collision_energy_from_distance(d_min)
    if w_smooth > 0:
        e += w_smooth * neighbor_smoothness_energy(q)
    return e


def _fast_pose_jac(spec: RobotSpec, q: np.ndarray,
                   ca: np.ndarray, sa: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """One forward-kinematics pass -> (end_effector_pose 4x4, geometric Jacobian
    6xN), using the allocation-light _fast_chain. Numerically identical to V4's
    _fast_pose_jac and to core end_effector_pose + geometric_jacobian."""
    chain, _ = _fast_chain(spec, q, ca, sa)
    n = spec.n_joints
    pose = chain[n]
    z = chain[:n, :3, 2]
    p = chain[:n, :3, 3]
    d = chain[n, :3, 3] - p
    J = np.empty((6, n))
    J[0, :] = z[:, 1] * d[:, 2] - z[:, 2] * d[:, 1]
    J[1, :] = z[:, 2] * d[:, 0] - z[:, 0] * d[:, 2]
    J[2, :] = z[:, 0] * d[:, 1] - z[:, 1] * d[:, 0]
    J[3:, :] = z.T
    return pose, J


def _lm_polish_fast(spec: RobotSpec, q0: np.ndarray, T_target: np.ndarray,
                    pos_tol: float, orient_tol: float, max_steps: int,
                    ca: np.ndarray, sa: np.ndarray,
                    lam0: float = 0.08) -> tuple[np.ndarray, float, bool, int]:
    """V4's adaptive Levenberg-Marquardt endgame, using _fast_pose_jac and the
    cached identity. Behavior identical to V4's _lm_polish_fast; only faster."""
    q = q0.copy()
    pose, J = _fast_pose_jac(spec, q, ca, sa)
    err = pose_error(pose, T_target)
    pos_e = float(np.linalg.norm(err[:3])); orient_e = float(np.linalg.norm(err[3:]))
    e_cur = pos_e + 0.3 * orient_e
    lam = lam0
    steps_used = 0
    for _ in range(max_steps):
        if pos_e < pos_tol and orient_e < orient_tol:
            return q, e_cur, True, steps_used
        steps_used += 1
        dq = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * _I6, err)
        q_try = spec.clip(q + dq)
        pose_t, J_t = _fast_pose_jac(spec, q_try, ca, sa)
        err_t = pose_error(pose_t, T_target)
        p_t = float(np.linalg.norm(err_t[:3])); o_t = float(np.linalg.norm(err_t[3:]))
        e_try = p_t + 0.3 * o_t
        if e_try < e_cur:
            q, J, err, pos_e, orient_e, e_cur = q_try, J_t, err_t, p_t, o_t, e_try
            lam = max(lam * 0.5, 1e-4)
        else:
            lam = min(lam * 2.5, 2.0)
            if lam >= 2.0:
                break
    return q, e_cur, (pos_e < pos_tol and orient_e < orient_tol), steps_used


def _fold_once_v41(
    spec: RobotSpec, q0: np.ndarray, T_target: np.ndarray, rng: np.random.Generator,
    max_iters: int, pos_tol: float, orient_tol: float,
    stage2_iters: int, stuck_window: int, stuck_eps: float,
    steps: list | None, it_offset: int, ca: np.ndarray, sa: np.ndarray,
) -> tuple[np.ndarray, float, bool, int, int]:
    """One folding trajectory -- IDENTICAL to V4's _fold_once_v4, but the
    Metropolis funnel sweep scores single-joint perturbations via incremental
    FK, and every FK/Jacobian uses the allocation-light fast primitive."""
    n = spec.n_joints
    lo = spec.joint_limits[:, 0]
    hi = spec.joint_limits[:, 1]
    q = q0.copy()
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
        nonlocal best_q, best_combined
        _, _, c = _combined_err(spec, cand_q, T_target)
        if c < best_combined:
            best_combined = c
            best_q = cand_q.copy()

    # ---------- STAGE 2: coarse collapse (hydrophobic collapse) ----------
    for _ in range(stage2_iters):
        it += 1
        pose, J = _fast_pose_jac(spec, q, ca, sa)
        err = pose_error(pose, T_target)
        dq = J.T @ np.linalg.solve(J @ J.T + (0.15 ** 2) * _I6, err)
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
        T0, Tf = 0.3, 0.01
        frac = min(it / max_iters, 1.0)
        temp = T0 * (Tf / T0) ** frac

        # SAME full-chain Metropolis funnel search as V4 (same acceptance, same
        # RNG draw order) -- but each single-joint candidate is scored with an
        # incremental FK recompute instead of a full chain rebuild.
        if it % 2 == 0:
            chain, L = _fast_chain(spec, q, ca, sa)
            for i in range(n):
                cand = np.clip(q[i] + rng.uniform(-search_radius, search_radius), lo[i], hi[i])
                chain_try, Li = _incremental_chain(spec, chain, L, i, cand, ca, sa)
                q_try = q.copy(); q_try[i] = cand
                e_try = _energy_from_chain(spec, chain_try, q_try, T_target, *_W)
                if e_try < cur_energy or rng.uniform() < np.exp(-(e_try - cur_energy) / max(temp, 1e-6)):
                    q, cur_energy = q_try, e_try
                    chain = chain_try
                    L[i] = Li  # cache the one changed local; frames k>i reused it already

        # fused pose+Jacobian (one FK pass) for the downhill step
        pose, J = _fast_pose_jac(spec, q, ca, sa)
        err = pose_error(pose, T_target)
        pos_e, orient_e = float(np.linalg.norm(err[:3])), float(np.linalg.norm(err[3:]))

        if pos_e < 0.05 and orient_e < 0.2:
            q, _, conv, _ = _lm_polish_fast(spec, q, T_target, pos_tol, orient_tol, 12, ca, sa)
            cur_energy = total_energy_fast(spec, q, T_target, *_W)
            record("funnel_lm_endgame", energy=cur_energy)
            consider(q)
            if conv:
                return best_q, best_combined, True, it, rescues
        else:
            dq = J.T @ np.linalg.solve(J @ J.T + (0.05 ** 2) * _I6, err)
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
            if recent[0] - recent[-1] < stuck_eps:
                rescues += 1
                scope_sizes = sorted(set([
                    max(1, n // 6),
                    max(1, n // 2),
                    max(1, 5 * n // 6),
                    n,
                ]))
                scope = scope_sizes[min(rescues - 1, len(scope_sizes) - 1)]
                if scope >= n:
                    q = spec.random_config(rng)
                else:
                    contributions = frustration_index(spec, q, T_target)
                    worst = int(np.argmax(contributions))
                    half = scope // 2
                    lo_s = max(0, worst - half)
                    hi_s = min(n, lo_s + scope)
                    lo_s = max(0, hi_s - scope)
                    fresh = spec.random_config(rng)
                    for j in range(lo_s, hi_s):
                        q[j] = fresh[j]
                search_radius = 0.5
                recent = []
                cur_energy = total_energy_fast(spec, q, T_target, *_W)
                record(f"chaperone_rescue_{scope}", energy=cur_energy)
                consider(q)

    return best_q, best_combined, False, it, rescues


def solve_protein_fast_v41(
    spec: RobotSpec,
    q0: np.ndarray,
    T_target: np.ndarray,
    rng: np.random.Generator,
    max_iters: int = 150,
    pos_tol: float = 1e-3,
    orient_tol: float = 1e-2,
    collect_steps: bool = False,
    stage2_iters: int = 10,
    stuck_window: int = 10,
    stuck_eps: float = 2e-4,
    max_replicas: int = 6,
    lm_restarts: int = 4,
) -> SolveResult:
    """Barrierless-first ensemble driver. Phase A tries up to `lm_restarts`
    cheap barrierless (LM) folds -- the funnel-hypothesis fast path that
    resolves unfrustrated landscapes in ~TRAC-IK time. Only if no sterically
    clean native state is found does Phase B escalate to up to `max_replicas`
    full stochastic Metropolis folds (the collision-aware protein machinery).
    Uses the allocation-light / incremental FK primitives throughout."""
    n = spec.n_joints
    t0 = time.perf_counter()
    steps: list | None = [] if collect_steps else None

    # cos/sin of the (constant) link twists -- precomputed once per solve and
    # threaded through every FK call so the hot loop never recomputes them.
    ca = np.cos(spec.alpha); sa = np.sin(spec.alpha)

    # contact-order-inspired difficulty scaling of the per-replica budget
    reach_needed = float(np.linalg.norm(T_target[:3, 3]))
    max_reach = float(np.sum(np.abs(spec.a) + np.abs(spec.d)))
    reach_ratio = min(reach_needed / max_reach, 1.0) if max_reach > 0 else 0.0
    _, J0 = _fast_pose_jac(spec, q0, ca, sa)
    sv = np.linalg.svd(J0, compute_uv=False)
    cond = sv[0] / (sv[-1] + 1e-6)
    difficulty = 1.0 + min(reach_ratio, 1.0) + min(cond / 100.0, 1.0)
    s2 = int(stage2_iters * difficulty)

    global_best_q = None
    global_best_combined = np.inf
    total_iters = 0
    total_rescues = 0
    success = False
    converged_candidates: list[tuple[float, np.ndarray]] = []

    def _have_clean() -> bool:
        return any(d >= 0.0 for d, _ in converged_candidates)

    # ---------- PHASE A: barrierless folding restarts (funnel hypothesis) ----------
    # A smooth, unfrustrated landscape folds by downhill diffusion alone -- no
    # stochastic search needed. Try cheap LM folds from q0, then fresh random
    # seeds; most targets resolve here in ~TRAC-IK time. Stop as soon as one
    # converges to a sterically CLEAN native state (collision-aware policy,
    # identical to V4's: a clashing converged pose is kept only as a fallback).
    for r in range(max(1, lm_restarts)):
        seed = q0.copy() if r == 0 else spec.random_config(rng)
        q_lm, e_lm, conv, lm_steps = _lm_polish_fast(
            spec, seed, T_target, pos_tol, orient_tol, 30, ca, sa)
        total_iters += lm_steps
        if e_lm < global_best_combined:
            global_best_combined = e_lm
            global_best_q = q_lm.copy()
        if conv:
            success = True
            d = self_collision_min_distance(spec, q_lm)
            converged_candidates.append((d, q_lm.copy()))
            if d >= 0.0:
                break  # clean barrierless fold -- done on the fast path

    # ---------- PHASE B: stochastic funnel folding (frustrated landscapes) ----------
    # Only escalate if no clean barrierless solution exists. This is the full
    # staged fold -- Metropolis funnel + chaperone rescue + collision energy --
    # i.e. the protein machinery that wins success + steric quality on the hard
    # tail. Skipped entirely on the easy bulk, which is where the speedup comes
    # from.
    if not _have_clean():
        # Count only Phase-B (collision-aware) converged folds toward the early
        # stop -- Phase A's collision-BLIND LM candidates must not short-circuit
        # the collision-seeking search, or clutter targets keep a clashing pose
        # the full fold would have cleaned. Mirrors V4's "2 converged candidates"
        # budget, but measured on the collision-aware folds only.
        phase_b_converged = 0
        for replica in range(max_replicas):
            if _have_clean() or phase_b_converged >= 2:
                break
            seed = q0.copy() if replica == 0 else spec.random_config(rng)
            best_q, best_combined, conv, iters, rescues = _fold_once_v41(
                spec, seed, T_target, rng, max_iters, pos_tol, orient_tol,
                s2, stuck_window, stuck_eps, steps, total_iters, ca, sa,
            )
            total_iters += iters
            total_rescues += rescues
            if best_combined < global_best_combined:
                global_best_combined = best_combined
                global_best_q = best_q.copy()
            if conv:
                success = True
                phase_b_converged += 1
                d = self_collision_min_distance(spec, best_q)
                converged_candidates.append((d, best_q.copy()))
                if d >= 0.0:
                    break

    if converged_candidates:
        _, q_best = max(converged_candidates, key=lambda c: c[0])
        global_best_q = q_best

    q = global_best_q if global_best_q is not None else q0.copy()

    # ---------- stability-checked termination (native kinetic stability) ----
    if success:
        _, _, base_combined = _combined_err(spec, q, T_target)
        jitter_failures = 0
        n_jit = 5
        arm_reach = max(0.1, float(np.sum(np.abs(spec.a)) + np.sum(np.abs(spec.d))))
        jitter_std = float(np.clip(1e-3 / max(1.0, arm_reach), 1e-4, 5e-3))
        for _ in range(n_jit):
            qj = spec.clip(q + rng.normal(0, jitter_std, size=n))
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
        solver_name="protein_fast_v41",
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
