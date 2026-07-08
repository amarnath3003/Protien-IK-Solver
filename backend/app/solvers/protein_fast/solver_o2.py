"""
ProteinIK Fast -- OPT2 / o2 variant (faithful iterative-annealing warm start).

EXPERIMENTAL (2026-07-08). Ported from the `v4-speed-opt` fork's `solver_opt2.py`
onto the CURRENT corrected-modified-DH base V4, so it can be benchmarked on the
real-mesh oracles (PyBullet + MuJoCo). Base V4 (`solve_protein_fast`) is untouched;
this is an isolated, removable variant registered as `protein_fast_o2`.

Same folding architecture / energy / collision-aware native selection as
`solve_protein_fast`. The ONLY change is how Phase B's first stochastic fold is
seeded -- the *biologically faithful* warm start (contrast the plain o1 warm start,
which relaxed the trapped state in place and eroded the self-collision edge):

  GroEL / IAM PARTIAL UNFOLD. Phase A leaves a trapped intermediate -- a fold that
  reached the target but is sterically frustrated (clashing). The iterative
  annealing mechanism does not refold a trapped substrate from a random coil
  (base), nor relax it in place (o1); it *partially unfolds* the frustrated
  substructure and lets it re-descend the funnel. So the chaperone identifies the
  most frustrated joints (via `frustration_index`) and kicks ONLY those partway up
  the funnel with a bounded stochastic unfold -- not a full random reseed -- then
  the fold re-anneals. This keeps the stochastic re-exploration that produces the
  collision edge, while starting warm (near the funnel) for speed.

Everything else delegates to the base module's primitives, so the fold itself is
the same fold on the same (corrected) kinematics -- only its Phase-B entry differs.
"""
from __future__ import annotations

import numpy as np
import time

from app.core.kinematics import (
    end_effector_pose, pose_error, self_collision_min_distance,
)
from app.core.types import SolveResult
from app.solvers.protein_energy import frustration_index
from app.solvers.protein_fast.solver import (
    _fast_pose_jac, _lm_polish_fast, _fold_once, _combined_err,
)

# Bounded partial-unfold kick magnitude (rad). Large enough to escape the trapped
# clashing basin (re-diversify), small enough to stay near the funnel (fast) --
# i.e. a partial unfold, not a full random reseed.
_UNFOLD_KICK = 0.7


def solve_protein_fast_o2(
    spec,
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
) -> SolveResult:
    n = spec.n_joints
    t0 = time.perf_counter()
    steps: list | None = [] if collect_steps else None

    ca = np.cos(spec.alpha); sa = np.sin(spec.alpha)

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

    # ---------- PHASE A: barrierless LM restarts (unchanged) ----------
    for r in range(max_replicas):
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
                break

    # ---------- PHASE B: IAM partial-unfold warm start ----------
    if not _have_clean():
        warm_seed = None
        if converged_candidates:
            warm_seed = max(converged_candidates, key=lambda c: c[0])[1].copy()
        s2_warm = max(2, s2 // 4)

        phase_b_converged = 0
        for replica in range(max_replicas):
            if _have_clean() or phase_b_converged >= 2:
                break
            if replica == 0 and warm_seed is not None:
                # GroEL/IAM partial unfold of the frustrated substructure:
                # kick the top-half most-frustrated joints partway up the funnel,
                # keep the rest of the (already-formed) fold, then re-anneal.
                contributions = frustration_index(spec, warm_seed, T_target)
                k = max(1, n // 2)
                worst = np.argsort(contributions)[-k:]
                seed = warm_seed.copy()
                seed[worst] = seed[worst] + rng.uniform(-_UNFOLD_KICK, _UNFOLD_KICK, size=k)
                seed = spec.clip(seed)
                s2_use = s2_warm
            else:
                seed = q0.copy() if replica == 0 else spec.random_config(rng)
                s2_use = s2
            best_q, best_combined, conv, iters, rescues = _fold_once(
                spec, seed, T_target, rng, max_iters, pos_tol, orient_tol,
                s2_use, stuck_window, stuck_eps, steps, total_iters, ca, sa,
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

    # ---------- stability-checked termination (unchanged) ----------
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
        solver_name="protein_fast_o2",
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
