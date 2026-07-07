"""
Phase 3 payoff: drive the *real* (PyBullet) self-collision rate down.

Post-migration, PyBullet is our collision authority. A single solve lands in
whatever IK branch the solver's start config funnels to, which may self-collide on
the real meshes even when the target is perfectly reachable clean. `solve_clean`
generates several candidate solutions for the *same* target from diverse start
configs (which explore different IK branches — elbow up/down, wrist flips), scores
each by PyBullet's real mesh collision, and returns the one the simulator certifies
cleanest.

Key design (consistent with the whole migration, plan §4): the simulator is used
only at the **boundary** (scoring the K finished candidates), never inside a
solver's iteration loop, so the fast numpy core is untouched. This is an
**offline / planning-grade** mode — it costs ~K solves + K collision queries per
target — matching where the ProteinIK family is actually deployed (goal sampling,
fallback, offline clean-solve), not a real-time servo.

The candidate solver is arbitrary (`protein_fast` by default) — the wrapper lowers
real collision for whatever generator you hand it.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from app.core.kinematics import RobotSpec
from app.core.types import SolveResult
from app.solvers.registry import run_solver


@dataclass
class CleanSolveResult:
    """Outcome of a K-candidate, real-collision-selected solve."""
    result: SolveResult | None      # the selected candidate's full SolveResult (None if all failed)
    sim_min_self_distance: float    # PyBullet clearance of the selected candidate (m; neg = colliding)
    sim_in_collision: bool
    n_candidates: int               # successful candidates found (<= K)
    n_collision_free: int           # how many of them PyBullet certified clean
    # Candidate 0 == the honest single-shot (caller's q0), for a free before/after.
    single_success: bool = False
    single_sim_min_self_distance: float = float("nan")
    single_in_collision: bool = False


def solve_clean(backend, solver: str, spec: RobotSpec, q0: np.ndarray,
                T_target: np.ndarray, K: int = 16, seed: int = 0) -> CleanSolveResult:
    """Return the lowest-PyBullet-collision successful solve among K candidates.

    Candidate 0 uses the caller's ``q0`` (so it subsumes the honest single-shot
    result); candidates 1..K-1 start from random configs to reach different IK
    branches. ``backend`` is a live ``PyBulletBackend`` for ``spec``'s robot.
    """
    best: SolveResult | None = None
    best_clear = -np.inf
    n_ok = 0
    n_clean = 0
    single_success = False
    single_clear = float("nan")
    for k in range(max(1, K)):
        rng = np.random.default_rng(seed * 100_003 + k)
        q_start = np.asarray(q0, float) if k == 0 else spec.random_config(rng)
        r = run_solver(solver, spec, q_start, T_target, rng)
        if not r.success:
            continue
        n_ok += 1
        _, clear = backend.self_collision(np.asarray(r.q_final, float))
        n_clean += int(clear >= 0.0)
        if k == 0:                       # honest single-shot baseline
            single_success = True
            single_clear = float(clear)
        if clear > best_clear:
            best_clear, best = clear, r
    if best is None:
        return CleanSolveResult(None, float("nan"), False, 0, 0,
                                single_success, single_clear,
                                bool(single_clear < 0.0) if single_success else False)
    return CleanSolveResult(
        result=best,
        sim_min_self_distance=float(best_clear),
        sim_in_collision=bool(best_clear < 0.0),
        n_candidates=n_ok,
        n_collision_free=n_clean,
        single_success=single_success,
        single_sim_min_self_distance=single_clear,
        single_in_collision=bool(single_clear < 0.0) if single_success else False,
    )
