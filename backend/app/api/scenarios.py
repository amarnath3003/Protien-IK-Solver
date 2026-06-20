"""
Benchmark scenario generators.

Provides different target-pose distributions so the dashboard can compare
solvers honestly across difficulty regimes, not just one "easy" random
distribution. This matters specifically because the open-space random
distribution (our default so far) turned out to already be ~40%
near-singular by manipulability index, yet TRAC-IK-style still
outperformed ProteinIK there -- the more interesting open question is
whether a *collision-heavy* distribution changes that picture, since
that's the one place ProteinIK's collision-energy-aware staged search has
a plausible structural advantage over methods with no collision
awareness in their convergence loop (CCD, FABRIK) or that only re-seed
globally on stuck (TRAC-IK).
"""

from __future__ import annotations

import numpy as np

from app.core.kinematics import RobotSpec, end_effector_pose, geometric_jacobian, self_collision_min_distance


def _random_q(spec: RobotSpec, rng: np.random.Generator) -> np.ndarray:
    return rng.uniform(-np.pi, np.pi, spec.n_joints)


def generate_target(spec: RobotSpec, rng: np.random.Generator, scenario: str):
    """Returns (q0, T_target): a random start config and a target pose,
    generated according to the requested scenario distribution."""
    if scenario == "open_space":
        return _open_space(spec, rng)
    elif scenario == "near_singular":
        return _near_singular(spec, rng)
    elif scenario == "cluttered":
        return _cluttered(spec, rng)
    else:
        raise ValueError(f"Unknown scenario '{scenario}'. Use: open_space, near_singular, cluttered")


def _open_space(spec: RobotSpec, rng: np.random.Generator):
    """Default: uniform random joint configs for both start and target.
    (This is the distribution used in all benchmarking so far.)"""
    q_true = _random_q(spec, rng)
    T_target = end_effector_pose(spec, q_true)
    q0 = _random_q(spec, rng)
    return q0, T_target


def _near_singular(spec: RobotSpec, rng: np.random.Generator, max_tries: int = 50):
    """Bias target generation toward low-manipulability (near-singular)
    configurations -- rejection sampling on manipulability index."""
    best_q, best_m = None, np.inf
    for _ in range(max_tries):
        q = _random_q(spec, rng)
        J = geometric_jacobian(spec, q)
        m = np.sqrt(max(np.linalg.det(J @ J.T), 0))
        if m < best_m:
            best_m, best_q = m, q
        if m < 0.005:
            break
    T_target = end_effector_pose(spec, best_q)
    q0 = _random_q(spec, rng)
    return q0, T_target


def _cluttered(spec: RobotSpec, rng: np.random.Generator, max_tries: int = 200):
    """Bias target generation toward configurations close to self-collision
    -- rejection sampling favoring low min-self-distance targets, which is
    the regime where the collision-energy term in ProteinIK (and the
    complete absence of any collision-awareness in CCD/FABRIK/TRAC-IK's
    convergence loop) should matter most. Honesty note: TRAC-IK and DLS
    don't reason about self-collision at all during their search -- they
    can converge to colliding solutions without any penalty, so 'success'
    for them only means reaching the target pose, not avoiding clash. This
    scenario is most informative when read alongside min_self_distance in
    the results, not success_rate alone.

    Threshold note: measured empirically that random UR5 configs already
    have a MEDIAN min-self-distance of ~0.0197m (the 6-DOF UR5's wrist
    links are simply close together in most poses), with the distribution
    extending to clearly-penetrating values (5th percentile ~ -0.06m, i.e.
    actual overlap). An earlier version of this function used a 0.02m
    accept-threshold, which is approximately the *median* -- meaning the
    rejection sampling almost always accepted the first sample and never
    actually selected for clutter (verified: it produced statistically
    identical results to the open_space scenario). The threshold below is
    set near the 5th percentile so this scenario actually selects for
    distinctly tighter-than-typical configurations.
    """
    best_q, best_d = None, np.inf
    for _ in range(max_tries):
        q = _random_q(spec, rng)
        d = self_collision_min_distance(spec, q)
        if d < best_d:
            best_d, best_q = d, q
        if d < -0.03:
            break
    T_target = end_effector_pose(spec, best_q)
    q0 = _random_q(spec, rng)
    return q0, T_target
