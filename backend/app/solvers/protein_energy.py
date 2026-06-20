"""
Energy terms for the protein-folding-inspired IK solver ("ProteinIK").

Each function returns a scalar energy (lower is better) and is kept
independently testable. The staged solver (protein_ik.py) combines these
differently in each stage, rather than using one fixed weighted sum from
iteration 1 -- that staging is the actual point of the method (see module
docstring there), not the existence of these terms individually, which
mirror standard IK cost terms (target error, joint limits, collision,
smoothness).
"""

from __future__ import annotations

import numpy as np

from app.core.kinematics import (
    RobotSpec, forward_kinematics_chain, end_effector_pose, pose_error,
    self_collision_min_distance, self_collision_min_distance_from_chain,
)


def target_energy(spec: RobotSpec, q: np.ndarray, T_target: np.ndarray) -> float:
    """Pose-error energy: ||FK(q) - T_target||, position + orientation."""
    T_cur = end_effector_pose(spec, q)
    err = pose_error(T_cur, T_target)
    return float(np.linalg.norm(err[:3]) + 0.3 * np.linalg.norm(err[3:]))


def joint_limit_energy(spec: RobotSpec, q: np.ndarray) -> float:
    """Soft barrier penalty that grows sharply near joint limits.
    Zero in the interior, smoothly increasing near the boundary."""
    lo, hi = spec.joint_limits[:, 0], spec.joint_limits[:, 1]
    rng = hi - lo
    # normalized position in [0, 1] within the limit range
    frac = (q - lo) / rng
    # barrier: large near 0 or 1, ~0 in the middle (quadratic in log-barrier style, but bounded)
    margin = 0.05
    penalty = np.where(
        frac < margin, (margin - frac) ** 2,
        np.where(frac > 1 - margin, (frac - (1 - margin)) ** 2, 0.0)
    )
    return float(np.sum(penalty)) * 50.0


def collision_energy(spec: RobotSpec, q: np.ndarray) -> float:
    """Steric-clash-style repulsion: inverse-distance penalty for
    non-adjacent link pairs that are close together."""
    d_min = self_collision_min_distance(spec, q)
    return _collision_energy_from_distance(d_min)


def _collision_energy_from_distance(d_min: float) -> float:
    if d_min <= 0:
        return 100.0 + abs(d_min) * 100.0  # heavy penalty for actual penetration
    safe_margin = 0.05
    if d_min >= safe_margin:
        return 0.0
    return float((safe_margin - d_min) / safe_margin) ** 2 * 10.0


def neighbor_smoothness_energy(q: np.ndarray) -> float:
    """Encourages adjacent joints to not have wildly different angles --
    a 'coordinated motion' / local-coherence term, the IK analog of
    short-range residue-residue interactions in a folding chain."""
    diffs = np.diff(q)
    return float(np.sum(diffs ** 2)) * 0.5


def neutral_pose_energy(q: np.ndarray, q_neutral: np.ndarray) -> float:
    """Pulls joints toward a neutral/relaxed pose -- used ONLY in the
    target-blind local relaxation stage, where the chain 'settles' before
    any target awareness exists (the secondary-structure-formation
    analog). Never combined with target_energy in the same stage."""
    return float(np.sum((q - q_neutral) ** 2)) * 0.5


def total_energy_fast(spec: RobotSpec, q: np.ndarray, T_target: np.ndarray,
                       w_target: float, w_limit: float, w_collision: float, w_smooth: float) -> float:
    """Combined energy that computes the forward-kinematics chain exactly
    once and derives both target_energy and collision_energy from it,
    instead of each calling its own independent FK pass. Profiling showed
    this redundant double-FK-per-energy-call as the dominant remaining
    cost in the solver's hot inner loop after vectorizing collision
    distance; sharing one FK chain roughly halves the per-call cost
    again. Numerically identical to summing the individual energy
    functions -- verified by direct comparison in tests.
    """
    chain = forward_kinematics_chain(spec, q)
    e = 0.0
    if w_target > 0:
        T_cur = chain[-1]
        err = pose_error(T_cur, T_target)
        e += w_target * float(np.linalg.norm(err[:3]) + 0.3 * np.linalg.norm(err[3:]))
    if w_limit > 0:
        e += w_limit * joint_limit_energy(spec, q)
    if w_collision > 0:
        d_min = self_collision_min_distance_from_chain(spec, chain)
        e += w_collision * _collision_energy_from_distance(d_min)
    if w_smooth > 0:
        e += w_smooth * neighbor_smoothness_energy(q)
    return e
