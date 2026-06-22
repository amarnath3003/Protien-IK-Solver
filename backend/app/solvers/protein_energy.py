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


def ramachandran_pair_energy(q: np.ndarray, pair_wells: list) -> float:
    """2D joint-pair energy analogous to Ramachandran plot wells.
    For each joint pair (i, i+1), finds the distance to the nearest
    empirical well center in the 2D joint space."""
    e = 0.0
    for i in range(len(q) - 1):
        wells = pair_wells[i]
        pt = np.array([q[i], q[i+1]])
        dists = np.sum((wells - pt)**2, axis=1)
        e += float(np.min(dists))
    return e * 2.0


def go_contact_energy(chain: np.ndarray, native_contacts: list) -> float:
    """Gō-model inspired native contact energy.
    Pulls the chain toward a structurally favorable topology by penalizing
    deviation from native distances for specific non-adjacent link pairs.
    Takes a precomputed FK chain to avoid redundant computation."""
    if not native_contacts:
        return 0.0
    e = 0.0
    pts = chain[:, :3, 3]
    for i, j, target_d in native_contacts:
        # Distance between joint i and j origins
        d = float(np.linalg.norm(pts[i] - pts[j]))
        # Only penalize if they are further than target (attraction only)
        if d > target_d:
            e += (d - target_d)**2
    return e * 5.0


def frustration_index(spec: RobotSpec, q: np.ndarray, T_target: np.ndarray) -> np.ndarray:
    """Frustration index for chaperone rescue.
    Measures conflict between local preference (smoothness)
    and global need (gradient toward target). Highly frustrated joints
    are those that are locally trapped but globally required to move.
    """
    from app.core.kinematics import geometric_jacobian
    n = len(q)
    
    # 1. Global need (Jacobian gradient toward target)
    T_cur = end_effector_pose(spec, q)
    err = pose_error(T_cur, T_target)
    J = geometric_jacobian(spec, q)
    global_dq = J.T @ err
    q_global = q + global_dq
    
    # 2. Local preference (Smoothness pulls to neighbor average)
    q_local_min = np.zeros(n)
    for i in range(n):
        if i == 0:
            q_local_min[i] = q[1]
        elif i == n - 1:
            q_local_min[i] = q[n - 2]
        else:
            q_local_min[i] = (q[i - 1] + q[i + 1]) / 2.0
            
    # Frustration: absolute difference between where local and global want to go
    return np.abs(q_local_min - q_global)


def total_energy_fast(spec: RobotSpec, q: np.ndarray, T_target: np.ndarray,
                       w_target: float, w_limit: float, w_collision: float, w_smooth: float,
                       w_ramo: float = 0.0, pair_wells: list = None,
                       w_go: float = 0.0, native_contacts: list = None) -> float:
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
    if w_ramo > 0 and pair_wells is not None:
        e += w_ramo * ramachandran_pair_energy(q, pair_wells)
    if w_go > 0 and native_contacts is not None:
        e += w_go * go_contact_energy(chain, native_contacts)
    return e
