"""
CCH-IK core math — isolated from V1 protein_energy.py.

Provides:
  - compute_conflict()        : full-vector cosine similarity ∈ [-1, 1]
  - e_constraints()           : collision + joint-limit energy
  - fd_constraint_gradient()  : central finite-diff gradient of e_constraints
  - backtracking_line_search(): Armijo step-size selection
  - geometric_seed()          : warm-start initialisation (standard practice)
"""

from __future__ import annotations

import numpy as np

from app.core.kinematics import (
    RobotSpec, end_effector_pose, pose_error, self_collision_min_distance,
    geometric_jacobian,
)
from app.solvers.protein_energy import collision_energy, joint_limit_energy


# ---------------------------------------------------------------------------
# Conflict metric
# ---------------------------------------------------------------------------

def compute_conflict(g_target: np.ndarray, g_constr: np.ndarray,
                     eps: float = 1e-8) -> float:
    """
    Full-vector cosine similarity between the task gradient and the
    constraint gradient, both ∈ ℝⁿ (joint space).

    Returns C ∈ [-1, +1]:
      C = -1  gradients fully aligned  → constraints cooperate with task
      C =  0  orthogonal              → independent objectives
      C = +1  fully opposed           → maximum conflict; hold λ

    Using the full joint-space vector (not per-element scalars) gives a
    continuous, informative measure. Per-element scalar cosines reduce to
    ±1 (binary) because individual elements are scalars.

    No Hessian interpretation is claimed. C is an empirical proxy for
    objective incompatibility, not a convexity indicator.
    """
    dot  = float(g_target @ g_constr)
    norm = (float(np.linalg.norm(g_target)) *
            float(np.linalg.norm(g_constr))) + eps
    return -dot / norm   # negate so aligned = low C, conflicted = high C


# ---------------------------------------------------------------------------
# Energy helpers
# ---------------------------------------------------------------------------

def e_target(spec: RobotSpec, q: np.ndarray,
             T_target: np.ndarray, orient_w: float = 0.3) -> float:
    """Squared pose error: ||pos_err||² + orient_w * ||orient_err||²"""
    T_cur = end_effector_pose(spec, q)
    err = pose_error(T_cur, T_target)
    return float(np.linalg.norm(err[:3]) ** 2
                 + orient_w * np.linalg.norm(err[3:]) ** 2)


def e_constraints(spec: RobotSpec, q: np.ndarray) -> float:
    """Constraint energy = collision_energy + 0.5 * joint_limit_energy."""
    return collision_energy(spec, q) + 0.5 * joint_limit_energy(spec, q)


# ---------------------------------------------------------------------------
# Gradient
# ---------------------------------------------------------------------------

def fd_constraint_gradient(spec: RobotSpec, q: np.ndarray,
                            eps: float = 1e-5) -> np.ndarray:
    """
    Central finite-difference gradient of e_constraints w.r.t. q.

    Cost: 2*n FK evaluations per step (12 for a 6-DOF arm).
    Acceptable for the research solver. Profiling target if speed matters.
    """
    n = spec.n_joints
    g = np.zeros(n)
    for i in range(n):
        qp, qm = q.copy(), q.copy()
        qp[i] += eps
        qm[i] -= eps
        g[i] = (e_constraints(spec, qp) - e_constraints(spec, qm)) / (2.0 * eps)
    return g


# ---------------------------------------------------------------------------
# Line search
# ---------------------------------------------------------------------------

def backtracking_line_search(spec: RobotSpec, q: np.ndarray,
                              g: np.ndarray, T_target: np.ndarray,
                              lambda_: float,
                              alpha0: float = 0.08,
                              beta: float = 0.5,
                              max_halvings: int = 6) -> float:
    """
    Armijo backtracking: find α ∈ (0, alpha0] such that
      E(q - α*g, λ) < E(q, λ)
    where E(q, λ) = e_target(q) + λ * e_constraints(q).

    Returns alpha0 / 2^k for the first k where descent holds.
    Falls back to the smallest tried if no improvement found.
    """
    E_cur = e_target(spec, q, T_target) + lambda_ * e_constraints(spec, q)
    alpha = alpha0
    for _ in range(max_halvings):
        q_try = spec.clip(q - alpha * g)
        E_try = e_target(spec, q_try, T_target) + lambda_ * e_constraints(spec, q_try)
        if E_try < E_cur:
            return alpha
        alpha *= beta
    return alpha


# ---------------------------------------------------------------------------
# Warm-start seed (Component C)
# ---------------------------------------------------------------------------

def geometric_seed(spec: RobotSpec, q0: np.ndarray,
                   T_target: np.ndarray,
                   max_steps: int = 20) -> np.ndarray:
    """
    Warm-start initialisation: fast unconstrained gradient descent on
    E_target only (λ=0) for `max_steps` steps.

    Uses the analytical geometric Jacobian (one FK pass per step).
    g_true = -J.T @ err_vec  is the TRUE gradient of E_target (uphill).
    Descent step: q -= α * g_true  →  q += α * J.T @ err_vec

    This is standard IK practice — not claimed as biological.
    Returns the improved seed if it reduces task error, else q0.
    """
    q = q0.copy()
    T_cur = end_effector_pose(spec, q)
    e0 = float(np.linalg.norm(pose_error(T_cur, T_target)[:3]))

    for _ in range(max_steps):
        T_cur = end_effector_pose(spec, q)
        err = pose_error(T_cur, T_target)
        # Analytical geometric Jacobian — one FK pass, no FD overhead
        J = geometric_jacobian(spec, q)
        # TRUE gradient of E_target = -J.T @ err_vec (negate the descent dir)
        # Descent: q -= α * g_true  ⟺  q += α * J.T @ err
        q = spec.clip(q + 0.05 * (J.T @ err))

    T_new = end_effector_pose(spec, q)
    e1 = float(np.linalg.norm(pose_error(T_new, T_target)[:3]))
    return q if e1 < e0 else q0.copy()


# ---------------------------------------------------------------------------
# Gradient surgery (Component B — PCGrad-style projection)
# ---------------------------------------------------------------------------

def pcgrad_project(g_constr: np.ndarray, g_target_true: np.ndarray,
                   eps: float = 1e-8) -> np.ndarray:
    """
    PCGrad-style gradient surgery (Yu et al. NeurIPS 2020).

    Both inputs are TRUE gradients (uphill directions):
      g_target_true = -J.T @ err_vec   (gradient of E_target)
      g_constr      = fd_constraint_gradient()  (gradient of E_constraints)

    When g_constr and g_target point in OPPOSITE directions (dot < 0),
    constraints directly oppose task progress.  We remove the opposing
    component of g_constr — the part that climbs E_target — and keep only
    the component orthogonal-to or aligned-with g_target.

    Formally:
        if g_constr · g_target < 0:
            g_constr_proj = g_constr - (g_constr · g_target / ||g_target||²) * g_target
        else:
            g_constr_proj = g_constr   (already cooperative)

    Honest claim: this is a heuristic.  It does not guarantee Pareto
    optimality, but empirically reduces gradient interference.
    """
    dot = float(g_constr @ g_target_true)
    if dot >= 0:
        return g_constr  # cooperative or orthogonal — no surgery needed
    norm_sq = float(g_target_true @ g_target_true) + eps
    return g_constr - (dot / norm_sq) * g_target_true
