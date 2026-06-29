"""
Raw (V6) energy terms — Phase 1: van der Waals (Lennard-Jones 6-12).

Faithful to the coarse-grained Cα folding model (see ``raw_math.md`` §3.1):
the arm's joint origins are treated as beads, and every NON-ADJACENT bead
pair interacts via a FULL 6-12 Lennard-Jones potential

    E_LJ = Σ_{|i-j|>=2}  4 ε [ (σ_ij/d_ij)^12  −  S2·(σ_ij/d_ij)^6 ],   d_ij = ||p_i − p_j||

with S2 = 1 (attractive, the physics) or S2 = 0 (repulsion-only, the ablation
baseline). The retained −(σ/d)^6 attraction is the part with **no IK
equivalent**: every existing IK self-collision model keeps only the repulsive
wall. The attractive well (minimum at d = 2^(1/6)·σ, depth −ε) gives the chain
a *preferred* inter-bead spacing — emergent structure from physics, not a rule.

σ_ij = sigma_scale·(r_i + r_j) couples the well location to the link radii;
ε is **uniform** across all pairs (non-Gō — structure must emerge, not be
planted toward a known native state).

Public API:
  bead_positions(spec, q)            -> (n+1, 3) joint-origin beads
  lj_pair(d, sigma, epsilon, ...)    -> scalar pair potential (for tests/plots)
  lj_energy(spec, q, ...)            -> scalar E_LJ
  lj_energy_and_grad(spec, q, ...)   -> (E_LJ, dE/dq)   ANALYTIC force
"""

from __future__ import annotations

import functools

import numpy as np

from app.core.kinematics import RobotSpec, forward_kinematics_chain


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _bead_radii(spec: RobotSpec) -> np.ndarray:
    """Per-bead radius, length (n+1).

    Bead i is the proximal origin of link i; the distal end-effector bead
    (index n) reuses the last link's radius. Used only to set σ_ij.
    """
    r = spec.link_radius
    return np.concatenate([r, r[-1:]])


def bead_positions(spec: RobotSpec, q: np.ndarray) -> np.ndarray:
    """(n+1, 3) joint-origin 'bead' positions from one FK pass."""
    chain = forward_kinematics_chain(spec, q)
    return chain[:, :3, 3]


@functools.lru_cache(maxsize=8)
def _nonadjacent_pairs(n_beads: int) -> tuple[np.ndarray, np.ndarray]:
    """Index arrays (I, J) of every bead pair with |i-j| >= 2.

    Cached per bead-count; the pair set is fixed for a given robot.
    """
    I, J = [], []
    for i in range(n_beads):
        for j in range(i + 2, n_beads):
            I.append(i)
            J.append(j)
    return np.asarray(I, dtype=int), np.asarray(J, dtype=int)


# ---------------------------------------------------------------------------
# Pair potential (scalar) — for unit tests and visualisation
# ---------------------------------------------------------------------------

def lj_pair(d: float, sigma: float, epsilon: float = 1.0,
            attractive: bool = True) -> float:
    """Single-pair Lennard-Jones potential 4ε[(σ/d)^12 − S2(σ/d)^6].

    With attraction (S2=1): zero at d=σ, minimum −ε at d=2^(1/6)·σ.
    """
    s2 = 1.0 if attractive else 0.0
    sr6 = (sigma / d) ** 6
    return float(4.0 * epsilon * (sr6 * sr6 - s2 * sr6))


# ---------------------------------------------------------------------------
# Field energy (scalar)
# ---------------------------------------------------------------------------

def lj_energy(spec: RobotSpec, q: np.ndarray,
              sigma_scale: float = 1.0, epsilon: float = 1.0,
              attractive: bool = True) -> float:
    """Total E_LJ over all non-adjacent bead pairs (energy only, no force)."""
    pts = bead_positions(spec, q)
    radii = _bead_radii(spec)
    I, J = _nonadjacent_pairs(pts.shape[0])
    if I.size == 0:
        return 0.0
    d = np.maximum(np.linalg.norm(pts[I] - pts[J], axis=1), 1e-9)
    sigma = sigma_scale * (radii[I] + radii[J])
    sr6 = (sigma / d) ** 6
    s2 = 1.0 if attractive else 0.0
    return float(np.sum(4.0 * epsilon * (sr6 * sr6 - s2 * sr6)))


# ---------------------------------------------------------------------------
# Field energy + analytic gradient (the force the Langevin step consumes)
# ---------------------------------------------------------------------------

def lj_energy_and_grad(spec: RobotSpec, q: np.ndarray,
                       sigma_scale: float = 1.0, epsilon: float = 1.0,
                       attractive: bool = True) -> tuple[float, np.ndarray]:
    """Return (E_LJ, dE_LJ/dq) with the gradient computed analytically.

    Derivation (raw_math.md §3.1):
        ∂E/∂q_k = Σ_pairs (dE/dd)·(∂d_ij/∂q_k)
        dE/dd   = (24ε/d)·( S2·(σ/d)^6 − 2·(σ/d)^12 )
        ∂d_ij/∂q_k = û_ij·(∂p_i/∂q_k − ∂p_j/∂q_k),  û_ij = (p_i − p_j)/d
        ∂p_m/∂q_k  = z_k × (p_m − p_k)   if k < m  else 0   (revolute joint)
    All quantities come from the single FK chain pass.
    """
    chain = forward_kinematics_chain(spec, q)
    pts = chain[:, :3, 3]                 # (n+1, 3) beads
    n = spec.n_joints
    n_beads = n + 1
    grad = np.zeros(n)

    I, J = _nonadjacent_pairs(n_beads)
    if I.size == 0:
        return 0.0, grad

    radii = _bead_radii(spec)
    diff = pts[I] - pts[J]                # (P, 3)
    d = np.maximum(np.linalg.norm(diff, axis=1), 1e-9)
    sigma = sigma_scale * (radii[I] + radii[J])
    sr6 = (sigma / d) ** 6
    sr12 = sr6 * sr6
    s2 = 1.0 if attractive else 0.0

    E = float(np.sum(4.0 * epsilon * (sr12 - s2 * sr6)))

    dE_dd = (24.0 * epsilon / d) * (s2 * sr6 - 2.0 * sr12)   # (P,)
    u = diff / d[:, None]                                    # û_ij  (P, 3)

    z = chain[:n, :3, 2]                  # (n, 3) joint axes
    p = chain[:n, :3, 3]                  # (n, 3) joint origins

    # ∂E/∂q_k for each joint k. Joint k moves only beads m > k.
    for k in range(n):
        dP = np.zeros((n_beads, 3))
        dP[k + 1:] = np.cross(z[k], pts[k + 1:] - p[k])      # z_k × (p_m − p_k)
        contrib = dP[I] - dP[J]                              # (P, 3)
        grad[k] = float(np.sum(dE_dd * np.einsum("pi,pi->p", u, contrib)))

    return E, grad
