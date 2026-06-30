"""
Raw (V6) energy terms — Phase 1: van der Waals (LJ 6-12); Phase 2: directional H-bond.

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
  hbond_energy(spec, q, d0, ...)         -> scalar E_HB (directional)
  hbond_energy_and_grad(spec, q, d0, ...)-> (E_HB, dE/dq)  FD force
  calibrate_hbond_d0(spec, rng)          -> preferred H-bond distance d0
  config_entropy(spec, q, ...)           -> scalar S = log Omega (target-blind, clash-aware)
  config_entropy_and_grad(spec, q, ...)  -> (S, dS/dq)  FD force (common random numbers)
"""

from __future__ import annotations

import functools

import numpy as np

from app.core.kinematics import (
    RobotSpec, forward_kinematics_chain, self_collision_min_distance,
)


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


# ---------------------------------------------------------------------------
# Phase 2 — directional hydrogen bond (raw_math.md §3.2)
# ---------------------------------------------------------------------------
# Each INTERIOR bead carries a local backbone normal — the unit normal to the
# plane of its triplet (p_{i-1}, p_i, p_{i+1}). This is the faithful CG H-bond
# direction (Enciso & Rey), NOT the joint axis z_i. An H-bond between two
# non-adjacent interior beads is stabilising only when the inter-bead distance
# is near d0 AND both backbone normals are aligned with the inter-bead
# direction. That distance-AND-orientation gate is what makes real H-bonds
# directional and is what creates secondary structure — and it has no IK
# equivalent (the Jacobian captures influence, not preferred geometry).


def _bead_normals(pts: np.ndarray) -> np.ndarray:
    """(n+1, 3) unit backbone normals. Endpoint beads (0, n) and any collinear
    triplet are left as zero → they do not participate in H-bonds."""
    n_beads = pts.shape[0]
    t = np.zeros((n_beads, 3))
    if n_beads >= 3:
        v1 = pts[1:-1] - pts[:-2]        # p_i − p_{i-1}
        v2 = pts[2:] - pts[1:-1]         # p_{i+1} − p_i
        c = np.cross(v1, v2)
        nc = np.linalg.norm(c, axis=1)
        good = nc > 1e-9
        c[good] = c[good] / nc[good, None]
        c[~good] = 0.0
        t[1:-1] = c
    return t


@functools.lru_cache(maxsize=8)
def _interior_pairs(n_beads: int) -> tuple[np.ndarray, np.ndarray]:
    """Non-adjacent pairs (|i-j| >= 2) among INTERIOR beads (1..n_beads-2),
    i.e. beads that have a defined backbone normal. Chains shorter than 5
    beads have none (too short for secondary structure — like a tripeptide)."""
    I, J = [], []
    for i in range(1, n_beads - 1):
        for j in range(i + 2, n_beads - 1):
            I.append(i)
            J.append(j)
    return np.asarray(I, dtype=int), np.asarray(J, dtype=int)


def _hb_distance_factor(d, d0, sigma_d):
    """Gaussian distance gate F(d): 1 at d=d0, decaying with width sigma_d."""
    return np.exp(-((d - d0) ** 2) / (2.0 * sigma_d ** 2))


def _hb_angle_factor(x, kappa):
    """Angular gate: 1 when |x|=1 (normal aligned with the inter-bead
    direction), exp(-kappa) when perpendicular. x = t_hat · r_hat."""
    return np.exp(-kappa * (1.0 - np.abs(x)))


def hbond_energy(spec: RobotSpec, q: np.ndarray, d0: float, sigma_d: float,
                 kappa: float = 3.0, epsilon_hb: float = 1.0,
                 directional: bool = True) -> float:
    """Total directional H-bond energy (<= 0).

    directional=False drops the angular gates → a distance-only contact
    potential (the ablation: contacts WITHOUT orientation), used to show the
    directionality is what produces secondary structure.
    """
    pts = bead_positions(spec, q)
    I, J = _interior_pairs(pts.shape[0])
    if I.size == 0:
        return 0.0
    diff = pts[J] - pts[I]
    d = np.maximum(np.linalg.norm(diff, axis=1), 1e-9)
    F = _hb_distance_factor(d, d0, sigma_d)
    if directional:
        t = _bead_normals(pts)
        rhat = diff / d[:, None]
        a = np.einsum("pi,pi->p", t[I], rhat)
        b = np.einsum("pi,pi->p", t[J], rhat)
        ang = _hb_angle_factor(a, kappa) * _hb_angle_factor(b, kappa)
    else:
        ang = 1.0
    return float(-epsilon_hb * np.sum(F * ang))


def hbond_energy_and_grad(spec: RobotSpec, q: np.ndarray, d0: float,
                          sigma_d: float, kappa: float = 3.0,
                          epsilon_hb: float = 1.0, directional: bool = True,
                          fd_eps: float = 1e-6) -> tuple[float, np.ndarray]:
    """(E_HB, dE_HB/dq) with the force by central finite differences
    (raw_math.md §3.2: FD first, analytic later). 2n FK passes per call —
    acceptable for the research solver, like V5's FD constraint gradient."""
    E = hbond_energy(spec, q, d0, sigma_d, kappa, epsilon_hb, directional)
    n = spec.n_joints
    g = np.zeros(n)
    for i in range(n):
        qp, qm = q.copy(), q.copy()
        qp[i] += fd_eps
        qm[i] -= fd_eps
        ep = hbond_energy(spec, qp, d0, sigma_d, kappa, epsilon_hb, directional)
        em = hbond_energy(spec, qm, d0, sigma_d, kappa, epsilon_hb, directional)
        g[i] = (ep - em) / (2.0 * fd_eps)
    return E, g


def calibrate_hbond_d0(spec: RobotSpec, rng: np.random.Generator,
                       n_samples: int = 200) -> float:
    """Preferred H-bond distance d0 = median interior-pair bead distance over
    random configs (the analog of backbone geometry setting H-bond distance).
    Returns 0.0 for chains too short to have interior H-bond pairs."""
    I, J = _interior_pairs(spec.n_joints + 1)
    if I.size == 0:
        return 0.0
    ds = []
    for _ in range(n_samples):
        pts = bead_positions(spec, spec.random_config(rng))
        ds.append(np.linalg.norm(pts[J] - pts[I], axis=1))
    return float(np.median(np.concatenate(ds)))


# ---------------------------------------------------------------------------
# Phase 3 — configurational entropy  S = log Omega  (raw_math.md §3.3)
# ---------------------------------------------------------------------------
# The hydrophobic / free-energy term. The hydrophobic effect is entropic; in a
# coarse-grained chain the favourable SOLVENT entropy is already folded into the
# attractive LJ contacts (Phase 1), so the term that remains here is the CHAIN
# CONFORMATIONAL ENTROPY — the local clash-free, in-limits accessible volume.
# It is high for open/extended configs, low for compact / near-collision /
# near-limit ones, so it OPPOSES collapse (favours the unfolded ensemble at high
# T) and competes with E_LJ; temperature controls the balance ⇒ the folding
# transition.
#
# Two properties make it RAW (no IK equivalent), unlike manipulability:
#   • target-blind  — it never references the EE target (manipulability / null-
#                     space redundancy resolution is task-relative).
#   • collision-aware — it counts only clash-free accessible volume; manipulability
#                     (a Jacobian determinant) is blind to self-collision.
# It is the local free-volume estimator standard in polymer physics: sample a
# cloud of local perturbations, count the excluded-volume-respecting ones.


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.clip(x, -60.0, 60.0)))


def entropy_stencil(n: int, m: int, rho: float, seed: int = 12345) -> np.ndarray:
    """Fixed cloud of m local perturbations (m, n), Gaussian std rho. Fixing the
    cloud (common random numbers) is what makes the finite-difference entropy
    gradient smooth rather than Monte-Carlo-noisy."""
    rng = np.random.default_rng(seed)
    return rho * rng.standard_normal((m, n))


def config_entropy(spec: RobotSpec, q: np.ndarray,
                   rho: float = 0.15, m: int = 64, margin: float = 0.0,
                   alpha_clash: float = 50.0, alpha_lim: float = 30.0,
                   floor: float = 1e-6, stencil: np.ndarray | None = None,
                   seed: int = 12345) -> float:
    """S_conf(q) = log Omega(q).

    Omega = soft fraction of a local perturbation cloud around q that stays
    within joint limits AND clash-free (capsule proxy). Target-blind and
    collision-aware. S <= 0; S = 0 means fully open (maximal local freedom).
    """
    if stencil is None:
        stencil = entropy_stencil(spec.n_joints, m, rho, seed)
    lo, hi = spec.joint_limits[:, 0], spec.joint_limits[:, 1]
    qk = q[None, :] + stencil                                    # (m, n)
    # soft joint-limit feasibility per sample (Ramachandran-like accessible set)
    lim = np.prod(_sigmoid(alpha_lim * (qk - lo)) * _sigmoid(alpha_lim * (hi - qk)), axis=1)
    # soft clash (excluded-volume) feasibility per sample
    clash = np.empty(stencil.shape[0])
    for k in range(stencil.shape[0]):
        d = self_collision_min_distance(spec, qk[k])
        clash[k] = _sigmoid(alpha_clash * (d - margin))
    omega = float(np.mean(lim * clash))
    return float(np.log(max(omega, floor)))


def config_entropy_and_grad(spec: RobotSpec, q: np.ndarray,
                            rho: float = 0.15, m: int = 64, margin: float = 0.0,
                            alpha_clash: float = 50.0, alpha_lim: float = 30.0,
                            floor: float = 1e-6, seed: int = 12345,
                            fd_eps: float = 1e-3) -> tuple[float, np.ndarray]:
    """(S_conf, dS_conf/dq) by central finite differences with a FIXED stencil
    (common random numbers). Cost (2n+1)*m capsule evaluations — this is the
    expensive term, so keep m modest in the hot loop (like V5's FD gradient)."""
    stencil = entropy_stencil(spec.n_joints, m, rho, seed)
    kw = dict(margin=margin, alpha_clash=alpha_clash, alpha_lim=alpha_lim,
              floor=floor, stencil=stencil)
    S = config_entropy(spec, q, **kw)
    n = spec.n_joints
    g = np.zeros(n)
    for i in range(n):
        qp, qm = q.copy(), q.copy()
        qp[i] += fd_eps
        qm[i] -= fd_eps
        g[i] = (config_entropy(spec, qp, **kw) - config_entropy(spec, qm, **kw)) / (2.0 * fd_eps)
    return S, g
