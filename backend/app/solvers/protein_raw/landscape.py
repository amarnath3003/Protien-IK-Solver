"""
Raw (V6) — Phase 4: landscape topology (Σ ratio + glass temperature).

The 4th raw contribution: an IK difficulty predictor measured from the energy
LANDSCAPE *before* solving, not from trial-and-error (no IK equivalent — IK
difficulty is read from point quantities like manipulability/condition number,
or learned). It is the Bryngelson–Wolynes foldability criterion (raw_math.md §6):

    Σ = σ_E / ΔE          σ_E = std of E over random configs
                          ΔE  = mean(E over random) − E_native
    Σ < 1  ⇔  Z > 1  ⇔  funnelled landscape  → easy / fast fold
    Σ > 1            ⇔  glassy landscape       → kinetically trapped / hard

Σ is the reciprocal of the folding Z-score. E_native is unknown in IK (it is
the solution we are solving for), so we use a cheap warm-start (a few Jacobian
steps) as the native-energy proxy — the one IK-specific departure, stated openly.

The cooling target (raw_math.md §5):
    T_glass ≈ σ_E / sqrt(2 · S₀)     S₀ = configurational entropy scale (REM)
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from app.core.kinematics import (
    RobotSpec, end_effector_pose, pose_error, geometric_jacobian,
)
from app.solvers.protein_energy import target_energy
from app.solvers.protein_raw.energy import (
    bead_positions, lj_energy, hbond_energy, calibrate_hbond_d0,
    _bead_radii, _nonadjacent_pairs,
)

WELL = 2.0 ** (1.0 / 6.0)


# ---------------------------------------------------------------------------
# Calibrated parameters (one place; reused by the Phase-5 solver)
# ---------------------------------------------------------------------------

@dataclass
class RawParams:
    sigma_scale: float          # LJ length scale (well at median bead spacing)
    epsilon: float = 1.0        # LJ depth
    e_cap: float = 50.0         # per-pair LJ repulsion cap (landscape sampling)
    d0: float = 0.0             # H-bond preferred distance (0 ⇒ chain too short)
    sigma_d: float = 0.05       # H-bond distance width
    kappa: float = 2.0          # H-bond angular sharpness
    epsilon_hb: float = 1.0     # H-bond depth
    rho: float = 0.15           # entropy local-cloud scale (also sets S₀)

    @classmethod
    def calibrate(cls, spec: RobotSpec, rng: np.random.Generator,
                  n_samples: int = 200) -> "RawParams":
        """Fit geometry-dependent scales once per robot."""
        radii = _bead_radii(spec)
        I, J = _nonadjacent_pairs(spec.n_joints + 1)
        if I.size:
            ds = [np.linalg.norm(
                    bead_positions(spec, spec.random_config(rng))[I]
                    - bead_positions(spec, spec.random_config(rng))[J], axis=1)
                  for _ in range(n_samples)]
            d_med = float(np.median(np.concatenate(ds)))
            sigma_scale = d_med / (WELL * float(np.mean(radii[I] + radii[J])))
        else:
            sigma_scale = 1.0
        d0 = calibrate_hbond_d0(spec, rng)
        return cls(sigma_scale=sigma_scale, d0=d0,
                   sigma_d=(0.25 * d0 if d0 > 0 else 0.05))


# ---------------------------------------------------------------------------
# Potential (the enthalpic landscape; entropy excluded — Σ is over energy)
# ---------------------------------------------------------------------------

def bio_energy(spec: RobotSpec, q: np.ndarray, p: RawParams) -> float:
    """E_LJ (capped) + E_HB — the target-blind biophysical potential."""
    e = lj_energy(spec, q, p.sigma_scale, p.epsilon, True, e_cap=p.e_cap)
    if p.d0 > 0:
        e += hbond_energy(spec, q, p.d0, p.sigma_d, p.kappa, p.epsilon_hb, True)
    return e


def warm_start(spec: RobotSpec, q0: np.ndarray, T_target: np.ndarray,
               steps: int = 40, damping: float = 0.1) -> np.ndarray:
    """Cheap native-energy proxy: a few damped-least-squares (DLS) steps toward
    the target. DLS is stable through singularities (the λ²I term), so it
    reliably reduces task error; it still converges to a worse residual on
    genuinely hard targets, which correctly raises Σ there."""
    q = q0.copy()
    eye = np.eye(6)
    for _ in range(steps):
        err = pose_error(end_effector_pose(spec, q), T_target)
        J = geometric_jacobian(spec, q)
        dq = J.T @ np.linalg.solve(J @ J.T + (damping ** 2) * eye, err)
        q = spec.clip(q + dq)
    return q


def _sigma_from_energies(E: np.ndarray) -> dict:
    """Σ = σ_E / ΔE from an energy ensemble; native = the ensemble minimum.
    Funnelled (one deep minimum well below the rest) → ΔE large → Σ small.
    Glassy (many states near the minimum) → ΔE small → Σ large."""
    E = np.asarray(E, dtype=float)
    e_native = float(np.min(E))
    sigma_E = float(np.std(E))
    delta_E = float(np.mean(E) - e_native)
    return {"sigma": sigma_E / max(delta_E, 1e-8),
            "sigma_E": sigma_E, "delta_E": delta_E, "E_native": e_native}


def sigma_ratio(spec: RobotSpec, T_target: np.ndarray, p: RawParams,
                rng: np.random.Generator, n_seeds: int = 24,
                ws_steps: int = 60) -> dict:
    """Σ over the Bryngelson–Wolynes COMPACT ENSEMBLE — the set of approximate
    solutions reached by warm-starting from many random seeds (the IK analog of
    the molten-globule ensemble). Native = the best (lowest-energy) solution
    found. Task and bio are balanced to equal variance so the steric (bio)
    frustration is not swamped by the task term.

    Honest scope: Σ characterises the funnel quality of the COLLISION-AWARE
    landscape. It is complementary to V5's during-solve conflict diagnostic and
    to a collision-blind solver's notion of difficulty — see the Phase-4
    experiment for the measured (modest) correlation."""
    sols = [warm_start(spec, spec.random_config(rng), T_target, steps=ws_steps)
            for _ in range(n_seeds)]
    task = np.array([target_energy(spec, q, T_target) for q in sols])
    bio = np.array([bio_energy(spec, q, p) for q in sols])
    w = float(np.std(task) / max(np.std(bio), 1e-9))
    out = _sigma_from_energies(task + w * bio)
    out["w_bio"] = w
    return out


# ---------------------------------------------------------------------------
# Glass temperature (REM) — the cooling target
# ---------------------------------------------------------------------------

def configurational_entropy_scale(spec: RobotSpec, rho: float = 0.15) -> float:
    """S₀ ≈ Σ_j log( joint-range / cloud-cell ) — log of the number of
    rho-resolution cells the chain can occupy (a configurational entropy)."""
    lo, hi = spec.joint_limits[:, 0], spec.joint_limits[:, 1]
    cells = np.maximum((hi - lo) / (2.0 * rho), 1.0001)
    return float(np.sum(np.log(cells)))


def glass_temperature(sigma_E: float, s0: float) -> float:
    """T_glass = σ_E / sqrt(2·S₀)  (REM, Bryngelson 1987)."""
    return float(sigma_E / np.sqrt(2.0 * max(s0, 1e-6)))
