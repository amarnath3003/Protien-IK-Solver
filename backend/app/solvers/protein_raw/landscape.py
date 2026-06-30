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
               steps: int = 40, lr: float = 0.05) -> np.ndarray:
    """Cheap native-energy proxy: a few Jacobian-transpose steps toward the
    target. Converges fast on easy targets, stalls near singular ones — which
    correctly RAISES Σ (smaller gap) for hard targets."""
    q = q0.copy()
    for _ in range(steps):
        err = pose_error(end_effector_pose(spec, q), T_target)
        q = spec.clip(q + lr * (geometric_jacobian(spec, q).T @ err))
    return q


def sigma_ratio(spec: RobotSpec, T_target: np.ndarray, p: RawParams,
                rng: np.random.Generator, n_samples: int = 400) -> dict:
    """Σ = σ_E/ΔE for this target. Task and bio are balanced to equal variance
    (per target) so the diagnostic is comparable across targets, then E_native
    is the warm-start energy under the SAME balanced potential."""
    qs = [spec.random_config(rng) for _ in range(n_samples)]
    task = np.array([target_energy(spec, q, T_target) for q in qs])
    bio = np.array([bio_energy(spec, q, p) for q in qs])
    w = float(np.std(task) / max(np.std(bio), 1e-9))      # balance bio ↔ task

    E = task + w * bio
    q_nat = warm_start(spec, qs[int(np.argmin(task))], T_target)
    E_nat = float(target_energy(spec, q_nat, T_target) + w * bio_energy(spec, q_nat, p))

    sigma_E = float(np.std(E))
    delta_E = float(np.mean(E) - E_nat)
    sigma = sigma_E / max(delta_E, 1e-8)
    return {"sigma": sigma, "sigma_E": sigma_E, "delta_E": delta_E,
            "E_native": E_nat, "w_bio": w}


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
