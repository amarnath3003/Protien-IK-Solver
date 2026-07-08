"""
ProteinIK Fast -- CALIB variant (de-biased collision model).

EXPERIMENTAL (2026-07-08). Answers "is V4 specialised to the capsule proxy, and if we
un-bias it does it gain?" Identical to base `solve_protein_fast` in EVERY way except the
collision model it searches against: instead of the default capsule radii (blind in the
wrist cluster), it uses radii CALIBRATED against the real PyBullet mesh so the proxy is
conservative (few dangerous false-clears). See `bench/calibrate_radii.py`.

Calibrated per-link radii (uniform +Δr chosen to minimise weighted false-clear):
  UR5    +10 mm/link -> false-clear 0.0%, but false-alarm 62.5% (UR5 can't be de-biased
         cheaply -- the wrist blind spot needs so much inflation the arm turns over-cautious)
  Franka +14 mm/link -> false-clear 8.6%, false-alarm 0.1% (nearly free)

Base V4 (`solve_protein_fast`) is untouched; this is an isolated, removable variant
registered as `protein_fast_calib`. FK / success are unaffected (only link_radius changes,
so the fold reasons about a fatter, real-mesh-honest arm; pose-reaching is identical).
"""
from __future__ import annotations

import dataclasses
import numpy as np

from app.solvers.protein_fast.solver import solve_protein_fast

# Real-mesh-calibrated capsule radii, keyed by DOF (6=UR5, 7=Franka). Planar (3) has no
# URDF/mesh oracle, so it keeps the default radii (nothing to calibrate against).
_CALIB_RADIUS = {
    6: np.array([0.07, 0.06, 0.055, 0.05, 0.05, 0.045]),          # UR5  (+0.010)
    7: np.array([0.064, 0.054, 0.039, 0.039, 0.039, 0.034, 0.029]),  # Franka (+0.014)
}


def solve_protein_fast_calib(spec, q0, T_target, rng, collect_steps=False):
    new_r = _CALIB_RADIUS.get(spec.n_joints)
    spec2 = dataclasses.replace(spec, link_radius=new_r) if new_r is not None else spec
    return solve_protein_fast(spec2, q0, T_target, rng, collect_steps=collect_steps)
