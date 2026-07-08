"""
Solver registry: maps solver name -> a uniform callable signature so the
API layer doesn't need solver-specific branching logic.

Every entry is wrapped to accept (spec, q0, T_target, rng, collect_steps)
even though not every underlying solver needs an rng -- this keeps the
API and benchmark-runner code identical regardless of which solver is
selected.
"""

from __future__ import annotations

import numpy as np
from typing import Callable

from app.core.kinematics import RobotSpec
from app.core.types import SolveResult
from app.solvers.jacobian_dls import solve_dls
from app.solvers.ccd import solve_ccd
from app.solvers.fabrik import solve_fabrik
from app.solvers.trac_ik_style import solve_trac_ik
from app.solvers.multi_start import solve_multi_start
from app.solvers.protein_ik import solve_protein_ik
from app.solvers.protein_homotopy import solve_protein_homotopy
from app.solvers.fixed_lambda_ik import solve_fixed_lambda_ik
from app.solvers.protein_fast import solve_protein_fast
from app.solvers.protein_fast.solver_o2 import solve_protein_fast_o2
from app.solvers.protein_fast.solver_calib import solve_protein_fast_calib
from app.solvers.protein_raw import solve_protein_raw
from app.solvers.analytical_planar3dof import solve_analytical_planar3dof


def _wrap_no_rng(fn) -> Callable:
    def wrapped(spec, q0, T_target, rng, collect_steps=False):
        return fn(spec, q0, T_target, collect_steps=collect_steps)
    return wrapped


def _wrap_rng(fn) -> Callable:
    def wrapped(spec, q0, T_target, rng, collect_steps=False):
        return fn(spec, q0, T_target, rng, collect_steps=collect_steps)
    return wrapped


SOLVER_REGISTRY: dict[str, Callable[..., SolveResult]] = {
    "jacobian_dls": _wrap_no_rng(solve_dls),
    "ccd": _wrap_no_rng(solve_ccd),
    "fabrik": _wrap_no_rng(solve_fabrik),
    "trac_ik_style": _wrap_rng(solve_trac_ik),
    "multi_start": _wrap_rng(solve_multi_start),
    "protein_ik":         _wrap_rng(solve_protein_ik),
    "fixed_lambda_ik":    _wrap_rng(solve_fixed_lambda_ik),
    "protein_homotopy":   _wrap_rng(solve_protein_homotopy),
    "protein_fast":       _wrap_rng(solve_protein_fast),
    "protein_fast_o2":    _wrap_rng(solve_protein_fast_o2),
    "protein_fast_calib": _wrap_rng(solve_protein_fast_calib),
    "protein_raw":        _wrap_rng(solve_protein_raw),
    # Planar 3-DOF analytical solver — only valid when robot='planar3dof'
    "analytical_planar3dof": _wrap_rng(solve_analytical_planar3dof),
}

ROBOT_SOLVER_COMPAT: dict[str, list[str]] = {
    # analytical_planar3dof is only valid for the 2D planar arm.
    # Every other solver operates purely on RobotSpec and is DOF-agnostic.
    "planar3dof":   list(SOLVER_REGISTRY.keys()),
    "ur5":          [s for s in SOLVER_REGISTRY if s != "analytical_planar3dof"],
    "franka_panda": [s for s in SOLVER_REGISTRY if s != "analytical_planar3dof"],
}


def get_solvers_for_robot(robot: str) -> list[str]:
    """Return the ordered list of solver ids valid for the given robot."""
    return ROBOT_SOLVER_COMPAT.get(
        robot, [s for s in SOLVER_REGISTRY if s != "analytical_planar3dof"]
    )


SOLVER_DISPLAY_NAMES: dict[str, str] = {
    "jacobian_dls": "Jacobian (DLS)",
    "ccd": "CCD",
    "fabrik": "FABRIK",
    "trac_ik_style": "TRAC-IK style",
    "multi_start": "Multi-start",
    "protein_ik":       "ProteinIK (V1)",
    "fixed_lambda_ik":  "Fixed-λ Homotopy (Baseline)",
    "protein_homotopy": "ProteinIK Homotopy (CCH-IK)",
    "protein_fast":     "ProteinIK Fast (V4)",
    "protein_fast_o2":  "ProteinIK Fast (V4+o2 IAM)",
    "protein_fast_calib": "ProteinIK Fast (V4 real-calib)",
    "protein_raw":      "ProteinIK Raw Biology (V6)",
    "analytical_planar3dof": "Analytical IK (Planar 3-DOF, exact)",
}


def run_solver(name: str, spec: RobotSpec, q0: np.ndarray, T_target: np.ndarray,
                rng: np.random.Generator, collect_steps: bool = False) -> SolveResult:
    if name not in SOLVER_REGISTRY:
        raise ValueError(f"Unknown solver '{name}'. Available: {list(SOLVER_REGISTRY)}")
    return SOLVER_REGISTRY[name](spec, q0, T_target, rng, collect_steps=collect_steps)
