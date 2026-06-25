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
from app.solvers.protein_ik_v1 import solve_protein_ik_v1
from app.solvers.protein_ik import solve_protein_ik_v2
from app.solvers.protein_ik_v3 import solve_protein_ik_v3
from app.solvers.protein_ik_v4 import solve_protein_ik_v4


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
    "protein_ik": _wrap_rng(solve_protein_ik_v1),
    "protein_ik_v2": _wrap_rng(solve_protein_ik_v2),
    "protein_ik_v3": _wrap_rng(solve_protein_ik_v3),
    "protein_ik_v4": _wrap_rng(solve_protein_ik_v4),
}

SOLVER_DISPLAY_NAMES: dict[str, str] = {
    "jacobian_dls": "Jacobian (DLS)",
    "ccd": "CCD",
    "fabrik": "FABRIK",
    "trac_ik_style": "TRAC-IK style",
    "multi_start": "Multi-start",
    "protein_ik": "ProteinIK V1",
    "protein_ik_v2": "ProteinIK V2",
    "protein_ik_v3": "ProteinIK V3",
    "protein_ik_v4": "ProteinIK V4",
}


def run_solver(name: str, spec: RobotSpec, q0: np.ndarray, T_target: np.ndarray,
                rng: np.random.Generator, collect_steps: bool = False) -> SolveResult:
    if name not in SOLVER_REGISTRY:
        raise ValueError(f"Unknown solver '{name}'. Available: {list(SOLVER_REGISTRY)}")
    return SOLVER_REGISTRY[name](spec, q0, T_target, rng, collect_steps=collect_steps)
