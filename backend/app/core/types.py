"""
Shared types for solver results and step traces.

Every solver -- classical or protein-IK -- returns a `SolveResult` and
(optionally) a list of `SolveStep` snapshots. This is what makes the
benchmark dashboard and the live-streaming view possible: all solvers
are instrumented identically, so metrics are directly comparable and the
frontend doesn't need solver-specific rendering logic.
"""

from __future__ import annotations

import time
import numpy as np
from dataclasses import dataclass, field, asdict
from typing import Optional


@dataclass
class SolveStep:
    """One iteration snapshot, used for live-streaming / 'replay' views."""
    iteration: int
    q: list  # joint angles at this step
    pos_error: float  # meters
    orient_error: float  # radians
    min_self_distance: float  # meters (steric clash proxy)
    phase: str = ""  # solver-specific label, e.g. "local_relax", "collapse", "rescue"
    energy: Optional[float] = None  # solver-specific scalar, if applicable

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class SolveResult:
    """Final outcome + summary metrics for a single IK solve attempt."""
    solver_name: str
    success: bool
    q_final: list
    pos_error: float
    orient_error: float
    iterations: int
    wall_time_ms: float
    min_self_distance: float
    joint_limit_violations: int
    restarts: int = 0  # number of restarts/branch-resets used, if applicable
    steps: list = field(default_factory=list)  # list[SolveStep], optional full trace
    # CCH-IK diagnostic outputs (default 0.0 / 1.0 for all other solvers)
    conflict_index:   float = 0.0  # full-vector cosine conflict C ∈ [0,2] at final step
    lambda_final:     float = 1.0  # homotopy parameter λ at termination ∈ [0,1]
    difficulty_score: float = 0.0  # mean C over full trajectory — how hard was this solve?
                                   # 0 = objectives always cooperated, 2 = always opposed
                                   # valid even when success=False; pure diagnostic output
    # ProteinIK Raw (V6) diagnostic outputs (default 0.0 for all other solvers)
    sigma_ratio:  float = 0.0  # landscape funnel quality Σ=σ_E/ΔE; <1 funnelled, >1 glassy
    free_energy:  float = 0.0  # F(q,T)=E_task+E_LJ+E_HB−T·S_conf at the returned config
    t_glass:      float = 0.0  # REM glass temperature — the Langevin cooling target

    def to_dict(self, include_steps: bool = True) -> dict:
        d = {
            "solver_name": self.solver_name,
            "success": self.success,
            "q_final": self.q_final,
            "pos_error": self.pos_error,
            "orient_error": self.orient_error,
            "iterations": self.iterations,
            "wall_time_ms": self.wall_time_ms,
            "min_self_distance": self.min_self_distance,
            "joint_limit_violations": self.joint_limit_violations,
            "restarts": self.restarts,
            "conflict_index":   self.conflict_index,
            "lambda_final":     self.lambda_final,
            "difficulty_score": self.difficulty_score,
            "sigma_ratio":      self.sigma_ratio,
            "free_energy":      self.free_energy,
            "t_glass":          self.t_glass,
        }
        if include_steps:
            d["steps"] = [s.to_dict() if isinstance(s, SolveStep) else s for s in self.steps]
        return d


class Timer:
    """Tiny context-manager stopwatch returning milliseconds."""
    def __enter__(self):
        self._t0 = time.perf_counter()
        return self

    def __exit__(self, *exc):
        self.ms = (time.perf_counter() - self._t0) * 1000.0
