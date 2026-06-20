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
