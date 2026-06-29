"""Pydantic schemas for API requests and responses."""

from __future__ import annotations

from pydantic import BaseModel, Field
from typing import Optional


class TargetPose(BaseModel):
    """End-effector target pose as position + quaternion (xyzw), the
    conventional format for a 3D frontend (three.js) to send/receive."""
    position: list[float] = Field(..., min_length=3, max_length=3)
    quaternion: list[float] = Field(..., min_length=4, max_length=4)  # x, y, z, w


class SolveRequest(BaseModel):
    solver: str
    robot: str = "ur5"           # which arm to use; defaults to UR5 for backwards compat
    q0: Optional[list[float]] = None  # if omitted, a random valid seed is used
    target: TargetPose
    seed: Optional[int] = None
    collect_steps: bool = True


class RandomTargetRequest(BaseModel):
    robot: str = "ur5"
    seed: Optional[int] = None


class BatchBenchmarkRequest(BaseModel):
    solvers: list[str]
    robot: str = "ur5"           # which arm to benchmark on
    n_trials: int = Field(default=100, ge=1, le=2000)
    seed: int = 1
    scenario: str = "open_space"  # "open_space" | "near_singular" | "cluttered"


class RobotSpecResponse(BaseModel):
    name: str
    n_joints: int
    a: list[float]
    d: list[float]
    alpha: list[float]
    joint_limits: list[list[float]]
    link_radius: list[float]

