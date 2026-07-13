"""WSL environment setup shared by every native-benchmark entry point.

Applies exactly the two runtime overrides needed to reuse the repo's own
scoring + solver code unchanged inside WSL:
  1. point robot_descriptions at the same Windows cache the original run used;
  2. override ``resolve_urdf_path('ur5')`` to the identical original URDF file
     (robot_descriptions 3.0.0 renamed the ur5 module; the cached file is intact).
Nothing here edits a tracked repo file — these are process-local monkeypatches.
"""
from __future__ import annotations

import importlib
import os
import sys

BACKEND = "/mnt/c/Coding Projects/Protien IK/backend"
CACHE = "/mnt/c/Users/Amarnath/.cache/robot_descriptions"
UR5_URDF = CACHE + "/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf"


def apply() -> None:
    os.environ.setdefault("ROBOT_DESCRIPTIONS_CACHE", CACHE)
    if BACKEND not in sys.path:
        sys.path.insert(0, BACKEND)

    import app.sim.models as models
    _orig = models.resolve_urdf_path

    def _resolve(robot: str) -> str:
        return UR5_URDF if robot == "ur5" else _orig(robot)

    models.resolve_urdf_path = _resolve
    # Patch the names already bound inside the backends (they did `from ... import`).
    for modname in ("app.sim.pybullet_backend", "app.sim.mujoco_backend"):
        try:
            importlib.import_module(modname).resolve_urdf_path = _resolve
        except Exception:
            pass
