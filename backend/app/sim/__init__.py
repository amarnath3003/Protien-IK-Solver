"""
Simulator adapter + oracle layer (see ``sim_migration_plan.md``).

The solvers are already decoupled from our in-house kinematic sim: every one
consumes a ``RobotSpec`` and depends only on ``app.core.kinematics``. This
package does NOT rewrite any solver. It builds a thin layer that lets a real,
widely-trusted robotics simulator (PyBullet first, MuJoCo later) become the
*source of truth* for the three things that are currently our own private
definitions: the robot model, forward kinematics, and self-collision.

Phase status (see the plan's §5):
  * Phase 0  — acquire & pin models .................. DONE (see ``models.py``)
  * Phase 1  — DH<->URDF FK parity harness .......... DONE (see ``parity.py``)
  * Phase 2+ — evaluation oracle, collision reconcile, MuJoCo ... not yet

Everything here imports PyBullet lazily, so importing ``app`` (the FastAPI app,
the solvers, the existing test suite) never requires PyBullet to be installed.
Only code that actually talks to the sim pays that import cost, and it raises a
clear, actionable error if PyBullet is missing.
"""

from __future__ import annotations

__all__ = ["parity", "models"]
