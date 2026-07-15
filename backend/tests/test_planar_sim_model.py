"""The planar N-DOF sim model must stay faithful to the arm the DOF-scaling
experiment actually solves on, and to the proxy it is cross-checked against.

These lock in the two invariants the paper's Section 5.4 claim rests on:

  1. ``app.sim.planar_model.planar_ndof_spec`` == ``usecase_experiments.planar_ndof_spec``.
     If these drift, the engines would score a *different arm* than the solvers
     ran on, and the corroboration would be silently meaningless.

  2. With capsule geometry, each engine's self-collision distance equals the
     capsule proxy's to floating-point precision. This is what licenses the
     paper's "the DOF-scaling clean-solve rates are reproduced by both engines"
     sentence -- and it is a real regression guard on the URDF frame algebra
     (link_i must carry proxy segment i-1; get that wrong and the distances
     diverge immediately).

Engine tests skip cleanly where pybullet/mujoco aren't installed (they live in
WSL, not the Windows dev venv -- see native_bench/README.md).
"""
from __future__ import annotations

import sys

import numpy as np
import pytest

from app.core.kinematics import ROBOT_REGISTRY, self_collision_min_distance
from app.sim import planar_model

DOFS = [4, 8, 16]


def _reset(dof: int) -> str:
    """Drop a *generated* arm so it can be rebuilt under another geometry.

    Never touches built-in arms: ROBOT_REGISTRY is module-global, so popping
    planar3dof here would break every later test in the session (and would
    defeat test_refuses_to_shadow_builtin_planar3dof outright).
    """
    robot = f"planar{dof}dof"
    if robot in planar_model.GENERATED_URDFS:
        planar_model.GENERATED_URDFS.pop(robot, None)
        ROBOT_REGISTRY.pop(robot, None)
    return robot


@pytest.mark.parametrize("dof", DOFS)
def test_spec_matches_usecase_experiments(dof: int):
    """The sim layer's arm is byte-for-byte the experiment's arm."""
    sys.path.insert(0, "backend/scrap")
    usecase = pytest.importorskip("usecase_experiments")

    ours = planar_model.planar_ndof_spec(dof)
    theirs = usecase.planar_ndof_spec(dof)
    assert ours.n_joints == theirs.n_joints
    for field in ("a", "d", "alpha", "theta_offset", "link_radius", "joint_limits"):
        np.testing.assert_allclose(getattr(ours, field), getattr(theirs, field),
                                   err_msg=f"planar{dof}dof drift in '{field}'")


def test_refuses_to_shadow_builtin_planar3dof():
    """planar3dof is a hand-written spec with its own radii/limits, and the
    master survey scores against it -- generating a uniform DOF-sweep arm over
    the top of it would silently change that arm's geometry."""
    assert "planar3dof" in ROBOT_REGISTRY, "built-in planar3dof went missing"
    with pytest.raises(ValueError, match="already a built-in robot"):
        planar_model.register_planar_arm(3)


@pytest.mark.parametrize("dof", DOFS)
@pytest.mark.parametrize("engine", ["pybullet", "mujoco"])
def test_capsule_engine_matches_proxy(dof: int, engine: str):
    """Capsule geometry: engine distance == proxy distance, to fp precision.

    A capsule's surface gap IS segment-distance-minus-radii, so any real gap
    here means the URDF misplaces a link relative to the proxy's segments.
    """
    pytest.importorskip(engine)
    robot = _reset(dof)
    planar_model.register_planar_arm(dof, geom_type="capsule")

    if engine == "pybullet":
        from app.sim.pybullet_backend import PyBulletBackend as Backend
    else:
        from app.sim.mujoco_backend import MuJoCoBackend as Backend

    spec = planar_model.planar_ndof_spec(dof)
    be = Backend(robot)
    try:
        rng = np.random.default_rng(0)
        for q in rng.uniform(-np.pi, np.pi, size=(50, dof)):
            d_proxy = self_collision_min_distance(spec, q)
            in_col, d_eng = be.self_collision(q)
            # Only compare where the engine actually measured (it saturates far pairs).
            if d_eng < 0.49:
                assert abs(d_proxy - d_eng) < 1e-9, (
                    f"{engine} planar{dof}dof: proxy {d_proxy:.12f} vs engine "
                    f"{d_eng:.12f} -- URDF/proxy geometry disagree"
                )
            assert (d_proxy < 0.0) == in_col, "collision verdict disagrees"
    finally:
        be.close()


@pytest.mark.parametrize("dof", DOFS)
def test_generated_urdf_loads_in_pybullet_both_geoms(dof: int):
    """Bullet's importer chokes on our long single-line XML; we emit indented XML.

    Regression guard for the 16-DOF cylinder case, which failed to load with a
    bogus XML_ERROR_PARSING_ATTRIBUTE until build_planar_urdf started indenting.
    """
    p = pytest.importorskip("pybullet")
    for geom in ("capsule", "cylinder"):
        spec = planar_model.planar_ndof_spec(dof)
        path = planar_model.build_planar_urdf(spec, geom)
        cid = p.connect(p.DIRECT)
        try:
            p.loadURDF(path)  # raises if Bullet can't parse it
        finally:
            p.disconnect(cid)
