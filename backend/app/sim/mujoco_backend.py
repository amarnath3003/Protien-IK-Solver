"""
Phase 4 (sim_migration_plan.md §5): the MuJoCo **second oracle**.

Phase 2 built the PyBullet oracle; this is its independent cross-check. The point
of a second simulator is confidence: if our DH forward kinematics, PyBullet, *and*
MuJoCo all agree on FK, and PyBullet and MuJoCo agree on real-mesh self-collision,
then the Phase-2/Phase-3 findings are engine-independent facts about the robot, not
artifacts of one physics package.

The design mirrors ``pybullet_backend.PyBulletBackend`` deliberately, so the two are
drop-in comparable and the Phase-1 parity machinery (``parity.compute_parity``)
validates both the same way:

  * **Same model.** MuJoCo loads the *identical* URDF PyBullet does (classic UR5
    ``ur5_robot.urdf`` / franka_ros ``panda.urdf`` via ``robot_descriptions``), not
    Menagerie's ur5e/panda MJCF. So this is a pure engine-vs-engine comparison on
    the exact model our DH was validated against -- it isolates *engine* differences
    from *model* differences. MuJoCo can't resolve URDF ``package://`` mesh URIs and
    chokes on ``.dae`` visual meshes, so ``_mujoco_urdf`` rewrites the mesh paths to
    absolute, strips the (visual-only) ``.dae`` geometry, and injects a ``<mujoco>``
    compiler block. Collision meshes are ``.stl`` and load unchanged.

  * **Same frames.** ``fusestatic="false"`` keeps every URDF link as its own MuJoCo
    body, so the EE frame (``tool0`` / ``panda_link8``) exists to compare against our
    DH EE. Body poses are read from ``data.xmat`` (rotation matrix, row-major) and
    ``data.xpos`` -- reading the matrix directly avoids MuJoCo's **wxyz** quaternion
    convention (plan risk #2), which is the opposite of PyBullet/our-API **xyzw**.

  * **Same collision pairs.** ``mj_geomDistance`` (MuJoCo 3.x) is the exact analog of
    PyBullet's ``getClosestPoints``: signed closest distance between two geoms up to
    ``distmax`` (negative == penetrating). We query it over the *same* non-adjacent
    arm-link set PyBullet uses (base + one link per revolute joint; ee/tool/finger
    geoms excluded), with the same parent-child adjacency filter.

  * **Self-checking.** The constant Phase-1 frame offset ``C`` is (re)measured at
    construction via ``compute_parity`` and asserted ``< OFFSET_RESIDUAL_TOL``. If
    MuJoCo's link frame ever stopped matching our DH model (e.g. it silently returned
    the inertial frame instead of the link frame), construction would fail rather
    than score against a wrong robot.

MuJoCo is imported lazily; importing this module never requires it. Runs headless.
Meant to be driven from ``bench/sim_crosscheck.py`` inside ``.venv-sim``.
"""

from __future__ import annotations

import os
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np

from app.core.kinematics import RobotSpec, get_robot_spec, end_effector_pose
from app.sim.models import get_sim_model, resolve_urdf_path
from app.sim.parity import compute_parity
from app.sim.pybullet_backend import SimScore  # reuse the oracle-verdict dataclass


# Same tolerance the PyBullet oracle uses: Phase 1 measured a < 1e-6 residual for
# both arms; a bigger one means the model drifted and the oracle is invalid.
OFFSET_RESIDUAL_TOL = 1e-4


def _mujoco_urdf(robot: str) -> str:
    """Rewrite ``robot``'s URDF into a MuJoCo-loadable form and return the path.

    MuJoCo's URDF importer can't resolve ROS ``package://`` URIs and rejects the
    ``.dae`` meshes UR/Franka use for *visual* geometry. We therefore:
      1. replace every ``package://<pkg>/`` with the absolute on-disk package root,
      2. delete all ``<visual>`` elements (collision ``.stl`` meshes are what we
         need for self-collision; visuals are irrelevant and are the ``.dae`` ones),
      3. inject ``<mujoco><compiler .../></mujoco>`` with ``fusestatic="false"`` so
         fixed-joint links (``tool0``, ``ee_link``, ``panda_link8`` ...) survive as
         their own bodies -- we read the EE link frame for FK parity.

    The cleaned URDF is written beside a temp path; the original cache is untouched.
    """
    urdf = resolve_urdf_path(robot).replace("\\", "/")
    text = open(urdf).read()

    # Resolve every distinct ``package://<pkg>/`` prefix to ``<...>/<pkg>/`` by
    # locating <pkg> as an ancestor directory of the URDF on disk.
    for pkg in sorted({s.split("/")[0] for s in
                       (part.split("package://", 1)[1] for part in text.split()
                        if "package://" in part)}):
        idx = urdf.find(f"/{pkg}/")
        if idx == -1:
            continue
        root_abs = urdf[: idx + 1]  # up to and including the slash before <pkg>
        text = text.replace(f"package://{pkg}/", f"{root_abs}{pkg}/")

    root = ET.fromstring(text)
    for link in root.findall("link"):
        for vis in link.findall("visual"):
            link.remove(vis)
    mj = ET.Element("mujoco")
    ET.SubElement(mj, "compiler", {
        "discardvisual": "true",     # ignore any visual geoms we didn't strip
        "balanceinertia": "true",    # tolerate URDF inertias MuJoCo deems invalid
        "strippath": "false",        # keep our absolute mesh paths
        "fusestatic": "false",       # keep fixed-joint links as separate bodies
    })
    root.insert(0, mj)

    out = os.path.join(tempfile.gettempdir(), f"proteinik_mj_{robot}.urdf")
    ET.ElementTree(root).write(out)
    return out


@dataclass(frozen=True)
class _LinkPair:
    """A non-adjacent collision pair, by link name and the geoms to test."""
    name_a: str
    name_b: str
    geoms_a: tuple[int, ...]
    geoms_b: tuple[int, ...]


class MuJoCoBackend:
    """Headless MuJoCo oracle for one URDF-defined arm (Phase-4 second oracle).

    Public surface matches ``PyBulletBackend``: ``set_config``, ``fk``,
    ``self_collision``, ``dh_to_sim``, ``score`` -- so the same benchmark and
    cross-check code drives either engine, and ``compute_parity`` self-validates
    both against our DH model at construction.
    """

    def __init__(self, robot: str, pos_tol: float = 1e-3, orient_tol: float = 1e-2,
                 verify_samples: int = 300, verify_seed: int = 0,
                 collision_link_names: list[str] | None = None):
        import mujoco  # lazy

        self._mj = mujoco
        self.robot = robot
        self.spec: RobotSpec = get_robot_spec(robot)
        self.model_meta = get_sim_model(robot)
        self.pos_tol = pos_tol
        self.orient_tol = orient_tol
        self.ee_link = self.model_meta.ee_link_candidates[0]

        self._m = mujoco.MjModel.from_xml_path(_mujoco_urdf(robot))
        self._d = mujoco.MjData(self._m)

        # Name<->id helpers for bodies.
        self._body_name = [
            mujoco.mj_id2name(self._m, mujoco.mjtObj.mjOBJ_BODY, b)
            for b in range(self._m.nbody)
        ]
        self._body_id = {n: i for i, n in enumerate(self._body_name) if n is not None}

        # Map our q -> hinge-joint qpos addresses, in joint (tree) order. Hinges
        # only, so Franka's two prismatic finger joints are ignored -- exactly the
        # PyBullet backend's "revolute only" rule.
        hinges = [j for j in range(self._m.njnt)
                  if self._m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
        if len(hinges) < self.spec.n_joints:
            raise RuntimeError(
                f"MuJoCo model for '{robot}' exposes {len(hinges)} hinge joints; "
                f"DH spec has {self.spec.n_joints}. Joint mapping ambiguous."
            )
        self._hinges = hinges[: self.spec.n_joints]
        self._q_adr = [int(self._m.jnt_qposadr[j]) for j in self._hinges]

        self._collision_pairs = self._build_collision_pairs(collision_link_names)

        # (Re)measure the constant DH<->sim offset and fail if it isn't constant.
        self._init_offset(verify_samples, verify_seed)

    # -- model wiring ------------------------------------------------------

    def _geoms_of_body(self, body_id: int) -> tuple[int, ...]:
        return tuple(g for g in range(self._m.ngeom)
                     if int(self._m.geom_bodyid[g]) == body_id)

    def _build_collision_pairs(self, link_names: list[str] | None) -> list[_LinkPair]:
        """Non-adjacent arm-link pairs to query for real self-collision.

        The collision *link set* must match the engine it is cross-checked against,
        because PyBullet's set is exactly "{URDF root link ``-1``} + {child link of
        each revolute joint}" -- and that root may be a geomless ``world`` (UR5, so
        ``base_link``'s mesh is *not* checked) or a geom-bearing ``panda_link0``
        (Franka, so it *is*). There is no clean geometric rule for that choice, so
        ``bench/sim_crosscheck.py`` passes ``collision_link_names`` = PyBullet's exact
        link names, guaranteeing identical meaningful pairs. Standalone (no names),
        we default to {tree parent of the first arm link} + {revolute children},
        which is the natural arm set.

        Either way: pairs whose bodies are directly parent/child (adjacent) are
        skipped -- same rule as the capsule proxy and the PyBullet oracle -- and a
        link carrying no collision geom (e.g. a geomless ``world`` root, or Panda's
        frame-only ``panda_link8``) simply contributes no pair.
        """
        if link_names is not None:
            link_bodies = [self._body_id[n] for n in link_names if n in self._body_id]
        else:
            arm_bodies = [int(self._m.jnt_bodyid[h]) for h in self._hinges]
            link_bodies = [int(self._m.body_parentid[arm_bodies[0]])] + arm_bodies

        pairs: list[_LinkPair] = []
        for ia in range(len(link_bodies)):
            for ib in range(ia + 1, len(link_bodies)):
                a, b = link_bodies[ia], link_bodies[ib]
                if (self._m.body_parentid[a] == b) or (self._m.body_parentid[b] == a):
                    continue  # adjacent (share a joint)
                ga, gb = self._geoms_of_body(a), self._geoms_of_body(b)
                if not ga or not gb:
                    continue  # a frame-only link (e.g. panda_link8) -- nothing to test
                pairs.append(_LinkPair(self._body_name[a], self._body_name[b], ga, gb))
        return pairs

    # -- frame offset (self-check) -----------------------------------------

    def _init_offset(self, n: int, seed: int) -> None:
        res = compute_parity(self.spec, self, self.ee_link, n_samples=n, seed=seed)
        if res.residual > OFFSET_RESIDUAL_TOL:
            raise RuntimeError(
                f"[{self.robot}] MuJoCo link frame does not match our DH model "
                f"(residual={res.residual:.2e} > {OFFSET_RESIDUAL_TOL:g}). The "
                f"oracle would score against a wrong robot. Re-run Phase-1 parity."
            )
        self.offset_side = res.offset_side
        self.C = np.array(res.offset_transform)
        self.offset_residual = res.residual
        self.parity = res

    def dh_to_sim(self, T_dh: np.ndarray) -> np.ndarray:
        if self.offset_side == "tool":
            return T_dh @ self.C
        return self.C @ T_dh

    # -- SimBackend surface ------------------------------------------------

    def set_config(self, q: np.ndarray) -> None:
        """Kinematic teleport: write q into qpos and run FK only (no dynamics)."""
        for k, adr in enumerate(self._q_adr):
            if k >= len(q):
                break
            self._d.qpos[adr] = float(q[k])
        # mj_kinematics computes body AND geom world frames from qpos -- everything
        # fk() and self_collision() read. It is pure FK; no contacts, no stepping.
        self._mj.mj_kinematics(self._m, self._d)

    def fk(self, q: np.ndarray, link_name: str | None = None) -> np.ndarray:
        """4x4 world transform of a link's URDF frame at config ``q`` (sim frame).

        Reads ``data.xpos`` / ``data.xmat`` for the MuJoCo body that carries the
        URDF link's frame. ``xmat`` is the row-major rotation matrix -- used
        directly so no wxyz<->xyzw quaternion conversion is involved.
        """
        self.set_config(q)
        name = self.ee_link if link_name is None else link_name
        bid = self._body_id[name]
        T = np.eye(4)
        T[:3, :3] = np.array(self._d.xmat[bid]).reshape(3, 3)
        T[:3, 3] = np.array(self._d.xpos[bid])
        return T

    def self_collision(self, q: np.ndarray, threshold: float = 0.5) -> tuple[bool, float]:
        """Real mesh self-collision at ``q``: (in_collision, min_surface_distance).

        Queries ``mj_geomDistance`` (signed convex closest distance, neg ==
        penetrating) per non-adjacent arm-link geom pair, over the same link set
        PyBullet uses. Pairs farther apart than ``threshold`` saturate at
        ``threshold`` and don't set the minimum -- identical semantics to the
        PyBullet ``getClosestPoints`` loop.
        """
        self.set_config(q)
        fromto = np.zeros(6)
        min_d = float("inf")
        for pr in self._collision_pairs:
            for ga in pr.geoms_a:
                for gb in pr.geoms_b:
                    d = self._mj.mj_geomDistance(self._m, self._d, ga, gb,
                                                 threshold, fromto)
                    if d < min_d:
                        min_d = d
        if min_d == float("inf"):
            min_d = threshold
        return (min_d < 0.0), float(min_d)

    # -- the oracle verdict ------------------------------------------------

    def score(self, q_final: np.ndarray, T_target_dh: np.ndarray) -> SimScore:
        """Score a solver's ``q_final`` against a DH-frame target, in MuJoCo."""
        T_sim_target = self.dh_to_sim(T_target_dh)
        T_sim_actual = self.fk(q_final)
        pos_err = float(np.linalg.norm(T_sim_actual[:3, 3] - T_sim_target[:3, 3]))
        R_err = T_sim_target[:3, :3] @ T_sim_actual[:3, :3].T
        c = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
        orient_err = float(np.arccos(c))
        in_col, min_d = self.self_collision(q_final)
        return SimScore(
            sim_pos_error=pos_err,
            sim_orient_error=orient_err,
            sim_success=(pos_err < self.pos_tol and orient_err < self.orient_tol),
            sim_min_self_distance=min_d,
            sim_in_collision=in_col,
        )

    # -- lifecycle ---------------------------------------------------------

    def close(self) -> None:
        self._d = None
        self._m = None

    def __enter__(self) -> "MuJoCoBackend":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
