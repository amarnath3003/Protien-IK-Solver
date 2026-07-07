"""
Phase 2 (sim_migration_plan.md §4-§5): the PyBullet **evaluation oracle**.

Phase 1 (``parity.py``) proved our DH forward kinematics matches the URDF up to a
*constant* frame offset (UR5: a base ``Rz(180 deg)``; Panda: identity). This module
turns that validated model into an independent judge of the solvers. It never runs
inside a solver's hot loop -- the solver keeps consuming the fast numpy ``RobotSpec``
core. PyBullet is used only at the **boundaries** (plan §4):

  * **Target generation** -- sample a config, FK it, express the target in the sim
    frame so "reachable" means reachable *in the sim*.
  * **Evaluation (the oracle)** -- push the solver's ``q_final`` into the sim, read
    the sim's *actual* EE pose (real pos/orient error) and the sim's *real*
    self-collision (mesh closest-points), and score success against THAT, not
    against our own FK.
  * **Native-IK baseline** -- PyBullet's own ``calculateInverseKinematics`` gives a
    free, widely-trusted competitor column.

The constant Phase-1 offset ``C`` is applied so sim-frame scoring is consistent:
``T_sim = C @ T_dh`` (base offset) or ``T_sim = T_dh @ C`` (tool offset). Because
the parity residual is < 1e-6, feeding the solver a DH-frame target and scoring its
result in the sim frame is exact up to that residual -- any real model drift would
show up immediately as a large ``sim_pos_error``.

PyBullet is imported lazily; importing this module never requires it. Runs headless
(DIRECT). Meant to be driven from ``bench/sim_benchmark.py`` inside ``.venv-sim``.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np

from app.core.kinematics import RobotSpec, get_robot_spec, end_effector_pose
from app.api.quaternion import quaternion_to_matrix, matrix_to_quaternion
from app.sim.models import get_sim_model, resolve_urdf_path
from app.sim.parity import compute_parity


# Offset residual we tolerate at load. Phase 1 measured < 1e-6 for both arms; a
# bigger residual means the model drifted and the oracle would score nonsense, so
# we fail loudly instead of silently benchmarking against a wrong robot.
OFFSET_RESIDUAL_TOL = 1e-4


def _so3_angle(R: np.ndarray) -> float:
    """Rotation angle encoded by R (geodesic distance from identity)."""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(c))


@dataclass
class SimScore:
    """One config scored by the sim oracle."""
    sim_pos_error: float        # meters, sim FK vs sim-frame target
    sim_orient_error: float     # radians
    sim_success: bool           # meets the same tolerances, but in the sim
    sim_min_self_distance: float  # meters, real mesh closest-point (neg = penetrating)
    sim_in_collision: bool


class PyBulletBackend:
    """Headless PyBullet oracle for one URDF-defined arm.

    Loads the URDF once into a DIRECT server with self-collision enabled, maps our
    ``q`` onto the URDF's revolute joints in tree order, and exposes the plan's
    ``SimBackend`` surface: ``fk``, ``self_collision``, ``set_config``,
    ``reachable_target``, ``native_ik`` -- plus ``score`` which packages the
    oracle's verdict on a solver's ``q_final``.

    The Phase-1 constant frame offset is (re)measured at construction and asserted
    small, so the oracle is self-checking: if the DH model and the URDF ever stop
    agreeing, construction fails rather than producing misleading numbers.
    """

    def __init__(self, robot: str, pos_tol: float = 1e-3, orient_tol: float = 1e-2,
                 verify_samples: int = 300, verify_seed: int = 0):
        import pybullet as p  # lazy

        self.robot = robot
        self.spec: RobotSpec = get_robot_spec(robot)
        self.model = get_sim_model(robot)
        self.pos_tol = pos_tol
        self.orient_tol = orient_tol
        self.ee_link = self.model.ee_link_candidates[0]

        urdf = resolve_urdf_path(robot)
        self._p = p
        self._cid = p.connect(p.DIRECT)
        p.resetSimulation(physicsClientId=self._cid)
        p.setAdditionalSearchPath(os.path.dirname(urdf), physicsClientId=self._cid)
        # URDF_USE_SELF_COLLISION so intra-body closest-point queries have collision
        # shapes for every link; it excludes parent-child (adjacent) pairs by default,
        # matching our capsule proxy's "non-adjacent only" rule.
        self.body = p.loadURDF(
            urdf,
            useFixedBase=True,
            flags=p.URDF_USE_INERTIA_FROM_FILE | p.URDF_USE_SELF_COLLISION,
            physicsClientId=self._cid,
        )

        # Index links by name; collect revolute joints (skips Panda's prismatic
        # fingers); record parent link per link for adjacency filtering.
        self.link_index_by_name: dict[str, int] = {}
        self.revolute_joints: list[int] = []
        base_name = p.getBodyInfo(self.body, physicsClientId=self._cid)[0].decode("utf-8")
        self.link_index_by_name[base_name] = -1
        self._parent_of: dict[int, int] = {}
        for j in range(p.getNumJoints(self.body, physicsClientId=self._cid)):
            info = p.getJointInfo(self.body, j, physicsClientId=self._cid)
            child_link = info[12].decode("utf-8")
            parent_idx = info[16]
            self.link_index_by_name[child_link] = j
            self._parent_of[j] = parent_idx
            if info[2] == p.JOINT_REVOLUTE:
                self.revolute_joints.append(j)

        if len(self.revolute_joints) < self.spec.n_joints:
            raise RuntimeError(
                f"URDF for '{robot}' exposes {len(self.revolute_joints)} revolute "
                f"joints; DH spec has {self.spec.n_joints}. Joint mapping ambiguous."
            )
        self.ee_index = self.link_index_by_name[self.ee_link]

        # Non-adjacent link-index pairs among the revolute-arm links (plus base),
        # matching the capsule proxy's set as closely as the URDF allows.
        self._collision_pairs = self._build_collision_pairs()
        # The link *names* this oracle checks (root link -1 + revolute children), so
        # a second oracle can be asked to query the identical set (Phase-4 fairness).
        self._idx_to_name = {v: k for k, v in self.link_index_by_name.items()}
        self.collision_link_names = [self._idx_to_name[lid]
                                     for lid in ([-1] + list(self.revolute_joints))]

        # (Re)measure the Phase-1 constant offset C and fail if it isn't constant.
        self._init_offset(verify_samples, verify_seed)

    # -- frame offset ------------------------------------------------------

    def _init_offset(self, n: int, seed: int) -> None:
        """Measure C (constant DH<->sim frame offset) and its structural residual.

        Reuses the Phase-1 parity computation against our own EE link, so the
        oracle's frame convention is derived from the same validated numbers, not
        hard-coded. Raises if the residual exceeds tolerance (model drift).
        """
        # parity.compute_parity needs an object with .fk(q, link); we satisfy it.
        res = compute_parity(self.spec, self, self.ee_link, n_samples=n, seed=seed)
        if res.residual > OFFSET_RESIDUAL_TOL:
            raise RuntimeError(
                f"[{self.robot}] DH<->URDF offset is not constant "
                f"(residual={res.residual:.2e} > {OFFSET_RESIDUAL_TOL:g}); the DH "
                f"model and URDF disagree structurally -- oracle would be invalid. "
                f"Re-run Phase-1 parity (app.sim.parity)."
            )
        self.offset_side = res.offset_side              # 'base' | 'tool' | 'none'
        self.C = np.array(res.offset_transform)         # 4x4 constant offset
        self.offset_residual = res.residual

    def dh_to_sim(self, T_dh: np.ndarray) -> np.ndarray:
        """Express a DH-frame pose in the sim's frame using the constant offset."""
        if self.offset_side == "tool":
            return T_dh @ self.C
        return self.C @ T_dh  # 'base' (or 'none' where C == I)

    # -- SimBackend surface ------------------------------------------------

    def set_config(self, q: np.ndarray) -> None:
        p = self._p
        for k, jidx in enumerate(self.revolute_joints):
            if k >= len(q):
                break
            p.resetJointState(self.body, jidx, float(q[k]), physicsClientId=self._cid)

    def fk(self, q: np.ndarray, link_name: str | None = None) -> np.ndarray:
        """4x4 world transform of a link's URDF frame at config ``q`` (sim frame).

        Reads ``getLinkState`` indices 4/5 (worldLinkFramePosition/Orientation =
        the URDF link frame), NOT 0/1 (inertial/COM frame).
        """
        p = self._p
        self.set_config(q)
        idx = self.ee_index if link_name is None else self.link_index_by_name[link_name]
        if idx == -1:
            pos, orn = p.getBasePositionAndOrientation(self.body, physicsClientId=self._cid)
        else:
            st = p.getLinkState(self.body, idx, computeForwardKinematics=True,
                                physicsClientId=self._cid)
            pos, orn = st[4], st[5]
        T = np.eye(4)
        T[:3, :3] = quaternion_to_matrix(list(orn))  # orn is xyzw (PyBullet native)
        T[:3, 3] = pos
        return T

    def _build_collision_pairs(self) -> list[tuple[int, int]]:
        """Non-adjacent link-index pairs to query for real self-collision.

        Considers the revolute-arm links plus the base; excludes any pair that is
        directly connected (parent-child), because adjacent links share a joint and
        their overlap is not a meaningful clash (same rule as the capsule proxy).
        """
        # Links that carry the arm geometry: base (-1) and each revolute child link.
        link_ids = [-1] + list(self.revolute_joints)
        pairs: list[tuple[int, int]] = []
        for a_i in range(len(link_ids)):
            for b_i in range(a_i + 1, len(link_ids)):
                a, b = link_ids[a_i], link_ids[b_i]
                if self._parent_of.get(b) == a or self._parent_of.get(a) == b:
                    continue  # adjacent
                pairs.append((a, b))
        return pairs

    def self_collision(self, q: np.ndarray, threshold: float = 0.5) -> tuple[bool, float]:
        """Real mesh self-collision at ``q``: (in_collision, min_surface_distance).

        Queries PyBullet ``getClosestPoints`` per non-adjacent link pair (real
        collision geometry, not our capsule proxy). ``contactDistance`` is negative
        when the meshes interpenetrate. Pairs farther apart than ``threshold`` are
        treated as clearly clear (they just don't contribute the minimum).
        """
        p = self._p
        self.set_config(q)
        min_d = float("inf")
        for a, b in self._collision_pairs:
            pts = p.getClosestPoints(self.body, self.body, threshold,
                                     linkIndexA=a, linkIndexB=b,
                                     physicsClientId=self._cid)
            for c in pts:
                d = c[8]  # contactDistance (neg = penetrating)
                if d < min_d:
                    min_d = d
        if min_d == float("inf"):
            min_d = threshold  # nothing within threshold -> at least this clear
        return (min_d < 0.0), float(min_d)

    def self_collision_detail(self, q: np.ndarray, threshold: float = 0.5
                              ) -> tuple[bool, float, tuple[str, str] | None]:
        """Like ``self_collision`` but also returns which link *pair* drives the
        minimum -- the pair whose real meshes are closest (or deepest penetrating).

        Used by the Phase-3 breakdown to attribute real collision (and the proxy's
        false-clears) to specific link pairs, i.e. *where* the capsule proxy's
        geometry is wrong. Returns ``(in_collision, min_dist, (link_a, link_b))``;
        the pair is ``None`` only if nothing was within ``threshold``.
        """
        p = self._p
        self.set_config(q)
        min_d = float("inf")
        argmin: tuple[str, str] | None = None
        for a, b in self._collision_pairs:
            pts = p.getClosestPoints(self.body, self.body, threshold,
                                     linkIndexA=a, linkIndexB=b,
                                     physicsClientId=self._cid)
            for c in pts:
                d = c[8]
                if d < min_d:
                    min_d = d
                    argmin = (self._idx_to_name[a], self._idx_to_name[b])
        if min_d == float("inf"):
            min_d = threshold
        return (min_d < 0.0), float(min_d), argmin

    def native_ik(self, T_sim_target: np.ndarray, q0: np.ndarray,
                  iters: int = 100, residual_thresh: float = 1e-5,
                  refine: int = 20) -> np.ndarray:
        """PyBullet's built-in IK for the EE link toward a *sim-frame* target.

        Seeds from ``q0`` and *iteratively refines*: PyBullet's
        ``calculateInverseKinematics`` is a single damped-least-squares pass that
        under-converges for a 6/7-DOF arm on one call, so we re-seed from each
        solution and keep the best -- the standard way to give this baseline a fair
        shot rather than crippling it. Returns the mapped revolute ``q``.
        This is the free baseline competitor (plan §2, §6).
        """
        p = self._p
        n = self.spec.n_joints
        pos = T_sim_target[:3, 3].tolist()
        orn = matrix_to_quaternion(T_sim_target[:3, :3])  # xyzw
        q = np.asarray(q0, dtype=float).copy()
        best_q, best_err = q.copy(), float("inf")
        for _ in range(max(1, refine)):
            self.set_config(q)
            sol = p.calculateInverseKinematics(
                self.body, self.ee_index, pos, orn,
                maxNumIterations=iters, residualThreshold=residual_thresh,
                physicsClientId=self._cid,
            )
            # sol indexes movable (non-fixed) joints in order; the arm's revolute
            # joints are the first n for both UR5 and Panda.
            q = np.asarray(sol[:n], dtype=float)
            T = self.fk(q)
            e = (np.linalg.norm(T[:3, 3] - T_sim_target[:3, 3])
                 + _so3_angle(T_sim_target[:3, :3] @ T[:3, :3].T))
            if e < best_err:
                best_err, best_q = e, q.copy()
            if e < residual_thresh:
                break
        return best_q

    def reachable_target(self, rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
        """Sample a reachable target: a random config + its DH-frame EE pose.

        Returned in the DH frame so the solver (which lives on ``RobotSpec``) can
        consume it directly; convert to the sim frame with ``dh_to_sim`` for
        scoring. Kept for API completeness -- the benchmark runner drives targets
        through ``scenarios.generate_target`` so its cells match the master bench.
        """
        q_seed = self.spec.random_config(rng)
        return q_seed, end_effector_pose(self.spec, q_seed)

    # -- the oracle verdict ------------------------------------------------

    def score(self, q_final: np.ndarray, T_target_dh: np.ndarray) -> SimScore:
        """Score a solver's ``q_final`` against a DH-frame target, in the sim.

        Expresses the target in the sim frame via the constant Phase-1 offset,
        reads the sim's actual EE pose and real self-collision at ``q_final``, and
        reports pos/orient error + success measured entirely by the sim.
        """
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
        try:
            self._p.disconnect(physicsClientId=self._cid)
        except Exception:
            pass

    def __enter__(self) -> "PyBulletBackend":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
