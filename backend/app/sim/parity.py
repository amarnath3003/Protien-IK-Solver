"""
Phase 1 (sim_migration_plan.md §5): DH <-> URDF forward-kinematics parity.

**This is the real deliverable of the whole migration.** For ~10k random joint
configurations we compare our hand-typed DH forward kinematics
(``kinematics.end_effector_pose``) against PyBullet's forward kinematics
(``getLinkState``) on the *same* ``q``, and report the maximum position and
orientation deviation. That single number decides whether everything downstream
is "wire up an oracle" (our model already matches a real robot's) or "reconcile
the robot model" (§3 of the plan).

The comparison is done at three levels, mirroring the plan's §3 decision table:

  1. **direct** — raw pose deviation between our DH EE and a chosen URDF link
     frame.
  2. **constant-offset** — the relative transform ``inv(T_dh) @ T_sim`` for every
     config. If it is *invariant* across configs, our DH frame and the URDF frame
     differ only by a fixed tool/base transform (frame convention) that the
     adapter can absorb -- NOT a real disagreement.
  3. **residual** — the structural drift that remains after removing that constant
     offset. A tiny residual == "the models are the same robot"; a large residual
     == a genuine joint-offset/sign/axis mismatch (fix the DH table or derive the
     spec from the URDF).

PyBullet is imported lazily and only inside the oracle, so importing this module
(or the wider ``app`` package) never requires PyBullet to be installed. The
matching pytest skips cleanly when it is absent.

Run it directly for a full report:
    python -m app.sim.parity                 # UR5 + Panda, 10k configs each
    python -m app.sim.parity ur5 20000       # one robot, custom sample count
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field

import numpy as np

from app.core.kinematics import RobotSpec, get_robot_spec, end_effector_pose
from app.api.quaternion import quaternion_to_matrix  # xyzw -> 3x3 (same conv as PyBullet)
from app.sim.models import get_sim_model, resolve_urdf_path


# Parity thresholds. 1e-6 is the plan's "match" bar (§3). Deviations at the
# 1e-7 level are expected purely from PyBullet vs numpy doing the chain multiply
# in a different order.
TOL_POS = 1e-6      # meters
TOL_ORIENT = 1e-6   # radians


# ---------------------------------------------------------------------------
# small SO(3) helpers
# ---------------------------------------------------------------------------

def _rotation_angle(R: np.ndarray) -> float:
    """Magnitude of the rotation encoded by ``R`` (angle from identity)."""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(c))


def _rel_angle(R1: np.ndarray, R2: np.ndarray) -> float:
    """Angle between two rotation matrices (geodesic distance on SO(3))."""
    return _rotation_angle(R1.T @ R2)


# ---------------------------------------------------------------------------
# PyBullet forward-kinematics oracle
# ---------------------------------------------------------------------------

class PyBulletFK:
    """Headless PyBullet FK for a URDF-defined arm.

    Loads the URDF once into a DIRECT (no-GUI, no-physics) server, maps our
    ``q`` onto the URDF's revolute joints in tree order, and reads a link's
    *URDF-frame* pose via ``getLinkState`` for a given ``q``.

    Correctness notes (classic PyBullet gotchas, handled here):
      * We read ``getLinkState`` indices **4/5** (worldLinkFramePosition /
        Orientation = the URDF link frame), NOT 0/1 (the link's *inertial/COM*
        frame, which is offset by the link's center of mass).
      * We map ``q`` onto **revolute** joints only, so Panda's two prismatic
        finger joints are correctly ignored.
      * Joint angles are applied with ``resetJointState`` (kinematic teleport),
        never by stepping dynamics -- FK must not depend on physics.
      * The base is fixed at the world origin, so world-frame poses equal
        base-frame poses (our DH base == the URDF root link).
    """

    def __init__(self, urdf_path: str, base_link: str = "", fixed_base: bool = True):
        import pybullet as p  # lazy: only needed when actually talking to the sim

        self._p = p
        self._cid = p.connect(p.DIRECT)
        p.resetSimulation(physicsClientId=self._cid)
        # Let the URDF's own meshes resolve relative to its directory. Meshes are
        # irrelevant to FK (link frames come from joint origins), so package://
        # meshes that don't resolve are harmless -- the kinematic tree still loads.
        p.setAdditionalSearchPath(os.path.dirname(urdf_path), physicsClientId=self._cid)
        self.body = p.loadURDF(
            urdf_path,
            useFixedBase=fixed_base,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=self._cid,
        )

        # Index links by name and collect movable joints.
        self.link_index_by_name: dict[str, int] = {}
        self.revolute_joints: list[int] = []
        base_name = p.getBodyInfo(self.body, physicsClientId=self._cid)[0].decode("utf-8")
        self.link_index_by_name[base_name] = -1  # base link is index -1 in PyBullet
        for j in range(p.getNumJoints(self.body, physicsClientId=self._cid)):
            info = p.getJointInfo(self.body, j, physicsClientId=self._cid)
            joint_type = info[2]
            child_link = info[12].decode("utf-8")
            self.link_index_by_name[child_link] = j
            if joint_type == p.JOINT_REVOLUTE:
                self.revolute_joints.append(j)

    def link_names(self) -> list[str]:
        return list(self.link_index_by_name.keys())

    def n_revolute(self) -> int:
        return len(self.revolute_joints)

    def _set_q(self, q: np.ndarray) -> None:
        p = self._p
        for k, jidx in enumerate(self.revolute_joints):
            if k >= len(q):
                break
            p.resetJointState(self.body, jidx, float(q[k]), physicsClientId=self._cid)

    def fk(self, q: np.ndarray, link_name: str) -> np.ndarray:
        """4x4 world transform of ``link_name``'s URDF frame at config ``q``."""
        p = self._p
        self._set_q(q)
        link_idx = self.link_index_by_name[link_name]
        if link_idx == -1:  # base link
            pos, orn = p.getBasePositionAndOrientation(self.body, physicsClientId=self._cid)
        else:
            state = p.getLinkState(
                self.body, link_idx,
                computeForwardKinematics=True, physicsClientId=self._cid,
            )
            pos, orn = state[4], state[5]  # worldLinkFramePosition / Orientation
        T = np.eye(4)
        T[:3, :3] = quaternion_to_matrix(list(orn))  # orn is xyzw
        T[:3, 3] = pos
        return T

    def close(self) -> None:
        try:
            self._p.disconnect(physicsClientId=self._cid)
        except Exception:
            pass

    def __enter__(self) -> "PyBulletFK":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


# ---------------------------------------------------------------------------
# parity computation
# ---------------------------------------------------------------------------

@dataclass
class ParityResult:
    robot: str
    ee_link: str
    n_samples: int
    # direct pose deviation (no offset removal)
    max_pos_err: float
    mean_pos_err: float
    p95_pos_err: float
    max_orient_err: float
    mean_orient_err: float
    # best constant offset that reconciles DH with the URDF frame, and the
    # structural residual that remains after removing it.
    offset_side: str            # 'none' | 'base' | 'tool'
    const_offset_pos: float
    const_offset_orient: float
    offset_transform: list = field(default_factory=list)  # 4x4 nested list
    residual_max_pos: float = 0.0
    residual_max_orient: float = 0.0
    verdict: str = ""       # 'exact' | 'constant_offset' | 'structural_mismatch'

    @property
    def residual(self) -> float:
        """Single scalar for ranking: worst structural drift (pos + orient)."""
        return self.residual_max_pos + self.residual_max_orient

    def summary(self) -> str:
        return (
            f"[{self.robot} / {self.ee_link}]  n={self.n_samples}\n"
            f"  direct   : max_pos={self.max_pos_err:.3e} m  "
            f"max_orient={self.max_orient_err:.3e} rad  "
            f"(mean_pos={self.mean_pos_err:.3e}, p95_pos={self.p95_pos_err:.3e})\n"
            f"  offset   : side={self.offset_side}  |t|={self.const_offset_pos:.6f} m  "
            f"angle={self.const_offset_orient:.6f} rad "
            f"({np.degrees(self.const_offset_orient):.3f} deg)\n"
            f"  residual : max_pos={self.residual_max_pos:.3e} m  "
            f"max_orient={self.residual_max_orient:.3e} rad\n"
            f"  VERDICT  : {self.verdict.upper()}"
        )


def _offset_spread(offsets: np.ndarray) -> tuple[np.ndarray, float, float]:
    """Given (N,4,4) candidate constant offsets, return (reference, pos_spread,
    orient_spread). If the offset is truly constant, both spreads are ~0."""
    ref = offsets[0].copy()
    ref[:3, 3] = offsets[:, :3, 3].mean(axis=0)
    R_ref = ref[:3, :3]
    pos_spread = float(np.max(np.linalg.norm(offsets[:, :3, 3] - ref[:3, 3], axis=1)))
    orient_spread = max(_rel_angle(R_ref, offsets[i, :3, :3]) for i in range(len(offsets)))
    return ref, pos_spread, orient_spread


def _classify(max_pos, max_orient, res_pos, res_orient) -> str:
    if max_pos < TOL_POS and max_orient < TOL_ORIENT:
        return "exact"
    if res_pos < TOL_POS and res_orient < TOL_ORIENT:
        return "constant_offset"
    return "structural_mismatch"


def compute_parity(spec: RobotSpec, oracle: "PyBulletFK", ee_link: str,
                   n_samples: int, seed: int = 0) -> ParityResult:
    """Compare our DH FK vs the oracle's FK for ``ee_link`` over N random configs.

    Draws ``q`` from ``spec`` (respecting joint limits), evaluates both FKs, and
    tests BOTH kinds of constant frame offset (§3 of the plan):
      * **base** (left):  ``T_sim = C_base @ T_dh``  -> candidate ``T_sim @ inv(T_dh)``
      * **tool** (right): ``T_sim = T_dh @ C_tool``  -> candidate ``inv(T_dh) @ T_sim``
    Whichever is invariant across configs is the constant offset the adapter can
    absorb; the smaller residual wins. (A base flip, e.g. UR's ``base`` vs
    ``base_link`` 180 deg-about-Z, is a left offset that a tool-only check misses.)
    """
    rng = np.random.default_rng(seed)

    pos_errs = np.empty(n_samples)
    orient_errs = np.empty(n_samples)
    left = np.empty((n_samples, 4, 4))   # base-side offset candidates
    right = np.empty((n_samples, 4, 4))  # tool-side offset candidates

    for i in range(n_samples):
        q = spec.random_config(rng)
        T_dh = end_effector_pose(spec, q)
        T_sim = oracle.fk(q, ee_link)
        T_dh_inv = np.linalg.inv(T_dh)

        pos_errs[i] = np.linalg.norm(T_dh[:3, 3] - T_sim[:3, 3])
        orient_errs[i] = _rel_angle(T_dh[:3, :3], T_sim[:3, :3])
        left[i] = T_sim @ T_dh_inv
        right[i] = T_dh_inv @ T_sim

    ref_L, lp, la = _offset_spread(left)
    ref_R, rp, ra = _offset_spread(right)

    # Pick the side that reconciles better (smaller combined structural spread).
    if (lp + la) <= (rp + ra):
        side, ref, res_pos, res_orient = "base", ref_L, lp, la
    else:
        side, ref, res_pos, res_orient = "tool", ref_R, rp, ra

    max_pos = float(pos_errs.max())
    max_orient = float(orient_errs.max())

    return ParityResult(
        robot=spec.name,
        ee_link=ee_link,
        n_samples=n_samples,
        max_pos_err=max_pos,
        mean_pos_err=float(pos_errs.mean()),
        p95_pos_err=float(np.percentile(pos_errs, 95)),
        max_orient_err=max_orient,
        mean_orient_err=float(orient_errs.mean()),
        offset_side=side,
        const_offset_pos=float(np.linalg.norm(ref[:3, 3])),
        const_offset_orient=_rotation_angle(ref[:3, :3]),
        offset_transform=ref.tolist(),
        residual_max_pos=res_pos,
        residual_max_orient=res_orient,
        verdict=_classify(max_pos, max_orient, res_pos, res_orient),
    )


def scan_ee_links(spec: RobotSpec, oracle: "PyBulletFK", n_samples: int = 200,
                  seed: int = 0) -> list[ParityResult]:
    """Diagnostic: run parity against EVERY URDF link and rank by structural fit.

    Auto-discovers which URDF link frame corresponds to our DH EE (robust to a
    wrong EE guess). Uses a small sample since it evaluates every link. Returns
    results sorted best-first by (structural residual, then direct deviation).
    """
    results = []
    for name in oracle.link_names():
        try:
            results.append(compute_parity(spec, oracle, name, n_samples, seed))
        except Exception:
            continue
    results.sort(key=lambda r: (r.residual, r.max_pos_err + r.max_orient_err))
    return results


def run_parity(robot: str, n_samples: int = 10_000, seed: int = 0,
               ee_link: str | None = None) -> ParityResult:
    """End-to-end parity for one robot: load spec + URDF, pick EE link, compare.

    If ``ee_link`` is None, uses the pinned primary candidate but first scans all
    links (small sample) to confirm it is the best structural match, and switches
    to the discovered best link if the pinned one is not.
    """
    spec = get_robot_spec(robot)
    model = get_sim_model(robot)
    urdf = resolve_urdf_path(robot)

    with PyBulletFK(urdf, base_link=model.base_link) as oracle:
        n_rev = oracle.n_revolute()
        if n_rev < spec.n_joints:
            raise RuntimeError(
                f"URDF for '{robot}' exposes {n_rev} revolute joints but the DH "
                f"spec has {spec.n_joints}. Joint mapping is ambiguous."
            )

        if ee_link is None:
            # Confirm the pinned EE candidate is really the best-matching frame.
            ranked = scan_ee_links(spec, oracle, n_samples=min(300, n_samples), seed=seed)
            best = ranked[0]
            primary = model.ee_link_candidates[0]
            ee_link = primary if any(
                r.ee_link == primary and abs(r.residual - best.residual) < 1e-6
                for r in ranked
            ) else best.ee_link

        return compute_parity(spec, oracle, ee_link, n_samples, seed)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _main(argv: list[str]) -> int:
    robots = ["ur5", "franka_panda"]
    n = 10_000
    if len(argv) >= 1 and argv[0]:
        robots = [argv[0]]
    if len(argv) >= 2:
        n = int(argv[1])

    print(f"DH <-> URDF FK parity  (n={n} random configs/robot, tol={TOL_POS:g})\n")
    worst = 0.0
    for robot in robots:
        spec = get_robot_spec(robot)
        model = get_sim_model(robot)
        urdf = resolve_urdf_path(robot)
        with PyBulletFK(urdf, base_link=model.base_link) as oracle:
            print(f"URDF: {urdf}")
            print(f"  revolute joints in URDF: {oracle.n_revolute()} "
                  f"(DH n_joints={spec.n_joints})")
            # scan to show which link frame our DH EE lands on
            ranked = scan_ee_links(spec, oracle, n_samples=min(300, n))
            print("  best-matching URDF link frames (by structural residual):")
            for r in ranked[:3]:
                print(f"    {r.ee_link:16s} resid_pos={r.residual_max_pos:.2e} "
                      f"resid_orient={r.residual_max_orient:.2e} "
                      f"[{r.offset_side}] offset(|t|={r.const_offset_pos:.4f}, "
                      f"ang={np.degrees(r.const_offset_orient):.2f}deg)")
            ee = ranked[0].ee_link
            res = compute_parity(spec, oracle, ee, n)
            print(res.summary())
            print()
            worst = max(worst, res.residual_max_pos, res.residual_max_orient)

    print(f"worst structural residual across robots: {worst:.3e}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(_main(sys.argv[1:]))
