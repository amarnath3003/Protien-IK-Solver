"""
Phase 0 (sim_migration_plan.md §5): acquire & pin the robot models that the
simulator uses as the *source-of-truth* oracle, and tie each one back to the
hand-typed DH ``RobotSpec`` in ``app.core.kinematics``.

For every robot we record:
  * where the URDF comes from (provenance — so the model we benchmark is a
    known, canonical model, not an ad-hoc one),
  * the ordered list of movable (revolute) joints, so our ``q`` maps 1:1 onto
    the sim's joints,
  * the end-effector *link frame* whose pose we compare against our DH
    ``end_effector_pose`` (plus fallback candidates — the DH EE frame and the
    URDF tool frame often differ by a *constant* rotation, which the Phase-1
    parity harness is built to detect),
  * the base link (our DH base == this link's frame).

It also provides ``validate_joint_limits`` — the plan's explicit Phase-0 task of
checking that the limits we encoded (notably Panda's unusual always-negative
joint-4 range ``[-3.0718, -0.0698]``) actually match the chosen URDF.

Model files are resolved lazily through the pure-Python ``robot_descriptions``
package (canonical ROS-Industrial / franka_ros models, downloaded & cached on
first use). Nothing here imports a simulator, so this module is safe to import
anywhere. The parity harness (``parity.py``) is what actually loads PyBullet.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np

from app.core.kinematics import RobotSpec, get_robot_spec


# ---------------------------------------------------------------------------
# Model registry
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class SimModel:
    """Pins one robot's real-world model to our DH ``RobotSpec``.

    Attributes:
        robot: our robot id (matches ``kinematics.ROBOT_REGISTRY`` / RobotSpec.name)
        description_module: the ``robot_descriptions`` module name that resolves
            the URDF path (e.g. ``"ur5_description"``).
        base_link: URDF link whose frame equals our DH base frame.
        ee_link_candidates: ordered EE link frames to compare against our DH EE.
            The first is the primary; the rest are fallbacks. Our standard-DH EE
            frame and the URDF tool frame frequently differ by a fixed rotation,
            so the parity harness scans these and reports the best match plus any
            constant offset.
        n_movable: expected number of movable (revolute) arm joints == n_joints.
        provenance: human-readable note on where the model comes from.
        notes: parity-relevant caveats discovered in Phase 0.
    """

    robot: str
    description_module: str
    base_link: str
    ee_link_candidates: tuple[str, ...]
    n_movable: int
    provenance: str = ""
    notes: str = ""
    # --- filled in by the Phase-1 parity harness (parity.py) ---
    # Constant offset between our DH base frame and the URDF root link, as a
    # rotation about Z in degrees (UR encodes its DH in the 'base' frame, which
    # is base_link rotated 180 deg about Z). The Phase-2 adapter applies this so
    # targets/poses live in a consistent frame.
    base_offset_z_deg: float = 0.0
    # FK parity outcome for the CURRENT kinematics.py FK:
    #   'validated'            — DH matches the URDF up to a constant frame offset
    #   'mismatch_modified_dh' — params are modified-DH but FK is standard-DH
    fk_status: str = "unknown"


# The two arms the plan targets. planar3dof is intentionally excluded: it has no
# standard URDF and is already validated analytically (see plan §7, risk #5).
SIM_MODELS: dict[str, SimModel] = {
    "ur5": SimModel(
        robot="ur5",
        description_module="ur5_description",
        base_link="base_link",
        # tool0 / ee_link sit d6=0.0823 past wrist_3 — same offset our DH EE
        # carries in d[5]. wrist_3_link is the pre-tool frame (fallback).
        ee_link_candidates=("tool0", "ee_link", "wrist_3_link"),
        n_movable=6,
        provenance="robot_descriptions.ur5_description -> "
                   "example-robot-data ur_description/urdf/ur5_robot.urdf "
                   "(classic UR5; DH offsets reconcile: d1=0.089159, |a2|=0.425, "
                   "|a3|=0.39225, d4=0.13585-0.1197+0.093=0.10915, d5=0.09465, d6=0.0823).",
        notes="URDF elbow limit is +/-pi; our spec uses +/-2pi ('effectively "
              "unlimited'). Benign for FK; relevant when sampling reachable "
              "targets in Phase 2. PHASE-1 RESULT: our DH EE == URDF tool0 up to a "
              "CONSTANT base offset of exactly Rz(180 deg) (UR encodes DH in the "
              "'base' frame = base_link flipped 180 deg about Z). Residual < 8e-7 m/rad "
              "over 10k configs => our UR5 DH is VALIDATED against the real model.",
        base_offset_z_deg=180.0,
        fk_status="validated",
    ),
    "franka_panda": SimModel(
        robot="franka_panda",
        description_module="panda_description",
        base_link="panda_link0",
        # panda_link8 is the flange (link7 + d7=0.107 along z) — our DH EE.
        # panda_link7 is the pre-flange frame; panda_hand is +hand (fallback).
        ee_link_candidates=("panda_link8", "panda_link7", "panda_hand"),
        n_movable=7,
        provenance="robot_descriptions.panda_description -> "
                   "example-robot-data panda_description/urdf/panda.urdf "
                   "(franka_ros; DH offsets reconcile: d1=0.333, d3=0.316, "
                   "a4=0.0825, a5=-0.0825, d5=0.384, a7=0.088, flange d7=0.107).",
        notes="URDF ships 7 revolute arm joints + 2 prismatic fingers; the parity "
              "loader filters to revolute so fingers are ignored. PHASE-1 RESULT "
              "(after fix): franka_panda_spec holds the Franka *modified*-DH (Craig) "
              "table and is now evaluated with dh_convention='modified' in "
              "kinematics.py -> our DH EE matches URDF panda_link8 to ~6e-8 with "
              "IDENTITY offset => Panda is VALIDATED against the real model. (Before "
              "the fix these params were run through a standard-DH FK, putting the EE "
              "~1.4 m off the real robot; that change re-derived all Franka results.)",
        base_offset_z_deg=0.0,
        fk_status="validated",
    ),
}


def get_sim_model(robot: str) -> SimModel:
    if robot not in SIM_MODELS:
        raise ValueError(
            f"No sim model pinned for robot '{robot}'. "
            f"Available: {list(SIM_MODELS)} (planar3dof is analytically validated, "
            f"no URDF — see sim_migration_plan.md §7)."
        )
    return SIM_MODELS[robot]


def resolve_urdf_path(robot: str) -> str:
    """Return the on-disk URDF path for ``robot``.

    Procedurally generated arms (the planar N-DOF chains of the DOF-scaling
    study -- see ``app.sim.planar_model``) are synthetic and have no upstream
    description package, so their generated URDF is returned directly. Every
    other arm resolves through robot_descriptions, which lazily downloads &
    caches the model on first use, and raises a clear, actionable error if the
    package is missing.
    """
    from app.sim.planar_model import GENERATED_URDFS  # lazy: avoids import cycle

    if robot in GENERATED_URDFS:
        return GENERATED_URDFS[robot]

    model = get_sim_model(robot)
    try:
        import importlib

        mod = importlib.import_module(f"robot_descriptions.{model.description_module}")
    except ImportError as exc:  # pragma: no cover - env-dependent
        raise ImportError(
            "robot_descriptions is required to resolve robot URDFs. Install it "
            "with `pip install robot_descriptions` (pure Python, no compiler). "
            f"Original error: {exc}"
        ) from exc
    return mod.URDF_PATH


# ---------------------------------------------------------------------------
# URDF joint-limit parsing + validation (Phase 0 task)
# ---------------------------------------------------------------------------

def urdf_movable_joints(urdf_path: str) -> list[dict]:
    """Parse a URDF and return its movable joints in tree order.

    Each entry: {name, type, axis (3-vec), lower, upper}. Only revolute /
    continuous / prismatic joints are 'movable'; fixed joints are skipped.
    Pure XML — no simulator needed.
    """
    root = ET.parse(urdf_path).getroot()
    out: list[dict] = []
    for j in root.findall("joint"):
        jtype = j.get("type")
        if jtype not in ("revolute", "continuous", "prismatic"):
            continue
        lim = j.find("limit")
        lower = float(lim.get("lower")) if (lim is not None and lim.get("lower") is not None) else None
        upper = float(lim.get("upper")) if (lim is not None and lim.get("upper") is not None) else None
        axis_el = j.find("axis")
        axis = [float(x) for x in axis_el.get("xyz").split()] if axis_el is not None else [0.0, 0.0, 1.0]
        out.append({"name": j.get("name"), "type": jtype, "axis": axis,
                    "lower": lower, "upper": upper})
    return out


@dataclass
class JointLimitCheck:
    """Result of comparing one robot's DH joint_limits against its URDF."""
    robot: str
    n_dh_joints: int
    n_urdf_revolute: int
    per_joint: list[dict] = field(default_factory=list)  # {index,name,dh,urdf,match,note}
    count_matches: bool = False

    @property
    def all_match(self) -> bool:
        return self.count_matches and all(pj["match"] for pj in self.per_joint)

    @property
    def mismatches(self) -> list[dict]:
        return [pj for pj in self.per_joint if not pj["match"]]

    def summary(self) -> str:
        lines = [f"[{self.robot}] DH joints={self.n_dh_joints} "
                 f"URDF revolute={self.n_urdf_revolute} "
                 f"count_match={self.count_matches}"]
        for pj in self.per_joint:
            flag = "OK " if pj["match"] else "DIFF"
            lines.append(
                f"  {flag} q{pj['index']} {pj['name']:22s} "
                f"DH[{pj['dh'][0]:+.4f},{pj['dh'][1]:+.4f}] "
                f"URDF[{pj['urdf'][0]:+.4f},{pj['urdf'][1]:+.4f}]"
                + (f"  <- {pj['note']}" if pj.get("note") else "")
            )
        return "\n".join(lines)


def validate_joint_limits(robot: str, tol: float = 1e-4) -> JointLimitCheck:
    """Compare the DH ``RobotSpec`` joint limits against the pinned URDF.

    This is the plan's Phase-0 validation. A *DIFF* is not necessarily a bug:
    for UR5 we deliberately encode wider (+/-2pi) limits than the URDF, and the
    per-joint report says so. For Panda every limit should match to ``tol`` --
    in particular the always-negative joint-4 range ``[-3.0718, -0.0698]``.
    """
    spec = get_robot_spec(robot)
    urdf_path = resolve_urdf_path(robot)
    movable = urdf_movable_joints(urdf_path)
    revolute = [m for m in movable if m["type"] in ("revolute", "continuous")]

    check = JointLimitCheck(
        robot=robot,
        n_dh_joints=spec.n_joints,
        n_urdf_revolute=len(revolute),
        count_matches=(len(revolute) >= spec.n_joints),
    )

    for i in range(spec.n_joints):
        dh_lo, dh_hi = float(spec.joint_limits[i, 0]), float(spec.joint_limits[i, 1])
        if i < len(revolute):
            uj = revolute[i]
            u_lo = uj["lower"] if uj["lower"] is not None else -np.inf
            u_hi = uj["upper"] if uj["upper"] is not None else np.inf
            match = (abs(dh_lo - u_lo) <= tol and abs(dh_hi - u_hi) <= tol)
            note = ""
            if not match:
                # Distinguish "we chose wider limits" (benign) from a genuine
                # lower/upper disagreement.
                if dh_lo <= u_lo + tol and dh_hi >= u_hi - tol:
                    note = "DH wider than URDF (intentional; benign for FK)"
                else:
                    note = "genuine limit disagreement"
            check.per_joint.append({
                "index": i, "name": uj["name"], "dh": (dh_lo, dh_hi),
                "urdf": (u_lo, u_hi), "match": match, "note": note,
            })
        else:
            check.per_joint.append({
                "index": i, "name": "<missing>", "dh": (dh_lo, dh_hi),
                "urdf": (np.nan, np.nan), "match": False,
                "note": "no corresponding URDF revolute joint",
            })
    return check


if __name__ == "__main__":  # pragma: no cover - manual Phase-0 report
    for _robot in SIM_MODELS:
        m = get_sim_model(_robot)
        print(f"\n{'='*72}\n{_robot}\n{'='*72}")
        print(f"  provenance: {m.provenance}")
        print(f"  base_link : {m.base_link}")
        print(f"  ee_link   : {m.ee_link_candidates}")
        print(f"  urdf      : {resolve_urdf_path(_robot)}")
        print(validate_joint_limits(_robot).summary())
