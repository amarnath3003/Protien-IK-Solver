"""Procedural URDF models for the planar N-DOF arms of the DOF-scaling study
(``scrap/usecase_experiments.py`` EXP E / paper Table 5).

Why this exists
---------------
The planar arms are synthetic. Unlike UR5/Franka they have no manufacturer
URDF, so ``robot_descriptions`` resolves nothing and the sim oracles had no
model to load -- ``models.SIM_MODELS`` excludes ``planar3dof`` for exactly this
reason. The DOF-scaling clean-solve rate was therefore scored by the capsule
proxy alone, while every other headline number in the paper is scored by two
independent engines. This module closes that gap.

What the generated URDF is
--------------------------
A URDF whose collision solids are *exactly* the capsules the proxy models::

    proxy segment k  ==  URDF link_{k+1}'s capsule
      endpoints : DH frame k origin -> DH frame k+1 origin
      radius    : spec.link_radius[k]

so PyBullet and MuJoCo score the same solid
``kinematics.self_collision_min_distance_from_chain`` scores.

What this does and does not establish
-------------------------------------
The planar arm *is defined as* a capsule chain: there is no ground-truth mesh
for it to be wrong about, unlike UR5/Franka whose manufacturer CAD disagreed
with our capsules (paper Section 5.6). So engine agreement here validates our
collision *implementation* against two independent narrow-phase engines -- it
does not independently validate the geometric *idealisation*. The paper states
that distinction rather than claiming the stronger UR5/Franka-grade result.

Frame derivation (why link_i carries segment i-1)
-------------------------------------------------
DH-generated URDFs put the *previous* joint's DH fixed part on the *next*
joint's origin (see ``native_bench/genuine_solvers.py:_dh_urdf_standard``,
FK-parity 2e-16), so with ``T_i = Rz(q_i) Tz(d_i) Tx(a_i) Rx(al_i)``::

    link_1 frame = Rz(q_1)                      -> origin == chain[0] (base)
    link_2 frame = DH frame 1 . Rz(q_2)         -> origin == chain[1]
    link_i frame = DH frame (i-1) . Rz(q_i)     -> origin == chain[i-1]

and the far end of that link, ``chain[i]``, sits at local ``(a[i-1], 0,
d[i-1])`` because that is precisely the fixed origin the *next* joint carries.
Hence link_i's capsule spans local ``(0,0,0) -> (a[i-1], 0, d[i-1])`` with
radius ``radii[i-1]`` -- matching proxy segment ``i-1`` exactly.

Adjacency also lines up for free: the proxy skips ``|i-j| < 2``; the engines
skip URDF parent/child bodies, which are the same pairs. ``base_link`` and
``tool0`` carry no geometry, mirroring the proxy's ``n_joints`` segments.
"""
from __future__ import annotations

import os
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

from app.core.kinematics import RobotSpec, ROBOT_REGISTRY

# robot id -> on-disk URDF we generated. ``models.resolve_urdf_path`` consults
# this before falling back to robot_descriptions, so procedurally generated arms
# flow through the unmodified PyBullet/MuJoCo backends.
GENERATED_URDFS: dict[str, str] = {}

# Default planar-arm geometry. Mirrors ``usecase_experiments.planar_ndof_spec``
# exactly -- links summing to ``total_reach``, thin uniform radius chosen so a
# folded chain genuinely self-clashes. Kept in sync by ``test_planar_parity.py``.
TOTAL_REACH = 1.0
LINK_RADIUS = 0.02


def planar_ndof_spec(n: int, total_reach: float = TOTAL_REACH,
                     link_radius: float = LINK_RADIUS) -> RobotSpec:
    """An n-link planar (RRR...) arm, links summing to ``total_reach``.

    Byte-for-byte the spec ``usecase_experiments.planar_ndof_spec`` builds; it
    lives here too so the sim layer can construct the arm by name without
    importing from ``scrap/``. ``test_planar_parity.py`` asserts the two agree.
    """
    a = np.full(n, total_reach / n)
    d = np.zeros(n)
    alpha = np.zeros(n)
    theta_offset = np.zeros(n)
    joint_limits = np.array([[-np.pi, np.pi]] * n)
    radii = np.full(n, link_radius)
    return RobotSpec(name=f"planar{n}dof", a=a, d=d, alpha=alpha,
                     theta_offset=theta_offset, joint_limits=joint_limits,
                     link_radius=radii)


def _capsule_geom(parent: ET.Element, a: float, d: float, radius: float,
                  geom_type: str) -> None:
    """Attach link i's collision solid: the segment (0,0,0) -> (a, 0, d),
    swept by ``radius``. URDF primitives are centred on their own origin with
    the axis along +Z, so we place the centre at the segment midpoint and
    rotate +Z onto the segment direction.
    """
    length = float(np.hypot(a, d))
    col = ET.SubElement(parent, "collision")
    # Rotate +Z onto the segment direction. The planar arms have d == 0, so the
    # segment runs along +X and this is Ry(90 deg); the general form keeps the
    # generator honest for any DH table with a d-offset.
    pitch = float(np.arctan2(a, d)) if length > 0 else 0.0
    ET.SubElement(col, "origin", {
        "xyz": f"{a / 2.0!r} 0 {d / 2.0!r}",
        "rpy": f"0 {pitch!r} 0",
    })
    geom = ET.SubElement(col, "geometry")
    # A capsule of radius r about the segment IS the proxy's solid: the proxy
    # returns segment-segment distance minus (r_i + r_j), which is exactly the
    # surface gap between two such capsules. A cylinder would differ at the
    # flat end caps, so capsule is the faithful choice where the engine takes it.
    ET.SubElement(geom, geom_type, {"radius": f"{radius!r}",
                                    "length": f"{length!r}"})


def build_planar_urdf(spec: RobotSpec, geom_type: str = "capsule",
                      path: str | None = None) -> str:
    """Emit a URDF for ``spec`` (a planar DH arm) and return its path.

    ``geom_type`` is ``capsule`` (faithful to the proxy) or ``cylinder``
    (standard URDF, for engines whose importer rejects capsules).
    """
    a, d, al = spec.a, spec.d, spec.alpha
    n = spec.n_joints
    root = ET.Element("robot", {"name": f"{spec.name}_dh"})
    ET.SubElement(root, "link", {"name": "base_link"})

    for i in range(1, n + 1):
        link = ET.SubElement(root, "link", {"name": f"link_{i}"})
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "mass", {"value": "1.0"})
        ET.SubElement(inertial, "inertia", {"ixx": "0.01", "ixy": "0", "ixz": "0",
                                            "iyy": "0.01", "iyz": "0", "izz": "0.01"})
        ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        # link_i carries proxy segment i-1: (0,0,0) -> (a[i-1], 0, d[i-1]).
        _capsule_geom(link, float(a[i - 1]), float(d[i - 1]),
                      float(spec.link_radius[i - 1]), geom_type)

    ET.SubElement(root, "link", {"name": "tool0"})

    for i in range(1, n + 1):
        parent = "base_link" if i == 1 else f"link_{i-1}"
        ox = oz = oroll = 0.0
        if i > 1:  # the PREVIOUS joint's DH fixed part rides on this origin
            ox, oz, oroll = float(a[i - 2]), float(d[i - 2]), float(al[i - 2])
        lo, hi = float(spec.joint_limits[i - 1, 0]), float(spec.joint_limits[i - 1, 1])
        joint = ET.SubElement(root, "joint", {"name": f"joint_{i}", "type": "revolute"})
        ET.SubElement(joint, "parent", {"link": parent})
        ET.SubElement(joint, "child", {"link": f"link_{i}"})
        ET.SubElement(joint, "origin", {"xyz": f"{ox!r} 0 {oz!r}", "rpy": f"{oroll!r} 0 0"})
        ET.SubElement(joint, "axis", {"xyz": "0 0 1"})
        ET.SubElement(joint, "limit", {"lower": f"{lo!r}", "upper": f"{hi!r}",
                                       "effort": "100", "velocity": "3.14"})

    tool = ET.SubElement(root, "joint", {"name": "joint_tool", "type": "fixed"})
    ET.SubElement(tool, "parent", {"link": f"link_{n}"})
    ET.SubElement(tool, "child", {"link": "tool0"})
    ET.SubElement(tool, "origin", {"xyz": f"{float(a[n-1])!r} 0 {float(d[n-1])!r}",
                                   "rpy": f"{float(al[n-1])!r} 0 0"})

    out = path or os.path.join(tempfile.gettempdir(),
                               f"proteinik_{spec.name}_{geom_type}.urdf")
    tree = ET.ElementTree(root)
    # Indent before writing. Not cosmetic: Bullet's URDF importer fails to parse
    # our longer single-line documents (16-DOF cylinder, 9238 bytes -> "Cannot
    # load URDF file" with a bogus XML_ERROR_PARSING_ATTRIBUTE at <origin>),
    # while the identical tree with newlines loads fine. MuJoCo parses either.
    ET.indent(tree)
    tree.write(out)
    return out


def register_planar_arm(n: int, geom_type: str = "capsule",
                        total_reach: float = TOTAL_REACH,
                        link_radius: float = LINK_RADIUS) -> str:
    """Make ``planar{n}dof`` resolvable by the whole sim stack; return its id.

    Registers the DH spec (so ``get_robot_spec`` finds it) and the generated
    URDF (so ``resolve_urdf_path`` finds it), then pins a ``SimModel`` whose
    frames are the URDF's own -- the DH table generated the URDF, so the
    DH<->sim offset is the identity and ``compute_parity`` verifies that at
    backend construction rather than us asserting it.
    """
    from app.sim.models import SIM_MODELS, SimModel

    robot = f"planar{n}dof"
    if robot in ROBOT_REGISTRY and robot not in GENERATED_URDFS:
        # planar3dof is a real, hand-written spec with its own radii/limits --
        # never silently replace it with the uniform DOF-sweep geometry.
        raise ValueError(
            f"'{robot}' is already a built-in robot with its own DH spec; "
            f"refusing to overwrite it with the generated DOF-sweep arm."
        )

    spec = planar_ndof_spec(n, total_reach, link_radius)
    ROBOT_REGISTRY[robot] = lambda: planar_ndof_spec(n, total_reach, link_radius)
    GENERATED_URDFS[robot] = build_planar_urdf(spec, geom_type)
    SIM_MODELS[robot] = SimModel(
        robot=robot,
        description_module="",  # generated, not resolved via robot_descriptions
        base_link="base_link",
        ee_link_candidates=("tool0",),
        n_movable=n,
        provenance=f"generated by app.sim.planar_model from the {robot} DH table "
                   f"(links {total_reach}/{n} m, capsule radius {link_radius} m); "
                   f"synthetic arm, no manufacturer URDF exists.",
        notes="Collision solids are the capsules the proxy models, so the engines "
              "score the same solid as kinematics.self_collision_min_distance. "
              "Validates the collision implementation, not the idealisation.",
        base_offset_z_deg=0.0,
        fk_status="validated",
    )
    return robot
