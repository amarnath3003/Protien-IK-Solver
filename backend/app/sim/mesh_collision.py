"""
A FAST, mesh-FAITHFUL self-collision model (Phase 3 / V7 — the real V4 upgrade).

The hand-tuned capsule proxy in ``kinematics.py`` puts one thin capsule on each
*joint-origin segment*. That geometry misses the real link bulk, so it reports
"clear" while the real URDF meshes interpenetrate ~20% of the time (``collision_parity.md``).
A solver whose collision reasoning consumes that proxy (V4's clean-fold test,
candidate selection, collision energy) therefore stops at solutions that really
collide.

This module builds a faithful-but-fast alternative once per robot:

  1. Extract each link's real collision-mesh vertices (PyBullet ``getMeshData``,
     returned in the link's LOCAL frame).
  2. Fit a small set of covering spheres to each link (k-means centers + a radius
     that covers each cluster) — a swept-sphere volume, the standard fast collision
     proxy in motion planning.
  3. Record the URDF's own kinematic chain (per-joint origin transform + axis), so
     the model can compute each link's world frame *directly from q* by URDF
     forward kinematics — NOT from the DH chain, whose intermediate frames do not
     coincide with the URDF link frames (they agree only at the end-effector).

At query time ``faithful_min_distance(model, q)`` runs the small URDF FK in numpy,
places every sphere, and returns the minimum surface gap over non-adjacent link
pairs. No simulator in the loop. Building needs PyBullet (offline, once).

Deliberately V4-*specific* leverage: only the protein solvers have a collision term
to consume a better signal, so feeding an accurate model to V4 widens its edge over
collision-blind baselines (TRAC-IK, CCD, native IK) rather than equalizing.
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np

from app.core.kinematics import get_robot_spec


# ---------------------------------------------------------------------------
# small SE(3) helpers
# ---------------------------------------------------------------------------

def _rpy_to_R(rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx  # URDF fixed-axis roll-pitch-yaw


def _origin_T(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _rpy_to_R(rpy)
    T[:3, 3] = xyz
    return T


def _axis_rot(axis: np.ndarray, angle: float) -> np.ndarray:
    a = axis / (np.linalg.norm(axis) + 1e-12)
    x, y, z = a
    c, s = np.cos(angle), np.sin(angle)
    C = 1 - c
    T = np.eye(4)
    T[:3, :3] = np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])
    return T


# ---------------------------------------------------------------------------
# model dataclasses
# ---------------------------------------------------------------------------

@dataclass
class MeshLink:
    """Covering spheres for one URDF link + the joint path that positions it."""
    name: str
    centers: np.ndarray            # (S,3) sphere centers in the LINK frame
    radii: np.ndarray              # (S,)
    order: int                     # position in the base->tip link order (adjacency)
    # joint path from base to this link: list of (origin_4x4, axis_or_None, q_index_or_None)
    path: list = field(default_factory=list)


@dataclass
class SphereModel:
    robot: str
    links: list                    # list[MeshLink]
    adjacency_gap: int = 2         # skip link pairs whose order differs by < this
    max_fk_error: float = 0.0      # worst URDF-FK vs PyBullet link-frame error at build (m)


# ---------------------------------------------------------------------------
# URDF kinematic parsing
# ---------------------------------------------------------------------------

def _parse_urdf_joints(urdf_path: str):
    """Return (joints_by_child, root_link). joints_by_child[child] = dict with
    parent, origin(4x4), axis(3), type, name."""
    root = ET.parse(urdf_path).getroot()
    joints = {}
    children = set()
    parents = set()
    for j in root.findall("joint"):
        name = j.get("name")
        jtype = j.get("type")
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0").split())] if o is not None else [0, 0, 0]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0").split())] if o is not None else [0, 0, 0]
        ax_el = j.find("axis")
        axis = np.array([float(v) for v in ax_el.get("xyz").split()]) if ax_el is not None else np.array([0, 0, 1.0])
        joints[child] = dict(parent=parent, origin=_origin_T(xyz, rpy),
                             axis=axis, type=jtype, name=name)
        children.add(child)
        parents.add(parent)
    roots = list(parents - children)
    return joints, (roots[0] if roots else None)


def _link_path(joints_by_child: dict, link: str):
    """Ordered list of joints from the root down to ``link`` (each a dict)."""
    path = []
    cur = link
    while cur in joints_by_child:
        path.append(joints_by_child[cur])
        cur = joints_by_child[cur]["parent"]
    path.reverse()
    return path


# ---------------------------------------------------------------------------
# build (needs PyBullet)
# ---------------------------------------------------------------------------

def _fit_link_spheres(pts: np.ndarray, k: int, radius_pct: float = 95.0,
                      radius_scale: float = 1.0):
    """Fit ``k`` spheres along a link's MEDIAL AXIS (its principal axis), each with
    radius = the link's true *perpendicular thickness* there.

    k-means on surface points gives an elongated link one fat cover-sphere (radius
    ~= half its length) that bulges far beyond the real thin link — the cause of
    the wrist over-collision. Placing spheres down the centerline with a thickness
    radius keeps elongated links thin and compact links round, matching the mesh
    volume tightly. ``radius_pct`` trims outlier surface spikes; spheres are grown
    to at least half the axial spacing so there are no gaps along the axis.
    """
    c0 = pts.mean(axis=0)
    Vc = pts - c0
    # principal axis via SVD (largest singular direction)
    _, _, Vt = np.linalg.svd(Vc, full_matrices=False)
    axis = Vt[0]
    t = Vc @ axis                                   # projection onto axis
    perp = np.linalg.norm(Vc - np.outer(t, axis), axis=1)  # perpendicular distance
    tmin, tmax = float(t.min()), float(t.max())
    k = max(1, min(k, len(pts)))
    ts = np.linspace(tmin, tmax, k) if k > 1 else np.array([(tmin + tmax) / 2])
    spacing = (tmax - tmin) / max(1, k - 1) if k > 1 else 0.0
    centers = c0[None, :] + ts[:, None] * axis[None, :]
    # assign each vertex to nearest center along the axis; radius = thickness there
    assign = np.argmin(np.abs(t[:, None] - ts[None, :]), axis=1)
    radii = np.zeros(k)
    for j in range(k):
        pj = perp[assign == j]
        thick = float(np.percentile(pj, radius_pct)) if len(pj) else 0.0
        # cover axial gaps: sphere must reach at least halfway to its neighbour
        radii[j] = max(thick, 0.5 * spacing) * radius_scale
    return centers, radii


def build_sphere_model(robot: str, spheres_per_link: int = 6,
                       radius_pct: float = 100.0, radius_scale: float = 1.0,
                       check_samples: int = 20, seed: int = 0) -> SphereModel:
    """Build the faithful sphere model for ``robot`` (offline; needs PyBullet).

    Also validates the numpy URDF-FK against PyBullet's own link frames, so a
    parsing/convention bug can't silently place spheres in the wrong pose.
    """
    import pybullet as p
    from app.sim.models import resolve_urdf_path
    from app.api.quaternion import quaternion_to_matrix

    spec = get_robot_spec(robot)
    urdf = resolve_urdf_path(robot)
    joints_by_child, _ = _parse_urdf_joints(urdf)

    cid = p.connect(p.DIRECT)
    try:
        p.setAdditionalSearchPath(os.path.dirname(urdf), physicsClientId=cid)
        body = p.loadURDF(urdf, useFixedBase=True,
                          flags=p.URDF_USE_SELF_COLLISION, physicsClientId=cid)
        n_pb = p.getNumJoints(body, physicsClientId=cid)
        revolute = [j for j in range(n_pb)
                    if p.getJointInfo(body, j, physicsClientId=cid)[2] == p.JOINT_REVOLUTE]
        # PyBullet link index -> child link name
        name_of_link = {-1: p.getBodyInfo(body, physicsClientId=cid)[0].decode()}
        for j in range(n_pb):
            name_of_link[j] = p.getJointInfo(body, j, physicsClientId=cid)[12].decode()

        # collect links with a real collision mesh (local-frame vertices)
        link_mesh: dict[int, np.ndarray] = {}
        for li in range(-1, n_pb):
            try:
                md = p.getMeshData(body, li, flags=p.MESH_DATA_SIMULATION_MESH,
                                   physicsClientId=cid)
            except Exception:
                continue
            if md and md[0] and md[0] > 0:
                link_mesh[li] = np.array(md[1])

        # actuated-joint order == revolute order == q index order
        actuated_names = [name_of_link[j] for j in revolute]  # child link of each revolute joint
        q_index_of_joint = {}  # joint 'name' -> q index (by the child link's driving joint)
        for qi, j in enumerate(revolute):
            jn = p.getJointInfo(body, j, physicsClientId=cid)[1].decode()
            q_index_of_joint[jn] = qi

        # build MeshLink list, ordered by base->tip (PyBullet link index order)
        links: list[MeshLink] = []
        ordered_li = sorted(link_mesh.keys())
        for order, li in enumerate(ordered_li):
            V = link_mesh[li]
            C, R = _fit_link_spheres(V, spheres_per_link,
                                     radius_pct=radius_pct, radius_scale=radius_scale)
            keep = R > 1e-9
            if not np.any(keep):
                keep = np.ones(len(R), bool)
            lname = name_of_link[li]
            raw_path = _link_path(joints_by_child, lname)
            path = [(jd["origin"], (jd["axis"] if jd["type"] in ("revolute", "continuous") else None),
                     q_index_of_joint.get(jd["name"]))
                    for jd in raw_path]
            links.append(MeshLink(name=lname, centers=C[keep], radii=R[keep],
                                  order=order, path=path))

        model = SphereModel(robot=robot, links=links)

        # --- validate numpy URDF-FK vs PyBullet link frames ---
        rng = np.random.default_rng(seed)
        worst = 0.0
        for _ in range(check_samples):
            q = spec.random_config(rng)
            for m, j in enumerate(revolute):
                p.resetJointState(body, j, float(q[m]), physicsClientId=cid)
            for li, L in zip(ordered_li, links):
                T_np = _link_world_T(L, q)
                st = p.getLinkState(body, li, computeForwardKinematics=True,
                                    physicsClientId=cid)
                T_pb = np.eye(4)
                T_pb[:3, :3] = quaternion_to_matrix(list(st[5]))
                T_pb[:3, 3] = st[4]
                worst = max(worst, float(np.linalg.norm(T_np[:3, 3] - T_pb[:3, 3])))
        model.max_fk_error = worst
        return model
    finally:
        p.disconnect(physicsClientId=cid)


# ---------------------------------------------------------------------------
# fast query (numpy only, no simulator)
# ---------------------------------------------------------------------------

def _link_world_T(link: MeshLink, q: np.ndarray) -> np.ndarray:
    """World transform of a link's frame via URDF FK along its joint path."""
    T = np.eye(4)
    for origin, axis, qi in link.path:
        T = T @ origin
        if axis is not None and qi is not None:
            T = T @ _axis_rot(axis, float(q[qi]))
    return T


def faithful_min_distance(model: SphereModel, q: np.ndarray) -> float:
    """Minimum surface gap (m) over non-adjacent link pairs at config ``q``;
    negative == interpenetrating. Pure numpy URDF FK + sphere distances."""
    world = []
    for L in model.links:
        T = _link_world_T(L, q)
        ctr = (T @ np.c_[L.centers, np.ones(len(L.centers))].T).T[:, :3]
        world.append((ctr, L.radii, L.order))
    min_d = np.inf
    m = len(world)
    for a in range(m):
        Ca, Ra, oa = world[a]
        for b in range(a + 1, m):
            Cb, Rb, ob = world[b]
            if abs(oa - ob) < model.adjacency_gap:
                continue
            diff = Ca[:, None, :] - Cb[None, :, :]
            dist = np.sqrt(np.sum(diff * diff, axis=2)) - Ra[:, None] - Rb[None, :]
            d = float(dist.min())
            if d < min_d:
                min_d = d
    return min_d if np.isfinite(min_d) else 1.0
