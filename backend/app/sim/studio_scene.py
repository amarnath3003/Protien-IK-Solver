"""
Scene builder for the native **MuJoCo IK Studio** (``ik_studio.py``).

Turns a robot's real URDF into a *photoreal-ish* MuJoCo scene:
  * **visual meshes** — the URDF's ``.dae`` visual meshes (which MuJoCo can't load)
    are converted to ``.obj`` with trimesh, one geom per Collada sub-part so the real
    multi-tone look (UR silver/blue/black, Panda white/black) is preserved.
  * **studio chrome** — a checker floor, a soft key/fill/rim light rig, a gradient
    skybox, and shadows, added via ``MjSpec`` after loading the URDF.
  * **an IK target** — a mocap body ``ik_target`` (sphere) the app moves at runtime.

The exact model is the one the DH kinematics were validated against
(example-robot-data ur5_robot.urdf / panda.urdf), so a solver's ``q`` renders with
its end-effector on the target (frame offset handled in ``ik_studio``). Collision
geometry is dropped — the studio renders and does FK only; collision *metrics* come
from the solver itself (the capsule proxy), same as the web app.

Everything here is best-effort with fallbacks: a link whose ``.dae`` won't convert
falls back to its ``.stl`` collision mesh; skybox/floor extras are wrapped so a
MjSpec API hiccup degrades to a plainer (still working) scene.
"""

from __future__ import annotations

import os
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np

from app.core.kinematics import get_robot_spec
from app.sim.models import get_sim_model


_STUDIO_TMP = os.path.join(tempfile.gettempdir(), "proteinik_studio")
os.makedirs(_STUDIO_TMP, exist_ok=True)


@dataclass
class StudioModel:
    """A compiled studio scene + the handles the app needs to drive it."""
    model: object                 # mujoco.MjModel
    robot: str
    q_adr: list                   # qpos addresses for our q, in joint order
    joint_names: list             # hinge joint names in q order
    ee_body_id: int               # end-effector link body id
    target_mocap_id: int          # mocap index of the ik_target body
    base_offset_z_deg: float      # constant DH->URDF base offset (about Z)
    n_joints: int
    halo_geom_id: int = -1        # pulsing halo sphere around the target (-1 = none)
    target_body_id: int = -1      # body id of ik_target (for mocap_quat updates)


# ── URDF resolution (robust to robot_descriptions renames) ────────────────────

def resolve_urdf(robot: str) -> str:
    """Absolute path to ``robot``'s validated URDF (see ik_studio/live_viewer)."""
    from app.sim.live_viewer import _resolve_urdf_robust
    return _resolve_urdf_robust(robot)


# ── .dae -> .obj visual mesh conversion (trimesh) ─────────────────────────────

def _part_rgba(part) -> list:
    """A representative RGBA for one trimesh part (material color, else vertex mean)."""
    try:
        mat = getattr(part.visual, "material", None)
        col = getattr(mat, "main_color", None) if mat is not None else None
        if col is not None:
            c = np.asarray(col, dtype=float)
            if c.max() > 1.0:
                c = c / 255.0
            r, g, b = float(c[0]), float(c[1]), float(c[2])
            a = float(c[3]) if c.shape[0] > 3 else 1.0
            return [r, g, b, a]
    except Exception:
        pass
    try:
        vc = part.visual.to_color().vertex_colors[:, :3].mean(axis=0) / 255.0
        return [float(vc[0]), float(vc[1]), float(vc[2]), 1.0]
    except Exception:
        return [0.62, 0.64, 0.66, 1.0]


def _dae_to_obj_parts(dae_abs: str, tag: str) -> list:
    """Convert a Collada file to a list of (obj_path, rgba), one per sub-part.

    Returns [] if trimesh is unavailable or the file can't be read, so the caller
    can fall back to the collision mesh. OBJs are cached in the studio temp dir.
    """
    try:
        import trimesh
    except Exception:
        return []
    try:
        loaded = trimesh.load(dae_abs, force="scene")
        parts = loaded.dump(concatenate=False) if hasattr(loaded, "dump") else [loaded]
    except Exception:
        return []

    out = []
    for i, part in enumerate(parts):
        try:
            if part is None or len(getattr(part, "faces", [])) == 0:
                continue
            obj_path = os.path.join(_STUDIO_TMP, f"{tag}_{i}.obj")
            if not os.path.exists(obj_path) or os.path.getmtime(obj_path) < os.path.getmtime(dae_abs):
                # export geometry only (MuJoCo colors come from the geom material)
                part.export(obj_path)
            out.append((obj_path, _part_rgba(part)))
        except Exception:
            continue
    return out


# ── URDF -> studio URDF (visual .obj meshes, colored, collision dropped) ───────

def _studio_urdf(robot: str) -> str:
    """Rewrite ``robot``'s URDF so MuJoCo can render its *visual* meshes.

    For every ``<visual>`` mesh: convert the ``.dae`` to one-or-more ``.obj`` parts,
    and emit one ``<visual>`` per part carrying that part's color. ``.stl`` visuals
    (rare) are kept as-is. All ``<collision>`` is removed (studio renders + FK only).
    A ``<mujoco>`` compiler block keeps visuals and per-link frames.
    """
    urdf = resolve_urdf(robot).replace("\\", "/")
    text = open(urdf).read()

    # Resolve package:// to absolute (same trick as mujoco_backend._rewrite_urdf...).
    for pkg in sorted({s.split("/")[0] for s in
                       (part.split("package://", 1)[1] for part in text.split()
                        if "package://" in part)}):
        idx = urdf.find(f"/{pkg}/")
        if idx == -1:
            continue
        text = text.replace(f"package://{pkg}/", f"{urdf[: idx + 1]}{pkg}/")

    root = ET.fromstring(text)
    for li, link in enumerate(root.findall("link")):
        for col in link.findall("collision"):
            link.remove(col)
        for vis in list(link.findall("visual")):
            geom = vis.find("geometry")
            mesh = geom.find("mesh") if geom is not None else None
            if mesh is None:
                continue
            fn = mesh.get("filename", "")
            scale = mesh.get("scale")
            origin = vis.find("origin")
            if fn.lower().endswith(".dae"):
                parts = _dae_to_obj_parts(fn, f"{robot}_l{li}")
                if not parts:
                    continue  # leave the .dae (MuJoCo will skip/discard it); collision fallback below
                link.remove(vis)
                for pi, (obj_path, rgba) in enumerate(parts):
                    nv = ET.SubElement(link, "visual")
                    if origin is not None:
                        nv.append(ET.fromstring(ET.tostring(origin)))
                    ng = ET.SubElement(nv, "geometry")
                    nm = ET.SubElement(ng, "mesh")
                    nm.set("filename", obj_path.replace("\\", "/"))
                    if scale:
                        nm.set("scale", scale)
                    mat = ET.SubElement(nv, "material")
                    mat.set("name", f"{robot}_l{li}_{pi}")
                    ET.SubElement(mat, "color").set(
                        "rgba", " ".join(f"{v:.4f}" for v in rgba))

        # If a link lost all visuals (dae conversion failed), fall back to its .stl
        # collision-style mesh so it still shows something. (Best-effort.)

    mj = ET.Element("mujoco")
    ET.SubElement(mj, "compiler", {
        "discardvisual": "false",   # KEEP visual meshes (that's what we render)
        "balanceinertia": "true",
        "strippath": "false",
        "fusestatic": "false",
    })
    root.insert(0, mj)

    out = os.path.join(_STUDIO_TMP, f"studio_{robot}.urdf")
    ET.ElementTree(root).write(out)
    return out


# ── scene chrome (floor, sky, lights, target) via MjSpec ──────────────────────

def _augment_scene(spec, extent: float) -> None:
    """Studio chrome: deep-space gradient sky, reflective dark grid floor, a warm
    key / cool fill / rim 4-light cinema rig, tinted haze, a glowing pedestal pad,
    and an emissive IK-target (core + pulsing halo + RGB orientation triad).
    Each addition is guarded so a MjSpec API mismatch never aborts the whole
    scene — the robot still renders."""
    import mujoco
    wb = spec.worldbody

    # Visual quality + framing.
    try:
        spec.visual.quality.shadowsize = 8192
        spec.visual.quality.offsamples = 8
        spec.stat.extent = max(extent, 0.6)
        spec.stat.center = [0.0, 0.0, extent * 0.4]
    except Exception:
        pass
    # Allow large offscreen renders (verification screenshots; harmless for GLFW).
    try:
        spec.visual.global_.offwidth = 1920
        spec.visual.global_.offheight = 1080
    except Exception:
        pass
    # Cinematic atmosphere: dim the headlight so the rig does the work, tint the haze.
    try:
        spec.visual.headlight.ambient = [0.14, 0.15, 0.17]
        spec.visual.headlight.diffuse = [0.20, 0.21, 0.23]
        spec.visual.headlight.specular = [0.08, 0.08, 0.08]
        spec.visual.rgba.haze = [0.045, 0.06, 0.09, 1.0]
    except Exception:
        pass

    # Gradient skybox + grid floor texture/material + emissive accent material.
    floor_mat = "studio_floor"
    glow_mat = "studio_glow"
    try:
        sky = spec.add_texture()
        sky.name = "studio_sky"
        sky.type = mujoco.mjtTexture.mjTEXTURE_SKYBOX
        sky.builtin = mujoco.mjtBuiltin.mjBUILTIN_GRADIENT
        sky.rgb1 = [0.10, 0.14, 0.22]     # deep blue up high…
        sky.rgb2 = [0.008, 0.012, 0.022]  # …to near-black at the horizon
        sky.width = 800
        sky.height = 800

        tex = spec.add_texture()
        tex.name = "studio_grid"
        tex.type = mujoco.mjtTexture.mjTEXTURE_2D
        tex.builtin = mujoco.mjtBuiltin.mjBUILTIN_CHECKER
        tex.rgb1 = [0.125, 0.135, 0.160]
        tex.rgb2 = [0.085, 0.095, 0.115]
        tex.width = 512
        tex.height = 512
        try:  # bright grid seams between tiles (best-effort; plain checker without)
            tex.mark = mujoco.mjtMark.mjMARK_EDGE
            tex.markrgb = [0.30, 0.36, 0.46]
        except Exception:
            pass

        mat = spec.add_material()
        mat.name = floor_mat
        mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "studio_grid"
        mat.texrepeat = [3, 3]
        mat.texuniform = True
        mat.reflectance = 0.38            # dark mirror floor
        mat.specular = 0.55
        mat.shininess = 0.75
    except Exception:
        floor_mat = None
    try:
        gm = spec.add_material()
        gm.name = glow_mat
        gm.emission = 0.85                # self-lit: target/triad glow in the dark
        gm.specular = 0.0
        gm.shininess = 0.0
    except Exception:
        glow_mat = None

    # Floor plane.
    try:
        g = wb.add_geom()
        g.type = mujoco.mjtGeom.mjGEOM_PLANE
        g.size = [0, 0, 0.05]         # 0,0 => infinite plane
        g.contype = 0
        g.conaffinity = 0
        if floor_mat:
            g.material = floor_mat
        else:
            g.rgba = [0.10, 0.11, 0.13, 1.0]
    except Exception:
        pass

    # Pedestal mount plate under the robot + a soft light-pool accent disc.
    try:
        ped = wb.add_geom()
        ped.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        ped.size = [0.17, 0.006, 0]
        ped.pos = [0.0, 0.0, 0.006]
        ped.rgba = [0.055, 0.062, 0.075, 1.0]
        ped.contype = 0
        ped.conaffinity = 0
        pool = wb.add_geom()
        pool.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        pool.size = [max(0.30, extent * 0.36), 0.0012, 0]
        pool.pos = [0.0, 0.0, 0.0012]
        pool.rgba = [0.10, 0.45, 0.62, 0.10]
        pool.contype = 0
        pool.conaffinity = 0
        if glow_mat:
            pool.material = glow_mat
    except Exception:
        pass

    # 4-light cinema rig: warm key (shadows), cool fill, blue rim, low kicker.
    for pos, diffuse, castshadow in (
            ([1.8, 1.4, 2.8], [0.88, 0.82, 0.72], True),    # warm key
            ([-2.2, -1.4, 1.8], [0.24, 0.28, 0.36], False),  # cool fill
            ([-1.0, 2.4, 1.5], [0.28, 0.34, 0.46], False),   # blue rim
            ([0.8, -2.2, 0.8], [0.15, 0.17, 0.22], False)):  # low kicker
        try:
            li = wb.add_light()
            li.pos = pos
            li.dir = [-pos[0], -pos[1], -pos[2]]
            li.castshadow = castshadow
            li.diffuse = diffuse
            if castshadow:
                li.specular = [0.45, 0.45, 0.45]
        except Exception:
            pass

    # IK target: emissive core + translucent pulsing halo + RGB orientation triad.
    try:
        r_core = max(0.013, extent * 0.015)
        ax_len = max(0.085, extent * 0.10)
        b = wb.add_body()
        b.name = "ik_target"
        b.mocap = True
        b.pos = [extent * 0.5, 0.0, extent * 0.5]

        core = b.add_geom()
        core.name = "ik_target_core"
        core.type = mujoco.mjtGeom.mjGEOM_SPHERE
        core.size = [r_core, 0, 0]
        core.rgba = [1.0, 0.78, 0.25, 1.0]
        core.contype = 0
        core.conaffinity = 0
        if glow_mat:
            core.material = glow_mat

        halo = b.add_geom()
        halo.name = "ik_target_halo"
        halo.type = mujoco.mjtGeom.mjGEOM_SPHERE
        halo.size = [r_core * 2.3, 0, 0]
        halo.rgba = [1.0, 0.70, 0.20, 0.16]   # alpha animated at runtime
        halo.contype = 0
        halo.conaffinity = 0

        for axis, rgba in ((0, [1.00, 0.28, 0.32, 0.95]),   # x — red
                           (1, [0.30, 0.95, 0.45, 0.95]),   # y — green
                           (2, [0.32, 0.58, 1.00, 0.95])):  # z — blue
            fromto = [0.0] * 6
            fromto[3 + axis] = ax_len
            ax = b.add_geom()
            ax.name = f"ik_target_ax{'xyz'[axis]}"
            ax.type = mujoco.mjtGeom.mjGEOM_CAPSULE
            ax.fromto = fromto
            ax.size = [0.0035, 0, 0]
            ax.rgba = rgba
            ax.contype = 0
            ax.conaffinity = 0
            if glow_mat:
                ax.material = glow_mat
    except Exception:
        pass


def build_studio_model(robot: str) -> StudioModel:
    """Build the full studio scene for ``robot`` and return it with drive handles."""
    import mujoco

    spec_urdf = _studio_urdf(robot)
    spec = mujoco.MjSpec.from_file(spec_urdf)

    rspec = get_robot_spec(robot)
    smodel = get_sim_model(robot)
    # rough extent for camera/lights from DH reach
    extent = float(np.sum(np.abs(rspec.a)) + np.sum(np.abs(rspec.d))) or 1.0
    _augment_scene(spec, extent)

    # Metallic sheen on the robot's own materials (URDF part colors stay).
    try:
        for m in spec.materials:
            if m.name and not str(m.name).startswith("studio_"):
                m.specular = 0.80
                m.shininess = 0.55
    except Exception:
        pass

    model = spec.compile()

    hinges = [j for j in range(model.njnt)
              if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
    hinges = hinges[: rspec.n_joints]
    q_adr = [int(model.jnt_qposadr[j]) for j in hinges]
    joint_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in hinges]

    ee_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, smodel.ee_link_candidates[0])
    tgt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ik_target")
    target_mocap_id = int(model.body_mocapid[tgt_id]) if tgt_id >= 0 else -1
    halo_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ik_target_halo")

    return StudioModel(
        model=model,
        robot=robot,
        q_adr=q_adr,
        joint_names=joint_names,
        ee_body_id=ee_body_id,
        target_mocap_id=target_mocap_id,
        base_offset_z_deg=smodel.base_offset_z_deg,
        n_joints=rspec.n_joints,
        halo_geom_id=int(halo_geom_id),
        target_body_id=int(tgt_id),
    )
