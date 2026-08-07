"""
Native MuJoCo **live viewer** — watch a real robot arm solve IK, in a real engine.

Unlike the web dashboard (which renders a stylised cylinder *proxy* placed by a
re-implementation of our DH forward kinematics), this opens a native
``mujoco.viewer`` window rendering the **actual UR5 / Franka Panda meshes** — the
exact URDF models the sim oracle (``pybullet_backend`` / ``mujoco_backend``) and the
Phase-1 DH↔URDF parity were validated against. The robot is driven by the *same*
solver code the API/benchmarks use, run **in-process** (no server, no websocket):

    target ── generate_target(spec, scenario) ──►  DH-frame pose
      │
      ├─ run_solver(name, spec, q0, T_target)  ──►  step trace of joint vectors q
      │
      └─ mujoco.viewer:  qpos ← each q  ·  mj_forward  ·  sync   (real meshes move)

The target pose is drawn as a marker (sphere + RGB orientation triad) at
``Rz(base_offset) · T_target`` — the constant DH→URDF frame offset measured in
Phase 1 (UR5: Rz(180°), Panda: identity). Because that offset is exact to float
noise, the rendered arm's end-effector reaches the marker precisely on a good solve.

Rendering uses each link's **collision** mesh (the URDF's visual geometry is
``.dae``, which MuJoCo can't load — see ``_rewrite_urdf_for_mujoco``). For UR5 these
are full-detail; for Franka they are the (chunkier) convex collision meshes. Either
way it is the real robot model, not a proxy.

Run it (deps ``mujoco`` + ``robot_descriptions`` are in the core ``.venv``)::

    # from backend/ , with the venv python
    .venv/Scripts/python.exe -m app.sim.live_viewer                 # UR5, ProteinIK Fast
    .venv/Scripts/python.exe -m app.sim.live_viewer franka_panda multi_start
    .venv/Scripts/python.exe -m app.sim.live_viewer ur5 protein_ik --scenario cluttered --loop
    .venv/Scripts/python.exe -m app.sim.live_viewer --selftest      # headless pipeline check

Args: ``[robot] [solver]`` positional; ``--scenario`` open_space|near_singular|
cluttered; ``--seed``; ``--speed`` playback multiplier; ``--loop`` auto-resolve a new
target when the last one finishes; ``--selftest`` run the whole pipeline with no
window (used to verify without a display).
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

from app.core.kinematics import get_robot_spec, end_effector_pose
from app.api.scenarios import generate_target
from app.solvers.registry import run_solver, SOLVER_REGISTRY, SOLVER_DISPLAY_NAMES
from app.sim.models import get_sim_model, resolve_urdf_path
from app.sim.mujoco_backend import _rewrite_urdf_for_mujoco


# Robot-name aliases so the CLI is forgiving.
_ROBOT_ALIASES = {
    "panda": "franka_panda",
    "franka": "franka_panda",
    "franka_panda": "franka_panda",
    "ur5": "ur5",
}

# EE link frame per robot (first candidate is the validated one — see models.py).
_EE_LINK = {"ur5": "tool0", "franka_panda": "panda_link8"}


# ── model resolution ──────────────────────────────────────────────────────────

def _resolve_urdf_robust(robot: str) -> str:
    """Return an on-disk URDF path for ``robot``, resilient to robot_descriptions
    renames.

    ``resolve_urdf_path`` maps to a pinned ``robot_descriptions`` module, but that
    package renamed ``ur5_description`` (it no longer exists in 3.1.x). The model we
    validated against is example-robot-data's ``ur5_robot.urdf``, so on failure we
    fall back to it inside the robot_descriptions cache. Importing
    ``panda_description`` guarantees the whole example-robot-data repo is cloned, so
    this works on a fresh machine too — not only when UR5 happens to be cached.
    """
    try:
        return resolve_urdf_path(robot)
    except Exception:
        pass
    # Fallback: locate example-robot-data via the panda module (same repo), then
    # join the known relative path of the model we actually validated against.
    import robot_descriptions.panda_description as _pd  # side effect: clones the repo

    marker = "example-robot-data"
    root = _pd.URDF_PATH.replace("\\", "/")
    erd = root[: root.find(marker) + len(marker)]
    rel = {
        "ur5": ("robots", "ur_description", "urdf", "ur5_robot.urdf"),
        "franka_panda": ("robots", "panda_description", "urdf", "panda.urdf"),
    }.get(robot)
    if rel is None:
        raise
    path = os.path.join(erd, *rel)
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"Could not resolve a URDF for '{robot}'. Tried robot_descriptions and "
            f"the example-robot-data cache fallback ({path})."
        )
    return path


def _rz(deg: float) -> np.ndarray:
    """4×4 homogeneous rotation about +Z by ``deg`` degrees (the DH→URDF base offset)."""
    t = np.deg2rad(deg)
    c, s = np.cos(t), np.sin(t)
    m = np.eye(4)
    m[0, 0], m[0, 1] = c, -s
    m[1, 0], m[1, 1] = s, c
    return m


class RobotView:
    """Loads one robot's real MuJoCo model and maps our ``q`` onto its hinges.

    Owns the ``MjModel`` / ``MjData`` the viewer renders. ``dh_to_sim`` expresses a
    DH-frame target in the model's frame using the constant Phase-1 base offset, so
    marker placement lines up with the rendered end-effector.
    """

    def __init__(self, robot: str):
        import mujoco  # lazy — importing this module never requires mujoco

        self.mj = mujoco
        self.robot = robot
        self.spec = get_robot_spec(robot)
        self.ee_link = _EE_LINK[robot]
        self._offset = _rz(get_sim_model(robot).base_offset_z_deg)

        urdf = _resolve_urdf_robust(robot)
        self.model = mujoco.MjModel.from_xml_path(_rewrite_urdf_for_mujoco(urdf, f"view_{robot}"))
        self.data = mujoco.MjData(self.model)

        # Map our q -> hinge qpos addresses, in joint (tree) order. Hinges only, so
        # Franka's two prismatic finger joints are skipped (same rule as the oracle).
        hinges = [j for j in range(self.model.njnt)
                  if self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
        if len(hinges) < self.spec.n_joints:
            raise RuntimeError(
                f"MuJoCo model for '{robot}' exposes {len(hinges)} hinge joints; "
                f"DH spec has {self.spec.n_joints}."
            )
        self._q_adr = [int(self.model.jnt_qposadr[j]) for j in hinges[: self.spec.n_joints]]
        self._ee_bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.ee_link)

    def set_q(self, q) -> None:
        """Write joint vector ``q`` into qpos and run FK so poses/geoms are current."""
        for k, adr in enumerate(self._q_adr):
            if k < len(q):
                self.data.qpos[adr] = float(q[k])
        self.mj.mj_forward(self.model, self.data)

    def ee_pose(self):
        """4×4 world transform of the EE link at the current config (sim frame)."""
        T = np.eye(4)
        T[:3, :3] = np.array(self.data.xmat[self._ee_bid]).reshape(3, 3)
        T[:3, 3] = np.array(self.data.xpos[self._ee_bid])
        return T

    def dh_to_sim(self, T_dh: np.ndarray) -> np.ndarray:
        """Express a DH-frame pose in the model frame (constant Phase-1 base offset)."""
        return self._offset @ T_dh


# ── target / solve (shared by viewer and selftest) ────────────────────────────

def _make_solve(view: RobotView, solver: str, scenario: str, seed):
    """Generate a target and run one solve in-process. Returns
    (T_sim_target, q_start, steps, result)."""
    spec = view.spec
    rng = np.random.default_rng(seed)
    q0, T_target_dh = generate_target(spec, rng, scenario)
    result = run_solver(solver, spec, q0.copy(), T_target_dh, rng, collect_steps=True)
    T_sim_target = view.dh_to_sim(T_target_dh)
    # Trajectory of joint vectors: prefer the recorded step trace; fall back to a
    # start→final interpolation for any solver that records no steps.
    if result.steps:
        traj = [np.asarray(s.q, dtype=float) for s in result.steps]
        phases = [getattr(s, "phase", "") for s in result.steps]
        pos_errs = [s.pos_error for s in result.steps]
    else:
        qf = np.asarray(result.q_final, dtype=float)
        traj = [q0 + (qf - q0) * t for t in np.linspace(0.0, 1.0, 60)]
        phases = [""] * len(traj)
        pos_errs = [result.pos_error] * len(traj)
    return T_sim_target, q0, traj, phases, pos_errs, result


# ── marker drawing ────────────────────────────────────────────────────────────

_RED = np.array([0.95, 0.25, 0.2, 1.0], dtype=np.float32)
_GRN = np.array([0.3, 0.95, 0.45, 1.0], dtype=np.float32)
_BLU = np.array([0.3, 0.5, 0.95, 1.0], dtype=np.float32)
_AMBER = np.array([1.0, 0.75, 0.2, 0.95], dtype=np.float32)
_OK = np.array([0.35, 0.95, 0.5, 0.95], dtype=np.float32)


def _draw_target(scn, mj, T_sim_target: np.ndarray, reached: bool) -> None:
    """Populate ``scn`` (a viewer user-scene) with a target sphere + RGB pose triad.

    Sphere colour flags whether the arm reached it (green) or not yet (amber). The
    three capsules are the target frame's X/Y/Z axes (red/green/blue).
    """
    pos = T_sim_target[:3, 3]
    R = T_sim_target[:3, :3]
    scn.ngeom = 0
    eye = np.eye(3).flatten()

    # Target position sphere.
    mj.mjv_initGeom(scn.geoms[scn.ngeom], int(mj.mjtGeom.mjGEOM_SPHERE),
                    np.array([0.03, 0, 0]), pos, eye, _OK if reached else _AMBER)
    scn.ngeom += 1

    # Orientation triad — a thin capsule along each target axis.
    for axis, col in ((0, _RED), (1, _GRN), (2, _BLU)):
        p1 = pos + 0.10 * R[:, axis]
        g = scn.geoms[scn.ngeom]
        mj.mjv_initGeom(g, int(mj.mjtGeom.mjGEOM_CAPSULE),
                        np.zeros(3), np.zeros(3), eye, col)
        mj.mjv_connector(g, int(mj.mjtGeom.mjGEOM_CAPSULE), 0.005, pos, p1)
        scn.ngeom += 1


# ── console HUD ───────────────────────────────────────────────────────────────

def _status(robot, solver, phase, i, n, pos_err, orient_err, done=False, success=None):
    tag = SOLVER_DISPLAY_NAMES.get(solver, solver)
    ph = f" {phase:<12}" if phase else " " * 13
    line = (f"\r  {robot:<12} {tag:<26}{ph} step {i:>4}/{n:<4} "
            f"pos {pos_err*1000:7.2f} mm  orient {np.rad2deg(orient_err):6.2f} deg")
    if done:
        line += f"   ->  {'converged' if success else 'did not converge'}    "
    sys.stdout.write(line)
    sys.stdout.flush()


# ── main viewer loop ──────────────────────────────────────────────────────────

def run_viewer(robot: str, solver: str, scenario: str, seed, speed: float, loop: bool):
    import mujoco.viewer  # lazy — needs a display

    view = RobotView(robot)
    dt = max(0.006, 0.03 / max(speed, 1e-3))

    with mujoco.viewer.launch_passive(view.model, view.data,
                                      show_left_ui=False, show_right_ui=False) as viewer:
        # A gentle default camera looking at the arm's mid-height.
        viewer.cam.distance = 2.4
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        viewer.cam.lookat[:] = [0.0, 0.0, 0.4]

        cur_seed = seed
        while viewer.is_running():
            T_sim_target, q0, traj, phases, pos_errs, result = _make_solve(
                view, solver, scenario, cur_seed)
            n = len(traj)

            # Animate through the trajectory.
            for i, q in enumerate(traj):
                if not viewer.is_running():
                    break
                view.set_q(q)
                _draw_target(viewer.user_scn, view.mj, T_sim_target, reached=False)
                viewer.sync()
                oe = result.orient_error if i == n - 1 else 0.0
                _status(robot, solver, phases[i], i + 1, n, pos_errs[i], oe)
                time.sleep(dt)

            # Settle on q_final and flag the outcome on the marker + HUD.
            view.set_q(result.q_final)
            _draw_target(viewer.user_scn, view.mj, T_sim_target, reached=result.success)
            viewer.sync()
            _status(robot, solver, "", n, n, result.pos_error, result.orient_error,
                    done=True, success=result.success)
            print()

            if not loop:
                # Keep the window open until the user closes it.
                while viewer.is_running():
                    viewer.sync()
                    time.sleep(0.05)
                break

            # Brief hold, then re-solve a fresh target.
            for _ in range(int(1.2 / 0.05)):
                if not viewer.is_running():
                    break
                viewer.sync()
                time.sleep(0.05)
            cur_seed = None if cur_seed is None else cur_seed + 1


def run_selftest(seed) -> int:
    """Headless end-to-end check: resolve model, solve, animate qpos, and confirm the
    rendered EE reaches the target marker. Returns a process exit code."""
    print("live_viewer selftest (headless - no window)\n" + "=" * 60)
    combos = [
        ("ur5", "protein_fast", "open_space"),
        ("ur5", "trac_ik_style", "cluttered"),
        ("franka_panda", "multi_start", "open_space"),
    ]
    ok = True
    for robot, solver, scenario in combos:
        view = RobotView(robot)
        T_sim_target, q0, traj, phases, pos_errs, result = _make_solve(
            view, solver, scenario, seed)
        # Drive the whole trajectory through the model (exercises the render path).
        for q in traj:
            view.set_q(q)
        # Where did the *rendered* arm's EE actually end up vs the target marker?
        view.set_q(result.q_final)
        ee = view.ee_pose()
        marker_err = float(np.linalg.norm(ee[:3, 3] - T_sim_target[:3, 3]))
        # On a converged solve the rendered EE must sit on the marker; if the solver
        # failed, the arm simply won't reach it (expected) — so we only assert the
        # geometry is consistent (rendered EE == solver's own reported error).
        consistent = abs(marker_err - result.pos_error) < 1e-3
        status = "OK" if consistent else "INCONSISTENT"
        if not consistent:
            ok = False
        print(f"  {robot:<13} {solver:<16} {scenario:<13} steps={len(traj):>4} "
              f"success={str(result.success):<5} "
              f"solver_pos_err={result.pos_error*1000:6.2f}mm "
              f"rendered_EE_to_marker={marker_err*1000:6.2f}mm  [{status}]")
    print("=" * 60)
    print("selftest PASSED - pipeline + real-mesh FK consistent." if ok
          else "selftest FAILED - rendered EE disagrees with solver.")
    return 0 if ok else 1


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        prog="app.sim.live_viewer",
        description="Native MuJoCo viewer: watch a real robot arm solve IK live.")
    p.add_argument("robot", nargs="?", default="ur5",
                   help="ur5 | franka_panda (default: ur5)")
    p.add_argument("solver", nargs="?", default="protein_fast",
                   help=f"solver id (default: protein_fast). One of: {list(SOLVER_REGISTRY)}")
    p.add_argument("--scenario", default="open_space",
                   choices=["open_space", "near_singular", "cluttered"])
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--speed", type=float, default=1.0, help="playback speed multiplier")
    p.add_argument("--loop", action="store_true",
                   help="auto-generate a new target after each solve")
    p.add_argument("--selftest", action="store_true",
                   help="run the full pipeline headless (no window) and exit")
    args = p.parse_args(argv)

    if args.selftest:
        return run_selftest(args.seed)

    robot = _ROBOT_ALIASES.get(args.robot.lower())
    if robot is None:
        p.error(f"unknown robot '{args.robot}'. Use: ur5, franka_panda")
    if args.solver not in SOLVER_REGISTRY:
        p.error(f"unknown solver '{args.solver}'. Available: {list(SOLVER_REGISTRY)}")
    if args.solver == "analytical_planar3dof":
        p.error("analytical_planar3dof is only valid for planar3dof (no URDF mesh model).")

    print(f"Opening MuJoCo viewer - {robot} / "
          f"{SOLVER_DISPLAY_NAMES.get(args.solver, args.solver)} / {args.scenario}")
    print("Close the window (or Ctrl+C) to exit."
          + ("  [--loop: new target each solve]" if args.loop else ""))
    try:
        run_viewer(robot, args.solver, args.scenario, args.seed, args.speed, args.loop)
    except KeyboardInterrupt:
        print("\ninterrupted.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
