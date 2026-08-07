"""
ProteinIK **IK Studio** — a native MuJoCo application for exploring inverse
kinematics on the real robot models.

A single interactive window (GLFW + MuJoCo's own renderer) that shows the *actual*
UR5 / Franka Panda meshes on a studio floor and lets you:

  * **orbit / pan / zoom** the camera (mouse),
  * **place an IK target** by Ctrl+clicking anywhere on the robot or floor,
  * watch the selected **solver** drive the real arm to that target, live,
  * read **live metrics** (phase, iteration, position/orientation error, self-
    clearance, success) as it solves,
  * **switch robots** (UR5 / Franka) and **cycle solvers** with the keyboard,
  * **compare every solver** on the current target and see a ranked table.

All solving is the *same* in-process solver code the API and benchmarks use
(`app.solvers.registry.run_solver`) on the validated DH `RobotSpec`; the studio only
renders and does FK. The scene (real visual meshes + floor/sky/lights + the target
mocap) is built by `app.sim.studio_scene.build_studio_model`.

Run it (deps `mujoco`, `glfw`, `robot_descriptions`, `trimesh` are in the core `.venv`)::

    cd backend
    .venv/Scripts/python.exe -m app.sim.ik_studio                 # UR5
    .venv/Scripts/python.exe -m app.sim.ik_studio franka_panda protein_ik
    .venv/Scripts/python.exe -m app.sim.ik_studio --selftest      # headless checks + screenshots
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from app.core.kinematics import get_robot_spec, end_effector_pose
from app.api.scenarios import generate_target
from app.solvers.registry import run_solver, get_solvers_for_robot, SOLVER_DISPLAY_NAMES
from app.sim.studio_scene import build_studio_model


DT_STEP = 0.03            # seconds per recorded solver step during playback
_ROBOTS = ["ur5", "franka_panda"]
_ROBOT_LABEL = {"ur5": "UR5 (6-DOF)", "franka_panda": "Franka Panda (7-DOF)"}
_ALIASES = {"panda": "franka_panda", "franka": "franka_panda", "ur5": "ur5",
            "franka_panda": "franka_panda"}


def _rz3(deg: float) -> np.ndarray:
    """3×3 rotation about +Z (the constant DH->URDF base offset)."""
    t = np.deg2rad(deg)
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class IKStudio:
    """Owns the model/data/solve state. Rendering + input live in ``run`` (GLFW)."""

    def __init__(self, robot: str, solver: str, scenario: str = "open_space", speed: float = 1.0):
        self.robot = robot
        self.solver = solver
        self.scenario = scenario
        self.dt_step = DT_STEP / max(speed, 1e-3)
        self.show_help = True
        self.compare = None          # ranked list of solver rows, or None
        self.status = "ready"
        self.busy = None             # transient "computing…" message
        self._pending_robot = None   # set on robot switch; applied in the GL thread

        self._load(robot)
        self._new_target(solve=True)

    # ── model lifecycle ───────────────────────────────────────────────────────

    def _load(self, robot: str):
        import mujoco
        self.mj = mujoco
        self.robot = robot
        self.sm = build_studio_model(robot)
        self.model = self.sm.model
        self.data = mujoco.MjData(self.model)
        self.spec = get_robot_spec(robot)
        self.offset = self.sm.base_offset_z_deg
        self.solvers = [s for s in get_solvers_for_robot(robot)
                        if s != "analytical_planar3dof"]
        if self.solver not in self.solvers:
            self.solver = "protein_ik" if "protein_ik" in self.solvers else self.solvers[0]
        self.traj = None
        self.cur_idx = 0
        self.result = None
        self.T_target_dh = None
        self._set_q(np.zeros(self.sm.n_joints))

    def _set_q(self, q) -> None:
        for k, a in enumerate(self.sm.q_adr):
            if k < len(q):
                self.data.qpos[a] = float(q[k])
        self.mj.mj_forward(self.model, self.data)
        self.cur_q = np.asarray(q, dtype=float)

    def _update_marker(self) -> None:
        if self.sm.target_mocap_id >= 0 and self.T_target_dh is not None:
            p_sim = _rz3(self.offset) @ self.T_target_dh[:3, 3]
            self.data.mocap_pos[self.sm.target_mocap_id] = p_sim

    # ── solving ───────────────────────────────────────────────────────────────

    def _solve(self, T_dh: np.ndarray, solver: str, q0=None) -> None:
        rng = np.random.default_rng()
        if q0 is None:
            q0 = getattr(self, "cur_q", None)
            if q0 is None or len(q0) != self.sm.n_joints:
                q0 = self.spec.random_config(rng)
        self.status = f"solving · {SOLVER_DISPLAY_NAMES.get(solver, solver)}"
        result = run_solver(solver, self.spec, np.asarray(q0, float), T_dh, rng,
                            collect_steps=True)
        self.solver = solver
        self.result = result
        self.T_target_dh = T_dh
        self.traj = ([np.asarray(s.q, float) for s in result.steps]
                     if result.steps else [np.asarray(result.q_final, float)])
        self.step_meta = result.steps
        self.cur_idx = 0
        self.traj_start = time.perf_counter()
        self._update_marker()
        self.status = "done"

    def _new_target(self, solve: bool = True) -> None:
        rng = np.random.default_rng()
        q0, T_dh = generate_target(self.spec, rng, self.scenario)
        self.compare = None
        if solve:
            self._solve(T_dh, self.solver, q0=q0)
        else:
            self.T_target_dh = T_dh
            self._update_marker()

    def _compare_all(self) -> None:
        if self.T_target_dh is None:
            return
        self.busy = "comparing all solvers…"
        q0 = self.spec.random_config(np.random.default_rng(1234))
        rows = []
        for s in self.solvers:
            t0 = time.perf_counter()
            r = run_solver(s, self.spec, q0.copy(), self.T_target_dh,
                           np.random.default_rng(7), collect_steps=False)
            rows.append({
                "id": s, "name": SOLVER_DISPLAY_NAMES.get(s, s),
                "success": bool(r.success), "pos": float(r.pos_error),
                "orient": float(r.orient_error), "ms": float(r.wall_time_ms),
                "clr": float(r.min_self_distance),
            })
        rows.sort(key=lambda x: (not x["success"], x["pos"]))
        self.compare = rows
        self.busy = None
        self._solve(self.T_target_dh, rows[0]["id"], q0=q0)   # animate the winner

    def cycle_solver(self, step: int) -> None:
        i = (self.solvers.index(self.solver) + step) % len(self.solvers)
        self.compare = None
        if self.T_target_dh is not None:
            self._solve(self.T_target_dh, self.solvers[i], q0=self.spec.random_config(np.random.default_rng()))

    def place_target(self, p_sim: np.ndarray) -> None:
        """Set the IK target position to a clicked world point (keep orientation)."""
        T = (self.T_target_dh.copy() if self.T_target_dh is not None else np.eye(4))
        T[:3, 3] = _rz3(-self.offset) @ np.asarray(p_sim, float)   # sim -> DH
        self.compare = None
        self._solve(T, self.solver)

    # ── per-frame animation advance ───────────────────────────────────────────

    def advance(self) -> None:
        if not self.traj:
            return
        idx = min(int((time.perf_counter() - self.traj_start) / self.dt_step),
                  len(self.traj) - 1)
        if idx != self.cur_idx or idx == 0:
            self.cur_idx = idx
            self._set_q(self.traj[idx])
        else:
            self.mj.mj_forward(self.model, self.data)  # keep marker/geoms fresh

    # ── overlay text ──────────────────────────────────────────────────────────

    def _metrics_columns(self):
        r = self.result
        m = None
        if self.traj and getattr(self, "step_meta", None):
            m = self.step_meta[min(self.cur_idx, len(self.step_meta) - 1)]
        labels, values = [], []

        def row(k, v):
            labels.append(k); values.append(v)

        row("robot", _ROBOT_LABEL[self.robot])
        row("solver", SOLVER_DISPLAY_NAMES.get(self.solver, self.solver))
        row("status", self.status)
        if m is not None:
            row("phase", m.phase or "-")
            row("iteration", str(m.iteration))
            row("pos error", f"{m.pos_error * 1000:.2f} mm")
            row("orient error", f"{np.rad2deg(m.orient_error):.2f} deg")
            row("self-clearance", f"{m.min_self_distance * 1000:.1f} mm")
        if r is not None:
            row("result", "CONVERGED" if r.success else "did not converge")
            row("final pos err", f"{r.pos_error * 1000:.2f} mm")
            row("solve time", f"{r.wall_time_ms:.1f} ms")
        return "\n".join(labels), "\n".join(values)

    def _compare_text(self):
        if not self.compare:
            return None, None
        head = f"{'solver':<20}{'ok':>4}{'pos(mm)':>9}{'ms':>7}"
        lines = [head]
        for row in self.compare:
            lines.append(f"{row['name'][:20]:<20}"
                         f"{('YES' if row['success'] else 'no'):>4}"
                         f"{row['pos'] * 1000:>9.2f}{row['ms']:>7.1f}")
        return "COMPARE — all solvers, this target", "\n".join(lines)

    def _help_text(self):
        return (
            "IK STUDIO",
            "Ctrl+Click  place IK target\n"
            "L-drag orbit   R-drag pan   scroll zoom\n"
            "R  new target      C  compare all solvers\n"
            "[ ]  cycle solver     1/2  UR5 / Franka\n"
            "SPACE replay   H help   ESC quit"
        )

    # ── main interactive loop ─────────────────────────────────────────────────

    def run(self) -> None:
        import glfw
        mujoco = self.mj

        if not glfw.init():
            raise RuntimeError("Failed to initialize GLFW (no display?).")
        window = glfw.create_window(1360, 860, "ProteinIK — IK Studio", None, None)
        if not window:
            glfw.terminate()
            raise RuntimeError("Failed to create GLFW window.")
        glfw.make_context_current(window)
        glfw.swap_interval(1)

        cam = mujoco.MjvCamera()
        opt = mujoco.MjvOption()
        scene = [None]
        context = [None]

        def rebuild_gl():
            scene[0] = mujoco.MjvScene(self.model, maxgeom=20000)
            context[0] = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)
            self._frame_camera(cam)

        rebuild_gl()

        # ── input state ──
        state = {"lx": 0.0, "ly": 0.0, "left": False, "right": False, "mid": False}

        def on_mouse_button(win, button, action, mods):
            x, y = glfw.get_cursor_pos(win)
            state["lx"], state["ly"] = x, y
            pressed = action == glfw.PRESS
            if button == glfw.MOUSE_BUTTON_LEFT:
                state["left"] = pressed
                if pressed and (mods & glfw.MOD_CONTROL):
                    self._pick(win, x, y, opt, scene[0])
            elif button == glfw.MOUSE_BUTTON_RIGHT:
                state["right"] = pressed
            elif button == glfw.MOUSE_BUTTON_MIDDLE:
                state["mid"] = pressed

        def on_cursor(win, x, y):
            dx, dy = x - state["lx"], y - state["ly"]
            state["lx"], state["ly"] = x, y
            if not (state["left"] or state["right"] or state["mid"]):
                return
            w, h = glfw.get_window_size(win)
            h = max(h, 1)
            if state["left"] and (glfw.get_key(win, glfw.KEY_LEFT_CONTROL) == glfw.PRESS):
                return  # ctrl+left is "place target", not orbit
            if state["left"]:
                action = mujoco.mjtMouse.mjMOUSE_ROTATE_V
            elif state["right"]:
                action = mujoco.mjtMouse.mjMOUSE_MOVE_H
            else:
                action = mujoco.mjtMouse.mjMOUSE_ZOOM
            mujoco.mjv_moveCamera(self.model, action, dx / h, dy / h, cam)

        def on_scroll(win, xoff, yoff):
            mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_ZOOM,
                                  0.0, -0.05 * yoff, cam)

        def on_key(win, key, scancode, action, mods):
            if action != glfw.PRESS and action != glfw.REPEAT:
                return
            if key in (glfw.KEY_ESCAPE, glfw.KEY_Q):
                glfw.set_window_should_close(win, True)
            elif key == glfw.KEY_R:
                self._new_target(solve=True)
            elif key == glfw.KEY_C:
                self._compare_all()
            elif key == glfw.KEY_RIGHT_BRACKET:
                self.cycle_solver(+1)
            elif key == glfw.KEY_LEFT_BRACKET:
                self.cycle_solver(-1)
            elif key == glfw.KEY_SPACE:
                self.traj_start = time.perf_counter(); self.cur_idx = 0
            elif key == glfw.KEY_H:
                self.show_help = not self.show_help
            elif key in (glfw.KEY_1, glfw.KEY_2):
                self._pending_robot = _ROBOTS[0 if key == glfw.KEY_1 else 1]

        glfw.set_mouse_button_callback(window, on_mouse_button)
        glfw.set_cursor_pos_callback(window, on_cursor)
        glfw.set_scroll_callback(window, on_scroll)
        glfw.set_key_callback(window, on_key)

        print("IK Studio open. Ctrl+Click to place a target; R new target; "
              "C compare; [ ] solver; 1/2 robot; H help; ESC quit.")

        while not glfw.window_should_close(window):
            if self._pending_robot and self._pending_robot != self.robot:
                self._load(self._pending_robot)
                self._pending_robot = None
                self._new_target(solve=True)
                rebuild_gl()

            self.advance()

            w, h = glfw.get_framebuffer_size(window)
            viewport = mujoco.MjrRect(0, 0, w, h)
            mujoco.mjv_updateScene(self.model, self.data, opt, None, cam,
                                   mujoco.mjtCatBit.mjCAT_ALL, scene[0])
            mujoco.mjr_render(viewport, scene[0], context[0])

            font = mujoco.mjtFont.mjFONT_NORMAL
            gp = mujoco.mjtGridPos
            lab, val = self._metrics_columns()
            mujoco.mjr_overlay(font, gp.mjGRID_BOTTOMLEFT, viewport, lab, val, context[0])
            if self.show_help:
                t, b = self._help_text()
                mujoco.mjr_overlay(font, gp.mjGRID_TOPLEFT, viewport, t, b, context[0])
            if self.busy:
                mujoco.mjr_overlay(font, gp.mjGRID_TOP, viewport, self.busy, "", context[0])
            ct, cb = self._compare_text()
            if ct:
                mujoco.mjr_overlay(font, gp.mjGRID_BOTTOMRIGHT, viewport, ct, cb, context[0])

            glfw.swap_buffers(window)
            glfw.poll_events()

        glfw.terminate()

    def _frame_camera(self, cam) -> None:
        mujoco = self.mj
        mujoco.mj_forward(self.model, self.data)
        gp = self.data.geom_xpos
        lo, hi = gp.min(axis=0), gp.max(axis=0)
        cam.lookat[:] = (lo + hi) / 2.0
        cam.distance = float(np.linalg.norm(hi - lo)) * 1.6 + 0.4
        cam.azimuth = 135.0
        cam.elevation = -20.0

    def _pick(self, win, x, y, opt, scene) -> None:
        import glfw
        mujoco = self.mj
        ww, wh = glfw.get_window_size(win)
        fw, fh = glfw.get_framebuffer_size(win)
        sx, sy = fw / max(ww, 1), fh / max(wh, 1)
        relx = (x * sx) / max(fw, 1)
        rely = (fh - y * sy) / max(fh, 1)
        aspect = fw / max(fh, 1)
        selpnt = np.zeros(3, dtype=np.float64)
        geomid = np.zeros(1, dtype=np.int32)
        flexid = np.zeros(1, dtype=np.int32)
        skinid = np.zeros(1, dtype=np.int32)
        found = mujoco.mjv_select(self.model, self.data, opt, aspect, relx, rely,
                                  scene, selpnt, geomid, flexid, skinid)
        if found >= 0:
            self.place_target(selpnt.copy())


# ── headless self-test (no window) ────────────────────────────────────────────

def run_selftest(save_dir: str | None = None) -> int:
    """Exercise the whole studio pipeline with no display: build each robot, solve,
    compare all solvers, verify the rendered EE reaches the target marker, and
    (optionally) save an offscreen screenshot per robot."""
    import os
    import mujoco
    print("IK Studio selftest (headless)\n" + "=" * 64)
    ok = True
    for robot in _ROBOTS:
        st = IKStudio(robot, "protein_ik")
        # animate to the end of the trajectory
        st._set_q(st.result.q_final)
        ee = st.data.xpos[st.sm.ee_body_id].copy()
        marker = _rz3(st.offset) @ st.T_target_dh[:3, 3]
        err = float(np.linalg.norm(ee - marker))
        consistent = abs(err - st.result.pos_error) < 2e-3
        ok = ok and consistent
        st._compare_all()
        nwin = sum(1 for r in st.compare if r["success"])
        print(f"  {robot:<13} solve_ok={str(st.result.success):<5} "
              f"rendered_EE_to_marker={err*1000:6.2f}mm "
              f"[{'OK' if consistent else 'MISMATCH'}]   "
              f"compare: {len(st.compare)} solvers, {nwin} reached target")
        if save_dir:
            r = mujoco.Renderer(st.model, 720, 1080)
            cam = mujoco.MjvCamera()
            st._frame_camera(cam)
            r.update_scene(st.data, cam)
            try:
                from PIL import Image
                p = os.path.join(save_dir, f"ikstudio_{robot}.png")
                Image.fromarray(r.render()).save(p)
                print(f"      screenshot -> {p}")
            except Exception as e:
                print(f"      (screenshot skipped: {e})")
    print("=" * 64)
    print("selftest PASSED" if ok else "selftest FAILED")
    return 0 if ok else 1


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="app.sim.ik_studio",
                                description="Native MuJoCo IK Studio.")
    p.add_argument("robot", nargs="?", default="ur5", help="ur5 | franka_panda")
    p.add_argument("solver", nargs="?", default="protein_ik", help="solver id")
    p.add_argument("--scenario", default="open_space",
                   choices=["open_space", "near_singular", "cluttered"])
    p.add_argument("--speed", type=float, default=1.0)
    p.add_argument("--selftest", action="store_true")
    p.add_argument("--save-dir", default=None, help="selftest: dir to save screenshots")
    args = p.parse_args(argv)

    if args.selftest:
        return run_selftest(args.save_dir)

    robot = _ALIASES.get(args.robot.lower())
    if robot is None:
        p.error("unknown robot; use ur5 or franka_panda")
    print(f"Building IK Studio — {robot} / {args.solver} … (first load converts meshes)")
    studio = IKStudio(robot, args.solver, scenario=args.scenario, speed=args.speed)
    try:
        studio.run()
    except KeyboardInterrupt:
        print("\ninterrupted.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
