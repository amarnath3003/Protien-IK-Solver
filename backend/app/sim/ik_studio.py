"""
ProteinIK **IK Studio** — a native MuJoCo application for exploring inverse
kinematics on the real robot models.

A single interactive window (GLFW + MuJoCo's own renderer) that shows the *actual*
UR5 / Franka Panda meshes in a dark reflective studio and lets you:

  * **orbit / pan / zoom** the camera (mouse), or hit **O** for a cinematic
    auto-orbit,
  * **place an IK target** by Ctrl+clicking anywhere on the robot or floor —
    the target renders as a glowing core + pulsing halo + RGB orientation triad,
    with a light beam down to the floor and an expanding ground ring,
  * watch the selected **solver** drive the real arm there, leaving a glowing
    **motion trail** behind the end-effector,
  * read the custom **HUD**: top status bar, color-coded live telemetry panel,
    playback progress bar, and a live **convergence plot** (log₁₀ position /
    orientation error per iteration),
  * **switch robots** (UR5 / Franka) and **cycle solvers** with the keyboard,
  * **compare every solver** on the current target and see a ranked table.

All solving is the *same* in-process solver code the API and benchmarks use
(`app.solvers.registry.run_solver`) on the validated DH `RobotSpec`; the studio only
renders and does FK. The scene (real visual meshes + floor/sky/light rig + the
target mocap) is built by `app.sim.studio_scene.build_studio_model`.

Run it (deps `mujoco`, `glfw`, `robot_descriptions`, `trimesh` are in the core `.venv`)::

    cd backend
    .venv/Scripts/python.exe -m app.sim.ik_studio                 # UR5
    .venv/Scripts/python.exe -m app.sim.ik_studio franka_panda protein_ik
    .venv/Scripts/python.exe -m app.sim.ik_studio --selftest      # headless checks + screenshots
    .venv/Scripts/python.exe -m app.sim.ik_studio --frames 60 --shot out.png   # HUD smoke test
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np

from app.core.kinematics import get_robot_spec
from app.api.scenarios import generate_target
from app.solvers.registry import run_solver, get_solvers_for_robot, SOLVER_DISPLAY_NAMES
from app.sim.studio_scene import build_studio_model


DT_STEP = 0.03            # seconds per recorded solver step during playback
TRAIL_MAX = 500           # max end-effector trail points kept
_ROBOTS = ["ur5", "franka_panda"]
_ROBOT_LABEL = {"ur5": "UR5 (6-DOF)", "franka_panda": "Franka Panda (7-DOF)"}
_ALIASES = {"panda": "franka_panda", "franka": "franka_panda", "ur5": "ur5",
            "franka_panda": "franka_panda"}

# ── HUD palette ────────────────────────────────────────────────────────────────
C_BG = (0.024, 0.031, 0.048, 0.86)      # panel background
C_ACCENT = (0.22, 0.84, 1.00)           # cyan accent
C_TEXT = (0.90, 0.93, 0.97)
C_DIM = (0.52, 0.60, 0.72)
C_GOOD = (0.30, 0.95, 0.55)
C_BAD = (1.00, 0.38, 0.38)
C_AMBER = (1.00, 0.78, 0.28)

_HELP_ROWS = [
    ("Ctrl+Click", "place IK target"),
    ("drag / scroll", "orbit - pan - zoom"),
    ("R", "new random target"),
    ("C", "compare all solvers"),
    ("[  ]", "cycle solver"),
    ("1 / 2", "UR5 / Franka Panda"),
    ("O", "cinematic auto-orbit"),
    ("SPACE", "replay solve"),
    ("H", "toggle this panel"),
    ("ESC", "quit"),
]


def _rz3(deg: float) -> np.ndarray:
    """3×3 rotation about +Z (the constant DH->URDF base offset)."""
    t = np.deg2rad(deg)
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _setup_figure(mujoco, fig) -> None:
    """Style the live convergence plot (dark glass panel, cyan/amber lines)."""
    mujoco.mjv_defaultFigure(fig)
    fig.flg_extend = 0
    fig.flg_legend = 1
    fig.flg_ticklabel[0] = 1
    fig.flg_ticklabel[1] = 1
    fig.figurergba = [0.015, 0.024, 0.042, 0.88]
    fig.panergba = [0.020, 0.030, 0.052, 0.55]
    fig.legendrgba = [0.02, 0.03, 0.05, 0.55]
    fig.gridrgb = [0.16, 0.20, 0.27]
    fig.textrgb = [0.74, 0.82, 0.92]
    fig.gridsize = [6, 5]
    fig.linewidth = 1.8
    fig.title = "convergence  log10(err)"
    fig.xformat = "%.0f"
    fig.linergb[0] = [0.25, 0.85, 1.00]
    fig.linergb[1] = [1.00, 0.72, 0.25]
    fig.linename[0] = "pos mm"
    fig.linename[1] = "ori deg"


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
        self.orbit = False           # cinematic auto-orbit toggle
        self.trail = []              # end-effector world positions this solve
        self.fig_x = None            # convergence plot data (or None)
        self.fig_y = None
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
        self.trail = []
        self.fig_x = None
        self.fig_y = None
        self._set_q(np.zeros(self.sm.n_joints))

    def _set_q(self, q) -> None:
        for k, a in enumerate(self.sm.q_adr):
            if k < len(q):
                self.data.qpos[a] = float(q[k])
        self.mj.mj_forward(self.model, self.data)
        self.cur_q = np.asarray(q, dtype=float)

    def _update_marker(self) -> None:
        if self.sm.target_mocap_id >= 0 and self.T_target_dh is not None:
            R = _rz3(self.offset)
            self.data.mocap_pos[self.sm.target_mocap_id] = R @ self.T_target_dh[:3, 3]
            # orient the triad with the full target frame
            quat = np.zeros(4)
            self.mj.mju_mat2Quat(quat, (R @ self.T_target_dh[:3, :3]).ravel())
            self.data.mocap_quat[self.sm.target_mocap_id] = quat

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
        self.trail = []
        self._build_figure_data(result.steps or [])
        self.traj_start = time.perf_counter()
        self._update_marker()
        self.status = "done"

    def _build_figure_data(self, steps) -> None:
        """Precompute the convergence-plot series (log10 mm / log10 deg)."""
        if len(steps) < 2:
            self.fig_x = None
            self.fig_y = None
            return
        xs = np.arange(len(steps), dtype=float)
        pe = np.log10(np.maximum(
            np.array([s.pos_error for s in steps], float) * 1000.0, 1e-9))
        oe = np.log10(np.maximum(
            np.rad2deg(np.array([s.orient_error for s in steps], float)), 1e-6))
        if len(xs) > 1000:                      # mjMAXLINEPNT
            sel = np.linspace(0, len(xs) - 1, 1000).astype(int)
            xs, pe, oe = xs[sel], pe[sel], oe[sel]
        self.fig_x = xs
        self.fig_y = (pe, oe)

    def _new_target(self, solve: bool = True) -> None:
        rng = np.random.default_rng()
        self.compare = None

        # Reject targets whose EE would be at or below the floor (z ≤ 0).
        # The DH→sim transform is a pure Z-rotation, so T_dh[2,3] == sim z.
        # We keep trying until the EE z > MIN_Z, or fall back to the best
        # above-floor candidate found so far (avoids an infinite loop).
        MIN_Z = 0.05   # 5 cm clearance above the floor
        best_q0, best_T = None, None
        for _ in range(50):
            q0, T_dh = generate_target(self.spec, rng, self.scenario)
            if T_dh[2, 3] > MIN_Z:
                break          # good target — use it
            if best_T is None or T_dh[2, 3] > best_T[2, 3]:
                best_q0, best_T = q0, T_dh  # track highest z seen so far
        else:
            # All 50 samples were below MIN_Z — use the highest one found.
            if best_T is not None:
                q0, T_dh = best_q0, best_T

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
            ee = self.data.xpos[self.sm.ee_body_id].copy()
            if not self.trail or np.linalg.norm(ee - self.trail[-1]) > 1e-4:
                self.trail.append(ee)
                if len(self.trail) > TRAIL_MAX:
                    self.trail.pop(0)
        else:
            self.mj.mj_forward(self.model, self.data)  # keep marker/geoms fresh

    # ── scene decorations (EE trail, target beam/ring, halo pulse) ────────────

    def _add_decor(self, scene, tnow: float) -> None:
        mujoco = self.mj
        I3 = np.eye(3).ravel()

        def new_geom():
            if scene.ngeom >= scene.maxgeom:
                return None
            g = scene.geoms[scene.ngeom]
            scene.ngeom += 1
            return g

        # glowing end-effector trail: cyan -> white, growing + brightening
        pts = self.trail
        if len(pts) >= 2:
            step = max(1, len(pts) // 220)
            sel = pts[::step]
            m = len(sel)
            for i, p in enumerate(sel):
                g = new_geom()
                if g is None:
                    break
                f = i / max(m - 1, 1)
                mujoco.mjv_initGeom(
                    g, mujoco.mjtGeom.mjGEOM_SPHERE,
                    np.array([0.0032 + 0.0048 * f, 0, 0]),
                    np.asarray(p, float), I3,
                    np.array([0.25 + 0.65 * f, 0.78 + 0.22 * f, 1.0,
                              0.10 + 0.60 * f], np.float32))
                g.emission = 0.7

        # target: light beam to the floor + expanding ground ring + halo pulse
        if self.T_target_dh is not None and self.sm.target_mocap_id >= 0:
            p = np.asarray(self.data.mocap_pos[self.sm.target_mocap_id], float)
            pulse = 0.5 + 0.5 * math.sin(tnow * 2.6)
            if p[2] > 0.02:
                g = new_geom()
                if g is not None:
                    mujoco.mjv_initGeom(
                        g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3),
                        np.zeros(3), I3, np.array([1.0, 0.72, 0.25, 0.15], np.float32))
                    mujoco.mjv_connector(
                        g, int(mujoco.mjtGeom.mjGEOM_CAPSULE), 0.0018,
                        np.array([p[0], p[1], 0.001]), p)
                    g.emission = 0.6
            g = new_geom()
            if g is not None:
                r = 0.045 + 0.030 * pulse
                mujoco.mjv_initGeom(
                    g, mujoco.mjtGeom.mjGEOM_CYLINDER,
                    np.array([r, 0.0008, 0]),
                    np.array([p[0], p[1], 0.0014]), I3,
                    np.array([1.0, 0.72, 0.25, 0.30 * (1.0 - pulse) + 0.05], np.float32))
                g.emission = 0.5
            if self.sm.halo_geom_id >= 0:
                self.model.geom_rgba[self.sm.halo_geom_id, 3] = 0.08 + 0.16 * pulse

    # ── HUD ───────────────────────────────────────────────────────────────────

    def _metric_rows(self):
        """(label, value, color) rows for the telemetry panel."""
        r = self.result
        m = None
        if self.traj and getattr(self, "step_meta", None):
            m = self.step_meta[min(self.cur_idx, len(self.step_meta) - 1)]
        rows = []
        if m is not None:
            rows.append(("phase", m.phase or "-", C_TEXT))
            rows.append(("iteration", str(m.iteration), C_TEXT))
            pe = m.pos_error * 1000
            rows.append(("pos error", f"{pe:.2f} mm",
                         C_GOOD if pe < 1 else (C_AMBER if pe < 10 else C_BAD)))
            oe = np.rad2deg(m.orient_error)
            rows.append(("orient error", f"{oe:.2f} deg",
                         C_GOOD if oe < 1 else (C_AMBER if oe < 5 else C_BAD)))
            clr = m.min_self_distance * 1000
            rows.append(("self-clearance", f"{clr:.1f} mm",
                         C_BAD if clr < 0 else (C_AMBER if clr < 20 else C_GOOD)))
        if r is not None:
            rows.append(("result", "CONVERGED" if r.success else "NOT CONVERGED",
                         C_GOOD if r.success else C_BAD))
            fpe = r.pos_error * 1000
            rows.append(("final pos err", f"{fpe:.2f} mm",
                         C_GOOD if fpe < 1 else (C_AMBER if fpe < 10 else C_BAD)))
            rows.append(("solve time", f"{r.wall_time_ms:.1f} ms", C_TEXT))
        return rows

    def _fill_figure(self, fig) -> bool:
        """Load the convergence series into ``fig`` up to the playback cursor."""
        if self.fig_x is None:
            return False
        n = int(np.searchsorted(self.fig_x, self.cur_idx, side="right"))
        n = max(2, min(n, len(self.fig_x)))
        for li, ys in enumerate(self.fig_y):
            fig.linepnt[li] = n
            fig.linedata[li][0:2 * n:2] = self.fig_x[:n]
            fig.linedata[li][1:2 * n:2] = ys[:n]
        fig.range[0] = [0.0, max(float(self.fig_x[-1]), 1.0)]
        lo = min(float(ys.min()) for ys in self.fig_y)
        hi = max(float(ys.max()) for ys in self.fig_y)
        fig.range[1] = [lo - 0.25, hi + 0.25]
        return True

    def _draw_hud(self, mujoco, con, vp, fig, fps: float) -> None:
        w, h = vp.width, vp.height
        s = max(0.85, h / 1000.0)
        lh = int(32 * s)   # row height — must exceed glyph height (~20px) with margin
        F = mujoco.mjtFont.mjFONT_SHADOW
        FB = mujoco.mjtFont.mjFONT_BIG

        def text(x, y, msg, rgb, font=F):
            mujoco.mjr_text(font, msg, con, x / max(w, 1), y / max(h, 1),
                            rgb[0], rgb[1], rgb[2])

        def rect(x, y, ww, hh, rgba):
            mujoco.mjr_rectangle(mujoco.MjrRect(int(x), int(y), int(ww), int(hh)),
                                 rgba[0], rgba[1], rgba[2], rgba[3])

        def panel(x, y, ww, hh, accent):
            rect(x, y, ww, hh, C_BG)
            rect(x, y + hh - 3, ww, 3, (accent[0], accent[1], accent[2], 0.9))

        # ── top status bar ──
        # bar_h is taller so FONT_BIG and FONT_SHADOW can coexist without clipping
        bar_h = int(56 * s)
        rect(0, h - bar_h, w, bar_h, C_BG)
        rect(0, h - bar_h - 3, w, 3, (*C_ACCENT, 0.9))
        # FONT_BIG baseline sits ~10 px lower than FONT_SHADOW at the same y;
        # use separate y coords so both sit visually centred in the bar.
        ty_big = h - bar_h + int(16 * s)   # IK STUDIO (big)
        ty_sm  = h - bar_h + int(20 * s)   # all other labels (shadow)
        text(int(18 * s), ty_big, "IK STUDIO", C_ACCENT, FB)
        text(int(175 * s), ty_sm, "ProteinIK x MuJoCo", C_DIM)
        text(int(w * 0.36), ty_sm, _ROBOT_LABEL[self.robot], C_TEXT)
        text(int(w * 0.36) + int(200 * s), ty_sm, f"scenario: {self.scenario}", C_DIM)
        text(w - int(460 * s), ty_sm, "SOLVER", C_DIM)
        text(w - int(375 * s), ty_sm,
             SOLVER_DISPLAY_NAMES.get(self.solver, self.solver), C_ACCENT)
        text(w - int(95 * s), ty_sm, f"{fps:4.0f} fps", C_DIM)

        # ── inner viewport (excludes status bar at top) ────────────────────
        # Both overlay panels are rendered inside this rect so they can never
        # bleed into the status bar, regardless of window scale.
        inner_vp = mujoco.MjrRect(0, 0, w, h - bar_h - int(6 * s))

        # ── telemetry panel (bottom-left, via overlay) ───────────────────
        rows = self._metric_rows()
        if rows:
            lines_l = "LIVE TELEMETRY\n" + "\n".join(label for label, _, _ in rows)
            lines_r = "\n" + "\n".join(value for _, value, _ in rows)
            mujoco.mjr_overlay(
                mujoco.mjtFont.mjFONT_SHADOW,
                mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                inner_vp,
                lines_l,
                lines_r,
                con,
            )

        # ── playback progress (bottom-center) ──
        if self.traj and len(self.traj) > 1:
            n, i = len(self.traj), self.cur_idx
            bw = int(w * 0.28)
            bx, by = (w - bw) // 2, int(26 * s)
            rect(bx, by, bw, int(6 * s), (1, 1, 1, 0.10))
            rect(bx, by, max(int(bw * i / (n - 1)), 2), int(6 * s), (*C_ACCENT, 0.95))
            text(bx, by + int(12 * s), f"playback  {i + 1}/{n}    SPACE replay", C_DIM)

        # ── convergence plot (bottom-right) ──
        if self._fill_figure(fig):
            fw, fh = int(380 * s), int(250 * s)
            mujoco.mjr_figure(
                mujoco.MjrRect(w - fw - int(14 * s), int(14 * s), fw, fh), fig, con)

        # ── compare table (top-right, via overlay) ──
        if self.compare:
            # We want this table at the top right, but below the status bar.
            # Using mjGRID_TOPRIGHT in inner_vp accomplishes exactly this.
            # We construct a single string for labels (left) and values (right).
            header_l = "SOLVER SHOWDOWN\n"
            header_r = "same target - same seed\n"
            col_l = "solver\n"
            col_r = "ok       pos mm       ms\n"
            
            lines_l = header_l + col_l
            lines_r = header_r + col_r
            
            for k, row in enumerate(self.compare):
                prefix = "> " if k == 0 else "  "
                name = (row["name"][:22] + "..") if len(row["name"]) > 24 else row["name"]
                lines_l += f"{prefix}{name}\n"
                
                ok_str = "yes" if row["success"] else "no "
                pos_str = f"{row['pos'] * 1000:8.2f}"
                ms_str = f"{row['ms']:6.1f}"
                lines_r += f"{ok_str}  {pos_str}  {ms_str}\n"
                
            mujoco.mjr_overlay(
                mujoco.mjtFont.mjFONT_SHADOW,
                mujoco.mjtGridPos.mjGRID_TOPRIGHT,
                inner_vp,
                lines_l.rstrip(),
                lines_r.rstrip(),
                con,
            )

        # ── controls panel (top-left, via overlay in inner viewport) ─────
        # mjGRID_TOPLEFT inside inner_vp starts just below the status bar.
        if self.show_help:
            lines_l = "CONTROLS\n" + "\n".join(k for k, _ in _HELP_ROWS)
            lines_r = "\n" + "\n".join(d for _, d in _HELP_ROWS)
            mujoco.mjr_overlay(
                mujoco.mjtFont.mjFONT_SHADOW,
                mujoco.mjtGridPos.mjGRID_TOPLEFT,
                inner_vp,
                lines_l,
                lines_r,
                con,
            )

        # ── busy overlay ──
        if self.busy:
            rect(0, 0, w, h, (0, 0, 0, 0.35))
            text(int(w / 2 - len(self.busy) * 6.5 * s), int(h / 2), self.busy, C_AMBER, FB)

    # ── main interactive loop ─────────────────────────────────────────────────

    def run(self, frames: int | None = None, shot: str | None = None) -> None:
        import glfw
        mujoco = self.mj

        if not glfw.init():
            raise RuntimeError("Failed to initialize GLFW (no display?).")
        glfw.window_hint(glfw.SAMPLES, 8)          # MSAA — smooth mesh edges
        if frames is not None:
            glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
        window = glfw.create_window(1600, 1000, "ProteinIK — IK Studio", None, None)
        if not window:
            glfw.terminate()
            raise RuntimeError("Failed to create GLFW window.")
        glfw.make_context_current(window)
        glfw.swap_interval(1)

        try:      # crisper HUD text on high-DPI displays
            cs = glfw.get_window_content_scale(window)[0]
        except Exception:
            cs = 1.0
        fontscale = 150 if cs < 1.25 else (200 if cs < 1.75 else 250)

        cam = mujoco.MjvCamera()
        opt = mujoco.MjvOption()
        fig = mujoco.MjvFigure()
        _setup_figure(mujoco, fig)
        scene = [None]
        context = [None]

        def rebuild_gl():
            scene[0] = mujoco.MjvScene(self.model, maxgeom=20000)
            context[0] = mujoco.MjrContext(self.model, mujoco.mjtFontScale(fontscale))
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
                self.traj_start = time.perf_counter()
                self.cur_idx = 0
                self.trail = []
            elif key == glfw.KEY_O:
                self.orbit = not self.orbit
            elif key == glfw.KEY_H:
                self.show_help = not self.show_help
            elif key in (glfw.KEY_1, glfw.KEY_2):
                self._pending_robot = _ROBOTS[0 if key == glfw.KEY_1 else 1]

        glfw.set_mouse_button_callback(window, on_mouse_button)
        glfw.set_cursor_pos_callback(window, on_cursor)
        glfw.set_scroll_callback(window, on_scroll)
        glfw.set_key_callback(window, on_key)

        print("IK Studio open. Ctrl+Click to place a target; R new target; "
              "C compare; [ ] solver; 1/2 robot; O orbit; H help; ESC quit.")

        fps = 60.0
        t_last = time.perf_counter()
        fcount = 0
        while (not glfw.window_should_close(window)
               and (frames is None or fcount < frames)):
            if self._pending_robot and self._pending_robot != self.robot:
                self._load(self._pending_robot)
                self._pending_robot = None
                self._new_target(solve=True)
                rebuild_gl()

            self.advance()
            tnow = time.perf_counter()
            fps = 0.9 * fps + 0.1 / max(tnow - t_last, 1e-4)
            t_last = tnow
            if self.orbit:
                cam.azimuth = (cam.azimuth + 0.10) % 360.0

            w, h = glfw.get_framebuffer_size(window)
            viewport = mujoco.MjrRect(0, 0, w, h)
            mujoco.mjv_updateScene(self.model, self.data, opt, None, cam,
                                   mujoco.mjtCatBit.mjCAT_ALL, scene[0])
            self._add_decor(scene[0], tnow)
            mujoco.mjr_render(viewport, scene[0], context[0])
            self._draw_hud(mujoco, context[0], viewport, fig, fps)

            if shot and frames is not None and fcount == frames - 1:
                rgb = np.zeros((h, w, 3), dtype=np.uint8)
                mujoco.mjr_readPixels(rgb, None, viewport, context[0])
                try:
                    from PIL import Image
                    Image.fromarray(np.flipud(rgb)).save(shot)
                    print(f"screenshot -> {shot}")
                except Exception as e:
                    print(f"(screenshot skipped: {e})")

            glfw.swap_buffers(window)
            glfw.poll_events()
            fcount += 1

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
    p.add_argument("--frames", type=int, default=None,
                   help="debug: run N frames in a hidden window, then exit")
    p.add_argument("--shot", default=None,
                   help="debug: with --frames, save the last frame (incl. HUD) as PNG")
    p.add_argument("--compare", action="store_true",
                   help="debug: with --frames, open the compare table for the shot")
    args = p.parse_args(argv)

    if args.selftest:
        return run_selftest(args.save_dir)

    robot = _ALIASES.get(args.robot.lower())
    if robot is None:
        p.error("unknown robot; use ur5 or franka_panda")
    print(f"Building IK Studio — {robot} / {args.solver} … (first load converts meshes)")
    studio = IKStudio(robot, args.solver, scenario=args.scenario, speed=args.speed)
    if args.frames is not None and args.compare:
        studio._compare_all()
    try:
        studio.run(frames=args.frames, shot=args.shot)
    except KeyboardInterrupt:
        print("\ninterrupted.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
