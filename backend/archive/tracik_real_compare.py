"""
REAL TRAC-IK vs our `trac_ik_style` — runs INSIDE WSL (Ubuntu), where the
genuine TRAC-IK C++/KDL library (`tracikpy`) is installed.

Fairness trick: we generate a URDF *directly from our own UR5 DH table*
(app.core.kinematics.ur5_spec), so the real TRAC-IK solver and our solver
operate on IDENTICAL kinematics. The script asserts FK parity (tracikpy.fk
vs our end_effector_pose) before benchmarking, then runs both solvers on the
SAME set of reachable targets and prints a comparison table.

Run (from inside WSL):
    wsl -d Ubuntu-2204 -u root -- python3 /mnt/c/Coding\\ Projects/Protien\\ IK/backend/scrap/tracik_real_compare.py

Requires (in WSL): numpy, tracikpy.
"""
from __future__ import annotations

import os
import sys
import time
import argparse

import numpy as np

# Import ONLY our trac_ik_style solver (+ its kinematics dep). We avoid the full
# registry so we don't drag in scipy/other Windows-only deps into WSL python.
_BACKEND = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _BACKEND not in sys.path:
    sys.path.insert(0, _BACKEND)

from app.core.kinematics import (
    ur5_spec, end_effector_pose, pose_error, self_collision_min_distance,
)
from app.solvers.trac_ik_style import solve_trac_ik
from app.solvers.protein_fast import solve_protein_fast   # ProteinIK V4

POS_TOL = 1e-3      # 1 mm
ORIENT_TOL = 1e-2   # 10 mrad


def urdf_from_ur5_dh() -> str:
    """Emit a URDF whose kinematics exactly reproduce ur5_spec()'s standard-DH FK.

    Standard DH link transform:  T_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i).
    URDF applies each joint origin BEFORE its axis rotation, so we place the
    fixed part F_i = Trans(a_i,0,d_i) . Rx(alpha_i) as the origin of joint_{i+1}
    (and a final fixed tool joint carries F_n). Result:
        T_ee = Rz(q1) F1 Rz(q2) F2 ... Rz(qn) Fn  ==  our end_effector_pose.
    """
    s = ur5_spec()
    a, d, alpha = s.a, s.d, s.alpha
    n = s.n_joints
    lo, hi = s.joint_limits[:, 0], s.joint_limits[:, 1]

    def link(name):
        # minimal inertial so kdl_parser builds a valid chain
        return (f'  <link name="{name}">\n'
                f'    <inertial><mass value="1.0"/>'
                f'<inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>'
                f'<origin xyz="0 0 0" rpy="0 0 0"/></inertial>\n'
                f'  </link>\n')

    out = ['<?xml version="1.0"?>', '<robot name="ur5_from_dh">']
    # root link must NOT carry inertia (KDL doesn't support a root inertia).
    out.append('  <link name="base_link"/>')
    for i in range(1, n + 1):
        out.append(link(f"link_{i}").rstrip())
    out.append(link("tool0").rstrip())

    for i in range(1, n + 1):
        parent = "base_link" if i == 1 else f"link_{i-1}"
        if i == 1:
            ox = oy = oz = 0.0
            oroll = 0.0
        else:
            ox, oy, oz = float(a[i - 2]), 0.0, float(d[i - 2])
            oroll = float(alpha[i - 2])
        out.append(
            f'  <joint name="joint_{i}" type="revolute">\n'
            f'    <parent link="{parent}"/><child link="link_{i}"/>\n'
            f'    <origin xyz="{ox} {oy} {oz}" rpy="{oroll} 0 0"/>\n'
            f'    <axis xyz="0 0 1"/>\n'
            f'    <limit lower="{float(lo[i-1])}" upper="{float(hi[i-1])}" '
            f'effort="100" velocity="3.14"/>\n'
            f'  </joint>')
    # final fixed tool joint carries F_n
    out.append(
        f'  <joint name="joint_tool" type="fixed">\n'
        f'    <parent link="link_{n}"/><child link="tool0"/>\n'
        f'    <origin xyz="{float(a[n-1])} 0 {float(d[n-1])}" '
        f'rpy="{float(alpha[n-1])} 0 0"/>\n'
        f'  </joint>')
    out.append('</robot>\n')
    return "\n".join(out)


def pct(a, q):
    return float(np.percentile(a, q))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=100)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--timeout", type=float, default=0.005,
                    help="real TRAC-IK per-solve wall budget (s); TRAC-IK default 5ms")
    args = ap.parse_args()

    try:
        from tracikpy import TracIKSolver
    except Exception as e:  # noqa: BLE001
        print(f"FATAL: tracikpy not importable: {e}")
        return 2

    spec = ur5_spec()
    urdf_path = "/tmp/ur5_from_dh.urdf"
    with open(urdf_path, "w") as f:
        f.write(urdf_from_ur5_dh())

    ik = TracIKSolver(urdf_path, "base_link", "tool0", timeout=args.timeout,
                      epsilon=1e-5, solve_type="Speed")
    print(f"Real TRAC-IK loaded: {ik.number_of_joints} joints, "
          f"base->tip base_link->tool0, timeout={args.timeout*1000:.1f}ms")

    # --- FK parity check: URDF kinematics must match our DH FK ---
    rng = np.random.default_rng(args.seed)
    max_fk_err = 0.0
    for _ in range(200):
        q = spec.random_config(rng)
        T_ours = end_effector_pose(spec, q)
        T_trac = ik.fk(q)
        max_fk_err = max(max_fk_err, float(np.max(np.abs(T_ours - T_trac))))
    print(f"FK parity (ours vs TRAC-IK URDF): max abs elem diff = {max_fk_err:.2e}", end="")
    if max_fk_err < 1e-6:
        print("  -> IDENTICAL kinematics (apples-to-apples).")
    else:
        print("  -> WARNING: kinematics differ; comparison not exact.")

    # --- benchmark on identical reachable targets ---
    rng = np.random.default_rng(args.seed + 1)
    real = dict(ok=[], pos=[], orient=[], ms=[])
    ours = dict(ok=[], pos=[], orient=[], ms=[], restarts=0)
    v4 = dict(ok=[], pos=[], orient=[], ms=[], restarts=0, collfree=[])

    for t in range(args.trials):
        q_ref = spec.random_config(rng)
        T = end_effector_pose(spec, q_ref)      # == ik.fk(q_ref), verified above
        qinit = spec.random_config(rng)

        # real TRAC-IK
        t0 = time.perf_counter()
        q_sol = ik.ik(T, qinit=qinit)
        real["ms"].append((time.perf_counter() - t0) * 1000.0)
        if q_sol is None:
            real["ok"].append(False); real["pos"].append(np.nan); real["orient"].append(np.nan)
        else:
            e = pose_error(ik.fk(q_sol), T)
            pe, oe = float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:]))
            real["ok"].append(pe < POS_TOL and oe < ORIENT_TOL)
            real["pos"].append(pe); real["orient"].append(oe)

        # our trac_ik_style (same target, own rng)
        r = solve_trac_ik(spec, qinit.copy(), T, np.random.default_rng(2000 + t))
        ours["ms"].append(r.wall_time_ms)
        ours["ok"].append(bool(r.success) and r.pos_error < POS_TOL and r.orient_error < ORIENT_TOL)
        ours["pos"].append(r.pos_error); ours["orient"].append(r.orient_error)
        ours["restarts"] += r.restarts

        # ProteinIK V4 (same target, own rng)
        r4 = solve_protein_fast(spec, qinit.copy(), T, np.random.default_rng(3000 + t))
        v4["ms"].append(r4.wall_time_ms)
        v4_ok = bool(r4.success) and r4.pos_error < POS_TOL and r4.orient_error < ORIENT_TOL
        v4["ok"].append(v4_ok)
        v4["pos"].append(r4.pos_error); v4["orient"].append(r4.orient_error)
        v4["restarts"] += r4.restarts
        if v4_ok:
            v4["collfree"].append(r4.min_self_distance >= 0.0)

    N = args.trials
    print(f"\n=== REAL TRAC-IK vs trac_ik_style — UR5 (identical DH), "
          f"{N} reachable targets, seed={args.seed} ===\n")
    hdr = (f"{'Solver':<24} {'Success%':>9} {'Mean pos(mm)':>13} "
           f"{'Mean orient(mrad)':>18} {'Mean ms':>9} {'p50 ms':>8} {'p95 ms':>8}")
    print(hdr); print("-" * len(hdr))
    for label, dd in [("REAL TRAC-IK", real),
                      ("trac_ik_style (ours)", ours),
                      ("ProteinIK V4", v4)]:
        a = np.array(dd["ms"])
        print(f"{label:<24} "
              f"{100.0*np.mean(dd['ok']):>8.1f}% "
              f"{1000.0*np.nanmean(dd['pos']):>13.3f} "
              f"{1000.0*np.nanmean(dd['orient']):>18.3f} "
              f"{a.mean():>9.2f} {pct(a,50):>8.2f} {pct(a,95):>8.2f}")
    v4_cf = (100.0 * np.mean(v4["collfree"])) if v4["collfree"] else float("nan")
    v4_a = np.array(v4["ms"])
    print(f"\n(ours: {ours['restarts']} restarts, {ours['restarts']/N:.2f}/trial | "
          f"V4: {v4['restarts']} restarts, {v4['restarts']/N:.2f}/trial, "
          f"{v4_cf:.0f}% of V4 solves self-collision-free | "
          f"real TRAC-IK budget {args.timeout*1000:.1f}ms/solve)")
    print(f"V4 vs real TRAC-IK: mean latency {v4_a.mean()/max(np.array(real['ms']).mean(),1e-9):.1f}x, "
          f"success {100.0*np.mean(v4['ok']):.1f}% vs {100.0*np.mean(real['ok']):.1f}%.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
