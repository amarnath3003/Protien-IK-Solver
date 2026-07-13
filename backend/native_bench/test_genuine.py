"""FK-parity + solve smoke test for the genuine imported baselines.

Confirms each library's chain (built from the repo DH table) reproduces the
repo's own forward kinematics to ~machine epsilon (so it solves the identical
robot), then runs each genuine solver on reachable targets and reports success.
"""
from native_bench._env import apply
apply()

import numpy as np
from app.core.kinematics import get_robot_spec, end_effector_pose, pose_error
import native_bench.genuine_solvers as G


def kdl_parity(spec, n=500):
    ctx = G._get_kdl(spec)
    kdl, chain = ctx["kdl"], ctx["chain"]
    fk = kdl.ChainFkSolverPos_recursive(chain)
    rng = np.random.default_rng(0)
    dp = do = 0.0
    for _ in range(n):
        q = spec.random_config(rng)
        ja = kdl.JntArray(spec.n_joints)
        for i in range(spec.n_joints):
            ja[i] = float(q[i])
        f = kdl.Frame()
        fk.JntToCart(ja, f)
        T = end_effector_pose(spec, q)
        dp = max(dp, max(abs(f.p[k] - T[k, 3]) for k in range(3)))
        R = np.array([[f.M[r, c] for c in range(3)] for r in range(3)])
        do = max(do, float(np.max(np.abs(R - T[:3, :3]))))
    return dp, do


def rtb_parity(spec, n=500):
    _, robot = G._get_rtb(spec)
    rng = np.random.default_rng(1)
    dp = do = 0.0
    for _ in range(n):
        q = spec.random_config(rng)
        T = np.array(robot.fkine(q).A)
        Td = end_effector_pose(spec, q)
        dp = max(dp, float(np.max(np.abs(T[:3, 3] - Td[:3, 3]))))
        do = max(do, float(np.max(np.abs(T[:3, :3] - Td[:3, :3]))))
    return dp, do


SOLVERS = {
    "RTB-DLS  (jacobian_dls)": G.solve_rtb_dls,
    "RTB-multi(multi_start) ": G.solve_rtb_multistart,
    "TRAC-IK  (trac_ik_style)": G.solve_real_tracik,
}

for robot in ["ur5", "franka_panda", "planar3dof"]:
    spec = get_robot_spec(robot)
    print("=" * 68)
    print(robot)
    kp, ko = kdl_parity(spec)
    rp, ro = rtb_parity(spec)
    print(f"  KDL FK parity: pos {kp:.2e} m  orient {ko:.2e}   -> {'OK' if kp<1e-9 else 'DIFFER'}")
    print(f"  RTB FK parity: pos {rp:.2e} m  orient {ro:.2e}   -> {'OK' if rp<1e-9 else 'DIFFER'}")
    rng = np.random.default_rng(5)
    for nm, fn in SOLVERS.items():
        ok = 0
        tms = []
        pe = []
        for _ in range(20):
            qref = spec.random_config(rng)
            T = end_effector_pose(spec, qref)
            q0 = spec.random_config(rng)
            try:
                r = fn(spec, q0, T, rng)
                ok += int(r.success)
                tms.append(r.wall_time_ms)
                pe.append(r.pos_error * 1000.0)
            except Exception as e:
                import traceback
                traceback.print_exc()
                break
        if tms:
            print(f"    {nm}  succ {ok:2d}/20  mean {np.mean(tms):7.2f}ms  "
                  f"med_pos {np.median(pe):.3f}mm")
print("DONE")
