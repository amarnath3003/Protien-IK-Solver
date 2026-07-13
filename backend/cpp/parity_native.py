"""Parity test: native C++ (pik_native) vs Python, on all 3 arms.
(1) primitives FK/Jacobian/self-collision/energy/frustration must match to ~1e-12.
(2) V4 statistical parity: success%, collision-free%, mean iters close to Python."""
import sys
sys.path.insert(0, "/mnt/c/Coding Projects/Protien IK/backend")
sys.path.insert(0, "/mnt/c/Coding Projects/Protien IK/backend/cpp")

import numpy as np
import pik_native as pn
from app.core.kinematics import (
    get_robot_spec, end_effector_pose, geometric_jacobian, self_collision_min_distance,
)
from app.solvers.protein_energy import total_energy_fast, frustration_index
from app.solvers.protein_fast.solver import solve_protein_fast


def cpp_robot(spec):
    return pn.make_robot(
        spec.a.tolist(), spec.d.tolist(), spec.alpha.tolist(), spec.theta_offset.tolist(),
        spec.joint_limits[:, 0].tolist(), spec.joint_limits[:, 1].tolist(),
        spec.link_radius.tolist(), spec.dh_convention == "modified")


for robot in ["ur5", "franka_panda", "planar3dof"]:
    spec = get_robot_spec(robot)
    R = cpp_robot(spec)
    rng = np.random.default_rng(0)
    dfk = djac = dcol = den = dfr = 0.0
    for _ in range(300):
        q = spec.random_config(rng)
        dfk = max(dfk, np.max(np.abs(np.array(pn.fk(R, q)) - end_effector_pose(spec, q))))
        djac = max(djac, np.max(np.abs(np.array(pn.jacobian(R, q)) - geometric_jacobian(spec, q))))
        dcol = max(dcol, abs(pn.self_collision(R, q) - self_collision_min_distance(spec, q)))
        qt = spec.random_config(rng); T = end_effector_pose(spec, qt)
        den = max(den, abs(pn.total_energy(R, q, T, 3.0, 1.0, 2.0, 0.3)
                           - total_energy_fast(spec, q, T, 3.0, 1.0, 2.0, 0.3)))
        dfr = max(dfr, np.max(np.abs(np.array(pn.frustration(R, q, T)) - frustration_index(spec, q, T))))
    ok = "OK" if max(dfk, djac, dcol, den, dfr) < 1e-9 else "DIFFER"
    print("=" * 60)
    print(f"{robot}: FK {dfk:.1e}  Jac {djac:.1e}  coll {dcol:.1e}  energy {den:.1e}  frust {dfr:.1e}  -> {ok}")

    # ---- per-solver statistical parity (same N targets for C++ and Python) ----
    import importlib
    N = 80
    SOLVERS = [
        ("V4",       lambda R, q0, T, sd: pn.solve_v4(R, q0, T, sd, False),
         "app.solvers.protein_fast.solver", "solve_protein_fast"),
        ("V1",       lambda R, q0, T, sd: pn.solve_v1(R, q0, T, sd) if hasattr(pn, "solve_v1") else None,
         "app.solvers.protein_ik", "solve_protein_ik"),
        ("Homotopy", lambda R, q0, T, sd: pn.solve_homotopy(R, q0, T, sd) if hasattr(pn, "solve_homotopy") else None,
         "app.solvers.protein_homotopy", "solve_protein_homotopy"),
        ("FixedL",   lambda R, q0, T, sd: pn.solve_fixed_lambda(R, q0, T, sd) if hasattr(pn, "solve_fixed_lambda") else None,
         "app.solvers.fixed_lambda_ik", "solve_fixed_lambda_ik"),
        ("V6",       lambda R, q0, T, sd: pn.solve_raw(R, q0, T, sd) if hasattr(pn, "solve_raw") else None,
         "app.solvers.protein_raw", "solve_protein_raw"),
    ]
    for name, cfn, pymod, pyname in SOLVERS:
        try:
            probe = cfn(R, spec.random_config(np.random.default_rng(0)),
                        end_effector_pose(spec, spec.random_config(np.random.default_rng(1))), 0)
        except Exception:
            probe = None
        if probe is None:
            continue
        pyfn = getattr(importlib.import_module(pymod), pyname)
        gen = np.random.default_rng(7)
        cs = cf = ps = pf = 0
        ci, pi = [], []
        for t in range(N):
            T = end_effector_pose(spec, spec.random_config(gen))
            q0 = spec.random_config(gen)
            r = cfn(R, q0, T, 3000 + t)
            cs += int(r["success"]); ci.append(r["iterations"])
            if r["success"]:
                cf += int(r["min_self_distance"] >= 0)
            pr = pyfn(spec, q0, T, np.random.default_rng(3000 + t))
            ps += int(pr.success); pi.append(pr.iterations)
            if pr.success:
                pf += int(pr.min_self_distance >= 0)
        print(f"   {name:9s} C++: succ {cs:3d}/{N} collfree {cf:3d}/{max(cs,1):3d} iters~{np.mean(ci):5.0f}"
              f"  |  Py: succ {ps:3d}/{N} collfree {pf:3d}/{max(ps,1):3d} iters~{np.mean(pi):5.0f}")
print("DONE")
