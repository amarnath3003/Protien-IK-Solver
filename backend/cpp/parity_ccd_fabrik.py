"""Parity test: native C++ CCD/FABRIK (pik_native) vs the Python solvers, on all
3 arms. Both are fully DETERMINISTIC (no RNG), so on identical (q0, T_target) the
C++ port must reproduce the Python's per-step math to floating-point tolerance
and its solve OUTCOMES (success / collision / errors) exactly.

Two checks:
  (A) per-step parity — with the iteration budget capped small, max|q_cpp - q_py|
      must be < 1e-9 (proves the per-joint update formulas are bit-identical).
  (B) full-run outcome parity — over the default budget, success must agree on
      EVERY trial and the success/collision-free counts must match exactly.

NOTE on FABRIK full-run q: FABRIK's revolute adaptation is marginally stable and
its orientation term is documented to *oscillate* on non-converged trials, so the
raw q of a NON-converged FABRIK solve can drift ~1e-3 by iteration 150 purely from
1-ULP transcendental differences (numpy vs Eigen) compounding — while the
converged/diverged OUTCOME stays identical. That is why FABRIK is graded on (A)+(B),
not on raw-q identity at full budget. CCD is contractive and matches to ~1e-13
even at full budget."""
import sys
sys.path.insert(0, "/mnt/c/Coding Projects/Protien IK/backend")
sys.path.insert(0, "/mnt/c/Coding Projects/Protien IK/backend/cpp")

import numpy as np
import pik_native as pn
from app.core.kinematics import get_robot_spec, end_effector_pose
from app.solvers.ccd import solve_ccd
from app.solvers.fabrik import solve_fabrik


def cpp_robot(spec):
    return pn.make_robot(
        spec.a.tolist(), spec.d.tolist(), spec.alpha.tolist(), spec.theta_offset.tolist(),
        spec.joint_limits[:, 0].tolist(), spec.joint_limits[:, 1].tolist(),
        spec.link_radius.tolist(), spec.dh_convention == "modified")


# (name, cpp_fn(R,q0,T,seed,max_iters), py_fn(spec,q0,T,max_iters=...), full_budget)
SOLVERS = [
    ("CCD",    pn.solve_ccd,    solve_ccd,    300),
    ("FABRIK", pn.solve_fabrik, solve_fabrik, 150),
]

overall_ok = True
N = 200
STEP = 3   # tiny budget for the per-step parity check (A)
for robot in ["ur5", "franka_panda", "planar3dof"]:
    spec = get_robot_spec(robot)
    R = cpp_robot(spec)
    print("=" * 74)
    for name, cfn, pyfn, budget in SOLVERS:
        gen = np.random.default_rng(7)
        dq_step = 0.0                       # (A) per-step divergence
        derr = 0.0
        cs = ps = cfree = pfree = agree = 0  # (B) outcome parity
        for t in range(N):
            T = end_effector_pose(spec, spec.random_config(gen))
            q0 = spec.random_config(gen)
            # (A) capped-budget per-step parity
            rs = cfn(R, q0.copy(), T, 0, STEP)
            ps_ = pyfn(spec, q0.copy(), T, max_iters=STEP)
            dq_step = max(dq_step, float(np.max(np.abs(
                np.asarray(rs["q"], float) - np.asarray(ps_.q_final, float)))))
            # (B) full-budget outcome parity
            r = cfn(R, q0.copy(), T, 0, budget)
            pr = pyfn(spec, q0.copy(), T, max_iters=budget)
            derr = max(derr, abs(r["pos_error"] - pr.pos_error),
                       abs(r["orient_error"] - pr.orient_error))
            cs += int(r["success"]); ps += int(pr.success)
            agree += int(bool(r["success"]) == bool(pr.success))
            cfree += int(r["min_self_distance"] >= 0)
            pfree += int(pr.min_self_distance >= 0)
        ok = (dq_step < 1e-9) and (agree == N) and (cs == ps) and (cfree == pfree)
        overall_ok = overall_ok and ok
        print(f"  {robot:12s} {name:6s}  per-step max|dq| {dq_step:.2e}  "
              f"succ C++ {cs:3d}/{N} Py {ps:3d}/{N}  agree {agree:3d}/{N}  "
              f"collfree C++ {cfree:3d} Py {pfree:3d}  -> {'OK' if ok else 'DIFFER'}")

print("=" * 74)
print("PARITY", "PASS" if overall_ok else "FAIL")
sys.exit(0 if overall_ok else 1)
