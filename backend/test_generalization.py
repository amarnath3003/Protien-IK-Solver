"""
Generalization evidence for ProteinIK.

Question: is ProteinIK overfit to the UR5 6-DOF arm, or is it a general IK solver
for arbitrary serial revolute manipulators (different DOF, different kinematics,
different scale)?

This script runs ProteinIK V4 -- with ZERO code changes -- on robots it was never
tuned for, generating reachable full-pose targets (target = FK(random q)) so every
target is achievable regardless of DOF. It compares against Jacobian DLS and the
TRAC-IK-style baseline.

Run:  ./.venv/Scripts/python test_generalization.py

Summary of the design point being demonstrated: the solver is written entirely
against `RobotSpec` / `spec.n_joints`. The only "6" in the hot loop is the
TASK-space dimension (3 position + 3 orientation) of the geometric Jacobian
(6 x n), which is correct for any DOF -- it is not the joint count.
"""
from __future__ import annotations
import numpy as np
import time

from app.core.kinematics import RobotSpec, ur5_spec, end_effector_pose
from app.solvers.protein_ik_v4 import solve_protein_ik_v4
from app.solvers.jacobian_dls import solve_dls
from app.solvers.trac_ik_style import solve_trac_ik


def make_spec(name, a, d, alpha, radii):
    n = len(a)
    lim = np.array([2 * np.pi] * n)
    return RobotSpec(
        name=name, a=np.array(a, float), d=np.array(d, float), alpha=np.array(alpha, float),
        theta_offset=np.zeros(n), joint_limits=np.stack([-lim, lim], axis=1),
        link_radius=np.array(radii, float),
    )


def evaluate(sp, n_trials=80, seed=7):
    tg = np.random.default_rng(seed)
    targets = [(tg.uniform(-np.pi, np.pi, sp.n_joints), tg.uniform(-np.pi, np.pi, sp.n_joints))
               for _ in range(n_trials)]
    rows = []
    for label, fn, needs_rng in [
        ('Jacobian DLS', solve_dls, False),
        ('TRAC-IK', solve_trac_ik, True),
        ('ProteinIK V4', solve_protein_ik_v4, True),
    ]:
        ok = 0
        t = 0.0
        for k, (qt, q0) in enumerate(targets):
            T = end_effector_pose(sp, qt)
            s = time.perf_counter()
            r = fn(sp, q0, T, np.random.default_rng(100 + k)) if needs_rng else fn(sp, q0, T)
            t += (time.perf_counter() - s) * 1000.0
            ok += r.success
        rows.append((label, ok / n_trials * 100.0, t / n_trials))
    return rows


def main():
    base = ur5_spec()

    robots = {
        'UR5 (6DOF, the tuned-on robot)': base,
        'Puma560 (6DOF, different kinematics)': make_spec(
            'puma', [0, 0.4318, 0.0203, 0, 0, 0], [0, 0, 0.15005, 0.4318, 0, 0],
            [np.pi/2, 0, -np.pi/2, np.pi/2, -np.pi/2, 0], [0.06, 0.05, 0.045, 0.04, 0.04, 0.035]),
        'KUKA iiwa (7DOF, REDUNDANT)': make_spec(
            'iiwa', [0]*7, [0.34, 0, 0.40, 0, 0.40, 0, 0.126],
            [-np.pi/2, np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, np.pi/2, 0],
            [0.07, 0.06, 0.06, 0.05, 0.05, 0.04, 0.03]),
        'Synthetic 4DOF': make_spec(
            'arm4', [0.3, 0.25, 0.2, 0.1], [0.1, 0, 0, 0.05],
            [np.pi/2, 0, 0, 0], [0.05, 0.04, 0.035, 0.03]),
    }

    print('### Generality across DOF and kinematics (reachable full-pose targets)\n')
    for rname, sp in robots.items():
        print(f'{rname}  (n_joints={sp.n_joints})')
        for label, succ, ms in evaluate(sp):
            print(f'   {label:<14} success {succ:5.1f}%   mean {ms:6.1f} ms')
        print()

    # scale robustness: same UR5, link lengths scaled 0.1x .. 10x
    print('### Scale robustness (UR5 link lengths scaled)\n')
    for f in [10.0, 1.0, 0.1]:
        sp = RobotSpec(name=f'ur5x{f}', a=base.a*f, d=base.d*f, alpha=base.alpha.copy(),
                       theta_offset=base.theta_offset.copy(), joint_limits=base.joint_limits.copy(),
                       link_radius=base.link_radius*f)
        reach = float(np.sum(np.abs(sp.a) + np.abs(sp.d)))
        print(f'UR5 x{f}  (reach ~ {reach:.2f} m)')
        for label, succ, ms in evaluate(sp):
            if label == 'Jacobian DLS':
                continue
            print(f'   {label:<14} success {succ:5.1f}%   mean {ms:6.1f} ms')
        print()


if __name__ == '__main__':
    main()
