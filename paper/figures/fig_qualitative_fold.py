"""P0 -- the qualitative "money shot": a long planar arm reaching one target, solved
two ways. TRAC-IK-style reaches the pose but folds the chain through itself (red,
self-colliding); KineticFold reaches the same pose with a clean, self-avoiding fold
(green). This is the DOF-scaling result (fig_dof_scaling) made visceral: at high DOF
the arm behaves like a self-avoiding polymer and only the folding-inspired schedule
keeps it clean.

Unlike the CSV figures, this one RUNS the solvers, so it needs the backend package
on the path (run it with the backend venv python). It searches seeds for a
representative contrastive target (KineticFold clean, TRAC-IK colliding) and prints
the seed it used, so the figure is reproducible.

Run (from anywhere, with the backend venv python):
    python fig_qualitative_fold.py [--dof 12] [--max-seeds 400]
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _style as S
sys.path.insert(0, str(S.REPO / "backend"))          # make `app` importable

import numpy as np
import matplotlib.pyplot as plt

from app.core.kinematics import RobotSpec, joint_positions
from app.api.scenarios import generate_target
from app.solvers.registry import run_solver

POS_TOL, ORIENT_TOL = 1e-3, 1e-2


def planar_ndof_spec(n: int, total_reach: float = 1.0) -> RobotSpec:
    """n-link planar arm, links summing to total_reach (matches usecase EXP E)."""
    return RobotSpec(
        name=f"planar{n}dof",
        a=np.full(n, total_reach / n), d=np.zeros(n), alpha=np.zeros(n),
        theta_offset=np.zeros(n),
        joint_limits=np.array([[-np.pi, np.pi]] * n),
        link_radius=np.full(n, 0.02),
    )


def clean(r) -> bool:
    return (bool(r.success) and r.pos_error < POS_TOL
            and r.orient_error < ORIENT_TOL and r.min_self_distance >= 0.0)


def find_contrastive(spec, max_seeds):
    """Find a target where KineticFold folds clean but TRAC-IK self-collides."""
    best = None
    for seed in range(1, max_seeds + 1):
        g = np.random.default_rng(1000 + seed)
        q0, T = generate_target(spec, g, "cluttered")
        rng_kf = np.random.default_rng(seed * 1_000_003 + 7)
        rng_tr = np.random.default_rng(seed * 1_000_003 + 7)
        kf = run_solver("protein_fast", spec, q0, T, rng_kf)
        tr = run_solver("trac_ik_style", spec, q0, T, rng_tr)
        # ideal: KF clean, TRAC reaches pose but collides (both "solve", KF is cleaner)
        if clean(kf) and tr.success and tr.pos_error < POS_TOL and tr.min_self_distance < 0:
            return seed, T, kf, tr
        if best is None and clean(kf) and not clean(tr):
            best = (seed, T, kf, tr)
    return best if best else (None, None, None, None)


def draw_arm(ax, spec, q, target_xy, ok, title):
    P = joint_positions(spec, q)          # (n+1, 3), base frame; planar -> use x,y
    col = "#009E73" if ok else "#D7263D"
    ax.plot(P[:, 0], P[:, 1], "-", color=col, lw=2.4, solid_capstyle="round", zorder=3)
    ax.plot(P[1:-1, 0], P[1:-1, 1], "o", color=col, ms=4, zorder=4)   # interior joints
    ax.plot(P[0, 0], P[0, 1], "s", color="#333", ms=7, zorder=5)      # base
    ax.plot(P[-1, 0], P[-1, 1], "o", color=col, ms=7, mfc="white", mew=1.5, zorder=5)  # EE
    ax.plot(*target_xy, marker="*", color="#e0a500", ms=15, mec="#b98400",
            mew=0.6, zorder=6)                                         # target
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=8.5)
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_edgecolor("#ccc")
    ax.grid(False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dof", type=int, default=12)
    ap.add_argument("--max-seeds", type=int, default=400)
    args = ap.parse_args()

    S.use_paper_style()
    spec = planar_ndof_spec(args.dof)
    seed, T, kf, tr = find_contrastive(spec, args.max_seeds)
    if seed is None:
        print("No contrastive target found; try a higher --dof or --max-seeds.")
        return
    print(f"  using planar {args.dof}-DOF, seed {seed}: "
          f"KineticFold clr={kf.min_self_distance:+.4f} m, "
          f"TRAC-IK clr={tr.min_self_distance:+.4f} m")

    tx, ty = float(T[0, 3]), float(T[1, 3])
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(S.WIDE, S.WIDE * 0.42))
    draw_arm(a2, spec, np.asarray(tr.q_final), (tx, ty), clean(tr),
             f"TRAC-IK-style — self-collides\nclearance {tr.min_self_distance*1000:+.1f} mm")
    draw_arm(a1, spec, np.asarray(kf.q_final), (tx, ty), clean(kf),
             f"KineticFold — clean fold\nclearance {kf.min_self_distance*1000:+.1f} mm")
    fig.suptitle(f"Same target, {args.dof}-DOF planar arm: the folding schedule keeps the chain self-avoiding",
                 fontsize=10, fontweight="bold", y=1.02)
    fig.text(0.5, -0.01, "gold star = target pose    square = base    open circle = end-effector",
             ha="center", fontsize=6.4, color="#666")
    fig.subplots_adjust(wspace=0.05, bottom=0.06)

    S.save(fig, "fig_qualitative_fold")


if __name__ == "__main__":
    main()
