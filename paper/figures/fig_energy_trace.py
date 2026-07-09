"""P2 -- the staged fold as a convergence trace: connects the folding-funnel metaphor
to the actual solver run. For one UR5 `cluttered` solve of StagedFold it plots
pose error and self-collision clearance against iteration, with the folding stages
(local-blind relax -> coarse collapse -> funnelled search -> chaperone rescue ->
stability gate) shaded as bands. You can see the target-blind first stage not
touching pose error, the collapse dropping it, the funnel narrowing, and a chaperone
rescue firing on a stall.

Runs the solver with collect_steps=True, so it needs the backend venv python.
Run:  python fig_energy_trace.py [--solver protein_ik] [--max-seeds 60]
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _style as S
sys.path.insert(0, str(S.REPO / "backend"))

import numpy as np
import matplotlib.pyplot as plt

from app.core.kinematics import get_robot_spec
from app.api.scenarios import generate_target
from app.solvers.registry import run_solver

# a soft, distinct band colour per phase (matched by substring, order-preserving)
PHASE_COLORS = ["#eef4fb", "#e9f6ef", "#fdf3e6", "#fbeaef", "#f0ecf6",
                "#eef7f6", "#f6f1e8"]


def pick_solve(solver, robot, scenario, max_seeds):
    """Prefer a solve that fired a chaperone rescue (restarts>0) so the trace shows
    the full staged story; fall back to the longest successful trace."""
    fallback = None
    for seed in range(1, max_seeds + 1):
        g = np.random.default_rng(seed)
        q0, T = generate_target(get_robot_spec(robot), g, scenario)
        r = run_solver(solver, get_robot_spec(robot), q0, T,
                       np.random.default_rng(seed * 1_000_003 + 1), collect_steps=True)
        if not r.steps:
            continue
        if r.success and r.restarts and r.restarts > 0:
            return seed, r
        if r.success and (fallback is None or len(r.steps) > len(fallback[1].steps)):
            fallback = (seed, r)
    return fallback if fallback else (None, None)


def phase_spans(phases):
    """Contiguous runs of identical phase labels -> [(phase, i0, i1), ...]."""
    spans, start = [], 0
    for i in range(1, len(phases) + 1):
        if i == len(phases) or phases[i] != phases[start]:
            spans.append((phases[start], start, i - 1))
            start = i
    return spans


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--solver", default="protein_ik")
    ap.add_argument("--robot", default="ur5")
    ap.add_argument("--scenario", default="cluttered")
    ap.add_argument("--max-seeds", type=int, default=60)
    args = ap.parse_args()

    S.use_paper_style()
    seed, r = pick_solve(args.solver, args.robot, args.scenario, args.max_seeds)
    if r is None:
        print("No successful traced solve found; raise --max-seeds.")
        return
    print(f"  {S.label(args.solver)} on {args.robot}/{args.scenario}, seed {seed}: "
          f"{len(r.steps)} steps, restarts={r.restarts}")

    it = np.array([s.iteration for s in r.steps], dtype=float)
    pos_mm = np.array([s.pos_error * 1000.0 for s in r.steps])
    clr = np.array([s.min_self_distance for s in r.steps])
    phases = [s.phase or "—" for s in r.steps]

    spans = phase_spans(phases)
    uniq = list(dict.fromkeys(p for p, _, _ in spans))
    pcol = {p: PHASE_COLORS[i % len(PHASE_COLORS)] for i, p in enumerate(uniq)}

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(S.COL * 1.9, S.COL * 1.15),
                                   sharex=True, height_ratios=[2, 1])
    for ax in (ax1, ax2):
        for p, i0, i1 in spans:
            ax.axvspan(it[i0], it[i1] + 1, color=pcol[p], zorder=0)

    ax1.plot(it, pos_mm, color=S.color(args.solver), lw=1.5, zorder=3)
    ax1.axhline(1.0, ls=":", lw=0.8, color="#888")   # 1 mm success threshold
    ax1.text(it[-1], 1.05, "1 mm tol", fontsize=6, color="#888", ha="right", va="bottom")
    ax1.set_yscale("log")
    ax1.set_ylabel("Pose error (mm, log)")
    ax1.set_title(f"The staged fold in one solve — {S.label(args.solver)}")

    ax2.plot(it, clr, color="#1f6f6b", lw=1.5, zorder=3)
    ax2.axhline(0.0, ls="-", lw=0.8, color="#c44")   # collision boundary
    ax2.set_ylabel("Clearance (m)")
    ax2.set_xlabel("Iteration")

    # phase legend (band colours)
    from matplotlib.patches import Patch
    handles = [Patch(facecolor=pcol[p], edgecolor="#ccc", label=p) for p in uniq]
    ax1.legend(handles=handles, ncol=min(3, len(uniq)), loc="upper right",
               fontsize=6.2, title="folding stage")
    fig.subplots_adjust(hspace=0.10)

    S.save(fig, "fig_energy_trace")


if __name__ == "__main__":
    main()
