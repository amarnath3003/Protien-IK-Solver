"""F5 -- real-mesh self-collision across both physical arms, high-success solvers.

Two panels (UR5, Franka), each a grouped bar chart of PyBullet self-collision rate
by scenario for the four >=90%-success solvers. On the non-redundant UR5 KineticFold
has the LOWEST collision in every regime; on the redundant Franka every solver is
statistically tied (a spare joint lets any of them dodge). Only the high-success
field is shown -- the simple baselines fail too often for a collision rate to be
meaningful. MuJoCo agrees to within ~1 point (CSV `mj_collision_pct`).

Source: 10-seed run (master_10seed_fast.csv; needs pb_collision_pct).
Run:    python fig_collision.py [--csv path/to/master_10seed_fast.csv]
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _style as S

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch

SOLVERS = ["protein_ik", "multi_start", "trac_ik_style", "protein_fast"]


def panel(ax, rows, robot, solvers, scen):
    x = np.arange(len(scen))
    w = 0.8 / len(solvers)
    for i, sid in enumerate(solvers):
        vals = [S.cell(rows, robot, sc).get(sid, {}).get("pb_collision_pct", float("nan"))
                for sc in scen]
        bars = ax.bar(x + (i - (len(solvers) - 1) / 2) * w, vals, w,
                      color=S.color(sid), edgecolor="white", linewidth=0.4, zorder=2)
        ax.bar_label(bars, fmt="%.0f", padding=1.5, fontsize=5.6)
    ax.set_xticks(x)
    ax.set_xticklabels([S.SCEN_LABEL[s] for s in scen])
    ax.set_title({"ur5": "UR5 (6-DOF, non-redundant)",
                  "franka_panda": "Franka Panda (7-DOF, redundant)"}[robot])
    ax.grid(axis="x", visible=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=str(S.DEFAULT_COLLISION_CSV))
    args = ap.parse_args()

    S.use_paper_style()
    rows = S.load_rows(args.csv)
    arms = [a for a in ("ur5", "franka_panda") if S.cell(rows, a, "open_space")]
    scen = [s for s in S.SCENARIOS if S.cell(rows, arms[0], s)]

    fig, axes = plt.subplots(1, len(arms), figsize=(S.WIDE, S.WIDE * 0.40), sharey=True)
    if len(arms) == 1:
        axes = [axes]
    for ax, arm in zip(axes, arms):
        solvers = [s for s in SOLVERS if any(r["robot"] == arm and r["solver"] == s for r in rows)]
        panel(ax, rows, arm, solvers, scen)
    axes[0].set_ylabel("Real-mesh self-collision, PyBullet (%)")
    for ax in axes:
        ax.set_ylim(0, 92)

    handles = [Patch(facecolor=S.color(s), label=S.label(s)) for s in SOLVERS]
    fig.legend(handles=handles, ncol=4, loc="lower center", bbox_to_anchor=(0.5, -0.02))
    fig.suptitle("Self-collision: KineticFold cleanest on the non-redundant UR5, a tie on the redundant Franka",
                 fontsize=9.5, fontweight="bold", y=1.02)
    fig.subplots_adjust(bottom=0.20, wspace=0.06)

    S.save(fig, "fig_collision")


if __name__ == "__main__":
    main()
