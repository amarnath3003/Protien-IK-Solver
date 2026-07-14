"""F3 -- success rate across the whole solver field, faceted by arm.

Shows the two-tier structure the paper argues for: the simple / single-trajectory
baselines (CCD, FABRIK, Jacobian-DLS) collapse on the harder scenarios, while the
two production baselines (Multi-start, TRAC-IK-style) and KineticFold hold near
100%. Bars are coloured by scenario on a difficulty ramp (light=open -> dark=cluttered).

Source: master CSV (default: 3-seed all-arms sim run).
Run:    python fig_success.py [--csv path/to/master_full.csv]
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


def panel(ax, rows, robot):
    solvers = S.present_solvers(rows, robot)
    scen = [s for s in S.SCENARIOS if any(r["robot"] == robot and r["scenario"] == s
                                          for r in rows)]
    x = np.arange(len(solvers))
    w = 0.8 / max(1, len(scen))

    # subtle highlight behind the hero column
    if S.HERO in solvers:
        hx = solvers.index(S.HERO)
        ax.axvspan(hx - 0.5, hx + 0.5, color=S.color(S.HERO), alpha=0.07, zorder=0)

    for j, sc in enumerate(scen):
        c = S.cell(rows, robot, sc)
        vals = [c.get(s, {}).get("success_pct", float("nan")) for s in solvers]
        ax.bar(x + (j - (len(scen) - 1) / 2) * w, vals, w,
               color=S.SCEN_COLOR[sc], edgecolor="white", linewidth=0.4,
               label=S.SCEN_LABEL[sc], zorder=2)

    ax.axhline(99, ls=":", lw=0.8, color="#888", zorder=1)
    ax.set_xticks(x)
    ax.set_xticklabels([S.label(s) for s in solvers], rotation=32, ha="right")
    ax.set_ylim(0, 108)
    ax.set_title({"ur5": "UR5 (6-DOF, non-redundant)",
                  "franka_panda": "Franka Panda (7-DOF, redundant)"}.get(robot, robot))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=str(S.DEFAULT_MASTER_CSV))
    args = ap.parse_args()

    S.use_paper_style()
    rows = S.load_rows(args.csv)
    arms = [a for a in ("ur5", "franka_panda") if any(r["robot"] == a for r in rows)]

    fig, axes = plt.subplots(1, len(arms), figsize=(S.WIDE, S.WIDE * 0.34),
                             sharey=True)
    if len(arms) == 1:
        axes = [axes]
    for ax, arm in zip(axes, arms):
        panel(ax, rows, arm)
    axes[0].set_ylabel("Success rate (%)")

    handles = [Patch(facecolor=S.SCEN_COLOR[s], label=S.SCEN_LABEL[s])
               for s in S.SCENARIOS]
    fig.legend(handles=handles, ncol=3, loc="lower center",
               bbox_to_anchor=(0.5, -0.02), title="scenario (harder →)")
    fig.suptitle("KineticFold leads or ties the field on success across both arms",
                 fontsize=10, fontweight="bold", y=1.02)
    fig.subplots_adjust(bottom=0.30, wspace=0.06)

    S.save(fig, "fig_success")


if __name__ == "__main__":
    main()
