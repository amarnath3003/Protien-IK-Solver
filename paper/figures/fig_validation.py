"""F6 -- "solve once, score three ways": the capsule proxy is systematically
optimistic, and the two independent real-mesh oracles agree.

Per (arm, scenario) it plots three collision rates side by side -- the capsule
proxy the solvers optimise against, PyBullet, and MuJoCo -- all scoring the SAME
final configurations. The proxy sits well below both engines (real meshes collide
more), while PyBullet and MuJoCo track each other. This is why the paper reports
collision only as a solver *ranking*, never as an absolute rate, and why it shrank
its own proxy-based magnitude claim.

Averaged over all solvers in each cell (so it measures the proxy, not one solver).
Source: master CSV (needs our_collision_pct + pb_collision_pct + mj_collision_pct).
Run:    python fig_validation.py [--csv path/to/master_full.csv]
"""
from __future__ import annotations

import argparse
import os
import sys
from statistics import fmean

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _style as S

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch

ENGINES = [("our_collision_pct", "Capsule proxy", "#9A9A9A"),
           ("pb_collision_pct",  "PyBullet",      "#0072B2"),
           ("mj_collision_pct",  "MuJoCo",        "#D55E00")]


def _mean(rows, robot, scen, field):
    vals = [r[field] for r in rows
            if r["robot"] == robot and r["scenario"] == scen
            and field in r and r[field] == r[field]]        # drop NaN
    return fmean(vals) if vals else float("nan")


def panel(ax, rows, robot):
    scen = [s for s in S.SCENARIOS if any(r["robot"] == robot and r["scenario"] == s
                                          for r in rows)]
    x = np.arange(len(scen))
    w = 0.8 / len(ENGINES)
    for i, (field, _lab, col) in enumerate(ENGINES):
        vals = [_mean(rows, robot, sc, field) for sc in scen]
        ax.bar(x + (i - (len(ENGINES) - 1) / 2) * w, vals, w,
               color=col, edgecolor="white", linewidth=0.4, zorder=2)
    ax.set_xticks(x)
    ax.set_xticklabels([S.SCEN_LABEL[s] for s in scen])
    ax.set_title({"ur5": "UR5", "franka_panda": "Franka Panda"}.get(robot, robot))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=str(S.DEFAULT_MASTER_CSV))
    args = ap.parse_args()

    S.use_paper_style()
    rows = S.load_rows(args.csv)
    arms = [a for a in ("ur5", "franka_panda")
            if any(r["robot"] == a and r.get("pb_collision_pct") == r.get("pb_collision_pct")
                   for r in rows)]

    fig, axes = plt.subplots(1, len(arms), figsize=(S.WIDE, S.WIDE * 0.30),
                             sharey=True)
    if len(arms) == 1:
        axes = [axes]
    for ax, arm in zip(axes, arms):
        panel(ax, rows, arm)
    axes[0].set_ylabel("Self-collision rate (%)\nmean over solvers")

    handles = [Patch(facecolor=c, label=l) for _f, l, c in ENGINES]
    fig.legend(handles=handles, ncol=3, loc="lower center", bbox_to_anchor=(0.5, -0.02))
    fig.suptitle("The capsule proxy is optimistic; two real-mesh oracles agree",
                 fontsize=10, fontweight="bold", y=1.03)

    # PyBullet<->MuJoCo collide/clear agreement, as a footnote (range over cells).
    agree = [r["pb_mj_collision_agree_pct"] for r in rows
             if r.get("pb_mj_collision_agree_pct") == r.get("pb_mj_collision_agree_pct")
             and r["robot"] in arms]
    if agree:
        fig.text(0.5, -0.12,
                 f"PyBullet and MuJoCo agree on the collide/clear call "
                 f"{min(agree):.0f}–{max(agree):.0f}% of the time.",
                 ha="center", fontsize=6.4, color="#666")
    fig.subplots_adjust(bottom=0.26, wspace=0.06)

    S.save(fig, "fig_validation")


if __name__ == "__main__":
    main()
