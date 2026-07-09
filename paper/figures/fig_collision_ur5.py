"""F5 -- UR5 real-mesh self-collision vs. success, high/mid-success solvers.

Two stacked panels sharing the x-axis: (top) PyBullet self-collision rate; (bottom)
success rate. On the 10-seed run KineticFold has the LOWEST self-collision in every
regime -- open, near-singular, and cluttered -- while being the only solver at ~100%
success. StagedFold is shown to make the honesty explicit: it is dominated on BOTH
axes (higher collision AND lower success), so KineticFold's cleanliness is not bought
by dropping hard targets from its denominator. MuJoCo agrees (CSV `mj_collision_pct`).

Source: collision CSV (default: 10-seed UR5 run, needs pb_collision_pct + success_pct).
Run:    python fig_collision_ur5.py [--csv path/to/master_10seed_fast.csv]
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

# ordered so the high-success trio is grouped; StagedFold shown but de-emphasised
SOLVERS = ["protein_ik", "multi_start", "trac_ik_style", "protein_fast"]


def grouped(ax, rows, field, solvers, scen):
    x = np.arange(len(scen))
    w = 0.8 / len(solvers)
    for i, sid in enumerate(solvers):
        vals = [S.cell(rows, "ur5", sc).get(sid, {}).get(field, float("nan"))
                for sc in scen]
        ax.bar(x + (i - (len(solvers) - 1) / 2) * w, vals, w,
               color=S.color(sid), edgecolor="white", linewidth=0.4, zorder=2)
    ax.set_xticks(x)
    ax.set_xticklabels([S.SCEN_LABEL[s] for s in scen])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=str(S.DEFAULT_COLLISION_CSV))
    args = ap.parse_args()

    S.use_paper_style()
    rows = S.load_rows(args.csv)
    solvers = [s for s in SOLVERS if any(r["robot"] == "ur5" and r["solver"] == s
                                         for r in rows)]
    scen = [s for s in S.SCENARIOS if S.cell(rows, "ur5", s)]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(S.COL, S.COL * 1.25),
                                   sharex=True, height_ratios=[2, 1])
    grouped(ax1, rows, "pb_collision_pct", solvers, scen)
    ax1.set_ylabel("Self-collision, PyBullet (%)")
    ax1.set_title("UR5: KineticFold is cleanest and\nmost reliable in every regime")

    grouped(ax2, rows, "success_pct", solvers, scen)
    ax2.set_ylabel("Success (%)")
    ax2.set_ylim(70, 102)
    ax2.axhline(99, ls=":", lw=0.8, color="#888")

    handles = [Patch(facecolor=S.color(s), label=S.label(s)) for s in solvers]
    fig.legend(handles=handles, ncol=2, loc="lower center",
               bbox_to_anchor=(0.5, -0.04))
    fig.subplots_adjust(bottom=0.22, hspace=0.12)

    S.save(fig, "fig_collision_ur5")


if __name__ == "__main__":
    main()
