"""LangevinFold mini-result (UR5) — cleanest self-collision, at a latency cost.

Two stacked panels sharing the x-axis: (top) PyBullet real-mesh self-collision rate;
(bottom) mean wall-clock latency on a LOG axis. The single picture behind the paper's
one-paragraph LangevinFold result: the literal folding simulation gives the lowest
real-mesh collision of any solver on the non-redundant arm (top, LangevinFold bars
lowest) while costing seconds per solve (bottom, LangevinFold bars ~100x higher) —
faithful biophysics buys solution quality, not speed. MuJoCo agrees (CSV mj_collision_pct).

Reads the SEPARATE small-scale run produced by backend/bench/langevin_benchmark.py.
Run:  python fig_langevin.py [--csv path/to/langevin_bench.csv] [--robot ur5]
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

# LangevinFold first (the star), then the reference field.
SOLVERS = ["protein_raw", "protein_fast", "trac_ik_style", "multi_start"]


def grouped(ax, rows, robot, field, solvers, scen, log=False):
    x = np.arange(len(scen))
    w = 0.8 / len(solvers)
    for i, sid in enumerate(solvers):
        vals = [S.cell(rows, robot, sc).get(sid, {}).get(field, float("nan")) for sc in scen]
        ax.bar(x + (i - (len(solvers) - 1) / 2) * w, vals, w,
               color=S.color(sid), edgecolor="white", linewidth=0.4, zorder=2)
    if log:
        ax.set_yscale("log")
    ax.set_xticks(x)
    ax.set_xticklabels([S.SCEN_LABEL[s] for s in scen])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=str(S.DEFAULT_LANGEVIN_CSV))
    ap.add_argument("--robot", default="ur5")
    args = ap.parse_args()

    S.use_paper_style()
    try:
        rows = S.load_rows(args.csv)
    except FileNotFoundError:
        print("  [skip] fig_langevin: run backend/bench/langevin_benchmark.py first "
              "(needs results/langevin_bench.csv)")
        return
    solvers = [s for s in SOLVERS
               if any(r["robot"] == args.robot and r["solver"] == s for r in rows)]
    scen = [s for s in S.SCENARIOS if S.cell(rows, args.robot, s)]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(S.COL * 1.15, S.COL * 1.2),
                                   sharex=True, height_ratios=[1, 1])
    grouped(ax1, rows, args.robot, "pb_collision_pct", solvers, scen)
    ax1.set_ylabel("Self-collision, PyBullet (%)")
    ax1.set_title(f"LangevinFold: cleanest fold, at a\nlatency cost ({args.robot.replace('_',' ')})")

    grouped(ax2, rows, args.robot, "mean_ms", solvers, scen, log=True)
    ax2.set_ylabel("Mean latency (ms, log)")

    handles = [Patch(facecolor=S.color(s), label=S.label(s)) for s in solvers]
    fig.legend(handles=handles, ncol=2, loc="lower center", bbox_to_anchor=(0.5, -0.03))
    fig.subplots_adjust(bottom=0.22, hspace=0.12)

    S.save(fig, "fig_langevin")


if __name__ == "__main__":
    main()
