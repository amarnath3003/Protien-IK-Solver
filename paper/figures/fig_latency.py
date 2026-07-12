"""F4 -- latency distribution: median-fast, with a bounded tail concentrated exactly
where the frustration diagnosis predicts.

For four (arm, scenario) cells this draws, per solver, the p50 -> p99 interval on a
LOG time axis with markers at p50 / p95 / p99 and a caret at the mean. KineticFold's
barrierless-first schedule makes its median 2-2.5x below TRAC-IK everywhere, at the
cost of a heavier tail on the hardest cell (Franka cluttered) -- a story a table of
percentiles renders poorly and a picture renders instantly.

Source: master CSV (needs p50_ms/p95_ms/p99_ms/mean_ms columns).
Run:    python fig_latency_tail.py [--csv path/to/master_full.csv]
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _style as S

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

CELLS = [("ur5", "open_space"), ("ur5", "cluttered"),
         ("franka_panda", "open_space"), ("franka_panda", "cluttered")]
SOLVERS = ["trac_ik_style", "protein_fast"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=str(S.DEFAULT_MASTER_CSV))
    args = ap.parse_args()

    S.use_paper_style()
    rows = S.load_rows(args.csv)

    cells = [c for c in CELLS if S.cell(rows, *c)]
    fig, ax = plt.subplots(figsize=(S.COL * 2, S.COL * 1.05))

    yticks, ylabels, y = [], [], 0.0
    for robot, scen in cells:
        c = S.cell(rows, robot, scen)
        group_top = y
        for sid in SOLVERS:
            r = c.get(sid)
            if not r:
                continue
            col = S.color(sid)
            ax.plot([r["p50_ms"], r["p99_ms"]], [y, y], color=col, lw=1.4, zorder=2)
            ax.scatter(r["p50_ms"], y, color=col, s=34, zorder=4)            # p50
            ax.scatter(r["p95_ms"], y, color=col, s=30, marker="D", zorder=4)  # p95
            ax.scatter(r["p99_ms"], y, color="white", edgecolor=col, s=30,
                       linewidth=1.2, zorder=4)                              # p99
            ax.scatter(r["mean_ms"], y, color=col, marker="|", s=90,
                       linewidth=1.4, zorder=3)                              # mean
            ax.text(r["p50_ms"], y + 0.18, S.label(sid), fontsize=6.2,
                    color=col, ha="left", va="bottom")
            yticks.append(y)
            ylabels.append("")
            y += 1.0
        # cell label centred on the group
        ax.text(-0.02, (group_top + y - 1) / 2,
                f"{ {'ur5':'UR5','franka_panda':'Franka'}[robot] }\n{S.SCEN_LABEL[scen]}",
                transform=ax.get_yaxis_transform(), ha="right", va="center",
                fontsize=7, fontweight="bold")
        y += 0.7  # gap between cells

    ax.set_xscale("log")
    ax.set_xlabel("Wall-clock latency (ms, log scale)")
    ax.set_yticks([])
    ax.set_ylim(-0.6, y - 0.4)
    ax.margins(x=0.08)
    ax.grid(axis="y", visible=False)
    ax.set_title("KineticFold: faster median, tail only on the hardest cell")

    marker_legend = [
        Line2D([0], [0], marker="o", color="#444", ls="", label="p50 (median)"),
        Line2D([0], [0], marker="D", color="#444", ls="", label="p95"),
        Line2D([0], [0], marker="o", mfc="white", mec="#444", color="#444", ls="", label="p99"),
        Line2D([0], [0], marker="|", color="#444", ls="", label="mean"),
    ]
    ax.legend(handles=marker_legend, ncol=4, loc="upper center",
              bbox_to_anchor=(0.5, -0.16))
    fig.subplots_adjust(left=0.16, bottom=0.24)

    S.save(fig, "fig_latency_tail")


if __name__ == "__main__":
    main()
