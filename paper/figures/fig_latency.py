"""F4 -- per-solve latency across the whole solver field: median vs mean.

For each arm (UR5, Franka) this draws, for every solver, its median (p50) and mean
wall-clock latency as two grouped bars on a LOG time axis, with the millisecond
value printed on each bar. The open-space regime is used so every solver is timed
on targets it actually attempts (on the harder regimes the simple baselines fail
outright and LangevinFold carries a measurement outlier). KineticFold has the
fastest typical solve of the practical field; LangevinFold alone runs offline-slow.

Source: broad 3-seed survey (master_full.csv) -- the paper's speed source.
Run:    python fig_latency_tail.py [--csv path/to/master_full.csv]
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _style as S

import matplotlib.pyplot as plt
import numpy as np

SCENARIO = "open_space"
# weak -> strong (LangevinFold excluded: offline, seconds/solve -- off the scale)
ORDER = ["ccd", "fabrik", "jacobian_dls", "protein_ik",
         "multi_start", "trac_ik_style", "protein_fast"]
METRICS = [("p50_ms", "median", "#1f6f6b"), ("mean_ms", "mean", "#e08a3c")]
FLOOR = 0.7  # ms; log-scale bar base


def _fmt(v: float) -> str:
    if v != v:
        return ""
    if v < 10:
        return f"{v:.1f}"
    if v < 1000:
        return f"{v:.0f}"
    return f"{v / 1000:.1f}k"


def main():
    ap = argparse.ArgumentParser()
    # Latency is reported from the broad 3-seed survey (master_full); success and
    # collision come from the 10-seed run.
    ap.add_argument("--csv", default=str(S.REPO / "backend" / "results" / "master_full.csv"))
    args = ap.parse_args()

    S.use_paper_style()
    rows = S.load_rows(args.csv)
    arms = [a for a in ("ur5", "franka_panda") if S.cell(rows, a, SCENARIO)]

    fig, axes = plt.subplots(1, len(arms), figsize=(S.WIDE, S.WIDE * 0.44), sharey=True)
    if len(arms) == 1:
        axes = [axes]

    top = 0.0
    for ax, arm in zip(axes, arms):
        c = S.cell(rows, arm, SCENARIO)
        solvers = [s for s in ORDER if s in c]
        x = np.arange(len(solvers))
        w = 0.40
        for k, (metric, mlabel, mcolor) in enumerate(METRICS):
            vals = [c.get(s, {}).get(metric, float("nan")) for s in solvers]
            top = max([top] + [v for v in vals if v == v])
            heights = [(v - FLOOR) if v == v else float("nan") for v in vals]
            bars = ax.bar(x + (k - 0.5) * w, heights, w, bottom=FLOOR,
                          color=mcolor, edgecolor="white", linewidth=0.4,
                          label=mlabel, zorder=3)
            ax.bar_label(bars, labels=[_fmt(v) for v in vals], padding=1.5,
                         fontsize=5.4, rotation=90)
        ax.set_yscale("log")
        ax.set_xticks(x)
        ax.set_xticklabels([S.label(s) for s in solvers], rotation=32, ha="right")
        ax.set_title({"ur5": "UR5 (6-DOF, non-redundant)",
                      "franka_panda": "Franka Panda (7-DOF, redundant)"}[arm])
        ax.grid(axis="x", visible=False)
    axes[0].set_ylabel("Wall-clock latency (ms, log scale)")
    for ax in axes:
        ax.set_ylim(FLOOR, top * 3.0)
    axes[0].legend(loc="upper right", title="per-solve latency")
    fig.suptitle("KineticFold has the fastest typical solve of the practical solver field",
                 fontsize=10, fontweight="bold", y=1.02)
    fig.subplots_adjust(bottom=0.24, wspace=0.06)

    S.save(fig, "fig_latency")


if __name__ == "__main__":
    main()
