"""F4 -- per-solve latency across the whole solver field: median vs mean.

For each arm (UR5, Franka) this draws, for every solver, its median (p50), mean, and
p99-tail wall-clock latency as three grouped bars on a LOG time axis, with the
millisecond value printed on each bar (the p99 bar makes each solver's tail explicit). The open-space regime is used so every solver is timed
on targets it actually attempts (on the harder regimes the simple baselines fail
outright and LangevinFold carries a measurement outlier). KineticFold has the
fastest typical solve of the practical field; LangevinFold alone runs offline-slow.

Source: native 10-seed run (master_10seed_fast(cpp).csv) -- every solver native
(real TRAC-IK, RTB baselines, C++ ProteinIK/CCD/FABRIK), so timings are apples-to-apples.
Run:    python fig_latency.py [--csv path/to/master_10seed_fast(cpp).csv]
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
# weak -> strong (LangevinFold excluded: ~14 ms offline solver, off the sub-ms scale)
ORDER = ["ccd", "fabrik", "jacobian_dls", "protein_ik",
         "multi_start", "trac_ik_style", "protein_fast"]
METRICS = [("p50_ms", "median", "#1f6f6b"), ("mean_ms", "mean", "#e08a3c"),
           ("p99_ms", "p99 (tail)", "#b5462f")]
FLOOR = 0.05  # ms; log-scale bar base (native solvers run sub-millisecond)


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
    # Latency is reported from the native 10-seed run (all solvers compiled), the same
    # file as success and collision, so the speed comparison is apples-to-apples.
    ap.add_argument("--csv", default=str(S.REPO / "backend" / "results" / "master_10seed_fast(cpp).csv"))
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
        w = 0.27
        for k, (metric, mlabel, mcolor) in enumerate(METRICS):
            vals = [c.get(s, {}).get(metric, float("nan")) for s in solvers]
            top = max([top] + [v for v in vals if v == v])
            heights = [(v - FLOOR) if v == v else float("nan") for v in vals]
            bars = ax.bar(x + (k - (len(METRICS) - 1) / 2) * w, heights, w, bottom=FLOOR,
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
