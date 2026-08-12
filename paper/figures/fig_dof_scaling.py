"""F2 (flagship) -- single-shot clean-solve rate vs planar-arm DOF (chain length).

The paper's central scaling result, from the powered sweep
(``results/dof_scaling/dof_scaling_full.json``, n=1000/cell, seeds 1..10):

  (a) KineticFold's clean-solve advantage over BOTH production baselines grows
      monotonically with chain length -- 2.0x -> 7.5x against genuine TRAC-IK and
      1.6x -> 3.7x against genuine RTB Multi-start, the stronger of the two. The
      earlier "peaks near 8 DOF" shape was an artifact of the n=120 pilot.
  (b) The collapse in solve rate is a SEARCH limit, not a geometric one. The
      fraction of targets for which any clean solution can be demonstrated stays
      at 90-97% through 14 DOF and is still 75% at 16 DOF, while the solve rates
      fall by nearly two orders of magnitude. (An earlier 48-restart oracle put
      16 DOF at 24% and looked like a feasibility floor; at 384 restarts that
      reading disappears, so the oracle's own budget has to be reported.)

All three solvers reach the target ~100% of the time; the entire gap is
self-collision avoidance. Native and apples-to-apples: KineticFold as its
C++/Eigen port, TRAC-IK as genuine TRACLabs C++ (tracikpy), Multi-start as
genuine Robotics Toolbox ik_LM.

Log y-axis is deliberate: the claim is about *ratios*, and on a log axis a
constant ratio is a constant vertical gap, so "the advantage widens" is directly
legible as widening separation rather than something the reader must compute.

Run: python fig_dof_scaling.py [--json path/to/dof_scaling_full.json]
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _style as S

import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter

SERIES = ["protein_fast", "trac_ik_style", "multi_start"]
MARKER = {"protein_fast": "o", "trac_ik_style": "s", "multi_start": "^"}
LINESTYLE = {"protein_fast": "-", "trac_ik_style": "--", "multi_start": ":"}
FLOOR = 0.08          # log-axis floor (%); a 0/1000 cell is drawn here, labelled
FEAS_COLOR = "#7A7A7A"


def load(path):
    D = json.loads(Path(path).read_text(encoding="utf-8"))
    dofs = D["config"]["dofs"]

    def cell(d, s):
        return next(c for c in D["cells"] if c["dof"] == d and c["solver"] == s)

    series = {s: [cell(d, s) for d in dofs] for s in SERIES}
    feas = [cell(d, "protein_fast").get("feasible_pct") for d in dofs]
    ratios = {
        base: [next((k for k in D["contrasts"]
                     if k["dof"] == d and k["baseline"] == base), None) for d in dofs]
        for base in ("trac_ik_style", "multi_start")
    }
    return dofs, series, feas, ratios


def panel_rates(ax, dofs, series, feas):
    # Feasibility first, recessive: it is the ceiling the solvers work under, not
    # a competitor. Same units as the solver series (% of targets), so one axis.
    if any(f is not None for f in feas):
        ax.plot(dofs, feas, color=FEAS_COLOR, ls="-", lw=1.4, marker="", alpha=0.85,
                zorder=1, label="a clean solution exists (any solver)")
        ax.fill_between(dofs, FLOOR, feas, color=FEAS_COLOR, alpha=0.07, zorder=0)

    for sid in SERIES:
        cs = series[sid]
        y = [c["clean_pct"] for c in cs]
        lo = [max(c["ci95_lo"], FLOOR) for c in cs]
        hi = [max(c["ci95_hi"], FLOOR) for c in cs]
        col = S.color(sid)
        ax.fill_between(dofs, lo, hi, color=col, alpha=0.16, lw=0, zorder=2)
        yplot = [v if v > 0 else FLOOR for v in y]
        ax.plot(dofs, yplot, color=col, ls=LINESTYLE[sid], lw=2.0,
                marker=MARKER[sid], ms=4.6, mew=0, zorder=3,
                label=S.label(sid) if sid != "trac_ik_style" else "TRAC-IK (genuine)")
        # a 0/1000 cell cannot be drawn on a log axis -- mark it explicitly rather
        # than letting the line vanish, which would read as missing data
        for d, v, c in zip(dofs, y, cs):
            if v == 0:
                ax.plot([d], [FLOOR], marker="v", ms=5, color=col,
                        mfc="white", mew=1.3, zorder=4)
                ax.annotate(f"0/{c['n']}", (d, FLOOR), textcoords="offset points",
                            xytext=(-9, 5), ha="right", fontsize=5.8, color=col)

    ax.set_yscale("log")
    ax.set_ylim(FLOOR, 130)
    ax.yaxis.set_major_formatter(FuncFormatter(
        lambda v, _p: f"{v:g}" if v >= 1 else f"{v:.1f}"))
    ax.set_xticks(dofs)
    ax.set_xlabel("Planar arm DOF   (chain length → polymer)")
    ax.set_ylabel("Single-shot clean-solve rate (%)")
    ax.set_title("(a) The advantage holds at every chain length", loc="left")
    ax.legend(loc="lower left", fontsize=5.9, framealpha=0.92)


def panel_ratio(ax, dofs, ratios):
    for base, col, mk, ls in (("trac_ik_style", S.color("trac_ik_style"), "s", "--"),
                              ("multi_start", S.color("multi_start"), "^", ":")):
        xs, ys, weak = [], [], []
        for d, k in zip(dofs, ratios[base]):
            if k is None or k["ratio"] is None:
                continue
            xs.append(d)
            ys.append(k["ratio"])
            weak.append(k["fisher_p"] >= 0.05)
        ax.plot(xs, ys, color=col, ls=ls, lw=2.0, marker=mk, ms=4.6, mew=0, zorder=3)
        # hollow any point that is not significant at p<0.05 -- identity is never
        # carried by colour alone here, so this reads in print and under CVD too
        for x, y, w in zip(xs, ys, weak):
            if w:
                ax.plot([x], [y], marker=mk, ms=4.6, color=col, mfc="white",
                        mew=1.3, zorder=4)
        # label on the last cell that is actually significant -- the terminal point
        # can be a hollow, low-count cell where the ratio is not resolvable
        j = max([i for i, w in enumerate(weak) if not w], default=len(xs) - 1)
        ax.annotate(f"vs {S.label(base)}", (xs[j], ys[j]),
                    textcoords="offset points", xytext=(-6, 6), ha="right",
                    fontsize=6.2, color=col, fontweight="bold")

    ax.axhline(1.0, color="#BBBBBB", lw=1.0, ls="-", zorder=1)
    ax.annotate("parity", (dofs[0], 1.0), textcoords="offset points",
                xytext=(0, -9), fontsize=5.8, color="#888888")
    ax.set_xticks(dofs)
    ax.set_ylim(0, None)
    ax.set_xlabel("Planar arm DOF")
    ax.set_ylabel("KineticFold clean-solve advantage (×)")
    # "through 14 DOF", not "with chain length": at 16 DOF the clean counts are too
    # small to resolve a ratio (TRAC-IK 0/1000; Multi-start p=0.21), so the terminal
    # point is drawn but must not be read as the trend reversing
    ax.set_title("(b) …and widens through 14 DOF", loc="left")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default=str(S.DEFAULT_DOF_FULL_JSON))
    args = ap.parse_args()

    S.use_paper_style()
    dofs, series, feas, ratios = load(args.json)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(S.WIDE, S.WIDE * 0.40))
    panel_rates(ax1, dofs, series, feas)
    panel_ratio(ax2, dofs, ratios)

    fig.text(0.5, -0.04,
             "All three solvers reach the target ≈100%; the gap is entirely "
             "self-collision avoidance.  n = 1000 per cell; bands are Wilson 95% "
             "intervals.  Hollow marker in (b) = not significant at p<0.05.",
             ha="center", fontsize=6.1, color="#666")
    fig.tight_layout()
    S.save(fig, "fig_dof_scaling")


if __name__ == "__main__":
    main()
