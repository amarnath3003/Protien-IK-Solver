"""F2 (flagship) -- single-shot clean-solve rate vs planar-arm DOF (chain length).

The paper's central result: as a planar arm is lengthened 4 -> 16 joints (made
progressively more polymer-like), KineticFold's clean-solve advantage over genuine
TRAC-IK holds at every length and grows through the mid-DOF range (peaking near
3.2x at 8 DOF) until KineticFold is the only method still producing collision-free
folds. Both solvers reach the target ~100% of the time; the entire gap is
self-collision avoidance. Native, apples-to-apples: KineticFold runs as its
C++/Eigen port, TRAC-IK as the genuine TRACLabs C++ library (tracikpy).

Source: usecase_results.json, key "E" (EXP E, native run).
Run:    python fig_dof_scaling.py [--json path/to/usecase_results.json]
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


def load_E(path):
    E = json.loads(Path(path).read_text(encoding="utf-8"))["E"]
    dofs = sorted({int(r["dof"]) for r in E})
    series = {
        sid: [next((r["clean_pct"] for r in E
                    if int(r["dof"]) == d and r["solver"] == sid), float("nan"))
              for d in dofs]
        for sid in ("protein_fast", "trac_ik_style")
    }
    return dofs, series


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default=str(S.DEFAULT_USECASE_JSON))
    args = ap.parse_args()

    S.use_paper_style()
    dofs, series = load_E(args.json)
    kf, tr = series["protein_fast"], series["trac_ik_style"]

    fig, ax = plt.subplots(figsize=(S.COL, S.COL * 0.86))
    ax.plot(dofs, kf, color=S.color("protein_fast"), ls="-", marker="o",
            label=S.label("protein_fast"), zorder=3)
    ax.plot(dofs, tr, color=S.color("trac_ik_style"), ls="--", marker="s",
            label="TRAC-IK (genuine)", zorder=3)

    # Advantage ratio, annotated above each KineticFold point.
    for d, a, b in zip(dofs, kf, tr):
        if b > 0:
            txt = f"{a / b:.1f}×"
        elif a > 0:
            txt = "only\nsolver"
        else:
            continue
        ax.annotate(txt, (d, a), textcoords="offset points", xytext=(0, 8),
                    ha="center", fontsize=6.6, color=S.color("protein_fast"),
                    fontweight="bold")

    ax.set_xlabel("Planar arm DOF   (chain length → polymer)")
    ax.set_ylabel("Single-shot clean-solve rate (%)")
    ax.set_title("KineticFold is the last to fold the longest chains cleanly")
    ax.set_xticks(dofs)
    ax.set_ylim(-3, max([v for v in kf if v == v]) + 16)
    ax.legend(loc="upper right")
    ax.text(0.5, -0.30,
            "Both solvers reach the target ≈100%; the gap is entirely "
            "self-collision avoidance.",
            transform=ax.transAxes, ha="center", fontsize=6.3, color="#666")

    S.save(fig, "fig_dof_scaling")


if __name__ == "__main__":
    main()
