"""F7 -- speed vs quality: where each solver belongs.

A scatter over the UR5 `cluttered` cell: x = mean wall-clock latency (log ms),
y = success rate; each point annotated with its real-mesh (PyBullet) self-collision
rate, the "quality" axis. Latency zones are shaded to read off a deployment role:
tight real-time control (fast, bounded) vs planning / offline / quality-critical
generation (latency tolerated for cleaner, more reliable solves). KineticFold lands
high-success + low-collision at moderate latency -> planning/offline; the simple
baselines sit low-success; TRAC-IK is fast but collides more on this hard cell.

Single source, single definition per axis. Source: master CSV.
Run:    python fig_deployment.py [--csv path/to/master_full.csv] [--scenario cluttered]
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _style as S

import matplotlib.pyplot as plt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=str(S.DEFAULT_MASTER_CSV))
    ap.add_argument("--robot", default="ur5")
    ap.add_argument("--scenario", default="cluttered")
    args = ap.parse_args()

    S.use_paper_style()
    rows = S.load_rows(args.csv)
    c = S.cell(rows, args.robot, args.scenario)
    solvers = [s for s in S.SOLVER_ORDER if s in c]

    fig, ax = plt.subplots(figsize=(S.COL * 1.7, S.COL * 1.05))

    # deployment latency zones (ms), labelled along the top edge (out of the point field)
    ax.axvspan(0.1, 15, color="#e8f3f1", zorder=0)
    ax.axvspan(15, 1e5, color="#f4eef6", zorder=0)
    ax.text(3.2, 0.965, "tight real-time", transform=ax.get_xaxis_transform(),
            fontsize=6.8, color="#3a8a80", ha="center", va="top")
    ax.text(2000, 0.90, "planning / offline / quality-critical",
            transform=ax.get_xaxis_transform(), fontsize=6.8, color="#8a5a9a",
            ha="center", va="top")

    # Fan label offsets (points) per solver, routed into open space so nothing
    # collides with the title, the zone labels, or each other.
    OFF = {
        "protein_fast":  (58, 8), "trac_ik_style": (-10, -34), "multi_start": (58, -12),
        "protein_ik":    (40, 0), "jacobian_dls":  (-42, -24), "fabrik": (30, 16),
        "ccd":           (48, -14),
    }
    for sid in solvers:
        r = c[sid]
        self_col = r.get("pb_collision_pct", float("nan"))
        ax.scatter(r["mean_ms"], r["success_pct"], s=70, color=S.color(sid),
                   edgecolor="white", linewidth=0.8, zorder=5)
        tag = S.label(sid)
        if self_col == self_col:
            tag += f" · {self_col:.0f}% col"
        dx, dy = OFF.get(sid, (7, 6))
        ax.annotate(tag, (r["mean_ms"], r["success_pct"]),
                    textcoords="offset points", xytext=(dx, dy), fontsize=6.2,
                    color=S.color(sid), ha="center",
                    arrowprops=dict(arrowstyle="-", lw=0.5, color=S.color(sid),
                                    shrinkA=0, shrinkB=3), zorder=4)

    ax.set_xscale("log")
    ax.set_xlabel("Mean wall-clock latency (ms, log)")
    ax.set_ylabel("Success rate (%)")
    ax.set_ylim(-6, 112)
    ax.set_title(f"Speed vs quality — {args.robot.replace('_',' ')} · {S.SCEN_LABEL.get(args.scenario, args.scenario)}")
    ax.text(0.5, -0.22,
            "Point label = real-mesh (PyBullet) self-collision rate; lower is cleaner.",
            transform=ax.transAxes, ha="center", fontsize=6.2, color="#666")
    fig.subplots_adjust(bottom=0.20)

    S.save(fig, "fig_deployment")


if __name__ == "__main__":
    main()
