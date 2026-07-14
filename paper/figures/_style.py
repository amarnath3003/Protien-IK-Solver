"""Shared style, palette, data loaders, and helpers for every ProteinIK paper figure.

Importing from here makes the whole figure set read as ONE system:

  * one serif, print-tuned matplotlib style (no LaTeX install required);
  * one fixed solver -> colour map. Colour follows the SOLVER *entity*, never its
    rank, so a solver is the same colour in every figure (a dataviz non-negotiable);
  * one CSV loader that tolerates BOTH the narrow schema written by
    `master_benchmark.py` (`collision_pct`) and the wide sim-scored schema
    (`our_collision_pct` + `pb_*`/`mj_*`), so the same code works before and after
    the PyBullet/MuJoCo scoring pass.

Colours are the Okabe-Ito colour-blind-safe qualitative palette, assigned once.
Dependencies: matplotlib + numpy + stdlib csv only (no pandas).

Solver id -> paper name mapping (the paper renames the code's V1/V4/V6):
    protein_ik   -> StagedFold      protein_fast -> KineticFold
    protein_raw  -> LangevinFold    trac_ik_style-> TRAC-IK (real TRACLabs, native)
"""
from __future__ import annotations

import csv
from pathlib import Path

# matplotlib is imported lazily inside use_paper_style()/save() so that the table
# generator (make_tables.py), which needs no plotting, runs with stdlib + numpy only.

# --------------------------------------------------------------------------- #
# Repository layout (paths resolve relative to this file, so CWD doesn't matter)
# --------------------------------------------------------------------------- #
FIG_DIR = Path(__file__).resolve().parent          # paper/figures
PAPER_DIR = FIG_DIR.parent                          # paper
REPO = PAPER_DIR.parent                             # repo root
TABLE_DIR = PAPER_DIR / "tables"                    # paper/tables

# Default data locations. THE real, paper-grade benchmark is the NATIVE re-run produced by
# backend/native_bench/run_native_master.py ("solve once, score three ways" in PyBullet +
# MuJoCo, every solver native: real TRAC-IK via tracikpy, RTB Jacobian-DLS/Multi-start, and
# C++/Eigen ProteinIK + CCD/FABRIK ports); its committed output is
# results/master_10seed_fast(cpp).csv (UR5 + Franka, seeds 1..10, n=1000/cell). Both
# success/speed AND real-mesh collision come from that single file. The pre-native
# `master_10seed_fast.csv` / `master_full.csv` are superseded. Override any of these on the CLI.
DEFAULT_MASTER_CSV    = REPO / "backend" / "results" / "master_10seed_fast(cpp).csv"  # THE master sim benchmark (native)
DEFAULT_COLLISION_CSV = REPO / "backend" / "results" / "master_10seed_fast(cpp).csv"  # same file (carries pb_/mj_ collision)
DEFAULT_USECASE_JSON  = REPO / "backend" / "scrap"   / "usecase_results.json"    # DOF scaling (SEPARATE experiment)
DEFAULT_LANGEVIN_CSV  = REPO / "backend" / "results" / "langevin_bench.csv"      # LangevinFold mini-benchmark (SEPARATE, small-scale)

# --------------------------------------------------------------------------- #
# Paper-facing names + fixed per-solver colours
# --------------------------------------------------------------------------- #
LABEL = {
    "protein_ik":            "StagedFold",
    "protein_fast":          "KineticFold",
    "protein_raw":           "LangevinFold",
    "trac_ik_style":         "TRAC-IK",
    "multi_start":           "Multi-start",
    "jacobian_dls":          "Jacobian-DLS",
    "ccd":                   "CCD",
    "fabrik":                "FABRIK",
    "analytical_planar3dof": "Analytical",
    "protein_homotopy":      "CCH-IK",
    "fixed_lambda_ik":       "Fixed-$\\lambda$",
}

# Okabe-Ito, assigned by entity. Hero = KineticFold (green); rival = TRAC-IK (vermillion).
COLOR = {
    "protein_fast":          "#009E73",  # bluish green   -- KineticFold (hero)
    "trac_ik_style":         "#D55E00",  # vermillion     -- TRAC-IK (main rival)
    "multi_start":           "#0072B2",  # blue
    "protein_ik":            "#CC79A7",  # reddish purple -- StagedFold
    "protein_raw":           "#E69F00",  # orange         -- LangevinFold
    "jacobian_dls":          "#56B4E9",  # sky blue
    "ccd":                   "#9A9A9A",  # grey  (weak field, recessive)
    "fabrik":                "#5F5F5F",  # dark grey
    "analytical_planar3dof": "#000000",
    "protein_homotopy":      "#117733",
    "fixed_lambda_ik":       "#AA4499",
}
HERO = "protein_fast"

# Natural weak -> strong ordering for bar charts.
SOLVER_ORDER = ["ccd", "fabrik", "jacobian_dls", "protein_ik",
                "multi_start", "trac_ik_style", "protein_fast"]

# Scenarios are difficulty-ordered -> a single-hue sequential ramp (light->dark).
SCENARIOS = ["open_space", "near_singular", "cluttered"]
SCEN_LABEL = {"open_space": "open", "near_singular": "near-sing.", "cluttered": "cluttered"}
SCEN_COLOR = {"open_space": "#cfe8e3", "near_singular": "#6bbcae", "cluttered": "#1f6f6b"}

# Figure widths (inches) for a two-column paper.
COL = 3.5     # single column
WIDE = 7.16   # full text width


def label(sid: str) -> str:
    return LABEL.get(sid, sid)


def color(sid: str) -> str:
    return COLOR.get(sid, "#444444")


# --------------------------------------------------------------------------- #
# Matplotlib style
# --------------------------------------------------------------------------- #
def use_paper_style() -> None:
    import matplotlib as mpl
    mpl.rcParams.update({
        "figure.dpi": 130,
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.02,
        "pdf.fonttype": 42,          # embed TrueType (editable/searchable text)
        "ps.fonttype": 42,
        "font.family": "serif",
        "font.serif": ["Times New Roman", "STIXGeneral", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "font.size": 8.5,
        "axes.titlesize": 9.5,
        "axes.titleweight": "bold",
        "axes.labelsize": 8.5,
        "axes.linewidth": 0.6,
        "axes.edgecolor": "#333333",
        "axes.grid": True,
        "axes.axisbelow": True,
        "grid.color": "#dddddd",
        "grid.linewidth": 0.5,
        "xtick.labelsize": 7.5,
        "ytick.labelsize": 7.5,
        "xtick.color": "#333333",
        "ytick.color": "#333333",
        "legend.fontsize": 7.5,
        "legend.frameon": False,
        "lines.linewidth": 1.6,
        "lines.markersize": 5,
    })


def save(fig, name: str) -> None:
    """Write <name>.pdf (for \\includegraphics) and <name>.png (quick preview)."""
    import matplotlib.pyplot as plt
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    for ext in ("pdf", "png"):
        fig.savefig(FIG_DIR / f"{name}.{ext}")
    plt.close(fig)
    print(f"  wrote figures/{name}.pdf  +  .png")


# --------------------------------------------------------------------------- #
# Data loading (stdlib csv; no pandas)
# --------------------------------------------------------------------------- #
_STR_COLS = {"robot", "scenario", "solver", "display_name", "status"}


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float("nan")


def load_rows(path) -> list[dict]:
    """Load a benchmark CSV into a list of dicts, coercing numeric columns to float.

    Normalises the collision-column name: the wide sim-scored schema calls the
    capsule-proxy rate `our_collision_pct`; the narrow schema calls it
    `collision_pct`. After this call both are available under `our_collision_pct`.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(
            f"Benchmark file not found: {path}\n"
            f"Run the benchmark first, or pass the correct path on the CLI."
        )
    out: list[dict] = []
    with open(path, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            row = {k: (v if k in _STR_COLS else _f(v)) for k, v in r.items()}
            if "our_collision_pct" not in row and "collision_pct" in row:
                row["our_collision_pct"] = row["collision_pct"]
            out.append(row)
    return out


def cell(rows: list[dict], robot: str, scenario: str) -> dict[str, dict]:
    """Map solver_id -> row for a single (robot, scenario) cell."""
    return {r["solver"]: r for r in rows
            if r["robot"] == robot and r["scenario"] == scenario}


def present_solvers(rows: list[dict], robot: str, order=SOLVER_ORDER) -> list[str]:
    """Solvers from `order` that actually appear for `robot`, in that order."""
    have = {r["solver"] for r in rows if r["robot"] == robot}
    return [s for s in order if s in have]
