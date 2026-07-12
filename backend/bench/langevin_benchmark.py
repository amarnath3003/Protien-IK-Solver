"""Dedicated LangevinFold (V6 / ``protein_raw``) benchmark — small scale, by design.

LangevinFold runs a coarse-grained overdamped-Langevin folding *simulation* and costs
~seconds per solve, so it is deliberately EXCLUDED from the main 10-seed master sweep
(``master_10seed_fast``, n=1000/cell). This runs it at an honest smaller scale against
the same reference field as the UR5 collision table, scored the SAME "solve once, score
three ways" way (capsule proxy + PyBullet + MuJoCo) — it literally *reuses*
``master_sim_benchmark``'s validated cell runner and oracles, so every number is
apples-to-apples with Tables 5–8 and cross-checked on two engines.

Point it establishes for the paper's one-paragraph LangevinFold result: on the
non-redundant UR5, the literal folding simulation produces the CLEANEST real-mesh
self-collision profile of any solver in the study, at a latency cost (seconds/solve)
that restricts it to offline, quality-critical use — i.e. faithful biophysics buys
solution *quality*, not speed.

Environment
-----------
Needs ``pybullet`` + ``mujoco`` (the real-mesh oracles), which on this project live in
``backend/.venv-sim``. Run it from ``backend/`` as a module, exactly like the master:

    PYTHONPATH=. .venv-sim/Scripts/python -m bench.langevin_benchmark              # default (~10–15 min)
    PYTHONPATH=. .venv-sim/Scripts/python -m bench.langevin_benchmark --quick      # smoke
    PYTHONPATH=. .venv-sim/Scripts/python -m bench.langevin_benchmark --trials 60  # more samples
    PYTHONPATH=. .venv-sim/Scripts/python -m bench.langevin_benchmark --resume     # continue a crashed run

Any flag not listed below is forwarded verbatim to ``master_sim_benchmark`` (e.g.
``--no-mujoco`` to run PyBullet-only, ``--robots ur5 franka_panda``).

Output: ``results/langevin_bench.{csv,md,manifest.json}`` (same wide schema as the
master), plus a focused, paragraph-ready summary printed at the end (and the master's
own "lowest-collision solver per cell" verdict block inside the ``.md``).
"""
from __future__ import annotations

import argparse
import csv
import sys

from bench.master_sim_benchmark import main as master_main

# LangevinFold (the star) first, then the reference field it must beat on cleanliness —
# the exact solvers in the UR5 collision table, so the comparison is consistent.
FIELD = ["protein_raw", "protein_fast", "trac_ik_style", "multi_start", "protein_ik"]

# code id -> paper name, for the summary print
PAPER = {"protein_raw": "LangevinFold", "protein_fast": "KineticFold",
         "trac_ik_style": "TRAC-IK-style", "multi_start": "Multi-start",
         "protein_ik": "StagedFold"}


def _print_summary(csv_path: str) -> None:
    """Read the finished CSV and print a compact, paragraph-ready comparison:
    where does LangevinFold rank on real-mesh collision, and at what latency?"""
    try:
        rows = list(csv.DictReader(open(csv_path, encoding="utf-8")))
    except FileNotFoundError:
        return

    def f(r, k):
        try:
            return float(r[k])
        except (KeyError, ValueError, TypeError):
            return float("nan")

    print("\n" + "=" * 74)
    print("LangevinFold summary — real-mesh self-collision & latency (paragraph-ready)")
    print("=" * 74)
    robots = sorted({r["robot"] for r in rows})
    for robot in robots:
        for scen in ["open_space", "near_singular", "cluttered"]:
            cell = [r for r in rows if r["robot"] == robot and r["scenario"] == scen
                    and r["solver"] in FIELD]
            if not cell:
                continue
            cell.sort(key=lambda r: (f(r, "pb_collision_pct")
                                     if f(r, "pb_collision_pct") == f(r, "pb_collision_pct")
                                     else 1e9))
            print(f"\n  {robot} / {scen}   (ranked by PyBullet collision %, cleanest first)")
            print(f"    {'solver':<14} {'succ%':>6} {'PBcol%':>7} {'MJcol%':>7} "
                  f"{'mean ms':>9} {'p99 ms':>9}")
            for r in cell:
                star = "*" if r["solver"] == "protein_raw" else " "
                print(f"  {star} {PAPER.get(r['solver'], r['solver']):<14} "
                      f"{f(r,'success_pct'):>6.1f} {f(r,'pb_collision_pct'):>7.1f} "
                      f"{f(r,'mj_collision_pct'):>7.1f} {f(r,'mean_ms'):>9.1f} "
                      f"{f(r,'p99_ms'):>9.1f}")
    print("\n  (* = LangevinFold. 'Cleanest' = lowest PB/MJ collision among ≥90%% success.)")
    print("=" * 74)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="LangevinFold-focused sim benchmark (small scale; reuses the master harness).")
    ap.add_argument("--trials", type=int, default=40,
                    help="trials/seed/cell (default 40; LangevinFold is ~s/solve, keep modest)")
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--scenarios", nargs="+",
                    default=["open_space", "near_singular", "cluttered"])
    ap.add_argument("--robots", nargs="+", default=["ur5"],
                    help="default ur5 — the arm where the paper claims LangevinFold wins")
    ap.add_argument("--out", default="results/langevin_bench", help="output stem")
    ap.add_argument("--quick", action="store_true", help="tiny smoke run (forwarded)")
    ap.add_argument("--resume", action="store_true", help="continue a crashed run (forwarded)")
    args, extra = ap.parse_known_args(argv)

    forward = [
        "--solvers", *FIELD,
        "--robots", *args.robots,
        "--scenarios", *args.scenarios,
        "--trials", str(args.trials),
        "--seeds", *map(str, args.seeds),
        "--out", args.out,
        "--no-native",   # PyBullet's native IK isn't part of the LangevinFold quality story
    ]
    if args.quick:
        forward.append("--quick")
    if args.resume:
        forward.append("--resume")
    forward += extra   # pass through --no-mujoco, --skip-validation, etc.

    rc = master_main(forward)
    if rc == 0:
        _print_summary(args.out + ".csv")
    return rc


if __name__ == "__main__":
    sys.exit(main())
