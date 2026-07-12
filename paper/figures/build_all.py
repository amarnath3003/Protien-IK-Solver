"""Build every figure and table in one shot.

By default builds the data-driven figures (read CSV/JSON) and the LaTeX tables.
Pass --with-solvers to ALSO build the two figures that run the solvers themselves
(qualitative fold, energy trace) -- those are slower and need the backend package
importable, so run this with the backend venv python.

Run:
    python build_all.py                 # CSV/JSON figures + tables
    python build_all.py --with-solvers  # + qualitative fold + energy trace
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

DATA_FIGS = ["fig_dof_scaling.py", "fig_success.py", "fig_latency.py",
             "fig_collision.py", "fig_validation.py", "fig_deployment.py"]
# fig_langevin.py is parked for future work (see backend/bench/langevin_benchmark.py);
# not part of the active paper build. Run it manually if/when needed.
SOLVER_FIGS = ["fig_qualitative_fold.py", "fig_energy_trace.py"]
TABLES = ["make_tables.py"]


def run(script: str) -> bool:
    print(f"\n=== {script} ===")
    r = subprocess.run([sys.executable, str(HERE / script)])
    if r.returncode != 0:
        print(f"  [FAILED] {script} (exit {r.returncode})")
        return False
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--with-solvers", action="store_true")
    args = ap.parse_args()

    scripts = DATA_FIGS + TABLES + (SOLVER_FIGS if args.with_solvers else [])
    ok = sum(run(s) for s in scripts)
    print(f"\nDone: {ok}/{len(scripts)} scripts succeeded.")
    if not args.with_solvers:
        print("(Skipped solver-driven figures; add --with-solvers to build them.)")


if __name__ == "__main__":
    main()
