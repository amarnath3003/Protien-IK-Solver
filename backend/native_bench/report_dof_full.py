"""Summarise ``dof_scaling_full.json`` into the numbers the paper actually quotes.

Reads the expanded DOF sweep (``run_dof_scaling_full.py``) and prints:
  * the main table -- clean% with Wilson 95% CI, per solver, n=1000/cell
  * nondeterminism -- per-solver spread across the R sweep repeats
  * contrasts -- ratio + Fisher exact p vs BOTH baselines
  * feasibility -- oracle-feasible fraction and the conditional clean rate

Run: PYTHONPATH=. python3 native_bench/report_dof_full.py \
        [--json results/dof_scaling/dof_scaling_full.json]
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

SHORT = {"protein_fast": "KineticFold", "trac_ik_style": "TRAC-IK",
         "multi_start": "Multi-start"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default="results/dof_scaling/dof_scaling_full.json")
    args = ap.parse_args()
    D = json.loads(Path(args.json).read_text(encoding="utf-8"))
    cells = D["cells"]
    cfg = D["config"]
    dofs = cfg["dofs"]

    def cell(dof, solver):
        return next(c for c in cells if c["dof"] == dof and c["solver"] == solver)

    print(f"n = {cfg['n_per_cell']}/cell, {cfg['repeats']} repeats, "
          f"seeds {cfg['seeds'][0]}-{cfg['seeds'][-1]}, wall {cfg['wall_time_s']}s\n")

    print("=" * 78)
    print("CLEAN-SOLVE RATE (%) with Wilson 95% CI   [repeat 1 of "
          f"{cfg['repeats']}]")
    print("=" * 78)
    print(f"{'DOF':>4} | {'KineticFold':>22} | {'TRAC-IK':>22} | {'Multi-start':>22}")
    print("-" * 78)
    for d in dofs:
        row = f"{d:>4} |"
        for s in ("protein_fast", "trac_ik_style", "multi_start"):
            c = cell(d, s)
            row += (f" {c['clean_pct']:6.2f} [{c['ci95_lo']:5.2f},"
                    f"{c['ci95_hi']:5.2f}] |")
        print(row)

    print("\n" + "=" * 78)
    print("NONDETERMINISM across the 5 sweep repeats (pp spread)")
    print("=" * 78)
    for s in ("protein_fast", "trac_ik_style", "multi_start"):
        spreads = [cell(d, s)["repeat_range_pp"] for d in dofs]
        det = all(cell(d, s)["deterministic"] for d in dofs)
        print(f"  {SHORT[s]:<12} max spread {max(spreads):5.2f} pp   "
              f"{'DETERMINISTIC (bit-exact)' if det else 'nondeterministic'}")
    print(f"\n  {'DOF':>4} | " + " | ".join(f"{SHORT[s]:>26}" for s in
                                            ("protein_fast", "trac_ik_style", "multi_start")))
    for d in dofs:
        row = f"  {d:>4} |"
        for s in ("protein_fast", "trac_ik_style", "multi_start"):
            c = cell(d, s)
            row += f" {c['repeat_mean']:6.2f} ({c['repeat_min']:5.2f}-{c['repeat_max']:5.2f}) |"
        print(row)

    print("\n" + "=" * 78)
    print("CONTRASTS -- KineticFold vs each baseline (Fisher exact, two-sided)")
    print("=" * 78)
    print(f"{'DOF':>4} | {'vs TRAC-IK':>28} | {'vs Multi-start':>28}")
    print("-" * 78)
    for d in dofs:
        row = f"{d:>4} |"
        for base in ("trac_ik_style", "multi_start"):
            k = next(c for c in D["contrasts"]
                     if c["dof"] == d and c["baseline"] == base)
            r = f"{k['ratio']:.2f}x" if k["ratio"] else "n/a"
            p = k["fisher_p"]
            ps = "<1e-12" if p < 1e-12 else f"{p:.2g}"
            sig = "*" if p < 0.05 else " "
            row += f" {r:>8}  p={ps:>8}{sig} |"
        print(row)

    if any("feasible_n" in c for c in cells):
        print("\n" + "=" * 78)
        print("FEASIBILITY -- does a clean solution exist at all?")
        print("(union oracle, lower bound on feasibility => conditional rate is an upper bound)")
        print("=" * 78)
        print(f"{'DOF':>4} | {'feasible':>9} | " +
              " | ".join(f"{SHORT[s]+' cond%':>22}" for s in
                         ("protein_fast", "trac_ik_style", "multi_start")))
        print("-" * 78)
        for d in dofs:
            c0 = cell(d, "protein_fast")
            if "feasible_n" not in c0:
                continue
            row = f"{d:>4} | {c0['feasible_pct']:7.2f}% |"
            for s in ("protein_fast", "trac_ik_style", "multi_start"):
                c = cell(d, s)
                v = c.get("conditional_clean_pct")
                if v is None:
                    row += f" {'--':>22} |"
                else:
                    row += (f" {v:6.2f} [{c['conditional_ci95_lo']:5.2f},"
                            f"{c['conditional_ci95_hi']:5.2f}] |")
            print(row)
        print("\n  HOW LOOSE IS THE BOUND -- single-shot clean solves the oracle "
              "alone failed to re-find\n  (these are unioned in, but they show the "
              "oracle misses real solutions; more misses = looser bound):")
        any_miss = False
        for d in dofs:
            parts = []
            for s in ("protein_fast", "trac_ik_style", "multi_start"):
                c = cell(d, s)
                m, k = c.get("clean_missed_by_oracle"), c["clean_k"]
                if m:
                    any_miss = True
                    parts.append(f"{SHORT[s]} {m}/{k}")
            if parts:
                print(f"    {d:>2}-DOF  " + ",  ".join(parts))
        if not any_miss:
            print("    none -- the oracle independently re-found every clean solve.")

    if D.get("replication"):
        bad = [r for r in D["replication"] if abs(r["delta"]) > 1e-9]
        print("\n" + "=" * 78)
        print(f"REPLICATION vs committed n=120: "
              f"{len(D['replication']) - len(bad)}/{len(D['replication'])} exact")
        for r in bad:
            print(f"  DRIFT {r['dof']:>2}-DOF {r['solver']:<15} "
                  f"{r['committed']:.3f} -> {r['subset']:.3f} ({r['delta']:+.3f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
