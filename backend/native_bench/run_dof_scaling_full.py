"""DOF-scaling sweep, expanded — the statistically-powered replacement for EXP E.

Why this file exists instead of a bigger ``usecase_experiments.exp_E``
---------------------------------------------------------------------
The committed Table 5 (``results/dof_scaling/dof_scaling_native.json``) runs
``n = 120`` per cell against a single baseline. Three problems, all real:

1. **The 16-DOF headline is one solve.** 0.833% of 120 = 1 clean solve. The
   paper's "KineticFold is the only one still producing collision-free
   solutions" rests on that single trial.
2. **TRAC-IK is wall-clock budgeted, hence nondeterministic**, and the drift is
   already visible *between two committed artifacts* that run the identical
   loop with the identical target stream and per-solve RNG: 8-DOF TRAC-IK is
   13.33% in ``dof_scaling_native.json`` and 10.83% in
   ``dof_scaling_sim_scored.json``, i.e. the "peak ratio" is 3.2x or 4.0x
   depending on which file you read. The paper claims repeats bound it to
   3.2-3.7x. They do not.
3. **One baseline.** Section 5.7 already confesses this. ``multi_start``
   (genuine Robotics Toolbox ``ik_LM``) is wired into the native registry and
   is the *other* production baseline the planar 3-DOF result in Section 5.3
   compares against, so the DOF sweep omitting it is an internal inconsistency.

``exp_E`` is left untouched so the committed Table 5 stays reproducible; this
runner reuses its exact target stream and per-solve RNG, so the original 120
trials are a strict *subset* of the new 2000 for seeds 1-2 and must reproduce
verbatim. That is asserted, not assumed (``--check-replication``).

What it produces
----------------
* **Main sweep** -- DOF in {4,6,8,10,12,14,16} (10 and 14 are new; the old
  "peak around 8" was interpolated across a 8->12 gap), 3 solvers, seeds 1-10 x
  n=100 = **n = 1000 per cell**, matching the 10-seed protocol already used for
  the collision sweep (collision-derived measures are the seed-sensitive ones,
  and clean-solve is collision-gated).
* **Repeats** -- the whole sweep R times. Per-solve RNG is identical across
  repeats, so any drift is *purely* wall-clock nondeterminism. This measures
  which solvers are deterministic rather than assuming it.
* **Feasibility oracle** -- the honest version of "is 16 DOF even solvable?".
  Uniform rejection sampling cannot answer this: a random draw in a 16-D joint
  space never lands within 1mm/10mrad of a specific target, so it would report
  0/1e6 at every DOF and measure dimensionality, not feasibility. Instead each
  target gets a large multi-restart budget spread across **all three** solvers
  (a union oracle -- using KineticFold alone would define feasibility in terms
  of the solver under test). A target is FEASIBLE if any restart returns a
  clean solve. This is a **lower bound** on true feasibility, so the conditional
  rate clean/feasible is an **upper bound**. Both are reported as such.

Run (inside WSL Ubuntu-2204, from backend/):
    export ROBOT_DESCRIPTIONS_CACHE="$HOME/.cache/robot_descriptions"
    PYTHONPATH=. python3 native_bench/run_dof_scaling_full.py \
        --out results/dof_scaling/dof_scaling_full.json
"""
from __future__ import annotations

import argparse
import json
import math
import platform
import sys
import time
from pathlib import Path

import numpy as np

# Side effect: swaps SOLVER_REGISTRY to genuine TRAC-IK / RTB multi-start and the
# native C++/Eigen KineticFold port. Importing this IS the native setup.
import native_bench.run_native_usecase  # noqa: F401

import usecase_experiments as U  # noqa: E402  (path set by the import above)
from app.solvers.registry import run_solver, SOLVER_DISPLAY_NAMES  # noqa: E402

DOFS = [4, 6, 8, 10, 12, 14, 16]
SOLVERS = ["protein_fast", "trac_ik_style", "multi_start"]
SEEDS = list(range(1, 11))
N_PER_SEED = 100
REPEATS = 5

# Union-oracle restart budget per target, split across solvers. KineticFold is
# cheapest per solve so it carries the most restarts; the two baselines are
# included so feasibility is not defined by the solver under test.
ORACLE_RESTARTS = {"protein_fast": 24, "trac_ik_style": 12, "multi_start": 12}

Z95 = 1.959963984540054


def wilson(k: int, n: int, z: float = Z95) -> tuple[float, float]:
    """Wilson score interval, in percent. Correct near 0% and 100%, where the
    normal approximation produces intervals that cross the boundary."""
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (100.0 * max(0.0, centre - half), 100.0 * min(1.0, centre + half))


def fisher_p(k1: int, n1: int, k2: int, n2: int) -> float:
    """Two-sided Fisher exact on the 2x2 clean/dirty contingency table."""
    from scipy.stats import fisher_exact
    return float(fisher_exact([[k1, n1 - k1], [k2, n2 - k2]])[1])


def targets_for(spec, seed: int, n: int):
    """exp_E's target stream, verbatim: one generator per seed, n draws.

    Drawing n=100 from default_rng(1000+seed) yields the original n=60 run's
    targets as its first 60 elements, which is what makes the replication
    check below possible."""
    g = np.random.default_rng(1000 + seed)
    return [U.generate_target(spec, g, "cluttered") for _ in range(n)]


def run_cell(spec, solver: str, seeds, n: int):
    """One (dof, solver) cell. Returns per-trial booleans, not just counts, so
    the feasibility join and the exact tests can be computed downstream."""
    solved_flags, clean_flags = [], []
    for seed in seeds:
        for i, (q0, T) in enumerate(targets_for(spec, seed, n)):
            # identical to exp_E: per-solve RNG depends only on (seed, i), so it
            # is stable across repeats and any drift is wall-clock, not sampling
            rng = np.random.default_rng(seed * 1_000_003 + i)
            r = run_solver(solver, spec, q0, T, rng)
            solved_flags.append(bool(U.solved(r)))
            clean_flags.append(bool(U.clean(r)))
    return solved_flags, clean_flags


def oracle_feasible(spec, seeds, n: int, restarts: dict, rng_base: int = 90210):
    """Union oracle: does ANY clean solution exist for each target?

    Each target is attacked by all three solvers -- first from the target's own
    q0 (the start the single-shot sweep uses, so the oracle can never be weaker
    than the sweep for that seed), then from many fresh random restarts.
    Returns a flat list of booleans aligned with run_cell's trial order.

    Direction of the bound: a True is *proof* the target admits a clean solve;
    a False is failure-to-find, not proof of impossibility. Feasible counts are
    therefore lower bounds and conditional rates upper bounds. The caller
    additionally unions in every single-shot clean solve observed in the main
    sweep, and reports how many of those the oracle missed on its own --
    a direct, quantitative measure of how loose the bound is per cell.
    """
    feasible = []
    for seed in seeds:
        for i, (q0, T) in enumerate(targets_for(spec, seed, n)):
            g = np.random.default_rng(rng_base + seed * 7919 + i)
            ok = False
            for solver, k in restarts.items():
                # seed 0 = the sweep's own start config, then k random restarts
                for attempt in range(k + 1):
                    q_start = q0 if attempt == 0 else spec.random_config(g)
                    try:
                        r = run_solver(solver, spec, q_start, T,
                                       np.random.default_rng(g.integers(1 << 62)))
                    except Exception:
                        continue
                    if U.clean(r):
                        ok = True
                        break
                if ok:
                    break
            feasible.append(ok)
    return feasible


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="results/dof_scaling/dof_scaling_full.json")
    ap.add_argument("--dofs", type=int, nargs="+", default=DOFS)
    ap.add_argument("--seeds", type=int, nargs="+", default=SEEDS)
    ap.add_argument("--n", type=int, default=N_PER_SEED)
    ap.add_argument("--repeats", type=int, default=REPEATS)
    ap.add_argument("--no-oracle", action="store_true")
    ap.add_argument("--oracle-dofs", type=int, nargs="+", default=None,
                    help="restrict the feasibility oracle to these DOF (default: all)")
    ap.add_argument("--oracle-scale", type=float, default=1.0,
                    help="multiply the per-solver restart budget (high DOF needs more: "
                         "at 16 DOF the 48-restart oracle missed 8 of KineticFold's 11 "
                         "single-shot clean solves, i.e. the bound was demonstrably loose)")
    ap.add_argument("--merge-oracle", default=None,
                    help="path to an earlier run whose feasibility flags should be "
                         "OR-ed in (feasibility is a union of proofs, so merging two "
                         "oracle passes is valid and only tightens the bound)")
    args = ap.parse_args(argv)

    t_start = time.perf_counter()
    n_per_cell = len(args.seeds) * args.n
    print(f"[dof-full] DOF={args.dofs} solvers={SOLVERS}", flush=True)
    print(f"[dof-full] n={n_per_cell}/cell ({len(args.seeds)} seeds x {args.n}), "
          f"repeats={args.repeats}", flush=True)

    specs = {d: U.planar_ndof_spec(d) for d in args.dofs}

    # ---- main sweep, repeated ------------------------------------------------
    # trials[(dof, solver, repeat)] = (solved_flags, clean_flags)
    trials = {}
    for rep in range(args.repeats):
        print(f"\n=== repeat {rep + 1}/{args.repeats} ===", flush=True)
        for dof in args.dofs:
            line = f"  [planar {dof:>2}-DOF]"
            for solver in SOLVERS:
                sv, cl = run_cell(specs[dof], solver, args.seeds, args.n)
                trials[(dof, solver, rep)] = (sv, cl)
                line += f"  {solver}: {100.0 * sum(cl) / len(cl):5.2f}%"
            print(line, flush=True)

    # ---- replication check against the committed n=120 run --------------------
    # seeds 1,2 x first 60 targets are exactly the original EXP E trial set.
    replication = None
    if 1 in args.seeds and 2 in args.seeds and args.n >= 60:
        committed = {  # dof -> {solver: clean_pct} from dof_scaling_native.json
            4: {"protein_fast": 71.66666666666667, "trac_ik_style": 36.666666666666664},
            6: {"protein_fast": 63.33333333333333, "trac_ik_style": 23.333333333333332},
            8: {"protein_fast": 43.333333333333336, "trac_ik_style": 13.333333333333334},
            12: {"protein_fast": 7.5, "trac_ik_style": 3.3333333333333335},
            16: {"protein_fast": 0.8333333333333334, "trac_ik_style": 0.0},
        }
        i1, i2 = args.seeds.index(1), args.seeds.index(2)
        replication = []
        for dof, exp in committed.items():
            if dof not in args.dofs:
                continue
            for solver, want in exp.items():
                _sv, cl = trials[(dof, solver, 0)]
                sub = ([cl[i1 * args.n + j] for j in range(60)]
                       + [cl[i2 * args.n + j] for j in range(60)])
                got = 100.0 * sum(sub) / len(sub)
                replication.append(dict(dof=dof, solver=solver, committed=want,
                                        subset=got, delta=got - want))
        print("\n=== replication vs committed n=120 (seeds 1-2, first 60 each) ===",
              flush=True)
        for r in replication:
            flag = "OK " if abs(r["delta"]) < 1e-9 else "DRIFT"
            print(f"  {flag} {r['dof']:>2}-DOF {r['solver']:<15} "
                  f"committed {r['committed']:6.3f}  now {r['subset']:6.3f} "
                  f"({r['delta']:+.3f})", flush=True)

    # ---- feasibility oracle ---------------------------------------------------
    prior = {}
    if args.merge_oracle:
        pj = json.loads(Path(args.merge_oracle).read_text(encoding="utf-8"))
        prior = {int(k): v for k, v in pj.get("feasibility_flags", {}).items()}
        print(f"\n[dof-full] merging oracle flags from {args.merge_oracle} "
              f"(DOF {sorted(prior)})", flush=True)

    feasibility = {}
    oracle_only = {}
    if not args.no_oracle:
        odofs = args.oracle_dofs if args.oracle_dofs is not None else args.dofs
        restarts = {s: int(round(k * args.oracle_scale))
                    for s, k in ORACLE_RESTARTS.items()}
        print(f"\n=== feasibility oracle (union, "
              f"{sum(restarts.values())} restarts/target + own q0) ===", flush=True)
        for dof in odofs:
            t0 = time.perf_counter()
            fl = oracle_feasible(specs[dof], args.seeds, args.n, restarts)
            # feasibility is a union of existence proofs, so OR-ing passes is valid
            if dof in prior and len(prior[dof]) == len(fl):
                fl = [a or b for a, b in zip(fl, prior[dof])]
            oracle_only[dof] = list(fl)
            # every single-shot clean solve is itself a proof of feasibility
            merged = list(fl)
            for solver in SOLVERS:
                _sv, cl = trials[(dof, solver, 0)]
                merged = [a or b for a, b in zip(merged, cl)]
            feasibility[dof] = merged
            add = sum(merged) - sum(fl)
            lo, hi = wilson(sum(merged), len(merged))
            print(f"  [planar {dof:>2}-DOF] feasible {sum(merged):>4}/{len(merged)} = "
                  f"{100.0 * sum(merged) / len(merged):6.2f}%  [{lo:.2f}, {hi:.2f}]  "
                  f"(oracle alone {sum(fl)}, +{add} from single-shot; "
                  f"{time.perf_counter() - t0:.0f}s)", flush=True)

    # ---- assemble ------------------------------------------------------------
    cells = []
    for dof in args.dofs:
        for solver in SOLVERS:
            per_rep = []
            for rep in range(args.repeats):
                _sv, cl = trials[(dof, solver, rep)]
                per_rep.append(100.0 * sum(cl) / len(cl))
            sv0, cl0 = trials[(dof, solver, 0)]
            k, n = sum(cl0), len(cl0)
            lo, hi = wilson(k, n)
            row = dict(
                dof=dof, solver=solver, n=n,
                solved_pct=100.0 * sum(sv0) / len(sv0),
                clean_pct=100.0 * k / n, clean_k=k,
                ci95_lo=lo, ci95_hi=hi,
                repeats_pct=per_rep,
                repeat_mean=float(np.mean(per_rep)),
                repeat_min=float(np.min(per_rep)),
                repeat_max=float(np.max(per_rep)),
                repeat_range_pp=float(np.max(per_rep) - np.min(per_rep)),
                deterministic=bool(np.ptp(per_rep) == 0.0),
            )
            if dof in feasibility:
                fl = feasibility[dof]
                nf = sum(fl)
                # union includes every single-shot clean solve, so k of k are
                # feasible by construction; the conditional rate is clean/feasible
                row["feasible_n"] = nf
                row["feasible_pct"] = 100.0 * nf / len(fl)
                # how many of this solver's clean solves the *oracle alone* missed:
                # a direct, per-cell measure of how loose the lower bound is
                only = oracle_only.get(dof)
                row["clean_missed_by_oracle"] = (
                    int(sum(1 for c, f in zip(cl0, only) if c and not f))
                    if only else None)
                row["conditional_clean_pct"] = (100.0 * k / nf) if nf else None
                clo, chi = wilson(k, nf) if nf else (None, None)
                row["conditional_ci95_lo"], row["conditional_ci95_hi"] = clo, chi
            cells.append(row)

    # pairwise Fisher exact vs each baseline, on repeat 0
    contrasts = []
    for dof in args.dofs:
        _s, kf_cl = trials[(dof, "protein_fast", 0)]
        for base in ("trac_ik_style", "multi_start"):
            _s2, b_cl = trials[(dof, base, 0)]
            k1, n1 = sum(kf_cl), len(kf_cl)
            k2, n2 = sum(b_cl), len(b_cl)
            contrasts.append(dict(
                dof=dof, baseline=base, kf_k=k1, kf_n=n1, base_k=k2, base_n=n2,
                kf_pct=100.0 * k1 / n1, base_pct=100.0 * k2 / n2,
                ratio=(k1 / k2) if k2 else None,
                fisher_p=fisher_p(k1, n1, k2, n2),
            ))

    out = dict(
        config=dict(dofs=args.dofs, solvers=SOLVERS, seeds=args.seeds,
                    n_per_seed=args.n, n_per_cell=n_per_cell,
                    repeats=args.repeats,
                    oracle_restarts=(None if args.no_oracle else ORACLE_RESTARTS),
                    scenario="cluttered",
                    display_names={s: SOLVER_DISPLAY_NAMES.get(s, s) for s in SOLVERS},
                    python=platform.python_version(), numpy=np.__version__,
                    wall_time_s=None),
        cells=cells,
        contrasts=contrasts,
        replication=replication,
        feasibility_flags={str(d): v for d, v in feasibility.items()},
        oracle_only_flags={str(d): v for d, v in oracle_only.items()},
        feasibility_note=("Union oracle over all three solvers "
                          "(own q0 + random restarts), OR-ed with every single-shot "
                          "clean solve observed in the sweep. A True is proof of "
                          "feasibility; a False is failure-to-find. Feasible counts "
                          "are LOWER bounds and conditional rates UPPER bounds. "
                          "'clean_missed_by_oracle' reports, per cell, how many clean "
                          "solves the oracle alone failed to re-find -- the direct "
                          "measure of how loose the bound is."),
    )
    out["config"]["wall_time_s"] = round(time.perf_counter() - t_start, 1)

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)
    print(f"\nDONE in {out['config']['wall_time_s']}s -> {args.out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
