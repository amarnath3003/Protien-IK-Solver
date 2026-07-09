# Paper figures & tables — generators

Every figure and results table is generated from the committed benchmark files, so
after a fresh benchmark run they regenerate verbatim. Nothing is hand-transcribed.

**The one authoritative benchmark** is `backend/bench/master_sim_benchmark.py`
("solve once, score three ways" — PyBullet + MuJoCo), whose committed output is
`backend/results/master_10seed_fast.csv` (UR5 + Franka, seeds 1..10, n=1000/cell).
All success / speed / collision / validation figures read that single file. The
older `master_full.csv` is **not** authoritative and is no longer a default. The
DOF-scaling figure is the one exception — it comes from a *separate* experiment
(`usecase_experiments.py` → `usecase_results.json`), which the master sweep doesn't
cover (it needs planar N-DOF arms).

## What's here

| Script | Output | Reads | Priority |
| :-- | :-- | :-- | :-- |
| `fig_dof_scaling.py`     | `fig_dof_scaling.{pdf,png}`     | `usecase_results.json` (E) | **P0 flagship** |
| `fig_qualitative_fold.py`| `fig_qualitative_fold.{pdf,png}`| runs the solvers | **P0** |
| `fig_success.py`         | `fig_success.{pdf,png}`         | master CSV | P1 |
| `fig_latency_tail.py`    | `fig_latency_tail.{pdf,png}`    | master CSV | P1 |
| `fig_collision_ur5.py`   | `fig_collision_ur5.{pdf,png}`   | 10-seed collision CSV | P1 |
| `fig_validation.py`      | `fig_validation.{pdf,png}`      | master CSV | P2 |
| `fig_deployment.py`      | `fig_deployment.{pdf,png}`      | master CSV | P2 |
| `fig_energy_trace.py`    | `fig_energy_trace.{pdf,png}`    | runs a solve (`collect_steps`) | P2 |
| `make_tables.py`         | `../tables/tab_*.tex`           | master + collision CSV, usecase JSON | — |
| `../tables/tables_static.tex` | hand-authored T1–T4 (isomorphism, robots, thresholds, baselines) | — | — |

`_style.py` holds the shared style, the fixed **solver → colour** map (colour
follows the solver entity across every figure; Okabe–Ito colour-blind-safe), and the
CSV loader. It also holds the **code-id → paper-name** map:
`protein_ik→StagedFold`, `protein_fast→KineticFold`, `protein_raw→LangevinFold`.

## Setup

```bash
cd backend
.venv/Scripts/python -m pip install -r ../paper/figures/requirements-figures.txt   # matplotlib
```

`make_tables.py` needs neither matplotlib nor numpy (stdlib only), so tables build
even without the install above.

## Run

```bash
# from paper/figures, with the backend venv python (so `app` imports for the
# solver-driven figures):
python build_all.py                 # all CSV/JSON figures + all LaTeX tables
python build_all.py --with-solvers  # + qualitative fold + energy trace

# or individually, with explicit inputs:
python fig_dof_scaling.py   --json ../../backend/scrap/usecase_results.json
python fig_success.py       --csv  ../../backend/results/master_10seed_fast.csv
python fig_collision_ur5.py --csv  ../../backend/results/master_10seed_fast.csv
python make_tables.py       --csv ... --collision-csv ... --json ...
```

## Default input paths (override on the CLI)

- master benchmark (success / speed / collision / validation), from
  `bench/master_sim_benchmark.py`: `backend/results/master_10seed_fast.csv`
- DOF scaling, from `usecase_experiments.py`: `backend/scrap/usecase_results.json`

Point the flags at whatever `--out` stem you actually run
(`master_sim_benchmark.py --out results/<stem>` writes `<stem>.csv`).

## LaTeX preamble

The generated tables use `booktabs`; the static tables also use `makecell`/`array`:

```latex
\usepackage{booktabs}
\usepackage{makecell}
\usepackage{array}
```

Figures are vector PDF — include with `\includegraphics[width=\columnwidth]{figures/fig_dof_scaling.pdf}`.
