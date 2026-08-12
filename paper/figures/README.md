# Paper figures & tables — generators

Every figure and results table is generated from the committed benchmark files, so
after a fresh benchmark run they regenerate verbatim. Nothing is hand-transcribed.

**The authoritative benchmark** is `backend/bench/master_sim_benchmark.py` ("solve
once, score three ways" — capsule proxy + PyBullet + MuJoCo), run through
`backend/native_bench/run_native_master.py` so every solver is native compiled code.
It has two committed sweeps, and figures read whichever one the paper reads:

| Source file | What it carries | Used by |
| :-- | :-- | :-- |
| `backend/results/master_full(cpp).csv` | 3 seeds, `n=300`/cell, **all three arms** — success + latency | success, latency, validation, deployment |
| `backend/results/master_10seed_fast(cpp).csv` | 10 seeds, `n=1000`/cell, UR5 + Franka — **real-mesh collision** | collision |
| `backend/results/dof_scaling/dof_scaling_native.json` | separate DOF-scaling study, `n=120`/cell, planar 4→16 DOF | DOF scaling (flagship) |

The pre-native Python runs under `backend/results/archive/python-run/` are
**superseded** and must never be used for paper numbers. See
[`backend/results/README.md`](../../backend/results/README.md) for the full
claim → file map.

## What's here

| Script | Output | Reads | In `academic.md` |
| :-- | :-- | :-- | :-- |
| `fig_pipeline.svg` → `.pdf`/`.png` | KineticFold's compute schedule | hand-authored vector art | **Fig. 1** |
| `fig_success.py` | `fig_success.{pdf,png}` | 3-seed survey CSV | **Fig. 2** |
| `fig_latency.py` | `fig_latency.{pdf,png}` | 3-seed survey CSV | **Fig. 3** |
| `fig_collision.py` | `fig_collision.{pdf,png}` | 10-seed collision CSV | **Fig. 4** |
| `fig_dof_scaling.py` | `fig_dof_scaling.{pdf,png}` | DOF-scaling JSON (key `E`) | §5.4 currently ships as **Table 5**; the figure is built and available |
| `fig_validation.py` | `fig_validation.{pdf,png}` | 3-seed survey CSV (FK / collision agreement) | §4.6/5.6 report it in prose |
| `fig_deployment.py` | `fig_deployment.{pdf,png}` | 3-seed survey CSV (clean-goal yield) | §5.5 reports it in prose |
| `fig_qualitative_fold.py` | `fig_qualitative_fold.{pdf,png}` | runs the solvers | not placed |
| `fig_energy_trace.py` | `fig_energy_trace.{pdf,png}` | runs one solve (`collect_steps=True`) | not placed |
| `fig_langevin.py` | `fig_langevin.{pdf,png}` | `langevin_bench.csv` | **parked** (future-work solver) |
| `make_tables.py` | `../tables/tab_*.tex` | survey + collision CSV, DOF JSON | Table 5 + supplements |
| `../tables/tables_static.tex` | hand-authored Tables 1–4 (isomorphism, robots, thresholds, baselines) | — | Tables 1–4 |

The four "not placed / in prose" figures are kept generated deliberately: they back
claims the current draft states in text, and are the drop-ins if a reviewer asks to
see them plotted.

`_style.py` holds the shared style, the fixed **solver → colour** map (colour follows
the solver entity across every figure; Okabe–Ito colour-blind-safe), the default input
paths, and the **code-id → paper-name** map:

| Code id | Paper name |
| :-- | :-- |
| `protein_ik` | StagedFold |
| `protein_fast` | KineticFold |
| `protein_raw` | LangevinFold (future work) |

## Setup

```bash
cd backend
.venv/Scripts/python -m pip install -r ../paper/figures/requirements-figures.txt   # matplotlib
```

`make_tables.py` needs neither matplotlib nor numpy (stdlib only), so the tables
build even without that install.

## Run

```bash
# from paper/figures, with the backend venv python (so `app` imports for the
# two solver-driven figures):
python build_all.py                 # every CSV/JSON figure + all LaTeX tables
python build_all.py --with-solvers  # + qualitative fold + energy trace

# or individually, with explicit inputs:
python fig_dof_scaling.py --json ../../backend/results/dof_scaling/dof_scaling_native.json
python fig_success.py     --csv  "../../backend/results/master_full(cpp).csv"
python fig_collision.py   --csv  "../../backend/results/master_10seed_fast(cpp).csv"
python make_tables.py     --csv ... --collision-csv ... --json ...
```

`fig_pipeline` is the one non-generated figure: it is hand-authored vector art
(`fig_pipeline.svg`, exported to `.pdf`/`.png`), with its content spec kept beside it
in `fig_pipeline_SPEC.md` / `fig_pipeline_CONTEXT.md` / `fig_pipeline_PROMPT.md`.

## LangevinFold mini-benchmark (parked)

LangevinFold (`protein_raw`) costs ~seconds/solve, so it is excluded from the master
sweep and gets its own small run — scored the *same* three-way way (it reuses the
master harness). It is future work in the current paper (§6), so `build_all.py` does
not build its figure. To run it:

```bash
cd backend
PYTHONPATH=. .venv-sim/Scripts/python -m bench.langevin_benchmark   # -> results/langevin_bench.{csv,md}
# then, from paper/figures:
python fig_langevin.py       # -> fig_langevin.{pdf,png}
python make_tables.py        # -> ../tables/tab_langevin.tex  (auto-picks up langevin_bench.csv)
```

`fig_langevin.py` / `tab_langevin` self-skip with a message until
`langevin_bench.csv` exists, so nothing fails when the run hasn't happened yet.

## LaTeX preamble

The generated tables use `booktabs`; the static tables also use `makecell`/`array`:

```latex
\usepackage{booktabs}
\usepackage{makecell}
\usepackage{array}
```

Figures are vector PDF — include with
`\includegraphics[width=\columnwidth]{figures/fig_dof_scaling.pdf}`.
