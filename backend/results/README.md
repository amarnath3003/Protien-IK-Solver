# Committed results — which file backs which claim

Everything the paper asserts numerically comes from a file in this directory. This
page is the map. Nothing in the paper is hand-transcribed: the figures and LaTeX
tables in [`paper/figures`](../../paper/figures) read these files directly, so
regenerating them reproduces the manuscript's numbers verbatim.

Each sweep writes four files off one output stem:

| Extension | Contents |
| :-- | :-- |
| `.csv` | one row per (arm, scenario, solver) cell — every metric, machine-readable; this is what the figure/table scripts read |
| `.md` | the same sweep as a human-readable report, plus the oracle-validation blocks and the per-cell verdict |
| `.manifest.json` | run provenance: config, seeds, cell count, wall time, and library versions (Python / NumPy / PyBullet / MuJoCo) |
| `logs/*.run.log` | the console transcript of the run that produced it |

## The two authoritative sweeps

Both are the *native* re-run: every solver is either a genuine upstream library or a
C++/Eigen port, so the latency columns are apples-to-apples (see
[`../native_bench/README.md`](../native_bench/README.md)). Produced by
`bench/master_sim_benchmark.py` via `native_bench/run_native_master.py`, under WSL
Ubuntu 22.04 / Python 3.10.12 / NumPy 1.26.4 / PyBullet 202010061 / MuJoCo 3.10.0.

### `master_full(cpp).*` — the 3-seed survey

100 trials × seeds `[1, 2, 3]` = **n = 300 per cell**, warm-up 8 untimed solves,
**all three arms** (planar 3-DOF, UR5, Franka Panda) × three scenarios, 99 cells.

Backs: **§5.1 success**, **§5.2 latency (mean/p50/p95/p99)**, **§5.3's planar-arm
paragraph**, **§5.5 deployment roles**, and the FK/collision oracle-agreement blocks
of **§4.6 / §5.6**. It is the only sweep that carries the planar arm.

### `master_10seed_fast(cpp).*` — the 10-seed collision sweep

100 trials × seeds `[1 … 10]` = **n = 1000 per cell**, **UR5 + Franka only** (the two
arms with a manufacturer URDF, hence a real-mesh oracle), 66 cells.

Backs: every **real-mesh self-collision** number in **§5.3** and **Figure 4**.
Collision is the seed-sensitive measure — it moves about twice as much between draws
as success does — which is why it gets the extra seeds (§4.4).

## The DOF-scaling study — `dof_scaling/`

A separate experiment, because the master sweep does not cover planar N-DOF arms.

### `dof_scaling_full.json` — the authoritative sweep (**Table 5**, **Figure 5**)

**n = 1000 per cell** (seeds `[1 … 10]` × 100, the same 10-seed protocol the collision
sweep uses — clean-solve is collision-gated, hence the seed-sensitive measure),
**seven** DOF points (4, 6, 8, 10, 12, 14, 16), and **three** solvers: the C++/Eigen
KineticFold port, genuine TRAC-IK (`tracikpy`), and genuine Robotics Toolbox
Multi-start (`ik_LM`) — the *stronger* baseline, whose absence §5.7 used to confess.
Produced by `native_bench/run_dof_scaling_full.py`; summarise with
`native_bench/report_dof_full.py`.

It carries four things the n = 120 pilot could not support:

- **Wilson 95% intervals** per cell, correct near 0% where the normal approximation
  crosses the boundary.
- **Five full sweep repeats.** The per-solve RNG depends only on `(seed, index)`, so
  it is identical across repeats and any movement is *purely* the wall-clock
  nondeterminism of the two library baselines. Measured, not assumed: KineticFold is
  bit-identical across all five; TRAC-IK moves up to ~1.2 pp and Multi-start up to
  ~3.3 pp.
- **Fisher exact** per cell against *both* baselines.
- **A union feasibility oracle** — every target attacked from its own `q0` plus
  hundreds of random restarts by all three solvers, to separate "the solver failed"
  from "no clean solution exists". Feasible counts are lower bounds (a `True` is a
  proof; a `False` is only failure-to-find), and the per-cell field
  `clean_missed_by_oracle` reports how loose that bound is by counting single-shot
  clean solves the oracle could not re-find.

Its replication block re-derives the committed n = 120 numbers exactly: seeds 1–2 ×
the first 60 targets are literally the pilot's trial set, and KineticFold reproduces
all five DOF cells bit-for-bit. TRAC-IK does not — which is the point.

### `dof_scaling_native.json` — the superseded n = 120 pilot

`bench/usecase_experiments.py` EXP E: **n = 120 per cell**, 4/6/8/12/16 joints,
KineticFold vs. TRAC-IK only. Kept reproducible, but **not** a source for paper
numbers: its 16-DOF cell is a single solve, and its "peak at 8 DOF" shape does not
survive n = 1000.

| File | Backs |
| :-- | :-- |
| `dof_scaling_full.json` | **Table 5**, **Figure 5**, all of **§5.4** (rows 4–14 DOF). |
| `dof_scaling_16dof_n5000.json` | **Table 5's 16-DOF row**, at `n = 5000`. Clean solves are rare events there: at `n = 1000` the KineticFold/Multi-start contrast was not significant (`p = 0.21`) and TRAC-IK read as an exact 0/1000, both sampling artifacts. The table and figure generators read this file *alongside* the sweep above and take the larger-n row per cell (`_style.load_dof`), so neither committed file is ever hand-edited. |
| `dof_scaling_full_pass1.json` | the same sweep with a **48-restart** oracle instead of 384 — backs §5.4's sentence that the weaker oracle put 16-DOF feasibility at 24.2% and looked like a geometric floor. Reproduce with `--oracle-scale 1`. |
| `dof_scaling_native.json` | superseded pilot; produced by `native_bench/run_native_usecase.py --only E`. |
| `dof_scaling_sim_scored.json` | PyBullet/MuJoCo re-scoring — still at the pilot's `n = 120` (§5.4's last paragraph says so explicitly). |
| `dof_scaling_sim_scored.json` | **§5.4's last paragraph** — the same sweep re-scored in PyBullet *and* MuJoCo through generated planar URDFs (`app/sim/planar_model.py`), under two collision solids: `capsule` (equals the proxy to ~1e-16 **by construction**, so it validates the implementation, not the geometry) and `cylinder` (a genuinely different idealisation; the proxy is conservative by 1–6 pp and every ranking holds). Produced by `native_bench/run_dof_sim_scored.py`. |
| `dof_scaling_native.log` | console transcript of the native run |

## Oracle validation — `validation/`

Two dedicated runs establish that the three evaluators are the same robot before any
solver is compared. They are the source of §4.6's agreement figures and §5.6's
"the proxy is optimistic" result — the master sweeps also carry their own
smaller-`n` agreement block, reproduced at the top of each `.md`.

| File | Backs | Produced by |
| :-- | :-- | :-- |
| `validation/sim_crosscheck.md` / `.csv` | **§4.6**: three-way FK agreement (`n=2000`/arm: DH↔PyBullet 9.5e-7 m UR5 / 6.6e-7 m Franka; DH↔MuJoCo 4.2e-8 / 8.7e-16; PyBullet↔MuJoCo 4.11e-8 / 5.90e-8 m) and collision agreement (`n=3000`/arm: PB↔MJ sign-agree 97.8% UR5 / 99.0% Franka, correlation 0.991 / 0.880) — Eqs. 24–25 | `bench/sim_crosscheck.py` |
| `validation/collision_parity.md` | **§5.6**: the capsule proxy is systematically *optimistic* — over `n=3000` random configs the real meshes collide 36.5% (UR5) / 9.9% (Franka) of the time against the proxy's 16.9% / 0.5%, a 20.2 / 9.5 pp "false-clear" band the solvers cannot see. This is why collision is reported as a ranking, never as an absolute rate. | `bench/collision_parity.py` |

## Solver ids → paper names

The CSV/Markdown columns use code ids. The paper renames the three ProteinIK solvers:

| Code id | Paper name | In the paper's compared field? |
| :-- | :-- | :-- |
| `protein_ik` | **StagedFold** | yes |
| `protein_fast` | **KineticFold** | yes |
| `protein_raw` | **LangevinFold** | **no** — future work (§6); see the note below |
| `trac_ik_style` | TRAC-IK (genuine TRACLabs C++) | yes |
| `multi_start` | Multi-start (genuine Robotics Toolbox `ik_LM` + restarts) | yes |
| `jacobian_dls` | Jacobian-DLS (genuine Robotics Toolbox `ik_LM`, single-shot) | yes |
| `ccd`, `fabrik` | CCD, FABRIK (in-repo algorithm, native C++ port) | yes |
| `analytical_planar3dof` | Analytical (exact closed form, planar only) | ground-truth validator |
| `protein_fast_o2`, `protein_fast_calib` | KineticFold variants (IAM warm-start; calibrated capsule radii) | ablation rows |
| `PyBullet native IK` | the simulator's own IK on the identical targets | reference column |

**Note on the `protein_raw` / LangevinFold rows.** The sweeps score LangevinFold too,
and its rows are in both CSVs — but the paper treats it as **future work** (§6) and
excludes it from the evaluated field, so no figure plots it and no §5 claim is made
about it. It is not in that field for a reason: it costs 13–22 ms per solve against
KineticFold's 0.1–0.7 ms, two orders of magnitude off real-time. Read literally, the
`.md` verdict tables ("lowest real-mesh-collision solver per cell") do name it on the
UR5 cells, where it posts the lowest collision of anything in the file. §5.3's
"cleanest of the field" is therefore a statement about the compared field, not about
every row of the CSV.

Homotopy (`protein_homotopy` / CCH-IK) and `fixed_lambda_ik` are excluded from the
native sweeps outright — the code ships (`app/solvers/protein_homotopy/`) and their
development record is archived under
[`docs/archive/v5-cchik/`](../../docs/archive/v5-cchik), but they are not part of this
paper.

## Column glossary

| Column | Meaning |
| :-- | :-- |
| `Succ%` | single-shot success on our DH core: ‖Δp‖ < 1 mm **and** ‖Δω‖ < 10 mrad |
| `PB succ% / MJ succ%` | the same configuration re-scored by PyBullet / MuJoCo real-mesh FK |
| `Mean ms`, `p95`, `p99` | wall-clock per solve, pooled over all seeds in the cell (not per-seed means) |
| `Iters` | solver iterations to termination |
| `PB col% / MJ col%` | real-mesh self-collision rate over non-adjacent link pairs, per engine |
| `PB clr / MJ clr` | mean signed clearance (m); negative = interpenetration depth |
| `PB pos mm` | end-effector position residual measured in the engine's mesh frame |
| `JLV` | joint-limit violations per trial |
| `–` | not applicable (planar arm has no URDF → no mesh oracle; PyBullet native IK has no DH-core row) |

## Archive — superseded, kept for provenance

| Path | What it is |
| :-- | :-- |
| `archive/python-run/` | the pre-native sweeps (`master_full.*`, `master_10seed_fast.*`), where the ProteinIK solvers and CCD/FABRIK were still interpreted Python. Same algorithms, ~10–500× slower wall-clock, so the latency columns are **not** comparable across solvers. Superseded by the `(cpp)` files. |
| `archive/native-intermediate/` | staging runs from the native port (`quick`, `quick2`, `cppquick`, `master_full_native`) — the intermediates that were folded into `master_full(cpp)`. |
| `archive/` (top level) | one-off probes and calibration runs from development: collision-radius calibration, o2 variant tests, per-arm oracle spot-checks, early timestamped master sweeps. |

No **result** in `archive/` backs a paper claim. If a number in the manuscript
disagrees with a file in there, the `(cpp)` file is right and the archived one is
stale. (The Phase-2 single-engine oracle sweeps — `archive/sim_oracle_*` — and their
runner `backend/archive/sim_benchmark.py` are the development record of the validation
work whose final artifacts are in `validation/`.)
