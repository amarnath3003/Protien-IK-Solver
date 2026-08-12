# Reproducing the paper

Every number, figure and table in [`paper/academic.md`](../paper/academic.md) is
regenerable from this repository. This page is the end-to-end recipe, from a clean
checkout to a rebuilt manuscript.

There are three levels of reproduction, in increasing cost:

| Level | What you get | Needs |
| :-- | :-- | :-- |
| **A — rebuild the paper** | every figure and LaTeX table, from the committed result files | Python + matplotlib, ~1 minute |
| **B — re-run the benchmarks** | the result files themselves | Linux/WSL, the genuine baseline libraries, a C++ toolchain, ~1–2 hours |
| **C — explore interactively** | live solver comparison in the browser or a MuJoCo window | Node + the sim extras |

If you only want to check that the manuscript matches its data, do **A**.

---

## Level A — rebuild every figure and table

```bash
python -m venv .venv-fig
.venv-fig/Scripts/python -m pip install -r paper/figures/requirements-figures.txt   # matplotlib
cd paper/figures
../../.venv-fig/Scripts/python build_all.py
```

Writes `paper/figures/fig_*.{pdf,png}` and `paper/tables/tab_*.tex`. The two
solver-driven figures (`fig_qualitative_fold`, `fig_energy_trace`) are skipped unless
you add `--with-solvers`, which additionally needs the backend package importable
(run it with `backend/.venv/Scripts/python`).

The scripts read only the committed result files — see
[`backend/results/README.md`](../backend/results/README.md) for the claim → file map.
Nothing is hand-transcribed, so a figure that changes means the data changed.

`make_tables.py` is stdlib-only, so the LaTeX tables build even without matplotlib.

---

## Level B — re-run the benchmarks

### B.0 What "native" means here, and why it matters

The paper's latency comparison is only meaningful if no solver is penalised by the
Python interpreter. So every solver in the reported sweeps runs as compiled code:

- **Genuine upstream libraries** — TRAC-IK (TRACLabs C++/KDL/NLopt via `tracikpy`),
  Jacobian-DLS and Multi-start (Peter Corke's Robotics Toolbox `ik_LM`), PyBullet's
  own `calculateInverseKinematics`.
- **Native C++/Eigen ports of the in-repo algorithms** — StagedFold, KineticFold (and
  its two variants), LangevinFold, CCD and FABRIK, all in `backend/cpp/`, exposed as
  the `pik_native` pybind11 module. FK and energy parity against the Python reference
  is verified to ≤1e-11 (`backend/cpp/parity_native.py`); CCD/FABRIK are bit-identical
  per step (`backend/cpp/parity_ccd_fabrik.py`).

CCD and FABRIK are ported rather than imported because no upstream library solves a DH
manipulator to a 6-DOF pose — the reference implementations (Caliko, Wang & Chen) are
graphics point-solvers. Bridging one would itself be a reimplementation, so the repo's
own algorithm is compiled instead, and labelled `(in-repo; native C++)` in the tables.

### B.1 Environment

The reported runs were produced under **WSL2 Ubuntu 22.04, system Python 3.10.12**,
because that is where the genuine TRAC-IK build lives. Any Linux box with these
packages works:

```bash
# genuine baselines + real-mesh oracles, all importable in one process
pip install tracikpy roboticstoolbox-python pybullet mujoco robot_descriptions "numpy<2"
# numpy is pinned <2 for the tracikpy ABI
sudo apt install libeigen3-dev            # for the C++ ports
pip install pybind11
export ROBOT_DESCRIPTIONS_CACHE="$HOME/.cache/robot_descriptions"
```

Verified versions: `tracikpy`, `PyKDL 1.5.1`, `roboticstoolbox 1.3.1`, `pybullet`,
`mujoco 3.10.0`, NumPy 1.26.4.

> On Windows, `backend/bench/run_master_benchmark.ps1` runs the *same* driver against
> `backend/.venv-sim` (PyBullet + MuJoCo, Python 3.12). That path scores the Python
> solvers, not the native ones, so its latency columns are not the paper's — use it for
> smoke tests and oracle work, not for reported timings.

### B.2 Build the C++ solvers

```bash
cd backend
bash cpp/build_native.sh          # -> cpp/pik_native.cpython-310-x86_64-linux-gnu.so
```

A prebuilt Linux/CPython-3.10 `.so` is committed so the sweeps can be re-run without a
toolchain; rebuild it if your Python differs. Verify the port before trusting a run:

```bash
PYTHONPATH=. python3 cpp/parity_native.py        # FK / energy / collision / frustration parity
PYTHONPATH=. python3 cpp/parity_ccd_fabrik.py    # per-step bit-identity for CCD + FABRIK
```

### B.3 The two authoritative sweeps

```bash
cd backend

# 3-seed survey — success + latency, all three arms (n=300/cell, 99 cells)
PYTHONPATH=. python3 native_bench/run_native_master.py --resume \
  --solvers jacobian_dls ccd fabrik trac_ik_style multi_start \
            protein_ik protein_fast protein_fast_o2 protein_fast_calib \
            protein_raw analytical_planar3dof \
  --out "results/master_full(cpp)"

# 10-seed collision sweep — UR5 + Franka (n=1000/cell, 66 cells)
PYTHONPATH=. python3 native_bench/run_native_master.py --resume \
  --seeds 1 2 3 4 5 6 7 8 9 10 --robots ur5 franka_panda \
  --out "results/master_10seed_fast(cpp)"
```

Each writes `<stem>.csv`, `<stem>.md` and `<stem>.manifest.json`. The driver is
crash-safe (the CSV is rewritten after every completed cell) and `--resume` skips cells
already present, so an interrupted overnight run can simply be relaunched. To force
specific cells to re-run, strip their rows first:

```bash
PYTHONPATH=. python3 native_bench/_strip_solvers.py "results/master_full(cpp).csv" protein_fast
```

Before reporting anything, check the run's own oracle-validation block at the top of
the `.md`: DH↔PyBullet and DH↔MuJoCo FK agreement, and PyBullet↔MuJoCo collision
sign-agreement. A run whose oracles disagree is a broken run.

### B.4 The DOF-scaling study (Table 5)

```bash
cd backend

# Table 5 + Figure 5 — the authoritative sweep: n=1000/cell (seeds 1..10), seven DOF
# points, three solvers, 5 repeats, Wilson CIs + Fisher exact, feasibility oracle.
# ~15 min. Add --no-oracle for the ~4 min version (main sweep + stats only).
PYTHONPATH=. python3 native_bench/run_dof_scaling_full.py \
  --out results/dof_scaling/dof_scaling_full.json
PYTHONPATH=. python3 native_bench/report_dof_full.py   # human-readable summary

# superseded n=120 pilot — kept reproducible, NOT a source for paper numbers
PYTHONPATH=. python3 native_bench/run_native_usecase.py --only E \
  --out results/dof_scaling/dof_scaling_native.json

# §5.4's engine cross-check — re-score the same sweep in PyBullet + MuJoCo,
# under capsule and cylinder collision solids
PYTHONPATH=. python3 native_bench/run_dof_sim_scored.py \
  --out results/dof_scaling/dof_scaling_sim_scored.json
```

Two caveats the paper states and you should expect to see:

- **Both library baselines are wall-clock budgeted**, so their clean rates move with
  machine load; KineticFold is deterministic given the seed. The sweep measures this
  rather than assuming it: the per-solve RNG depends only on `(seed, index)` and so is
  identical across repeats, which makes any movement attributable to wall-clock alone.
  Across five repeats KineticFold is **bit-identical in every cell**, TRAC-IK moves up
  to ~1.2 pp and Multi-start up to ~3.3 pp. This is also why the run's replication
  block finds KineticFold reproducing the committed n=120 pilot exactly while TRAC-IK
  drifts by up to 1.7 pp — the same effect that leaves the two committed pilot
  artifacts disagreeing on 8-DOF TRAC-IK (13.3% vs 10.8%).
- **The capsule scoring is exact by construction.** A capsule's surface gap *is* the
  proxy's segment-distance-minus-radii, so both engines reproduce the proxy to ~1e-16.
  That validates the collision *implementation*, not the geometry — use `--geoms
  cylinder` for a genuinely independent solid.

### B.5 Optional — the LangevinFold mini-benchmark

LangevinFold (`protein_raw`) is future work in this paper (§6) and costs ~seconds per
solve at full fidelity, so it has its own small run, scored the same three ways:

```bash
cd backend
PYTHONPATH=. .venv-sim/Scripts/python -m bench.langevin_benchmark   # -> results/langevin_bench.{csv,md}
```

Then `paper/figures/fig_langevin.py` and `make_tables.py` pick it up automatically.

### B.6 Then rebuild the paper

Re-run Level A. Because every figure and table reads the result files by path, the
manuscript's numbers follow the new run with no manual editing.

---

## Level C — run the system interactively

### Backend API + web dashboard

```bash
cd backend
python -m venv .venv && .venv/Scripts/python -m pip install -r requirements.txt
PYTHONPATH=. .venv/Scripts/python -m uvicorn app.main:app --reload      # :8000

cd ../frontend
npm install && npm run dev                                              # :5173
```

The dashboard runs several solvers on the same target and shows the arm, the energy
funnel and a live metric panel. Override the API base with `VITE_API_BASE`.

### Native MuJoCo IK Studio

```bash
cd backend
PYTHONPATH=. .venv-sim/Scripts/python -m app.sim.ik_studio
```

An interactive MuJoCo window with real meshes and click-to-place targets. Needs the
sim extras (`glfw`, `trimesh`, `pycollada`) from `requirements.txt`.

---

## Tests

```bash
cd backend && PYTHONPATH=. python -m pytest tests/ -v     # kinematics, solvers, planar sim model, raw energy
cd frontend && npm test                                   # JS kinematics parity
```

Sim-dependent tests skip cleanly when PyBullet/MuJoCo are absent. CI
(`.github/workflows/ci.yml`) runs both suites plus the frontend production build on
every push.

---

## Known reproduction gaps

Stated so a reviewer does not have to discover them:

- **DOF-scaling 16-DOF cell.** Table 5 now runs `n = 1000` per cell and every row is
  significant against both baselines (Fisher exact `p < 0.05`) **except** 16 DOF vs.
  Multi-start (`p = 0.21`, 11 clean solves against 5). The 16-DOF lead over TRAC-IK is
  resolved (`p = 9.5e-4`); the one over Multi-start is not, and is not claimed.
- **The DOF sweep's engine cross-check is still at `n = 120`.**
  `run_dof_sim_scored.py` (PyBullet + MuJoCo, capsule and cylinder solids) was run
  against the superseded pilot, not against Table 5's sweep. Its finding is a property
  of the collision model rather than of the sample, but the engine-scored ratios quoted
  in §5.4's last paragraph are the pilot's.
- **Incremental-FK bit-identity** for the Python reference solver is verified on the
  UR5 and planar arms (500 configurations each), not on Franka (§5.7).
- **Timing noise.** Wall-clock columns (mean/p95/p99) carry OS scheduling noise;
  success, collision and error columns are deterministic given the seed.
- **Self-collision only.** No solver here reasons about workspace obstacles.
