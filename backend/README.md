# Backend — solvers, validation harness, benchmarks

Python 3.11 for the API and the reference solvers; the reported benchmark numbers come
from the native (C++/genuine-library) path described below.

```
app/
  core/      DH kinematics — FK chain, geometric Jacobian, axis-angle pose error,
             capsule self-collision. THE shared core: every solver and every metric
             in the study is computed against this one implementation.
  solvers/   all solvers behind one uniform signature
  sim/       the validation harness (PyBullet + MuJoCo) and the interactive viewers
  api/       FastAPI routes + pydantic schemas behind the dashboard
cpp/         C++/Eigen ports of every in-repo solver -> `pik_native` (pybind11),
             plus the parity checkers that prove they match the Python
bench/       the authoritative benchmark drivers
native_bench/ runs those drivers with genuine/native solvers swapped in
results/     every committed result file (see results/README.md)
tests/       pytest suite
```

## The solver contract

Every solver — classical baseline or folding-inspired — is a callable

```python
solve(spec: RobotSpec, q0: np.ndarray, T_target: np.ndarray,
      rng: np.random.Generator, collect_steps: bool = False) -> SolveResult
```

registered in [`app/solvers/registry.py`](app/solvers/registry.py). That uniformity is
what makes the benchmark fair and the dashboard solver-agnostic: no caller branches on
which solver it is holding.

| Solver id | Paper name / role |
| :-- | :-- |
| `protein_ik` | **StagedFold** — folding's ordered stages |
| `protein_fast` | **KineticFold** — StagedFold + kinetic partitioning as a compute schedule |
| `protein_fast_o2` / `protein_fast_calib` | KineticFold variants (IAM warm-start; capsule radii calibrated against real mesh) |
| `protein_raw` | **LangevinFold** — coarse-grained Langevin folding simulation (future work) |
| `protein_homotopy` / `fixed_lambda_ik` | CCH-IK and its ablation — ships, but not part of the paper |
| `trac_ik_style` | TRAC-IK-style restart solver (the benchmark swaps in **genuine** TRAC-IK) |
| `multi_start`, `jacobian_dls` | restart / single-shot LM (the benchmark swaps in **genuine** Robotics Toolbox) |
| `ccd`, `fabrik` | geometric heuristics (compiled to native C++ for the benchmark) |
| `analytical_planar3dof` | exact closed form — ground truth, planar arm only |

`SolveResult` ([`app/core/types.py`](app/core/types.py)) carries success, `q_final`,
position/orientation error, iterations, wall time, min self-distance, joint-limit
violations, restarts, an optional per-iteration trace, and the solver-specific
diagnostics (CCH-IK's conflict index; LangevinFold's Σ ratio, free energy and glass
temperature).

## Robots

| Id | DOF | Notes |
| :-- | :-- | :-- |
| `planar3dof` | 3 | link lengths `[0.4, 0.3, 0.2]` m; exact closed-form solution exists → the ground-truth validator |
| `ur5` | 6 | non-redundant, **standard DH**; the primary tuning arm |
| `franka_panda` | 7 | redundant, **modified (Craig) DH** — feeding its official table through the standard-DH transform silently places the end effector ≈1.4 m from the real robot; tight asymmetric limits, incl. joint 4 confined to `[−3.07, −0.07]` rad |

`app/sim/planar_model.py` additionally generates planar N-DOF arms (4…16) *with* a URDF
whose collision solids are exactly the capsule proxy's, which is what makes the
DOF-scaling study engine-scorable.

## The validation harness — `app/sim/`

"Solve once, score three ways": a solver runs once on the DH core, and that identical
`q_final` is scored by the capsule proxy **and** by two full-mesh engines it never
queried while solving.

| Module | Role |
| :-- | :-- |
| `pybullet_backend.py` / `mujoco_backend.py` | the two oracles: real-mesh FK + closest-point self-collision over the same non-adjacent link pairs, on the same URDF. Each self-validates its FK against our DH at construction. |
| `mesh_collision.py` | shared non-adjacent-pair logic so both engines are asked the identical question |
| `parity.py` | FK / collision agreement checks (DH ≡ PyBullet ≡ MuJoCo) |
| `models.py`, `studio_scene.py` | URDF resolution (via `robot_descriptions`) and scene setup |
| `planar_model.py` | procedural URDF for planar N-DOF arms |
| `live_viewer.py`, `ik_studio.py` | MuJoCo viewers; the studio is interactive (click-to-place targets, live metrics, solver comparison) |
| `clean_solve.py` | the clean-solve (success **and** collision-free) predicate used by the use-case study |

## Benchmarks

| Driver | Produces |
| :-- | :-- |
| `bench/master_sim_benchmark.py` | **the** paper sweep — every solver × arm × scenario, scored three ways, crash-safe and `--resume`-able |
| `bench/usecase_experiments.py` | the deployment-role studies; EXP E is the DOF-scaling sweep behind Table 5 |
| `bench/sim_crosscheck.py` | PyBullet vs MuJoCo vs our DH — FK and collision agreement |
| `bench/collision_parity.py` | capsule proxy vs real mesh (the "the proxy is optimistic" result) |
| `bench/langevin_benchmark.py` | the LangevinFold mini-run (future work) |
| `bench/run_master_benchmark.ps1` | Windows launcher for the master sweep against `.venv-sim` |
| `native_bench/*` | the same drivers with genuine libraries + C++ ports swapped in — **this is what the paper reports** |

Reported timings require the native path: see
[`native_bench/README.md`](native_bench/README.md) for what is genuine vs. ported, and
[`../docs/REPRODUCE.md`](../docs/REPRODUCE.md) for the command lines.

## Environments

| Venv | Python | For |
| :-- | :-- | :-- |
| `.venv` | 3.11 | API, solvers, tests, figure scripts |
| `.venv-sim` | 3.12 | PyBullet + MuJoCo work on Windows (the oracles and viewers) |
| WSL Ubuntu 22.04 system `python3` | 3.10 | the native benchmark — the only place `tracikpy` (genuine TRAC-IK) and the C++ build live |

```bash
python -m venv .venv && .venv/Scripts/python -m pip install -r requirements.txt
PYTHONPATH=. .venv/Scripts/python -m uvicorn app.main:app --reload    # :8000
PYTHONPATH=. .venv/Scripts/python -m pytest tests/ -v
```

`requirements.txt` marks the sim extras `python_version < "3.13"` (no PyBullet cp313
wheel), and `app/sim/` degrades gracefully when they are absent — its tests skip rather
than fail.

## Archive

`archive/` and `results/archive/` hold superseded runners and runs kept for provenance
(the pre-native Python sweeps, the staging runs of the C++ port, one-off calibration
probes). Nothing there backs a paper claim.
