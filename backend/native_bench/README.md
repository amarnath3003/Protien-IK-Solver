# `native_bench` — `master_full.md`, re-run ENTIRELY as native compiled code

This directory reproduces the whole master benchmark
(the pre-native Python run, kept at
[`results/archive/python-run/master_full.md`](../results/archive/python-run/master_full.md))
**in the native system** —
the WSL environment where the real TRAC-IK C++ library lives — with **every solver
running as native compiled code, no interpreted Python**:

- the borrowed baselines are the **genuine upstream libraries** (real TRAC-IK, real
  Orocos KDL / Robotics Toolbox), imported and called directly — not reimplemented;
- the project's **own ProteinIK solvers are native C++/Eigen ports** (in
  [`../cpp/`](../cpp/), built into the `pik_native` pybind module) — same idea, logic,
  weights, tolerances and budgets as the Python, verified to parity.

The final output is [`results/master_full(cpp).md`](../results/master_full%28cpp%29.md):
the same 3 arms × 3 scenarios × 100 trials × 3 seeds, the same metric columns, the
same dual real-mesh scoring in **PyBullet + MuJoCo**. Because *nothing* is interpreted,
the speed columns are finally apples-to-apples — e.g. ProteinIK V4 runs sub-millisecond
and competes with TRAC-IK on latency as well as success/collision.

> Numbers differ from the archived Python `master_full.md` **by design**: native
> compiled solvers are not the Python originals. The point is the *same algorithms*,
> run natively.

## Every solver is native compiled code

| master_full row | Native implementation |
|:--|:--|
| **TRAC-IK style** | **REAL TRAC-IK** — TRACLabs C++/KDL/NLopt via `tracikpy` |
| **Jacobian (DLS)** | **REAL Robotics Toolbox** (Corke) Levenberg–Marquardt, single-shot from the seed |
| **Multi-start** | **REAL Robotics Toolbox** `ik_LM` with native random restarts (`slimit=100`) |
| **ProteinIK V1 / V4 / V4+o2 / V4-calib / V6** | **native C++/Eigen ports** of the project's own solvers (`../cpp/pik_*.hpp` → `pik_native`) — statistically identical to the Python (FK/energy parity ≤1e-11; success/collision match, only the RNG stream differs) |
| **CCD, FABRIK** | **native C++/Eigen ports** of the in-repo algorithm (`../cpp/pik_ccd.hpp`, `pik_fabrik.hpp` → `pik_native`) — *no genuine DH-native upstream exists* (see below), so the project's own code is compiled rather than faked as an import. Deterministic (no RNG): per-step math bit-identical to Python, quality columns match to ≤0.7 pt (`../cpp/parity_ccd_fabrik.py`) |
| Analytical (planar3dof) | the project's **exact closed-form** |
| PyBullet native IK | **REAL PyBullet** `calculateInverseKinematics` |

Homotopy (CCH-IK) and Fixed-λ are **excluded** from this benchmark (per request).

### The C++ ProteinIK ports (`../cpp/`)

`dh_robot.hpp` is a generic runtime-DH kinematics + energy library (variable DOF,
standard **and** modified/Craig DH) — the UR5-only `ur5_dh.hpp` generalized so the
solvers run on planar3dof / ur5 / franka_panda. On top of it: `pik_v4.hpp` (V4, plus
o2's IAM warm-start as a flag and calib as calibrated radii), `pik_v1.hpp` (V1 staged
solver), `pik_raw.hpp` (V6 "Raw Biology" — its own LJ/HB/entropy free-energy model +
Langevin annealing). `pik_native.cpp` is the pybind11 module; `parity_native.py`
proves FK/energy/collision/frustration match Python to machine epsilon and each
solver's success/collision distribution matches. `cpp_solvers.py` (here) adapts them
to the `(spec,q0,T,rng)->SolveResult` contract so the driver runs them unchanged.

### Why CCD and FABRIK are *ported*, not swapped for an upstream

Unlike TRAC-IK / KDL / Robotics Toolbox — real *robotics* libraries that accept a
DH robot and return joint angles — the genuine reference implementations of
**FABRIK** (Caliko, Aristidou & Lasenby) and **CCD** (Wang & Chen 1991) are
**graphics / animation point-solvers**: they operate on abstract 3-D point chains
and return bone bend-angles, not DH joint angles for a UR5 / Franka. There is no
original-author library that solves a DH manipulator to a 6-DOF pose. Bridging one
to these robots would itself be a reimplementation — so rather than fake a "genuine
import", these two solvers keep the repo's own algorithm, but (like the ProteinIK
family) that algorithm is now compiled to **native C++/Eigen** (`../cpp/pik_ccd.hpp`,
`pik_fabrik.hpp`) so the speed columns are apples-to-apples with the other native
solvers instead of penalised by the Python interpreter. Both are fully deterministic
(no RNG); `../cpp/parity_ccd_fabrik.py` proves the per-joint update math is
bit-identical to the Python (CCD ≤1e-13 at full budget; FABRIK ≤1e-13 per step) and
that the benchmark quality columns match to ≤0.7 pt — only a couple of non-converged
boundary solves per cell flip from 1-ULP numpy-vs-Eigen differences. They are
labelled `(in-repo; native C++)` in the table.

## Faithfulness — every genuine solver solves the *identical* robot

Each library builds its internal chain from the repo's own `RobotSpec` DH table and
is FK-parity-checked against `end_effector_pose` before use:

| Arm (DH convention) | KDL FK parity | RTB FK parity |
|:--|--:|--:|
| ur5 (standard) | 2.2e-16 m | 0 |
| franka_panda (**modified/Craig**) | 2.2e-16 m | 0 |
| planar3dof (standard) | 2.2e-16 m | 0 |

- **Modified DH (Franka)** needed a shift construction in KDL — `Frame.DH_Craig1989`
  places the joint rotation *before* the fixed link transform, but modified DH needs
  it *after*; the chain is rebuilt as `Fpre₀·[Rz(q₀)·Fpre₁]·…` (see `genuine_solvers._kdl_chain`).
- **TRAC-IK** solves the *real* URDF (`ur5_robot.urdf` / `panda.urdf`, the identical
  files the original run scored against) and its DH-frame target is mapped through the
  same validated constant frame offset the PyBullet/MuJoCo oracles use (ur5: base
  Rz(180°), residual 9.5e-7; panda: tool, residual 6.6e-7).
- The real-mesh oracles are the repo's own `PyBulletBackend` / `MuJoCoBackend`, reused
  unchanged; both self-validate at construction (PB↔MJ FK agreement ≤5e-8 m, collision
  sign-agreement ≥99%).

## Environment (WSL Ubuntu 22.04, Python 3.10)

Genuine solver + scoring libraries, all confirmed importable in one process:
`tracikpy` (real TRAC-IK), `PyKDL 1.5.1` (Orocos KDL), `roboticstoolbox 1.3.1`
(Corke), `pybullet`, `mujoco 3.10.0`. numpy pinned `<2` to preserve the tracikpy ABI.

## Files

- `genuine_solvers.py` — thin adapters wrapping each genuine library to the repo's
  `SolveResult` contract (build chain from DH → solve → score with repo DH machinery).
- `cpp_solvers.py` — the same adapter layer for the `pik_native` C++/Eigen ports
  (ProteinIK V1/V4/o2/calib/V6 + CCD/FABRIK).
- `run_native_master.py` — swaps the genuine + native adapters into `SOLVER_REGISTRY`
  and runs the repo's own `bench/master_sim_benchmark.py` driver unchanged.
- `run_native_usecase.py` — same swap for `bench/usecase_experiments.py` (paper
  Table 5, the DOF-scaling sweep, EXP E).
- `run_dof_sim_scored.py` — re-scores the DOF sweep in PyBullet + MuJoCo through the
  generated planar URDFs (`app/sim/planar_model.py`), capsule *and* cylinder solids.
- `_strip_solvers.py` — deletes named solver rows from an existing output CSV so
  `--resume` re-runs exactly those cells.
- `_env.py` — WSL path + URDF-resolver setup (ur5 module was renamed in
  robot_descriptions 3.0.0; we point at the identical cached original URDF).
- `test_genuine.py` — FK-parity + solve smoke test for the genuine baselines.

## Run

```bash
# inside WSL Ubuntu-2204, from backend/ (system python3.10, NOT .venv-sim)
export ROBOT_DESCRIPTIONS_CACHE="$HOME/.cache/robot_descriptions"

# 1. build the C++ module once
bash cpp/build_native.sh                    # -> cpp/pik_native.cpython-310-*.so

# 2. the 3-seed survey — success + latency, all three arms
PYTHONPATH=. python3 native_bench/run_native_master.py --resume \
  --solvers jacobian_dls ccd fabrik trac_ik_style multi_start \
            protein_ik protein_fast protein_fast_o2 protein_fast_calib \
            protein_raw analytical_planar3dof \
  --out "results/master_full(cpp)"

# 3. the 10-seed collision sweep — UR5 + Franka only
PYTHONPATH=. python3 native_bench/run_native_master.py --resume \
  --seeds 1 2 3 4 5 6 7 8 9 10 --robots ur5 franka_panda \
  --out "results/master_10seed_fast(cpp)"
```

`--resume` skips cells already present in the output CSV, so a crashed overnight
run picks up where it stopped; to force specific cells to re-run, strip their rows
first with `_strip_solvers.py`. Full command lines, including the DOF sweep and the
figure/table rebuild, are in [`docs/REPRODUCE.md`](../../docs/REPRODUCE.md).
