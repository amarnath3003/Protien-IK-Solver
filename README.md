# ProteinIK — Inverse Kinematics as a Protein-Folding Process

[![CI](https://github.com/amarnath3003/Protien-IK-Solver/actions/workflows/ci.yml/badge.svg)](https://github.com/amarnath3003/Protien-IK-Solver/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![DOI](https://img.shields.io/badge/DOI-10.5281%2Fzenodo.21905183-blue)](https://doi.org/10.5281/zenodo.21905183)

_A robot arm and a protein backbone are the same kind of object: a chain of rigid
segments whose only freedom is the rotation between neighbours, searching a rugged,
constrained landscape for a configuration that satisfies its boundary conditions._

This repository implements that correspondence as a set of inverse-kinematics solvers
and benchmark harnesses. The code reproduces the results in the draft manuscript
**[ProteinIK: Inverse Kinematics as a Protein-Folding Process](https://doi.org/10.5281/zenodo.21905183)**
and provides the data and figure generators used to produce every reported number.

| I want to… | Go to |
| :-- | :-- |
| read the paper | [`paper/academic.md`](paper/academic.md) — or the plain-English mirror, [`paper/academic_simple.md`](paper/academic_simple.md) |
| see which file backs which claim | [`backend/results/README.md`](backend/results/README.md) |
| reproduce the results | [`docs/REPRODUCE.md`](docs/REPRODUCE.md) |
| read the algorithms in detail | paper §3, or [`docs/METHODOLOGY.md`](docs/METHODOLOGY.md) |
| find the solver code | [`backend/app/solvers/`](backend/app/solvers) (Python reference) · [`backend/cpp/`](backend/cpp) (native ports) |

---

## Contents

- [The idea](#the-idea)
- [The three solvers](#the-three-solvers)
- [The baseline field](#the-baseline-field)
- [Headline results](#headline-results)
- [How the results are validated](#how-the-results-are-validated)
- [Repository layout](#repository-layout)
- [Quickstart](#quickstart)
- [Running the benchmarks](#running-the-benchmarks)
- [Tests and CI](#tests-and-ci)
- [Scope and limitations](#scope-and-limitations)
- [Citing this work](#citing-this-work)

---

## The idea

Classical IK treats the arm as a single objective to be minimised from the first
iteration — damped least squares, cyclic coordinate descent, reaching heuristics,
restart-based search — driving pose error down a landscape they treat as one basin to
descend. A protein does not fold that way. It folds in *stages*, and it decides *how
much search to spend* per molecule.

The correspondence is structural, not a metaphor:

| Protein folding | Inverse kinematics |
| :-- | :-- |
| Backbone dihedral angles φ/ψ (soft DOF) | Joint angles `q` (the DOF) |
| Rigid bonds / fixed bond lengths | Fixed link lengths (FK constraints) |
| Native (folded) state | The IK solution configuration |
| Free-energy funnel | Convergence basin to the target |
| Rugged landscape / kinetic traps | Local minima / failed solves |
| Excluded volume (sterics) | Self-collision avoidance |
| Hydrophobic collapse | Coarse approach to the target region |
| Secondary structure (local order) | Local joint settling |
| Molecular chaperone (GroEL) | Restart / rescue from a stuck state |
| Kinetic partitioning (fast vs. slow folders) | Easy vs. hard targets |

Algorithms already cross between the two fields — CCD, a robotics IK method, was
adopted into structural biology for protein loop closure — but **every prior crossing
runs robotics → biology, and carries one *move* at a time**. This work runs the other
way and carries the *process*: the ordered sequence nature uses to fold becomes the
engine of the solver. Every numerical ingredient is standard IK, so any advantage comes
from the **sequencing and the schedule**, not from a new energy function.

## The three solvers

| Paper name | Solver id | Idea | Status |
| :-- | :-- | :-- | :-- |
| **StagedFold** | `protein_ik` | Folding's ordered *sequence*: local settling **before the target is consulted** → coarse collapse → funnelled narrowing search → scoped chaperone rescue →[...] |
| **KineticFold** | `protein_fast` | StagedFold **plus kinetic partitioning as a compute schedule**: try a cheap downhill fold first, and pay for the full staged search only on genuinely frustrate[...] |
| **LangevinFold** | `protein_raw` | The correspondence at its physical limit: the arm coarse-grained to one bead per joint origin and evolved by overdamped **Langevin dynamics** on `F = E_task + [...]`

Two moves are, to our knowledge, new in this setting: a **target-blind first stage**
(the arm settles into a relaxed, in-limits pose before it is ever told where to go) and
a **scoped-then-escalating rescue** (when stuck, re-randomise only a contiguous window
of joints centred on the misfolded one, escalating to a global reseed only as a last
resort — where TRAC-IK's stall response is *always* a full random restart).

**KineticFold's schedule** is the contribution that makes it competitive:

- **Phase A (barrierless)** — up to 6 replicas run a cheap adaptive
  Levenberg–Marquardt polish (≤30 steps); the first replica that converges *clash-free*
  wins and the solve ends. This path takes **79% of targets** across the two physical
  arms (93% on UR5 open-space, down to 50% on Franka cluttered) — the gate tracks
  scenario difficulty rather than firing at a fixed rate.
- **Phase B (the full staged fold)** — fires only when no converged Phase-A replica is
  clash-free, i.e. the target is *frustrated*. It runs StagedFold's stages with a
  Metropolis-accepted funnel under geometric cooling and an analytic rescue that reads
  its joint off the already-computed Jacobian.

The diagnosis behind it: StagedFold's problem was never the average solve, it was the
tail — the slowest ~10% of targets consumed ~57% of total wall time. Micro-optimising
the inner loop bought only 1.1–1.4×, because the cost is not *how* the per-fold search
runs but *whether a target enters it at all*. Naive budget cuts (cap replicas, bail
earlier) destroyed the result: at `cap_replicas = 2`, Franka open-space success falls to
71.7% against ~100%. Full write-up:
[`docs/design/kineticfold-barrierless-first.md`](docs/design/kineticfold-barrierless-first.md).

## The baseline field

Six baselines spanning the IK literature. **Every solver in the reported sweeps runs as
native compiled code**, so the latency comparison is apples-to-apples:

| Baseline | Implementation |
| :-- | :-- |
| **TRAC-IK** | genuine TRACLabs C++/KDL/NLopt via `tracikpy`, `solve_type=Speed`, 5 ms timeout |
| **Jacobian-DLS** | genuine Robotics Toolbox (Corke) `ik_LM`, single-shot from `q0` |
| **Multi-start** | genuine Robotics Toolbox `ik_LM` with up to 100 random restarts |
| **CCD** / **FABRIK** | in-repo algorithm compiled to native C++/Eigen |
| **Analytical (planar)** | exact closed form — the ground-truth validator |
| **PyBullet native IK** | the simulator's own `calculateInverseKinematics`, on identical targets |

CCD and FABRIK are *ported*, not imported, for a stated reason: no upstream library
solves a DH manipulator to a 6-DOF pose — the reference implementations (Caliko,
Wang & Chen) are graphics point-solvers that return bone bend-angles. Bridging one
would itself be a reimplementation, so the repo's own algorithm is compiled instead and
labelled `(in-repo; native C++)`. Both are deterministic and verified bit-identical per
step against the Python (≤1e-13).

The ProteinIK solvers are likewise C++/Eigen ports (`backend/cpp/` → `pik_native`),
FK- and energy-parity-checked against the Python reference to ≤1e-11.

## Headline results

Three arms (planar 3-DOF, UR5 6-DOF, Franka Panda 7-DOF redundant) × three scenarios
(`open_space`, `near_singular`, `cluttered`, the latter two reject-sampled against a
hardness criterion). Every solver sees the identical target draw.

### Success — single-shot, ‖Δp‖ < 1 mm and ‖Δω‖ < 10 mrad

3-seed survey, `n = 300` per cell, from `master_full(cpp).md`:

| Solver | UR5 open / near-sing. / cluttered | Franka open / near-sing. / cluttered |
| :-- | :-- | :-- |
| **KineticFold** | 99.7 / **100** / **100** | **100** / **100** / **98.3** |
| TRAC-IK | **100** / 99.3 / **100** | 98.7 / 99.0 / 94.7 |
| Multi-start | **100** / **100** / **100** | 99.3 / 98.0 / 93.7 |
| StagedFold | 97.3 / 88.3 / 89.3 | 96.3 / 92.3 / 80.7 |
| Jacobian-DLS | 72.3 / 69.7 / 77.0 | 28.3 / 31.0 / 19.0 |
| FABRIK | 49.3 / 34.7 / 36.7 | 18.0 / 11.3 / 22.7 |
| CCD | 43.7 / 32.0 / 41.0 | 23.0 / 11.7 / 12.3 |

[...the rest of the README remains unchanged...]
