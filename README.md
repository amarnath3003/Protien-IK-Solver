# ProteinIK — Inverse Kinematics as a Protein-Folding Process

[![CI](https://github.com/amarnath3003/Protien-IK---An-IK-Solver-inspired-by-protein-folding/actions/workflows/ci.yml/badge.svg)](https://github.com/amarnath3003/Protien-IK---An-IK-Solver-inspired-by-protein-folding/actions)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Paper](https://img.shields.io/badge/paper-draft-informational)](paper/academic.md)

**A robot arm and a protein backbone are the same kind of object**: a chain of rigid
segments whose only freedom is the rotation between neighbours, searching a rugged,
constrained landscape for a configuration that satisfies its boundary conditions. This
repository takes that correspondence literally and builds an inverse-kinematics solver
out of the *process* proteins use to fold — then benchmarks it against the IK
literature and validates every number on two independent physics engines.

This is the research-code repository for the paper
**[*ProteinIK: Inverse Kinematics as a Protein-Folding Process*](paper/academic.md)**
(draft). It contains the solvers, the baseline field, the benchmark harness, every
committed result file, and the generators for every figure and table in the manuscript.

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
| **StagedFold** | `protein_ik` | Folding's ordered *sequence*: local settling **before the target is consulted** → coarse collapse → funnelled narrowing search → scoped chaperone rescue → native-state stability gate | evaluated (§3.2) |
| **KineticFold** | `protein_fast` | StagedFold **plus kinetic partitioning as a compute schedule**: try a cheap downhill fold first, and pay for the full staged search only on genuinely frustrated targets | evaluated — the paper's main solver (§3.3) |
| **LangevinFold** | `protein_raw` | The correspondence at its physical limit: the arm coarse-grained to one bead per joint origin and evolved by overdamped **Langevin dynamics** on `F = E_task + E_LJ + E_HB − T·S_conf`, cooled to a glass transition | future work (§6) — runs, but excluded from the compared field |

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

The field splits in two. The single-trajectory baselines collapse on the hard scenarios
— exactly the single-basin failure mode the design predicts for methods with no restart
mechanism. **UR5 is a saturated tie** at the top (99.3–100%), so the easy arm does not
separate the leaders. **The redundant Franka does**: KineticFold is the only solver
above 98% on all three cells, and its worst case (98.3% on `cluttered`) tops the best
baseline there by **3.7 points**. StagedFold — folding's process *without* its schedule
— clears every simple baseline but plateaus below the production ones, which is the gap
kinetic partitioning closes.

### Speed — with the whole field compiled

Mean / p99 ms per solve, `open_space`:

| Solver | UR5 | Franka |
| :-- | :-- | :-- |
| **KineticFold** | **0.1** / **1.5** | **0.1** / **1.4** |
| TRAC-IK | 0.5 / 2.6 | 0.9 / 5.1 |
| Multi-start | 0.6 / 1.0 | 0.8 / 1.9 |
| CCD | 0.4 / 0.7 | 0.6 / 0.8 |
| FABRIK | 0.3 / 0.4 | 0.5 / 0.6 |
| PyBullet native IK | 4.0 / 6.8 | 4.4 / 9.7 |

KineticFold has the fastest typical solve of the field on both arms — roughly 1.7–7×
under TRAC-IK and Multi-start — with its median near the measurement floor (≈0.04 ms on
UR5). Its **worst p99 anywhere in the survey is 4.7 ms** (Franka `cluttered`), below
TRAC-IK's 5.1 ms there. The small tail is the direct signature of Phase A: most targets
never enter the expensive fold. Wall-clock columns carry OS scheduling noise; success,
collision and error columns are deterministic given the seed.

### Self-collision — real mesh, and conditional on redundancy

10-seed sweep, `n = 1000` per cell, PyBullet real-mesh rate (MuJoCo agrees within ~1
point and preserves every ranking):

| Solver | UR5 open / near-sing. / cluttered | Franka cluttered |
| :-- | :-- | :-- |
| **KineticFold** | **26.2** / **40.4** / **56.4** | 82.4 |
| StagedFold | 32.9 / 43.4 / 64.6 | 77.3 |
| TRAC-IK | 35.3 / 49.9 / 74.2 | **77.1** |
| Multi-start | 35.8 / 47.6 / 74.7 | **77.0** |

On the **non-redundant UR5** KineticFold is the cleanest of the field in every regime —
1.24–1.35× lower than TRAC-IK — while matching the top of the success field, and it
penetrates about half as deeply when it does clash. On the **redundant Franka** the
field converges into a wash: a spare 7th joint gives every method a null-space
direction to dodge with, so collision-directed search has much less room to matter.
**That conditionality is evidence for the mechanism, not against it** — the edge appears
where the chain has nowhere to hide from its own search.

Counting only *clean* goals (a success that is also collision-free on real mesh), on
UR5 `cluttered` KineticFold returns **43.6 usable goals per 100 attempts against 25.8
(TRAC-IK) and 25.3 (Multi-start)**.

### The result that tests the thesis — scaling into a polymer

Lengthen a planar arm from 4 to 16 joints, making it progressively more polymer-like,
and measure single-shot clean-solve rate (`n = 1000` per cell, seeds 1–10) against both
production baselines. All three solvers reach the target ~100% of the time — the entire
gap is self-collision avoidance:

| DOF | n | KineticFold % [95% CI] | TRAC-IK % | Multi-start % | ×TI | ×MS | feasible % |
| --: | --: | --: | --: | --: | --: | --: | --: |
| 4 | 1000 | **74.80** [72.02, 77.39] | 37.54 | 44.10 | 2.0× | 1.7× | 89.9 |
| 6 | 1000 | **61.10** [58.04, 64.07] | 24.92 | 30.68 | 2.5× | 1.9× | 94.6 |
| 8 | 1000 | **40.10** [37.11, 43.17] | 11.00 | 17.62 | 3.6× | 2.1× | 94.5 |
| 10 | 1000 | **23.60** [21.07, 26.33] | 5.14 | 8.74 | 4.8× | 2.8× | 95.0 |
| 12 | 1000 | **9.30** [7.65, 11.26] | 2.00 | 3.32 | 5.2× | 2.7× | 97.0 |
| 14 | 1000 | **4.50** [3.38, 5.97] | 0.72 | 0.98 | 6.4× | **4.5×** | 93.9 |
| 16 | 5000 | **1.04** [0.79, 1.36] | 0.10 | 0.35 | **10.4×** | 3.5× | 75.1 |

The advantage holds at every chain length against both baselines and **widens as the
chain lengthens** — monotonically 2.0×→10.4× over TRAC-IK, and 1.7×→3.5–4.5× over
Multi-start, from 4 to 16 joints. **Every cell is significant** (Fisher exact
`p < 0.05`); even the sparsest, 16 joints, lands at `p = 5.7e-11` vs. TRAC-IK and
`p = 6e-6` vs. Multi-start.

The 16-joint row runs at `n = 5000` for a reason worth knowing: at `n = 1000` TRAC-IK
read as an exact 0.0% and the Multi-start margin was unresolvable (`p = 0.21`). Both
were artifacts of sample size — at 5000 trials TRAC-IK returns 5 clean solves, not
none. Rare events need big samples.

The falling absolute rates are a *search* limit, not a geometric one: the `feasible`
column is the fraction of targets for which a restart oracle can still demonstrate some
clean fold, and it stays at 89.9–97.0% through 14 joints and 75% at 16. Clean folds remain
available for three targets in four at 16 DOF while every method's single shot finds at
most one in a hundred. **The advantage is largest exactly where the arm behaves most
like a folding chain** — which is the point: the method wins because the problem
*becomes* folding.

## How the results are validated

**Solve once, score three ways.** Each solver runs a single time on the shared DH
`RobotSpec` core, and that identical `q_final` is then judged by three independent
evaluators: our capsule proxy (what the solvers optimise against) and two full-mesh
physics engines the solvers never query — **PyBullet** and **MuJoCo**, loading the same
URDF over the same non-adjacent link pairs. Both engine queries are purely kinematic
(`resetJointState`/`getLinkState`; `qpos` + `mj_kinematics`), so no dynamics rollout is
being compared against a kinematic model.

What that buys, concretely:

- **Forward kinematics agree to floating-point noise** — DH↔PyBullet 9.5e-7 m (UR5) and
  6.6e-7 m (Franka), DH↔MuJoCo 4.2e-8 and 8.7e-16, engine↔engine ≈4–6e-8 m. This is
  load-bearing: a target generated from a wrong FK gets "solved" successfully against
  that same error, so only a second model can expose it. It also independently confirms
  the Franka's **modified (Craig) DH** table — feeding it through the standard-DH
  transform silently places the end effector ≈1.4 m from the real robot.
- **The two oracles corroborate each other** — collide/clear sign-agreement 97.8% (UR5)
  to 99.0% (Franka), correlation 0.88–0.99 — so a proxy-vs-oracle disagreement is
  attributable to the proxy, not to noise between engines.
- **The validation corrects our own claim.** The capsule proxy is systematically
  *optimistic*: real meshes collide 36.5% of the time on UR5 where the proxy says
  16.9%. So collision is reported only as a **ranking** of solvers, never as an absolute
  rate, and the UR5 margin over TRAC-IK is stated at the real-mesh 1.24–1.35× rather
  than the larger number the proxy suggests.
- **The DOF-scaling result survives a different collision shape.** The planar arms carry
  no CAD, so they are re-emitted as URDFs and re-scored in both engines under *two*
  solids: capsules (which equal the proxy by construction — an implementation check, not
  a geometry one) and flat-capped **cylinders**, a genuinely different idealisation.
  Under cylinders every solver gains 0–4.7 pp, so the capsule model is the stricter
  reading — and across all **84 comparisons** (2 solids × 2 engines × 7 chain lengths ×
  2 baselines) KineticFold leads every single one. The scaling advantage is not an
  artifact of the capsule caps.

Artifacts: [`backend/results/validation/`](backend/results/validation) ·
harness: [`backend/app/sim/`](backend/app/sim) ·
runner: [`backend/bench/master_sim_benchmark.py`](backend/bench/master_sim_benchmark.py).

## Repository layout

```
paper/
  academic.md              THE PAPER (canonical draft)
  academic_simple.md       plain-English mirror, sentence by sentence
  figures/                 every figure + its generator (reads the committed results)
  tables/                  generated LaTeX tables + the hand-authored static ones

backend/
  app/
    core/                  DH kinematics, Jacobian, capsule self-collision  (the shared core)
    solvers/               all solvers, one uniform (spec, q0, T_target, rng) -> SolveResult
      protein_ik.py          StagedFold
      protein_fast/          KineticFold (+ o2 / calibrated variants)
      protein_raw/           LangevinFold (future work)
      protein_homotopy/      CCH-IK — not part of this paper
      ccd.py fabrik.py jacobian_dls.py multi_start.py trac_ik_style.py
      analytical_planar3dof.py
      registry.py            solver id -> callable, and per-robot compatibility
    sim/                   the validation harness: PyBullet + MuJoCo backends, mesh
                           collision, FK/collision parity, planar URDF generator,
                           live viewer + interactive MuJoCo IK Studio
    api/ main.py           FastAPI service behind the dashboard
  cpp/                     C++/Eigen ports of every in-repo solver -> pik_native (pybind11)
                           + the parity checkers that prove they match the Python
  bench/                   the authoritative benchmark drivers
    master_sim_benchmark.py  "solve once, score three ways" — the paper's sweep
    usecase_experiments.py   the DOF-scaling study (EXP E -> Table 5)
    sim_crosscheck.py        PyBullet vs MuJoCo vs DH oracle agreement
    collision_parity.py      capsule proxy vs real mesh
  native_bench/            runs those drivers with every solver swapped for its
                           genuine/native implementation (this is what the paper reports)
  results/                 every committed result file — see results/README.md
  tests/                   pytest suite (sim tests skip cleanly without PyBullet/MuJoCo)

frontend/                  React + Three.js dashboard: several solvers on one target,
                           live arm, energy funnel, metric panel

docs/
  REPRODUCE.md             clean checkout -> rebuilt paper, three levels of fidelity
  METHODOLOGY.md           the deep methods write-up
  design/                  per-solver design records (KineticFold, LangevinFold)
  archive/                 development history: drafts, research notes, the V5 line,
                           the dev log. Kept for the technical report; nothing here is
                           authoritative for the paper.
```

## Quickstart

```bash
# API + solvers (Python 3.11)
cd backend
python -m venv .venv && .venv/Scripts/python -m pip install -r requirements.txt
PYTHONPATH=. .venv/Scripts/python -m uvicorn app.main:app --reload        # :8000

# dashboard
cd ../frontend
npm install && npm run dev                                               # :5173
```

Solve one target from Python:

```python
import numpy as np
from app.core.kinematics import get_robot_spec, end_effector_pose
from app.solvers.registry import run_solver

spec = get_robot_spec("ur5")
rng  = np.random.default_rng(0)
q0   = np.zeros(spec.n_joints)
T_target = end_effector_pose(spec, rng.uniform(*spec.joint_limits.T))   # reachable by construction

res = run_solver("protein_fast", spec, q0, T_target, rng)   # KineticFold
print(res.success, res.pos_error, res.iterations)
```

Every solver shares that signature, so swapping `"protein_fast"` for `"trac_ik_style"`,
`"ccd"`, `"protein_ik"`, … changes nothing else. `get_solvers_for_robot(robot)` lists the
ids valid for an arm.

Interactive MuJoCo IK Studio (real meshes, click-to-place targets):

```bash
cd backend && PYTHONPATH=. .venv-sim/Scripts/python -m app.sim.ik_studio
```

## Running the benchmarks

The reported numbers come from the **native** sweeps — genuine baseline libraries plus
C++/Eigen ports — run under WSL2 Ubuntu 22.04 / Python 3.10:

```bash
cd backend
bash cpp/build_native.sh                                  # build pik_native
PYTHONPATH=. python3 cpp/parity_native.py                 # prove the port matches Python
PYTHONPATH=. python3 native_bench/run_native_master.py --resume \
    --out "results/master_full(cpp)"                      # the 3-seed survey
```

The driver is crash-safe (CSV rewritten after every cell) and `--resume` skips completed
cells, so an interrupted overnight run just gets relaunched. Full command lines for all
four sweeps, plus the environment and the figure rebuild, are in
**[`docs/REPRODUCE.md`](docs/REPRODUCE.md)**.

Rebuild every figure and table from the committed results in about a minute:

```bash
cd paper/figures && python build_all.py
```

## Tests and CI

```bash
cd backend && PYTHONPATH=. python -m pytest tests/ -v     # kinematics, solvers, planar sim model, raw energy
cd frontend && npm test                                   # JS kinematics parity
```

[CI](.github/workflows/ci.yml) runs both suites and the frontend production build on
every push. Tests that need PyBullet or MuJoCo skip cleanly when those are absent.

## Scope and limitations

Stated up front, because a reviewer should not have to find them:

- **Self-collision only.** No solver here reasons about workspace obstacles. An
  `E_obstacle` term folds into the same staged, kinetically partitioned machinery
  without changing either solver's logic, and is the immediate next step.
- **All results are in simulation.** No hardware experiments.
- **The collision proxy is hand-tuned**, not derived from CAD, and is optimistic
  relative to real mesh — hence the ranking-only reporting above.
- **The 16-DOF cell needed `n = 5000` to resolve.** At `n = 1000` the Multi-start
  contrast was not significant and TRAC-IK's rate read as an exact zero; both were
  sampling artifacts. Rare-event cells should be read with their sample size in view,
  and extending past 16 joints would need a larger one still.
- **Both library baselines are wall-clock budgeted** in the DOF sweep, so their clean
  rates move with machine load (TRAC-IK up to 0.8 pp, Multi-start up to 3.3 pp across
  five sweep repeats); KineticFold is bit-identical across all five.
- **The DOF-scaling comparison is single-shot.** A clearance-selecting wrapper (solve K
  times, keep the cleanest) lifts every solver — the feasibility oracle above is that
  wrapper at its limit, recovering clean folds for 75–97% of targets. The claim is a
  per-solve advantage, and does not by itself predict the ordering under a large
  restart budget.
- **The DOF sweep's engine cross-check re-solves under each collision solid.** Only
  KineticFold's capsule-to-cylinder difference is purely geometric (it is deterministic
  and returns identical configurations both times); the two wall-clock-budgeted
  baselines mix the change of solid with their own run-to-run movement.
- **Incremental-FK bit-identity** for the Python reference covers UR5 and the planar arm
  (500 configurations each), not Franka.
- **LangevinFold is future work.** It runs and is scored, but it is excluded from the
  compared field and no §5 claim rests on it — at 13–22 ms per solve it is two orders of
  magnitude off the real-time solvers. Its rows do appear in the committed CSVs; see
  [`backend/results/README.md`](backend/results/README.md) for how to read them.
- **CCH-IK / V5** (`protein_homotopy`, `fixed_lambda_ik`) ships in the codebase and in
  the dashboard but is not part of this paper; its research record is in
  [`docs/archive/v5-cchik/`](docs/archive/v5-cchik).

## Citing this work

See [`CITATION.cff`](CITATION.cff). The manuscript is a draft — author and venue
metadata carry `TODO` placeholders until it is submitted.

## License

[MIT](LICENSE). The UR5 and Franka Panda URDFs and meshes are resolved at runtime from
[`robot_descriptions`](https://github.com/robot-descriptions/robot_descriptions.py) and
remain under their own upstream licenses; they are not vendored here.

## Acknowledgements

The benchmark leans on work by other people: TRAC-IK (Beeson & Ames, TRACLabs), the
Robotics Toolbox for Python (Peter Corke), Orocos KDL, PyBullet, and MuJoCo. The
folding theory this borrows from is cited in full in the paper's reference list.
