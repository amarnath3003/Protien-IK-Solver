# `app/sim` — simulator adapter + oracle layer

Validates the ProteinIK solver family against a **real robotics simulator**
instead of only our own DH forward kinematics. Full rationale and phasing:
[`sim_migration_plan.md`](../../../sim_migration_plan.md).

The solvers are **not** touched. This is an adapter + oracle around them: a
widely-trusted simulator becomes the source of truth for the robot model, the
forward kinematics, and self-collision — the three things that are currently our
own private definitions.

## Status

| Phase | What | State |
|---|---|---|
| 0 | Acquire & pin UR5 + Panda models; validate joint limits | ✅ `models.py` |
| 1 | DH ↔ URDF FK parity harness (**the deliverable**) | ✅ `parity.py`, `tests/test_sim_parity.py` |
| 2 | PyBullet evaluation oracle (fk/collision/reachable-target/native-IK) | ✅ `pybullet_backend.py`, `bench/sim_benchmark.py` |
| 3 | Reconcile the capsule collision proxy vs real mesh collision (headline) | ✅ `bench/collision_parity.py` → `collision_parity.md` |
| 4 | MuJoCo second oracle (three-way FK + collision cross-check) | ✅ `mujoco_backend.py`, `bench/sim_crosscheck.py` → `sim_crosscheck.md` |

## The environment gotcha (read this first)

**PyBullet has no prebuilt Windows wheel for any modern Python** (verified across
cp37–cp312). On Windows it *always* compiles from source, which needs the MSVC
C++ Build Tools. The core backend runs on **Python 3.13**, where a source build
is the only option. So the sim deps live in a **separate Python 3.12 venv**,
kept out of the core `requirements.txt` (which stays pure-`pip`, no compiler).

### One-time setup

Requires the MSVC C++ Build Tools (for the PyBullet source build) and
[`uv`](https://github.com/astral-sh/uv):

```bash
# MSVC C++ build tools (once, ~2 GB) — needed only to compile PyBullet on Windows
winget install --id Microsoft.VisualStudio.2022.BuildTools -e \
  --override "--quiet --wait --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"

cd backend
uv python install 3.12
uv venv --python 3.12 .venv-sim
uv pip install --python .venv-sim/Scripts/python.exe pybullet numpy robot_descriptions
uv pip install --python .venv-sim/Scripts/python.exe mujoco   # Phase 4 second oracle
```

> `mujoco` ships prebuilt wheels for Windows/Python 3.12 (no compiler needed), so
> only PyBullet forces the source build above. Phase 4 needs both installed.

`.venv-sim/` is git-ignored. `robot_descriptions` downloads & caches the UR5 and
Panda URDFs on first use (to `~/.cache/robot_descriptions`).

> On Linux/macOS, PyBullet installs from a wheel with no compiler and any of
> Python 3.10–3.13 works — just `pip install pybullet robot_descriptions` into
> the normal backend env and skip the separate venv.

## Running the parity harness (Phase 1)

```bash
# full report — UR5 + Panda, 10k random configs each
backend/.venv-sim/Scripts/python.exe -m app.sim.parity

# one robot, custom sample count
backend/.venv-sim/Scripts/python.exe -m app.sim.parity ur5 20000

# Phase-0 model + joint-limit report
backend/.venv-sim/Scripts/python.exe -m app.sim.models

# as a guarded pytest (skips if pybullet/robot_descriptions absent)
backend/.venv-sim/Scripts/python.exe -m pytest tests/test_sim_parity.py -v
```

Run `python -m` commands from the `backend/` directory so `app` is importable.

## What the parity numbers mean (plan §3 decision table)

For each config we compare our DH `end_effector_pose(spec, q)` against PyBullet's
`getLinkState(ee_link, q)` and report three things:

- **direct** — raw pose deviation.
- **offset** — the constant transform `inv(T_dh) @ T_sim`. If it's the same for
  every config, our DH frame and the URDF frame differ only by a fixed
  tool/base convention the adapter absorbs — not a real disagreement.
- **residual** — structural drift left after removing that constant offset.

| Verdict | Meaning | Consequence |
|---|---|---|
| `exact` | direct deviation < 1e-6 | DH already matches the URDF; sim is a pure oracle |
| `constant_offset` | residual < 1e-6, offset ≠ 0 | frames differ by a fixed `T_tool`/`T_base`; absorb it in the adapter |
| `structural_mismatch` | residual ≥ 1e-6 | genuine joint offset/sign/axis difference; fix the DH table or derive the spec from the URDF |

A tiny residual on both arms means the migration is "wire up an oracle" (easy).
A large residual means "reconcile the robot model" first. **That is the single
number Phase 1 exists to produce.**

## Running the evaluation oracle (Phase 2)

```bash
# full sim-oracle sweep — both arms, all scenarios, all fast solvers + native-IK
backend/.venv-sim/Scripts/python.exe -m bench.sim_benchmark --skip-slow --out sim_oracle

# one arm / one scenario / a solver subset
backend/.venv-sim/Scripts/python.exe -m bench.sim_benchmark \
    --robots ur5 --scenarios cluttered --solvers protein_fast trac_ik_style --trials 50
```

Each solver runs on our fast DH `RobotSpec` core exactly as production does; its
`q_final` is then **re-scored inside PyBullet** — real FK and real mesh
self-collision (`getClosestPoints`). PyBullet's own `calculateInverseKinematics`
(iteratively refined for a fair shot) is added as a baseline column. The runner
lives in `bench/sim_benchmark.py`; the oracle it drives is
`app/sim/pybullet_backend.py`.

### The two comparisons to read

| Pair | Question | Phase-1 clean ⇒ expectation |
|---|---|---|
| `our_succ` vs `sim_succ` | Does a solve we call good survive an independent simulator's FK? | agree (a gap = model-parity leak) |
| `our_col` vs `sim_col` | Does our capsule proxy match real mesh collision? | **this is Phase 3** — divergence is the finding |

The solver never touches the sim in its hot loop: the sim is used only at the
**boundaries** (target frame + scoring), so benchmarks stay fast (plan §4, §7-6).
The constant Phase-1 offset `C` is (re)measured at `PyBulletBackend` construction
and asserted `< 1e-4`, so the oracle is self-checking — if the DH model and the
URDF ever stop agreeing, construction fails instead of scoring against a wrong robot.

## Running the collision reconciliation (Phase 3)

```bash
# per-config proxy vs real-mesh comparison + per-link-pair mechanism, both arms
backend/.venv-sim/Scripts/python.exe -m bench.collision_parity        # n=3000/arm
backend/.venv-sim/Scripts/python.exe -m bench.collision_parity ur5 5000
```

Compares our capsule `self_collision_min_distance` against PyBullet's real-mesh
`getClosestPoints` per random config, and — the paper-grade addition — **attributes
each real collision and each dangerous *false-clear* to the link pair that drives
it**. The headline (`collision_parity.md`): the proxy is systematically optimistic
(UR5 real 36% vs proxy 17%; ~20% false-clear), and that optimism is **not diffuse —
it localizes to the tight forearm↔wrist cluster** (UR5: 73% of false-clears are
`forearm_link|wrist_2_link`; Franka: 73% are `panda_link5|panda_link7`), exactly
where a thin joint-axis capsule can't represent the bulky link mesh. No single
safety margin `δ` rescues it — the conclusion that the collision *rate* from the
proxy can't be trusted (only the *ordering* survives; see `sim_oracle_findings.md`).

## Running the second oracle + cross-check (Phase 4 — MuJoCo)

Needs the sim venv to also have MuJoCo (`uv pip install --python .venv-sim/Scripts/python.exe mujoco`):

```bash
# three-way FK agreement + collision cross-check + solver-edge replication
backend/.venv-sim/Scripts/python.exe -m bench.sim_crosscheck          # both arms, all scenarios
backend/.venv-sim/Scripts/python.exe -m bench.sim_crosscheck \
    --robots ur5 --solvers protein_fast trac_ik_style --trials 60
```

MuJoCo loads the **identical** URDF PyBullet does (not Menagerie's ur5e/panda), so
this isolates *engine* differences from *model* differences. It answers three
escalating questions (`sim_crosscheck.md`): **(A)** do our DH, PyBullet, and MuJoCo
agree on FK? (yes — to float noise: UR5 same base Rz(180°) offset, Franka identity,
residuals ~1e-8; an independent re-confirmation of the corrected modified-DH Panda);
**(B)** do two independent real-mesh engines agree the capsule proxy is optimistic?
(yes — sign-agree ~98%, corr ~0.99 on UR5; the Phase-3 finding is engine-independent);
**(C)** does V4's collision edge replicate on a second engine? (yes — the
V4 < TRAC < Multi < V1 ordering on UR5 holds identically on MuJoCo). Absolute rates
are engine-dependent (convex-hull mesh treatment); the *ordering* the paper rests on
is not.

## Files

- `models.py` — Phase 0: pins UR5/Panda URDFs (via `robot_descriptions`), records
  EE-link candidates & provenance, validates DH joint limits vs the URDF.
- `parity.py` — Phase 1: headless PyBullet FK oracle + the DH↔URDF comparison,
  constant-offset analysis, and an EE-link auto-scan.
- `pybullet_backend.py` — Phase 2: the evaluation oracle. `PyBulletBackend`
  (`fk`, `self_collision`, `self_collision_detail`, `set_config`, `reachable_target`,
  `native_ik`, `score`) with the constant Phase-1 frame offset baked in and
  self-verified at load. `self_collision_detail` attributes a collision to its
  driving link pair (Phase 3); `collision_link_names` exposes the checked link set
  so a second oracle can query the identical pairs (Phase 4 fairness).
- `mujoco_backend.py` — Phase 4: the MuJoCo second oracle. `MuJoCoBackend`, same
  surface as `PyBulletBackend`, on the **identical** URDF. `_mujoco_urdf` rewrites
  the URDF so MuJoCo can load it (absolute mesh paths, visuals stripped, `<mujoco>`
  compiler block, `fusestatic="false"` to keep the EE link frame). Reads link frames
  from `data.xmat` (no wxyz/xyzw hazard) and distances from `mj_geomDistance`.
- `../../bench/sim_benchmark.py` — Phase 2 runner: mirrors `master_benchmark.py`'s
  target distributions, then re-scores every `q_final` in the sim + a native-IK column.
- `../../bench/collision_parity.py` — Phase 3: per-config proxy vs real-mesh
  comparison + per-link-pair attribution of where the proxy fails.
- `../../bench/sim_crosscheck.py` — Phase 4: PyBullet vs MuJoCo vs our-DH cross-check
  (FK agreement, collision agreement, solver-edge replication).
- `__init__.py` — package marker; imports nothing heavy.
