# Migrating the IK Solvers to Real Robotics Simulators

**Status:** ✅ COMPLETE (Phases 0–4) as of 2026-07-07. Phase 1 FK parity validated
both arms (incl. the corrected modified-DH Panda); Phase 2 PyBullet oracle + sweep;
Phase 3 collision proxy reconciled (`backend/collision_parity.md`) with per-link-pair
mechanism; Phase 4 MuJoCo second oracle three-way-agrees on FK and cross-checks
collision (`backend/sim_crosscheck.md`). Deliverables in `backend/app/sim/` +
`backend/bench/`; findings in `backend/sim_oracle_findings.md`. Phase 5 (frontend
overlay/video) and Phase 6 (Isaac) remain optional/deferred. Original plan preserved
below.

**(historical) Status:** ACTIVE next task as of 2026-07-06 (no code written yet — read the Kickoff section first)
**Author:** Engineering
**Goal:** Validate and benchmark the ProteinIK solver family inside real robotics
simulators (PyBullet, MuJoCo, optionally Isaac Sim) instead of only our own
in-house kinematic simulation.

---

## 0. Why this document exists

Today every solver is validated against **our own forward kinematics** and a
**capsule-based self-collision proxy** (`backend/app/core/kinematics.py`). A solve
is called a "success" when our FK says the end-effector is within
`pos_err < 1e-3 m` and `orient_err < 1e-2 rad` of the target.

That answers *"does the solver converge against our math?"* It does **not** answer
*"does the solver actually work on the robot a roboticist would recognize?"* The
two can diverge whenever our DH model, our frame conventions, or our collision
proxy disagree with a real robot model. The only way to know is to run the same
solves inside an independent, widely-trusted simulator and re-score them there.

This document is the plan to do exactly that.

---

## Kickoff — START HERE (updated 2026-07-06, post-benchmark)

**This is the active next task.** A fresh chat should read this section, then §1–§9 below.

**Why this matters *now* (concrete, from the latest benchmarks — `backend/v1v4_full_benchmark.md`, `backend/opt2_full.md`, N=300):**
- **V4 (base) is the paper's star:** beats TRAC-IK on success in all 9 (arm × scenario) cells (98–100%); competitive speed; and a **real self-collision edge on planar + UR5** — e.g. UR5 open_space V4 collides **3%** vs TRAC-IK **17%**; on UR5/planar cluttered V4's mean clearance is **positive (clash-free)** where TRAC-IK's is negative (colliding).
- **BUT that collision edge is measured only against our own capsule proxy** (`self_collision_min_distance` in `kinematics.py`), and that proxy is **degenerate on Franka** — *every* solver, TRAC-IK included, "collides" 65–99% because the elbow link-pair (2,4) pins the metric (clearance set by the fixed offset `a[3]=0.0825` + joint q4; radii were hand-tuned, not from CAD). So on the one redundant arm we **cannot measure the collision edge at all**, and we can't fully trust it on the others.
- ⇒ **The single most important open validation for the paper is: is V4's collision edge REAL under true mesh collision, or an artifact of the capsule proxy?** Only a real simulator answers that. It is **Phase 3 below (the headline)**, and it is why this migration is worth more to the paper than any further latency optimization.

**Paper story arc this feeds (confirmed with the user, 2026-07-06):** V1 (the origin — staged fold) → **V4 base (the star)** → **o2** (kept to *one light, honest paragraph*: we pushed V4 for speed via warm-starting the fold; found a real **~25%-faster-for-~1–2 pp-collision** tradeoff, and even the *biologically-faithful* GroEL/IAM version (o2) couldn't beat it — an honest speed/quality tradeoff, not a headline) → **V5/V6 kept minimal** (mechanistically-explained negatives, full detail in the companion technical report). A real collision oracle would also let V6's "biophysical energy → better *quality*" thesis finally be **tested or definitively closed**.

**The one first step (do this before anything else):** `pip install pybullet`, then write the **Phase-1 FK-parity test for UR5** — compare our DH `end_effector_pose(spec, q)` against PyBullet's `getLinkState` over ~10k random configs and report the max position/orientation deviation. That single number decides whether the rest is "wire up an oracle" (easy) or "reconcile the robot model" (more work). Everything downstream is gated on it. See §3 and §5-Phase-1.

**Context a fresh chat needs:** deep per-solver code + finding notes are in `research_notes/` (files 00–07); full chronological history is in `raw_notes.md`; the V4 speed-opt experiment (o1/o2) lives in git branch `v4-speed-opt` (at `C:/Users/subik/v4opt`) with **base V4 untouched and nothing landed**. The solvers are already decoupled from our sim (uniform `RobotSpec` signature), so this is an **adapter + oracle** effort, not a solver rewrite (see §1, §4).

---

## 1. The key insight that makes this cheap

**The solvers are already decoupled from our simulation.** Every solver — classical
or protein-inspired — has the identical signature enforced by the registry
(`backend/app/solvers/registry.py`):

```python
solve(spec: RobotSpec,
      q0: np.ndarray,
      T_target: np.ndarray,        # 4x4 homogeneous target pose
      rng: np.random.Generator,
      collect_steps: bool = False) -> SolveResult
```

and depends only on `backend/app/core/kinematics.py` (DH-based FK/Jacobian plus the
capsule self-collision proxy). **No solver touches the frontend.** The frontend
(`frontend/`) is a React-Three-Fiber visualizer that only *replays* the recorded
step-trace — it simulates nothing.

Therefore this is **not a rewrite of the solvers**. It is building an
**adapter + oracle layer** around them so that a real simulator becomes the source
of truth for three things that are currently *our own private definitions*:

| Concern | Today (our sim) | Real simulator |
|---|---|---|
| Robot model | hand-typed DH tables in `kinematics.py` (`ur5_spec`, `franka_panda_spec`, `planar3dof_spec`) | URDF / MJCF mesh + joint model |
| Forward kinematics | our `_dh_transform` chain | sim's FK (`getLinkState` / `mj_forward`) |
| "Collision" | capsule segment-distance proxy (`self_collision_min_distance`) | real mesh / convex collision + self-collision |
| Success criterion | `pos_err<1e-3, orient_err<1e-2` vs **our** FK | same tolerances vs **sim's** FK |

The entire value of this migration is exposing the gap between rows 2–4. If a
solver "succeeds" by our FK but the sim's FK puts the end-effector somewhere else,
we have a **model-parity bug** — and catching that is the whole point.

---

## 2. Which simulator(s) — recommendation

Do **not** try to support everything at once. Phase them.

### Primary: PyBullet  *(do this first)*
- Pure `pip install pybullet`. Runs headless on Windows. No GPU / ROS needed.
- Ships UR5 and Franka Panda URDFs (`pybullet_data`).
- Built-in FK (`getLinkState`), real self-collision (`getClosestPoints` /
  `getContactPoints`), and its own IK (`calculateInverseKinematics`) — which gives
  us a **free baseline competitor** to benchmark our solvers against.
- Lowest friction to first signal. This is the validation workhorse.

### Secondary: MuJoCo  *(do this second)*
- `pip install mujoco`. Also Windows-friendly, no ROS.
- MuJoCo Menagerie repo has high-quality UR5e and Panda MJCF models.
- Best contact/collision fidelity and very fast — good for large benchmark sweeps
  and for cross-checking PyBullet. If PyBullet, MuJoCo, *and* our DH all agree, we
  are on very solid ground.

### Later / optional: Isaac Sim (Isaac Lab)
- Only if we need GPU-parallel benchmarking at scale (thousands of solves) or
  photoreal / sensor-in-the-loop. Heavy install (Omniverse, NVIDIA GPU). **Defer**
  until PyBullet + MuJoCo have proven the solvers out.

### Skipped for now
- **Gazebo / ROS-MoveIt** — Linux/ROS overhead; we are on Windows 11. Revisit only
  if MoveIt's planning ecosystem or ROS integration becomes an explicit deliverable.
- **Drake** — excellent kinematics, steeper setup. Revisit if needed.

**Commitment: PyBullet → MuJoCo, in that order.**

---

## 3. The one hard problem: DH ↔ URDF model parity

~80% of the risk lives here, so it is addressed first.

Our UR5/Panda specs are **standard DH** tables. URDF/MJCF use a different
representation (per-link fixed transforms + joint axes), and even "the same" robot
can differ in:
- base frame placement,
- tool0 / flange / end-effector frame,
- joint-zero offsets,
- joint sign conventions.

If we naively send a solver's `q_final` to a URDF whose joint zeros differ, the EE
pose will not match and *every* solve will look like a failure even when the solver
is correct.

### Mitigation — build a parity test BEFORE anything else
1. Load the sim's URDF for UR5.
2. For ~10k random `q`, compare `end_effector_pose(spec, q)` (our FK) against the
   sim's `getLinkState(ee_link, q)`.
3. Compute the max position and orientation deviation.

Three possible outcomes drive the rest of the design:

| Outcome | Meaning | Action |
|---|---|---|
| Match `< 1e-6` | Our DH already matches that URDF | Proceed; sim is a pure independent oracle |
| Match up to a fixed base/tool transform | Frames differ by a constant offset | Insert constant `T_base` / `T_tool` in the adapter |
| Structural mismatch (joint offsets/signs) | Models genuinely differ | Fix the DH table, **or** derive `RobotSpec` from the URDF (recommended — see §4) |

**Recommendation:** treat the URDF as the source of truth so that *the model we
benchmark is the model we solve*.

---

## 4. Architecture: an adapter layer, solvers untouched

Introduce a thin abstraction so the simulator is pluggable, while our fast numpy
core stays the in-process path.

```
backend/app/sim/
  __init__.py
  base.py              # SimBackend protocol (abstract)
  pybullet_backend.py
  mujoco_backend.py
  parity.py            # DH-vs-sim FK parity test (also run in CI)
  urdf_to_spec.py      # build RobotSpec from a URDF (optional but recommended)
```

### The `SimBackend` protocol (what every backend implements)

```python
load(robot_name) -> handle
fk(q) -> T_ee                              # sim's forward kinematics (4x4)
self_collision(q) -> (in_collision: bool, min_dist: float)
set_config(q) -> None                      # for visualization / recording
reachable_target(rng) -> (q_seed, T_target)# sample a config in-sim, FK it in-sim
native_ik(T_target, q0) -> q               # optional baseline competitor
```

### Key design rule

The **solver keeps consuming `RobotSpec`** (fast, no sim in the hot loop). The
simulator is used only at the **boundaries**:

- **Target generation:** sample a config in the sim, FK it in the sim → that is the
  `T_target`. Targets are now guaranteed reachable *and* expressed in the sim's
  frame.
- **Evaluation (the oracle):** take the solver's `q_final`, push it into the sim,
  read the sim's actual EE pose → compute the **real** pos/orient error, and read
  the sim's **real** self-collision. Score success against *that*, not against our
  own FK.

This gives the honest answer to "does it truly work" without putting a slow physics
step inside the solver loop. If we later want the solver itself to run on sim FK
(to test robustness to model mismatch), we can swap a `SimKinematics`
implementation behind the same `RobotSpec`-shaped interface — but start with the
boundary approach.

---

## 5. Phased implementation plan

### Phase 0 — Acquire & pin models  *(~½ day)*
- Add `pybullet` to `backend/requirements.txt` (later `mujoco` + `robot_descriptions`).
- Vendor / locate UR5 and Panda URDFs.
- **Validate the Panda joint-4 limit** `[-3.07, -0.07]` we encoded
  (`franka_panda_spec`) actually matches the chosen URDF. That "always-negative
  elbow" constraint is unusual and is a prime parity-mismatch suspect.

### Phase 1 — Model parity harness  *(1–2 days) — DO NOT SKIP*
- Implement `sim/parity.py`: random-config FK comparison (our DH vs PyBullet) for
  UR5 and Panda.
- Resolve mismatches per §3 (offset transform, DH fix, or URDF-derived spec).
- Make it a pytest so parity is permanently guarded.
- **This phase is the real deliverable of the whole effort.** Once FK matches,
  everything downstream is bookkeeping.

### Phase 2 — PyBullet evaluation oracle  *(2–3 days)*
- Implement `pybullet_backend.py` (`load`, `fk`, `self_collision`, `set_config`,
  `reachable_target`, `native_ik`).
- Write a benchmark runner `backend/bench/sim_benchmark.py` that mirrors
  `_run_benchmark_sync` in `app/main.py` but: generates targets via the sim, runs
  each solver on `RobotSpec`, then **re-scores `q_final` against sim FK + sim
  collision**.
- Emit a comparison report: for each solver — *our-FK success rate* vs *sim-FK
  success rate*, *our capsule collision-rate* vs *sim real-collision-rate*, plus
  PyBullet's native IK as a baseline column.

### Phase 3 — Reconcile the collision metric  *(1–2 days) — HEADLINE RESULT* ✅ DONE
- Our capsule proxy (`self_collision_min_distance`) is geometric and approximate.
  Compare its `min_self_distance` against PyBullet's real `getClosestPoints` across
  many configs. Quantify the divergence.
- **Why this is a headline, not a sanity check:** the protein-inspired solvers'
  core differentiator is their collision-aware energy terms
  (`collision_energy`, `frustration_index` in `protein_energy.py`). Those terms
  optimize *against the capsule proxy*. This phase tests whether that advantage is
  **real** or an **artifact of an approximate collision signal**.
- **RESULT** (`backend/bench/collision_parity.py` → `backend/collision_parity.md`,
  `sim_oracle_findings.md` §2–3): the proxy is **systematically optimistic** (UR5
  real 36% vs proxy 17%, ~20% dangerous false-clear; Franka real 10% vs proxy 0.5%).
  The optimism **localizes** to the tight forearm↔wrist cluster (73% of UR5
  false-clears are `forearm_link|wrist_2_link`; 73% of Franka's are
  `panda_link5|panda_link7`). No safety margin `δ ≤ 0.15 m` rescues it. **Verdict:
  proxy collision *rates* can't be trusted; V4's collision *edge* (ordering vs
  baselines) survives but shrinks** — and the "improve V4 by giving it a better fast
  collision signal" idea (V7) was tried and scrapped for exactly this reason
  (`raw_notes.md` Entry 37).

### Phase 4 — MuJoCo backend  *(~2 days)* ✅ DONE
- Implement `mujoco_backend.py` to the same `SimBackend` protocol. **Decision that
  matters:** load the **identical** URDF PyBullet uses (classic `ur5_robot.urdf` /
  franka_ros `panda.urdf`), *not* Menagerie's ur5e/panda — so the cross-check
  isolates *engine* differences from *model* differences.
- Cross-check: PyBullet vs MuJoCo vs our DH. Three-way agreement = high confidence.
- **RESULT** (`backend/bench/sim_crosscheck.py` → `backend/sim_crosscheck.md`):
  **(A) FK** — DH ≡ PyBullet ≡ MuJoCo to float noise (UR5 same base Rz(180°),
  residual 8.9e-12 m; Franka identity, 8.5e-16 m) — an independent second engine
  re-confirms the corrected modified-DH Panda. **(B) Collision** — both real-mesh
  engines agree the proxy is optimistic (UR5 PB 38% ≈ MJ 36%, sign-agree ~98%,
  corr ~0.99), so Phase 3 is engine-independent. **(C) Solver edge** — V4's
  `V4 < TRAC < Multi < V1` collision ordering on UR5 replicates identically on
  MuJoCo. Guarded by `tests/test_sim_parity.py` (Phase-4 tests).

### Phase 5 — Reporting / visualization  *(optional, 1–2 days)*
- Either record solved trajectories as PyBullet/MuJoCo videos, or add a
  `?backend=sim` mode to the FastAPI app so the existing frontend overlays
  sim-validated poses. Lowest priority — the Phase 2 numbers are what answer
  "does it work."

### Phase 6 — Isaac Sim  *(only if scale demands it)*

---

## 6. Metric mapping (keep results comparable)

Keep the existing `SolveResult` schema; **add** sim-measured fields alongside the
self-reported ones so we can diff them directly:

| Self-reported (our FK) | Sim-measured (oracle) |
|---|---|
| `pos_error` | `sim_pos_error` |
| `orient_error` | `sim_orient_error` |
| `min_self_distance` (capsule) | `sim_min_self_distance` (real) |
| `success` (our FK) | `sim_success` (sim FK) |
| — | `pybullet_native_ik` success/time (baseline) |

V5 (`protein_homotopy`) diagnostics — `conflict_index`, `lambda_final`,
`difficulty_score` — have no sim equivalent. **Keep them as-is; do not replace
them.** They are internal solver diagnostics, orthogonal to sim validation.

**Headline result we are after:**
> "X% of solves our sim calls successful are also successful in PyBullet/MuJoCo,
> and our self-collision rate is within Y% of the real one."

---

## 7. Risks & gotchas (ranked)

1. **DH/URDF frame mismatch** (§3) — highest risk; Phase 1 exists solely to kill it.
2. **Quaternion / rotation conventions** — our API uses **xyzw** quaternions
   (`schemas.py: TargetPose`); PyBullet is **xyzw**, MuJoCo is **wxyz**. Easy to get
   a silent orientation flip. Add an explicit conversion test.
3. **Panda joint-4 constraint** — verify against the chosen URDF, or "reachable"
   targets won't actually be reachable in-sim.
4. **Collision-mesh fidelity** — URDF *visual* meshes differ from *collision*
   meshes; ensure the backend queries collision geometry, and decide whether to
   enable adjacent-link filtering to match our "non-adjacent only" rule
   (`self_collision_min_distance_from_chain`).
5. **`planar3dof` has no standard URDF** — either build a trivial 3-link URDF or
   keep it as an our-sim-only validation arm (it is our analytical ground truth via
   `analytical_planar3dof` anyway, so it arguably does not need a real sim).
6. **Performance** — never call the sim inside the solver loop; use it only at the
   target-generation and scoring boundaries, or benchmarks will crawl.

---

## 8. Effort summary & sequencing

| Milestone | Phases | Effort | Outcome |
|---|---|---|---|
| Core validation | 0–3 (PyBullet) | ~1 week | The "do the solvers truly work" answer + collision-proxy reality check |
| Cross-validation | 4 (MuJoCo) | ~2 days | Independent second oracle |
| Scale / fidelity | 5–6 | optional | Videos, frontend overlay, GPU-scale sweeps |

**Recommended first concrete step:** write the Phase 1 FK-parity test for UR5 in
PyBullet. It is small, and its single output number determines whether the rest is
"wire up an oracle" (easy) or "fix the model definitions" (more involved).
Everything else is gated on that number.

---

## 9. Appendix — relevant code references

- Solver dispatch & uniform signature: `backend/app/solvers/registry.py`
- Robot models (DH) + FK + Jacobian + capsule collision: `backend/app/core/kinematics.py`
  - `RobotSpec`, `ur5_spec`, `franka_panda_spec`, `planar3dof_spec`
  - `end_effector_pose`, `forward_kinematics_chain`, `geometric_jacobian`, `pose_error`
  - `self_collision_min_distance`, `self_collision_min_distance_from_chain`
- Result/step schema: `backend/app/core/types.py` (`SolveResult`, `SolveStep`)
- Energy terms used by protein solvers: `backend/app/solvers/protein_energy.py`
  (`collision_energy`, `frustration_index`, `total_energy_fast`)
- Existing benchmark runner to mirror: `backend/app/main.py` (`_run_benchmark_sync`)
- Target/scenario generation: `backend/app/api/scenarios.py`
  (`open_space`, `near_singular`, `cluttered`)
- API request/response schemas (quaternion convention): `backend/app/api/schemas.py`
- Quaternion ↔ transform conversions: `backend/app/api/quaternion.py`
