# ProteinIK — Detailed Methodology

> **How to read this:** this is the deep, precise version of "how the solvers work" — the backbone of the paper's
> Methods section. Every subsection opens with a **plain-English summary** (so you can follow it), then gives the
> **exact detail** (formulas, parameters, code anchors) so it's paper-grade and reproducible.
>
> Names: **StagedFold** (V1), **KineticFold** (V4), **LangevinFold** (V6). Umbrella method: **ProteinIK**.
> Every number here traces to the code in `backend/app/` and the committed result files.

---

## 1. Problem formulation — inverse kinematics as a folding search

**Plain version.** A robot arm and a protein are both *chains of rigid parts whose only freedom is the rotation
between neighbours*. "Solving IK" (find joint angles that put the hand at a target pose, without the arm hitting
itself) is the same kind of search as "folding" (find the backbone rotations that reach the low-energy native shape,
without atoms overlapping). We formalise that correspondence and then design solvers around it.

**Formal setup.**
- A robot with `n` revolute joints has configuration `q ∈ ℝⁿ` (the joint angles), bounded by limits `q ∈ [q⁻, q⁺]`.
- Forward kinematics `T(q) ∈ SE(3)` gives the end-effector pose. The target is `T_target`.
- The **pose error** is a 6-vector `e(q) = [Δp; Δω]` — position difference (3) and orientation as the rotation
  vector (axis-angle) of `R_target · R(q)ᵀ` (3). Success = `‖Δp‖ < 1 mm` **and** `‖Δω‖ < 10 mrad`.
- **Self-collision** is measured by a signed clearance `d(q)` (positive = clear, negative = interpenetrating);
  §2.3 defines it. A solve is **clean** if it reaches the target *and* `d(q) ≥ 0`.
- The solver searches the landscape of a combined objective `E(q) = E_target(q) + (constraint terms)` that is
  **rugged and multimodal** — many local minima, singular regions where the Jacobian loses rank, and self-collision
  basins — exactly the structure of a protein folding landscape.

**The correspondence we build on** (this becomes Figure 1 and the spine of the Introduction):

| Protein folding | Inverse kinematics |
|---|---|
| Backbone dihedral angles φ/ψ (the soft DOF) | Joint angles `q` (the DOF) |
| Rigid bonds / fixed bond lengths | Fixed link lengths (FK constraints) |
| Native (folded) state | The IK solution configuration |
| Free-energy funnel | Convergence basin to the target |
| Rugged landscape / kinetic traps | Local minima / failed solves |
| Excluded volume (sterics) | Self-collision avoidance |
| Hydrophobic collapse | Coarse approach to the target region |
| Secondary structure (local order) | Local joint settling |
| Molecular chaperone (GroEL) | Restart / rescue from a stuck state |
| Kinetic partitioning (fast vs. slow folders) | Easy vs. hard targets |

**Novelty scoping (stated up front, honestly).** We do **not** claim novel energy terms; each ingredient below has
precedent in the IK/optimisation literature. The contribution is (i) the **folding-inspired *sequencing*** of those
ingredients (StagedFold), and (ii) using folding's **kinetic partitioning** as a *compute schedule* that makes the
sequenced solver competitive (KineticFold).

---

## 2. Shared core (used by every solver, including the baselines)

**Plain version.** All solvers plug into one common interface and one common set of physics helpers (forward
kinematics, Jacobian, energy terms, a fast self-collision check), so no solver gets an unfair advantage and the
comparison is apples-to-apples.

### 2.1 Uniform interface
Every solver implements `solve(spec, q0, T_target, rng) → SolveResult`, where `SolveResult` carries
`success, q_final, pos_error, orient_error, iterations, wall_time_ms, min_self_distance` (the clearance `d`),
`joint_limit_violations, restarts`. This uniformity is what lets the benchmark share identical targets across all
solvers.

### 2.2 Kinematics
- **Forward kinematics** via Denavit–Hartenberg transforms `Tᵢ = Rot_z(θ)·Trans_z(d)·Trans_x(a)·Rot_x(α)`, chained
  along the arm. (Franka uses the **modified/Craig DH** convention — see §6.1; this was a real correctness fix.)
- **Geometric Jacobian** `J ∈ ℝ⁶ˣⁿ`: `J_v,i = z_i × (p_end − p_i)`, `J_w,i = z_i`.
- **Pose error** as above (axis-angle orientation). Validated against a closed-form planar solver to `< 1e-9 m`.

### 2.3 Energy terms (the "physics" the folding solvers minimise)
From `app/solvers/protein_energy.py`. `d` is the capsule clearance (§2.4).

| Term | Formula | Meaning |
|---|---|---|
| Target | `E_target = ‖Δp‖ + 0.3·‖Δω‖` | reach the goal pose |
| Joint-limit barrier | soft barrier active within `margin = 0.05 rad` of a limit, weight `×50` | stay in-range |
| Collision | `0` if `d ≥ 0.05`; `((0.05−d)/0.05)²·10` if `0 ≤ d < 0.05`; `100 + |d|·100` if `d < 0` | avoid self-overlap |
| Neighbour smoothness | `Σ (qᵢ₊₁−qᵢ)² · 0.5` | discourage kinked configurations |
| Neutral-pose anchor | `Σ (qᵢ − q_neutralᵢ)² · 0.5`, `q_neutral = 0` | a "relaxed" default (used only in Stage 1) |

### 2.4 The self-collision proxy (what the solvers optimise against)
`self_collision_min_distance(spec, q)`: model each link as a **capsule** (a line segment between consecutive joint
origins, inflated by a per-link radius). For every **non-adjacent** segment pair, compute the segment-to-segment
closest distance minus the two radii → a signed distance; return the global minimum. Negative = interpenetration.

This is a **fast approximation, not CAD geometry** — the radii are hand-tuned for self-consistency. It is the object
the solvers actually reason about *and* the thing our Validation section (§6.4) later checks against two real
physics engines. Key honest facts carried into the paper:
- It is **systematically optimistic** (real meshes collide more) — so we only ever quote collision as a *comparison
  between solvers*, never as an absolute rate.
- On the redundant Franka it is **dominated by one fixed structural pair (the elbow)**, so it is nearly insensitive
  to the 7th joint — the reason the Franka collision comparison is a tie (§4 results), not a lead.

---

## 3. StagedFold (was V1) — the folding *process* as an algorithm

**Plain version.** Instead of minimising one energy from step 1 (what every classical method does), StagedFold runs
the arm through the **same ordered stages a protein uses to fold**: first let the joints settle locally *without even
looking at the goal*, then pull coarsely toward the target region, then run a narrowing search that homes in, then —
if it gets stuck — call a "chaperone" that only re-jiggles the misbehaving joints, and finally check the solution is
*stable*, not balanced on a knife-edge. The individual moves are standard IK; the **order** is the idea.

Defaults: `max_iters = 200`, `pos_tol = 1e-3`, `orient_tol = 1e-2`.

### Stage 1 — Local-blind relaxation *(secondary-structure analog)*
Gradient-free coordinate descent, one joint at a time: try `qᵢ ± 0.3 rad`, keep whatever lowers a **target-blind**
local energy (`neutral + neighbour-smoothness + joint-limit` only — the target pose is *never consulted*). 6 sweeps.
- **Why it's unusual:** no production IK method starts by ignoring the target. This mirrors local secondary
  structure forming before the global fold, and it seeds the later stages from a relaxed, in-limits configuration.

### Stage 2 — Coarse collapse *(hydrophobic-collapse analog)*
A deliberately **detuned** damped-least-squares pull on the full 6-D pose error: damping `λ² = 0.15² = 0.0225`,
step scale `0.4`, 10 iterations. This is the **first stage that sees the target** — it moves the hand into the right
neighbourhood without trying to be precise (like a protein collapsing to a compact molten globule).

### Stage 3 — Funnelled narrowing search *(folding-funnel analog)*
The main refinement, a hybrid of:
- **(a)** a gradient-free coordinate-wise stochastic local search inside a **shrinking radius** (`r₀ = 0.5`, decays
  `×0.985` per iteration), firing every other iteration, **greedy accept-if-better** — *not* Metropolis (there is no
  temperature here; this is an important honesty point that distinguishes StagedFold from KineticFold and
  LangevinFold, both of which *do* use Metropolis/thermal acceptance);
- **(b)** one damped-least-squares gradient step per iteration (finer damping `0.05²`).
Weights over the combined energy: `w_target = 3.0, w_limit = 1.0, w_collision = 2.0, w_smooth = 0.3`.

### Stage 4 — Scoped chaperone rescue *(GroEL/chaperone analog)* — the key differentiator vs. TRAC-IK
- **Stall detection:** keep a window of the last 10 energies; if progress over the window `< 2e-4`, a rescue fires.
- **Identify the "misfolded" joints:** one-sided finite-difference sensitivity (perturb each joint by `0.05 rad`);
  the joint with the largest energy contribution is the culprit.
- **Escalation ladder:** scopes grow `[n/6, n/2, 5n/6, n]` (UR5 → `[1,3,5,6]`). A rescue re-randomises a
  **contiguous window of `scope` joints centred on the culprit**, leaving the rest of the settled chain untouched;
  the final rung is a **full random reseed** of the whole chain.
- **Honest framing (important):** StagedFold **starts scoped and escalates** — on a persistently stuck target its
  last rung *is* a global restart, i.e. it converges to TRAC-IK-like behaviour. So the accurate claim is "scoped
  first, global only as a last resort," not "never restarts globally."

### Stage 5 — Stability-gated termination *(native-state stability analog)*
Once converged, jitter the solution `5×` (jitter ≈ 1 mm of tip motion) and reject it if the energy jumps past a
threshold (`≥ 4/5` jittered trials failing ⇒ mark not-successful). This rejects knife-edge solutions the way
Anfinsen's native state must be a *stable* minimum, not any minimum.

### StagedFold — honest verdict + ablations (these are assets, not weaknesses)
- **Verdict:** beats the *simple* classical baselines (Jacobian-DLS, CCD, FABRIK) by wide margins, but does **not**
  beat the production baselines (TRAC-IK, Multi-start) on success — which is precisely the motivation for
  KineticFold.
- **Reverted experiments we report** (evidence that the specific choices matter): a pure neighbour-coupling Stage 1
  with no neutral anchor *dropped* cluttered success 90.0 → 86.0%; rotamer-library-biased proposals improved mean
  clearance but crashed cluttered success (90.0 → 67–76%); an allostery-inspired compensating step traded success
  for a small clearance gain and was removed. Each was tried, measured, and reverted — the paper cites them to show
  the sequencing/choices are empirically load-bearing, not decorative.

---

## 4. KineticFold (was V4) — kinetic partitioning makes it competitive *(the star)*

**Plain version.** StagedFold's problem wasn't the average solve — it was the **slow tail**: a small fraction of
hard targets ran the whole expensive stochastic machinery and ate most of the total time. KineticFold fixes this
with *another* folding idea. Real proteins undergo **kinetic partitioning**: some molecules fall straight down a
smooth funnel to the native state ("downhill" / barrierless folding, no search needed), while others get trapped and
need the chaperone. So KineticFold **tries the cheap downhill fold first** and only pays for the full staged search
on the targets that genuinely get *stuck*. The biology decides *when* to spend compute; the optimisation decides
*how well* each attempt solves.

### 4.1 The diagnosis (why a micro-optimisation couldn't fix it)
On the old always-run-everything fold, the slowest **~10%** of targets consumed **~57%** of total wall time. A
per-step speedup can't move a tail like that — measured, a bit-identical micro-pass gave only **1.1–1.4×**. The tail
is caused by *entering the expensive per-fold search at all*, so the fix has to be structural.

### 4.2 Layer 1 — the barrierless-first ensemble (the tail-killer)
A single budget `max_replicas = 6` governs two phases:
- **Phase A — barrierless (downhill) folds.** Each replica runs a cheap **Levenberg–Marquardt polish** (≤ 30 LM
  steps). Replica 0 seeds from `q0`; the rest from random configurations. As soon as a replica **converges to a
  sterically clean** solution (`d ≥ 0`), Phase A stops early with a success.
- **The frustration criterion.** The landscape is declared **"frustrated"** iff, after the LM restarts, **no**
  converged replica is clash-free. Only then does it escalate.
- **Phase B — the full staged fold**, `_fold_once`, fires **only on frustrated targets**. This is a StagedFold-style
  fold (coarse collapse → funnel → chaperone rescue → stability gate) but with a **true Metropolis-accepted funnel
  and an LM endgame** — a refinement over StagedFold's greedy Stage 3. Extra caps keep it cheap: stop on the first
  clean fold, or after at most two collision-aware converged folds.
- **Biological grounding:** trying spontaneous (barrierless) folding first and invoking the chaperone only on
  failure is *how GroEL actually works* — so this ordering is **more** faithful to folding than always running the
  full machinery, not a departure from it.

### 4.3 Layer 2 — allocation-light FK primitives (the per-step floor)
Independently, the inner loop is made cheap and **bit-identical** to the reference kinematics:
- `_fast_chain` builds the whole DH chain into a preallocated buffer (no per-joint array literals);
- `_incremental_chain` rebuilds only the **suffix** when the Metropolis sweep perturbs a single joint;
- `_fast_pose_jac` fuses the pose + Jacobian into one FK pass; a shared constant `6×6` identity replaces per-step
  allocation.
- **Verified** bit-identical to the reference FK (tested on UR5 + planar, 500 configs each; the paper will either
  extend this test to Franka or state the tested scope precisely — we will *not* over-claim "9000 configs across all
  three arms," which the committed tests don't cover).

### 4.4 What we tried and rejected (honesty)
Naive tail-edits that keep the fold order but just spend less — cap replicas, bail earlier, fewer iterations — bought
little speed and **destroyed the headline win** (e.g. Franka open-space success collapsed to **71.7%** at
`cap_replicas = 2`). The cost is the *per-fold* search, not the *number* of folds — which is exactly why the
kinetic-partitioning gate (skip the search entirely when unfrustrated) is the right lever.

---

## 5. LangevinFold (was V6) — the literal folding simulation *(condensed; full version → thesis)*

**Plain version.** StagedFold borrows folding's *process*; LangevinFold runs the *actual physics*. It treats the arm
as a coarse-grained molecule (one bead per joint) under thermal motion, and lets it **fold under a real biophysical
free energy**, cooling until it freezes into place. It is far too slow for normal use (seconds per solve), but under
real-mesh collision testing it produces the **cleanest** solutions of any solver — evidence that faithful biology
buys *quality*. The paper shows only this punchline; the full treatment goes in the thesis.

**The free energy** (minimised by the dynamics):
`F(q; T) = E_task + E_LJ + E_HB − T · S_conf`
- **E_task** — the only non-folding term: `w_task · (‖Δp‖ + 0.3·‖Δω‖)` (the boundary condition).
- **E_LJ** — a full 6-12 Lennard-Jones potential **with attraction**, uniform ε, over non-adjacent bead pairs
  (packing / contacts; the attractive well is what has no IK equivalent).
- **E_HB** — a directional "hydrogen-bond" term whose orientation is the **triplet-plane normal** of three
  consecutive beads (secondary-structure analog; interior pairs only, so the planar arm has none).
- **S_conf = log Ω** — a **conformational entropy**: the log of the clash-free, in-limits accessible micro-volume
  around `q`, estimated with a fixed Gaussian cloud (common random numbers). Target-blind, collision-aware; it
  opposes collapse and is provably *not* manipulability.

**Dynamics.** Overdamped Langevin (Euler–Maruyama): `q ← clip(q + clip_norm(−∇F·Δt + √(2TΔt)·ξ, max_step = 0.25))`,
with a single self-consistent temperature `T` cooling as `T_t = max(T_glass, T_start·e^{−t/τ})`. There is **no
Metropolis test** — motion is pure force + thermal noise (the defining distinction from simulated annealing).

**Endgame.** At `T → 0` the noise vanishes and the dynamics become a damped-Newton/LM consolidation (the native-state
selection). Among consolidated candidates it selects the **clash-free** one of minimum enthalpy (excluded volume as a
hard constraint), with a multi-start ensemble (`n_ws = 10 + 2·max(0, n−6)`) and an Anfinsen jitter stability check.

**Honest status.** Its measured collision advantage on UR5 traces partly to multi-start + hard clash-free selection,
and its core "biophysics → quality" claim is only *measurable* on a real mesh oracle (the capsule proxy can't see
it). Both nuances are carried in full in the thesis; the paper uses only the validated headline (cleanest UR5
self-collision, at a latency cost).

---

## 6. Experimental protocol

**Plain version.** Three arms of increasing hardness, three difficulty settings, a strong field of standard solvers,
every solver handed the *same* targets, and every solution independently re-checked in two real physics simulators.

### 6.1 Robots
| Arm | DOF | Notes |
|---|---|---|
| Planar 3-DOF (RRR) | 3 | link lengths `[0.4, 0.3, 0.2]`; has an **exact closed-form solver** → ground truth |
| UR5 | 6 | non-redundant; standard DH; the primary tuning + validation arm |
| Franka Panda | 7 | **redundant**; **modified/Craig DH** (a corrected kinematics fix — old standard-DH FK was ~1.4 m wrong); tight asymmetric limits |

### 6.2 Scenarios (target generators)
- **open_space** — uniform random reachable target, independent random start.
- **near_singular** — rejection-sample targets of low Yoshikawa manipulability `√det(JJᵀ)` (per-arm thresholds),
  i.e. near kinematic singularities.
- **cluttered** — rejection-sample **low-self-clearance** targets (early-exit at `d < −0.03`), i.e. poses that force
  the arm near self-collision. Read alongside the clearance metric, not success alone.

### 6.3 Baselines (the field to beat)
- **Jacobian-DLS** — single-trajectory damped least squares, no restart.
- **CCD** — cyclic coordinate descent (base→tip). *(Also the CCD that famously crossed into protein loop closure — our Related Work anchor.)*
- **FABRIK** — forward-and-backward reaching, adapted to revolute joints.
- **TRAC-IK-style** — DLS + stuck detection + **full random restart**. *The key baseline to beat* (global, not scoped, rescue).
- **Multi-start** — 8 independent DLS seeds, best by error.
- **Analytical (planar only)** — closed-form, exact, ground truth.

### 6.4 Metrics & fairness
- Targets are generated **once per (arm, scenario, seed)** and **shared across all solvers** — no solver sees an
  easier draw. Untimed warm-up per cell; multiple seeds; wall-clock timing.
- Reported: **success %** (`‖Δp‖<1 mm ∧ ‖Δω‖<10 mrad`); latency **mean + p50/p95/p99** (the tail is a first-class
  metric, not just the mean); **self-collision %** and **mean clearance**; joint-limit violations; restarts.
- **Where it wins** experiments (deployment roles): planning goal-sampling yield, offline clean-solve rate,
  fallback-rescue rate, and the **DOF-scaling** sweep (planar 4→16 joints) that is the paper's climax.

### 6.5 Validation harness (the differentiator)
Every solver's final configuration is **re-scored in two independent simulators** — PyBullet and MuJoCo — both
loading the *identical* URDF and querying the *identical* non-adjacent link pairs:
- **FK parity:** our DH ≡ PyBullet ≡ MuJoCo to floating-point noise → every *success* claim is independently true on
  two engines (including the corrected Franka kinematics).
- **Collision reality check:** our capsule proxy is optimistic; both engines agree it is, and agree with each other
  (sign-agreement ~98%, correlation ~0.99 on UR5). So collision is reported only as a *ranking* of solvers.
- **Edge replication:** the UR5 collision ranking (KineticFold cleanest of the practical solvers; LangevinFold
  cleanest overall) is **identical on both engines**; the Franka *tie* also reproduces on both. "Solve once, score
  three ways" (our proxy + PyBullet + MuJoCo) is the single reproducible artifact behind the results tables.

---

## 7. Reproducibility

- Solvers live in `backend/app/solvers/`; shared core in `backend/app/core/`; simulators in `backend/app/sim/`.
- Proxy benchmark: `backend/master_benchmark.py`. Real-engine benchmark (both simulators, solve-once-score-three-ways):
  `backend/bench/master_sim_benchmark.py` (runs in the `backend/.venv-sim` Python 3.12 environment with
  pybullet + mujoco). All result tables are regenerated from these scripts.
</content>
