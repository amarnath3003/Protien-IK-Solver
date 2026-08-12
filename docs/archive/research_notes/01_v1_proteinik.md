# V1 — ProteinIK Staged-Fold Solver

> Source files: `backend/app/solvers/protein_ik.py` (**PI**), `protein_energy.py` (**PE**),
> `rotamer_library.py` (**RL**), `backend/app/core/kinematics.py` (**K**).

## 0. Dispatch context (what actually runs in production)

The registry wires `protein_ik` through `_wrap_rng(solve_protein_ik)` (registry.py:49); `_wrap_rng`
(registry.py:37–40) calls the solver with only `(spec, q0, T_target, rng, collect_steps)`. So every keyword
in `solve_protein_ik` (PI:115–132) uses its **default**, and two feature flags are **always off**:

- `use_vectorial_folding=False` (PI:130) → the domain-decomposition variant (PI:214–309) never runs.
- `use_rotamer_bias=False` (PI:131) → the rotamer-biased proposal branch (PI:336–366) never fires.

Default budgets (PI:120–131): `max_iters=200`, `pos_tol=1e-3`, `orient_tol=1e-2`, `stage1_iters=6`,
`stage2_iters=10`, `stuck_window=10`, `stuck_eps=2e-4`, `max_rescues=6` (the last is vestigial — never referenced).

## 1. The five folding stages as actually implemented

**Stage 1 — Local-blind relaxation (secondary-structure analog) — PI:161–196.** Gradient-free coordinate
descent, one joint at a time (PI:185), trying `q[i]−0.3` and `q[i]+0.3` (PI:188), clipped to limits, keeping
any that lowers local energy. Objective = `neutral_pose_energy + neighbor_smoothness_energy + joint_limit_energy`
only (PI:187,191). **Target-blind** — `T_target` never referenced. `stage1_iters=6` sweeps; ±0.3 rad; neutral
pose `q_neutral = np.zeros(n)` (PI:135).

**Stage 2 — Coarse collapse (hydrophobic-collapse analog) — PI:198–213.** Damped-least-squares Jacobian pull
on the full 6-D pose error, deliberately detuned: damping `lam2=0.15²=0.0225` (PI:207), `coarse_scale=0.4`
(PI:209), `stage2_iters=10`. **First stage to consult the target.** Can set `success=True` early (PI:212–213).

**Stage 3 — Funneled narrowing search (folding-funnel analog) — PI:311–392.** Hybrid: (a) gradient-free
coordinate-wise stochastic local search within a shrinking radius, firing **every other iteration** (`it%2==0`,
PI:334) — greedy accept-if-better (PI:372–374), **NOT Metropolis** (no temperature); plus (b) one DLS gradient
step per iteration with damping `0.05²` (PI:382, finer than Stage 2). `search_radius` init 0.5 (PI:313), decays
`×0.985` each iter (PI:314,386). Weights `w_target=3.0, w_limit=1.0, w_collision=2.0, w_smooth=0.3` (PI:316).

**Stage 4 — Scoped stuck-rescue — PI:394–468.** See §2. Interleaved inside the Stage-3 loop.

**Stage 5 — Stability-checked termination (native-state analog) — PI:470–504.** See §3. Runs once if `success`.

## 2. Scoped chaperone rescue (Stage 4) — the claimed TRAC-IK differentiator

**Stall detection** (PI:406–410): after each Stage-3 iter, append `cur_energy` to `recent_energies`; once the
window (10) is full, `progress = window[0]−window[-1]`; if `progress < stuck_eps` (2e-4) a rescue fires.

**"Misfolded" joint identification** — `_per_joint_energy_contribution` (PI:88–112): one-sided finite-difference
sensitivity, perturb each joint by `eps=0.05` rad, `contributions[i]=|E(q_pert)−E(base)|`; base computed once.
Weights match Stage 3 (3,1,2,0.3). Worst joint = `argmax(contributions)` (PI:439).

**Escalation ladder (IAM / GroEL analog)** — PI:418–446. `scope_sizes = sorted(set([max(1,n//6), max(1,n//2),
max(1,5n//6), n]))` → UR5 `[1,3,5,6]`, Franka `[1,3,5,7]`. Scope grows with each consecutive rescue
(`scope_idx = min(rescues_used−1, …)`, PI:425).
- **Partial unfold** (`scope<n`, PI:432–446): a **contiguous** window of `scope` joints centered on `worst` is
  re-randomized (`q[j]=spec.random_config(rng)[j]`).
- **Full unfold** (`scope≥n`, PI:428–431): `q=spec.random_config(rng)` — a **global random restart of the whole
  chain**. After any rescue: `search_radius`→0.5, window cleared, energy recomputed.

**⚠️ Mechanism vs claim.** The docstring (PI:33–39) and README (README.md:80,83) describe Stage 4 as purely
scoped ("unlike TRAC-IK's global random restart"). The code's final rung *is* a global random restart. Accurate
framing: V1 **starts scoped and escalates**, converging to TRAC-IK-style behavior on persistently-stuck targets.

Claimed differentiator, quoted (PI:33–39): *"the joint(s) contributing most to the current energy (the
'misfolded' substructure) are identified and perturbed on their own, leaving the rest of the already-settled
chain untouched… This is scoped/local, unlike TRAC-IK's global random restart — the comparison this project
cares about most."*

## 3. Stability-gated termination (Stage 5) — PI:470–504

Runs only `if success`. Baseline `base_combined = ‖err_pos‖ + 0.3·‖err_orient‖` at converged `q`. Jitter
`jitter_std = clip(1e-3/max(1,arm_reach), 1e-4, 5e-3)` (≈1 mm tip displacement; UR5≈8.4e-4, Franka≈7.2e-4,
Planar=1e-3). 5 trials (PI:477); a trial fails if `combined_j > base_combined + 10·(pos_tol+0.3·orient_tol)`
(=0.04). If ≥4/5 fail → `success=False`, phase `stability_check_failed` (PI:500–502). Note: on failure the code
only sets the flag and returns — the "keep refining" comment (PI:499) is dead.

## 4. Energy functions (protein_energy.py) and which stages use them

| Function | Formula / meaning | Used by V1? |
| :-- | :-- | :-- |
| `target_energy` (PE:23) | `‖err_pos‖+0.3·‖err_orient‖` | imported but re-implemented inline |
| `joint_limit_energy` (PE:30) | soft barrier within `margin=0.05` of limits, `×50` | Stages 1, 3, 4 |
| `collision_energy` (PE:46) | on min self-dist `d`: 0 if `d≥0.05`; `((0.05−d)/0.05)²·10`; `100+|d|·100` if `d≤0` | Stages 3, 4 (via `total_energy_fast`) |
| `neighbor_smoothness_energy` (PE:62) | `Σ(diff q)²·0.5` | Stages 1, 3 |
| `neutral_pose_energy` (PE:70) | `Σ(q−q_neutral)²·0.5` | **Stage 1 only** |
| `ramachandran_pair_energy` (PE:78) | 2-D joint-pair well, `×2` | **not used** by V1 |
| `go_contact_energy` (PE:91) | Gō native-contact | **not used** by V1 |
| `frustration_index` (PE:109) | per-joint local/global conflict "for chaperone rescue" | **not used** — V1 uses `_per_joint_energy_contribution` instead |
| `total_energy_fast` (PE:139) | one-FK combined weighted energy; "numerically identical to summing" | workhorse for Stages 3, 4 |

## 5. Documented reverted mechanisms (honesty assets — keep)

- **Pure neighbor-coupling Stage 1 (no neutral anchor):** cluttered **90.0%→86.0%**, reverted (PI:166–182).
  Diagnosis: smoothness alone has a zero-energy minimum at any constant config, including near limits.
- **Rotamer-library-biased proposals:** improves mean self-distance every scenario but costs success —
  worst cluttered **90.0%→67.3%** unannealed, best variant still 76.0%. Disabled by default (PI:336–360, RL).
- **Allostery-inspired compensating step:** mean self-dist −0.0074→−0.0024 but success **90.0%→88.7%**,
  removed in favor of the escalation ladder (PI:448–463).
- **Vectorial/domain-decomposition fold:** naive version overshoot up to ~5.7 rad; fixed with step clip; kept
  behind `use_vectorial_folding=False` (PI:214–309).

## 6. Claimed contribution (verbatim, PI:49–57)

> *"every individual energy term and even most individual mechanisms here … have precedent elsewhere in the
> IK/motion-planning literature. **The claimed contribution is the specific staged sequencing** — target-blind-first,
> then coarse, then narrowing, then locally-scoped rescue, then stability-gated stop — not any single energy term
> in isolation. Whether this staging earns its complexity has to be settled empirically against the baselines…"*

## 7. Per-robot parameters

Robot specs (K:66–163): UR5 6-DOF, limits ±2π (effectively unlimited), reach ≈1.19 m; Franka 7-DOF, tight
asymmetric limits incl. **q4∈[−3.0718,−0.0698]**, reach ≈1.39 m; Planar 3-DOF `a=[0.4,0.3,0.2]`, limits ±π,
reach 0.9 m. Auto-adapting: Stage-5 jitter (∝ reach), Stage-4 scope ladder (∝ n). **Fixed constants (UR5-tuned):**
Stage-1 ±0.3, Stage-2 damping/scale, Stage-3 radius/decay/damping, `stuck_eps`, `eps`. **⚠️ `q_neutral=0` is
outside Franka's q4 range** → Stage 1 pulls q4 to its limit; the "mid-range neutral" comment is UR5-centric.
