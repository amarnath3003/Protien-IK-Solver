# Shared Core, Baselines, Robots & the Collision Proxy

> Source: `backend/app/core/{kinematics.py,types.py}`, `app/solvers/registry.py`, the baseline solvers,
> `app/api/scenarios.py`. The collision-proxy section (§3) is the load-bearing detail for the paper's
> measurement-limitation argument.

## 1. Uniform solver interface & `SolveResult`

Every solver: `solve(spec, q0, T_target, rng, collect_steps=False) -> SolveResult` (registry.py:31-40).
The registry wraps rng-less (`jacobian_dls`,`ccd`,`fabrik`) and rng-using solvers uniformly so the API/benchmark
never branch on solver identity. `SolveResult` (types.py:34-80): `solver_name, success, q_final, pos_error,
orient_error, iterations, wall_time_ms, min_self_distance` (**the collision metric**), `joint_limit_violations,
restarts, steps` + version-specific defaults `conflict_index=0, lambda_final=1, difficulty_score=0` (V5),
`sigma_ratio=0, free_energy=0, t_glass=0` (V6). Baselines leave the version fields at defaults.

## 2. Robot DH specs (kinematics.py)

Standard (non-modified) DH: `T_i = Rot_z(θ)·Trans_z(d)·Trans_x(a)·Rot_x(α)`.

- **UR5 (6-DOF, non-redundant)** ur5_spec():66-84. `a=[0,-0.425,-0.39225,0,0,0]`, `d=[0.089159,0,0,0.10915,
  0.09465,0.0823]`, `alpha=[π/2,0,0,π/2,-π/2,0]`, limits ±2π (effectively unlimited), `link_radius=[0.06,0.05,
  0.045,0.04,0.04,0.035]`. Primary tuning arm.
- **Franka Panda (7-DOF, redundant)** franka_panda_spec():87-129. `a=[0,0,0,0.0825,-0.0825,0,0.088]`,
  `d=[0.333,0,0.316,0,0.384,0,0.107]`, `alpha=[0,-π/2,π/2,π/2,-π/2,π/2,π/2]`. Tight asymmetric limits incl.
  **q4∈[-3.0718,-0.0698]** (permanently negative "elbow-down"), q6∈[-0.0175,3.7525]. `link_radius=[0.05,0.04,
  0.025,0.025,0.025,0.02,0.015]` (corrected — see §3). Reach ~0.855 m.
- **Planar 3-DOF RRR** planar3dof_spec():132-163. `a=[0.4,0.3,0.2]`, `d=α=0`, limits ±π, `link_radius=[0.03,
  0.025,0.02]`. Has a closed-form analytical solver (ground truth).

Only Franka is kinematically redundant (7 DOF for a 6-DOF pose) — exactly the case §3 undermines.

## 3. The self-collision proxy — CRITICAL

`self_collision_min_distance(spec,q)` (kinematics.py:281) → `self_collision_min_distance_from_chain` (298):
capsule (radius-inflated segment) model. Segments run between consecutive joint origins `pts=chain[:,:3,3]`.
For every **non-adjacent** segment pair `(i,i+1)` vs `(j,j+1)`, `j≥i+2`: segment-segment closest distance
(Ericson) **minus combined capsule radii** `r_i+r_j` → signed distance (negative = interpenetration). Returns
the global min. Uses a scalar loop (profiled faster than vectorized at this tiny pair count, 86 vs 224 ms/1000).

**Franka degeneracies & the radii fix** (docstring kinematics.py:111-124): original radii `[0.08,0.07,…]` caused
permanent false positives — (a) zero-length intermediate segments on pairs (0,2),(4,6) (Franka joints with d=0
AND a=0) → handled by a degenerate-segment skip (`_EPS2=1e-12`, kinematics.py:343-354); (b) pair (2,4)'s max
separation is the fixed elbow-offset `a[3]=0.0825 m`, and old `r[2]+r[4]=0.13 > 0.0825` flagged collision at
*every* config incl. home. **Fix:** radii cut so `r[2]+r[4]=0.05 < 0.058` (home (2,4) distance). Radii are
hand-tuned for self-consistency, **not from CAD**.

**THE FINDING (defensible, data-backed):**
1. The proxy is a crude geometric approximation with hand-tuned radii.
2. On Franka, the **elbow pair (2,4) is the argmin 88%** of sampled configs; 30 IK solutions 3.2 rad apart span
   only **0.004 m** of `min_self`; **null-space ascent on `min_self` = +0.000 gain**. UR5 (non-redundant) spans
   **0.057 m** across discrete multi-start branches. ⇒ the **7th DOF buys ≈0 clearance headroom under this
   proxy** — it cannot detect/reward redundancy-based collision avoidance.
3. **Do NOT** claim the proxy is a "degenerate constant −0.15" on Franka — tested false (min −0.075/max +0.032/
   std 0.029/809 unique over 3000 configs; that was the old-radii era). Correct framing = "structurally pinned /
   low-sensitivity to the redundant DOF."
4. Corroborated by `sim_migration_plan.md`, which proposes a PyBullet/MuJoCo mesh oracle to fix it.

Feedback into solvers: baselines call `self_collision_min_distance` **for reporting only** — none use it in their
loop (scenarios.py:107-110). The protein family consumes it via `protein_energy.collision_energy` (0 if d≥0.05,
quadratic ramp, `100+|d|·100` on penetration) — the collision-awareness whose *measurability on Franka* is what
the elbow-pinning finding undermines.

## 4. Scenario generators (scenarios.py)

- **open_space** (41-47): uniform random `q_true`→FK target; independent random `q0`. (Already ~40% near-singular
  by manipulability, yet TRAC-IK still wins there.)
- **near_singular** (71-97): rejection-sample lowest Yoshikawa manipulability `√det(JJᵀ)`; per-DOF threshold
  0.001 (planar) / 0.005 (UR5) / 0.015 (Franka, looser); ≤50 tries, keep best-so-far.
- **cluttered** (100-134): rejection-sample lowest `self_collision_min_distance`; early-exit at `d<-0.03` (≈5th
  percentile; the old 0.02 threshold ≈ median gave results identical to open_space); ≤200 tries. Most informative
  read alongside `min_self_distance`, not success alone (collision-blind baselines ignore it).

## 5. Baselines (one-liners for the paper's baseline field)

- **jacobian_dls**: single-trajectory DLS `dq=Jᵀ(JJᵀ+λ²I)⁻¹err`, damping 0.05, ≤200 iters, no restart.
- **ccd**: cyclic coordinate descent base→tip, ≤300 iters; last `n_wrist=min(3,n//2)` joints blend orientation.
- **fabrik**: FABRIK adapted to revolute joints via axis projection; alternating reach passes ≤150 iters +
  dedicated wrist-orientation step (0/50 → 77% success after that fix).
- **trac_ik_style**: DLS + stuck-detection + **full random restart** (the key baseline to beat); attempts of 50
  iters, budget 300, `stuck_window=8`, `stuck_eps=1e-5`; global (not scoped) rescue-on-stuck.
- **multi_start**: 8 independent DLS seeds (q0 + 7 random), 60 iters each, best-by-error; `restarts=7`.
- **analytical_planar3dof**: closed-form 2-link law-of-cosines (elbow-up/-down), planar only; ground truth
  (round-trips FK to <1e-9 m).

## 6. Kinematics core

`forward_kinematics_chain` returns the full `(n+1,4,4)` chain (201-220). `geometric_jacobian` (239-256): 6×N,
`J_v=z_i×(p_end−p_i)`, `J_w=z_i`, recomputed each call. `pose_error` (259-278): 6-vector `[pos diff(3),
axis-angle orient(3)]`, orient = rotation vector of `R_target·R_currᵀ`. FK validated by exact closed-form
comparison on the planar arm (`_planar_fk_exact`) and the analytical IK round-trip; V4's `_fast_chain` bit-identity
tested vs core (UR5+Planar ×500). No finite-difference Jacobian test found in the suite.
