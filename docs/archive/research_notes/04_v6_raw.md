# V6 — Raw Biology (coarse-grained Langevin folding IK) — for the technical report

> Source: `backend/app/solvers/protein_raw/{energy.py,landscape.py,solver.py}`, `raw_math.md` (live spec),
> `raw_audit.md`, `raw_design.md` (STALE on 2 terms — see below), `backend/raw_phase{1..4}_experiment.py`,
> `raw_quality_sweep.py`, `raw_notes.md`. **Status: documented NEGATIVE result.** Cite `raw_math.md`, not
> `raw_design.md`, for the live math.

## 1. Free energy `F(q;T) = E_task + E_LJ + E_HB − T·S_conf`

Reporter `free_energy(...)` solver.py:63-70; the Langevin loop uses its gradient (solver.py:194). A single
self-consistent `T` governs entropy weight + noise + cooling. Reduced units `k_B=γ=1`.

- **E_LJ** (energy.py:96-118) — 6-12 **with attraction**, uniform ε (non-Gō), all non-adjacent bead pairs
  `|i−j|≥2`; `σ_ij=sigma_scale·(r_i+r_j)`. `S2=1` attractive / `S2=0` repulsion-only ablation. **No-IK-equivalent
  part = the attractive well** (min −ε at `d=2^{1/6}σ`) → emergent inter-bead spacing (IK keeps only the repulsive
  wall). **Analytic gradient** (energy.py:125-170), matches FD <1e-4 on all arms.
- **E_HB** (energy.py:227-251) — directional H-bond, `= −ε_hb Σ F(d)·G(t̂_i·r̂)·H(t̂_j·r̂)`, `F`=Gaussian
  distance gate, `G,H`=angular gates. Direction = **triplet-plane normal** `t_i=norm((p_i−p_{i-1})×(p_{i+1}−p_i))`
  (energy.py:186-200), NOT joint axis. Interior non-adjacent pairs only → **Planar has none** (skipped).
  Two-condition (distance AND orientation) gate = no IK equivalent. **FD gradient** (energy.py:254-271).
- **S_conf = log Ω** (energy.py:322-345) — Ω = soft-feasible (in-limits × clash-free) free-volume over a **fixed
  Gaussian cloud (common random numbers)**; `target-blind`, `collision-aware`. Opposes collapse (chain
  conformational entropy). Distinct from manipulability (proven, §3). **FD gradient, fixed stencil**; cost
  `(2n+1)·m` capsule evals (hot-loop `m=16`).
- **E_task** — `w_task·(‖p_err‖+0.3·‖o_err‖)`, gradient `−Jᵀerr`. The only non-folding term (boundary condition).

## 2. Langevin dynamics (solver.py:160-196)

Euler–Maruyama overdamped: `grad_F = g_task + 0.4·g_lj + 0.4·g_hb − T·0.5·g_s`; `noise=√(2T·dt)·ξ`;
`q←clip(q + clip_norm(−grad_F·dt + noise, max_step=0.25))`. **No Metropolis** (pure force dynamics — the claimed
distinction from simulated annealing). Cooling `T_t=max(T_glass, T_start·e^{−t/τ})`, `T_start=max(4·T_glass,0.25)`,
`τ=n_lang/3`. Entropy force vanishes as T→0 (faithful). Phases: `raw_unfolded / raw_collapse / raw_consolidate`.

## 3. Endgame — T→0 consolidation, native selection (the Entry-15 fix), stability gate

`_consolidate` (solver.py:77-100): noise-off LM (`dq=Jᵀ(JJᵀ+λ²I)⁻¹err`, adaptive λ) = the T→0 limit of the same
dynamics. **Native selection** (solver.py:198-236): among consolidated target-reaching candidates, take the
**clash-free (min_self>0) one with min enthalpy** (T→0, entropy gone); least-bad clearance only if none clean.
This treats excluded volume as a **HARD Pauli constraint**, needed because the LJ well sits on joint-ORIGIN beads
but a capsule collides mid-span (min-E_LJ ≠ max-clearance). Multi-start seeds `n_ws=10+2·max(0,n−6)` (widened
4→10). Anfinsen jitter stability gate `_stable_native` (reported, not gated on).

## 4. Σ ratio + T_glass (landscape.py)

`Σ=σ_E/ΔE` over the **compact (warm-start) ensemble** (molten-globule analog), task+bio balanced to equal
variance; `E_native`=ensemble min (the one IK circularity, stated). Σ<1 funnelled, >1 glassy. `T_glass=σ_E/√(2S₀)`.

## 5. Phase-experiment results (terms work in isolation — NOT end-to-end quality)

- **P1 (LJ):** UR5 spacing std 0.54→0.28, in-well 61% vs 12% (repulsion-only); Planar E_LJ→negative, 83% vs 0%.
- **P2 (H-bond):** ideal 55× > perpendicular, 4× > off-distance. *Negative:* GD alone barely orients (UR5 align
  0.51→0.53; Franka flat) — emergence needs the Langevin thermal stage on these short (6–10 bead) chains.
- **P3 (entropy):** corr(clearance, S_conf) = **+0.90/+0.65/+0.91** vs manipulability **+0.08/+0.21/−0.27** →
  S ≠ manipulability. Entropy ascent opens configs (UR5 clearance −0.085→+0.020).
- **P4 (Σ):** UR5 ~0.77–0.87 funnelled; Franka/Planar ~1.0 glassy. corr(Σ, DLS difficulty) modest/mixed
  (−0.24/+0.16/−0.12) — complementary measure, not an oracle.
- **Solve reach (open_space):** UR5 10/10, Planar 9/10, Franka 9/10; ~2.1/0.68/3.2 s (slowest by design).

## 6. The quality NULL (raw_quality_sweep.py, UR5 cluttered N=24)

- **Entry 14:** Raw loses — V4 +0.0061 min_self / 0% baseline-win; Raw −0.016…−0.023 / 27–32% win. Two causes:
  (a) weights inert (selection was by task-error only); (b) bead-origin E_LJ ≠ capsule-surface min_self.
- **Entry 15 (after the hard clash-free selection fix + wider ensemble):** Raw −0.0228→**+0.0039** min_self
  (clash-free), success 92→100%, win-vs-V4 32→42% (V4 +0.0061). **A tie.** But the three weight configs are
  **byte-identical** — on non-redundant UR5 the Langevin *fold* adds nothing over multi-start + clash-free
  selection. The edge's active ingredients are selection + branch enumeration, **not the dynamics**.

## 7. Measurement boundary / pinned redundancy (Entry 17)

The core "biophysical energy → better quality" question is **untestable on the capsule proxy**: min-F ≠
max-clearance, and Franka self-collision is **structurally elbow-pinned** (pair (2,4) argmin 88%; 30 IK solutions
3.2 rad apart span 0.004 m; null-space ascent = +0.000). "Franka degenerate constant −0.15" is STALE/false
(std 0.029, 809 unique / 3000). Honest verdict = **"untestable under this proxy," not "useless"** → fair test
needs the mesh oracle (`sim_migration_plan.md`).

## 8. V6's two report-chapter negatives (both mechanistically explained)
1. **Fold redundant with selection** — the tie comes from multi-start + hard Pauli native selection, not Langevin.
2. **Measurement boundary** — the quality claim can't be scored on the current proxy on the only redundant arm.

These are the load-bearing evidence for the report's "staging, not physics" saturation thesis.
