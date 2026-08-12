# V5 — CCH-IK (Conflict-Controlled Homotopy IK)

> Source: `backend/app/solvers/protein_homotopy/{solver.py,core.py,__init__.py}`, `fixed_lambda_ik.py`,
> `v5_research_report.md`, `protein_ik_v5_deep_research.md`, `backend/v5_verify*.csv`, `v5_benchmark.py`.
>
> **Headline for the report:** the implemented solver and the narrative docs diverge in several load-bearing
> ways (energy form, conflict range, component count) and — most important — **the actual verification benchmark
> does NOT reproduce the success-rate win or the difficulty rank-ordering the report claims.** V5 is a NULL result
> whose only defensible contribution is a cheap trajectory diagnostic.

## 1. Homotopy energy `E(q,λ)`

As implemented: **`E(q,λ) = E_target(q) + λ·E_constraints(q)`** — there is **no `(1−λ)` on the task term**
(`core.py:114`, `solver.py:315`). README's `(1-λ)·E_task` (README.md:107) is wrong.
- `E_target` (`core.py:60-66`): `‖err[:3]‖² + 0.3·‖err[3:]‖²`.
- `E_constraints` (`core.py:69-71`): `collision_energy + 0.5·joint_limit_energy`.
- λ starts at 0.0 (pure task) and advances toward 1.0 (fully constrained).

## 2. Conflict index `C`

`compute_conflict` (`core.py:27-53`): `C = 1 − cosine(g_target, g_constr)`, **range [0,2]** (0 aligned/cooperative,
1 orthogonal, 2 opposed) — **not [−1,1]** as README.md:117 and the solver header (`solver.py:37-43`) claim.
`g_target=−Jᵀ·err` (analytic), `g_constr=` central FD (`core.py:78-93`). Measured every iteration (`solver.py:251`);
`C_final` is reported as `conflict_index`. A separate **per-joint** quantity `g_target*g_constr` (`solver.py:281`)
is used only to mask retreat joints — never averaged into the index.

## 3. λ advancement (Component A)

`solver.py:262-266`: if `C < CONFLICT_THRESHOLD (0.6)` and `λ<1`: `δλ = 0.10·exp(−3.84·C)`, `λ←min(1,λ+δλ)`;
else hold and increment the stuck counter. `LAMBDA_MAX_STEP=0.10`, `LAMBDA_BETA=3.84=ln10/0.6` (`solver.py:79,86,87`).
At C=0 step=0.10 (aggressive when cooperative); at C=0.6 step≈0.01 (nearly stopped). **A OFF** = linear
`λ=(it+1)/max_iters` (`solver.py:301-303`).

## 4. Conflict retreat / stuck rescue

Triggered after λ held `CONFLICT_RETREAT_AFTER=20` iters (`solver.py:278`). Deterministic constraint-descent on
conflicted joints only: `q[mask] −= RETREAT_ALPHA(0.15)·g_constr[mask]` (`solver.py:286-288`) — explicitly *not*
random (contrast V1). **λ handling:** with **Component E ON (default)**, λ is pushed *forward* by
`MIN_LAMBDA_PROGRESS=0.05`, never retreated (`solver.py:289-296`). **⚠️ The report's §3.5 λ-retraction (λ←λ·0.90)
is dead code** in the shipped config (only runs with E OFF, `solver.py:297-299`).

## 5. Components — 5 toggles, not 3

| Flag | Default | What it does | Code |
| :-- | :--: | :-- | :-- |
| A | True | conflict-controlled exponential λ advancement (**the contribution**); OFF→linear | `solver.py:262-303` |
| B | True | PCGrad gradient surgery when C≥0.6 (project out opposing component) | `core.py:165-192`, gate `solver.py:305-312` |
| C | True | geometric warm-start seed (short λ=0 task-only descent) | `core.py:129-158` |
| D | True | null-space constraint-aware endgame (declash returned config in task null space) | `solver.py:124-157,194-195` |
| E | True | monotonic predictor-corrector: λ never retreats | `solver.py:289-296` |

**⚠️ Component count disagrees three ways:** docs say 3 (A/B/C), solver header says "Two," code defines 5.
The ablation (`v5_benchmark.py:70-71`) patches only A/B/C — D and E stay ON in every row — so `A0B0C0` is **not**
the same solver as standalone `fixed_lambda_ik` (only D differs). Two "baselines" share a name.

## 6. Diagnostics (`types.py:49-52`)

- `conflict_index` ∈ [0,2] — final-step full-vector conflict.
- `lambda_final` ∈ [0,1] — λ at termination (λ<0.8 ⇒ constraints not fully introduced).
- `difficulty_score` — **mean C over trajectory** (`conflict_integral/iters`, meaned across restarts). Valid even
  on failure. `fixed_lambda_ik` doesn't set it (shows "—").

Measured (n=100 UR5): conflict_index ≈ 0.60/0.60/0.675; lambda_final ≈ 0.586/0.646/0.588; difficulty ≈
0.737/0.652/0.703 (open/near/cluttered).

## 7. Theoretical grounding (honest scoping, `solver.py:10-28`)

**Claimed (rigorous):** IFT (Allgower & Georg 1990) guarantees a *locally* smooth path q(λ) where the Hessian is
non-singular; λ bounded [0,1]; **no global convergence claim**; path breaks at singularities.
**Design-intuition only:** minimal frustration / protein-folding analogy — *"the design intuition only. All
algorithmic choices are justified by the optimisation theory above."*

## 8. The honest NULL result (the important part)

**8a. Docs concede:** V5 is not faster and doesn't beat V4 on success; the diagnostic (not a success win) is the
real contribution (`v5_research_report.md:367-381`, `deep_research:411-491`).

**8b. The benchmark shows CCH-IK does NOT beat fixed-λ** (`v5_verify_n100.md`, n=100, UR5):

| Scenario | CCH-IK (A1B1C1) | fixed_λ | protein_fast (V4) |
| :-- | --: | --: | --: |
| open_space | 94.0% | **95.0%** | 100% |
| near_singular | 93.0% | 93.0% (tie) | 100% |
| cluttered | 92.0% | **97.0%** | 100% |

Ablation confirms it: turning A (or all of A/B/C) **off matches or beats** the full solver (cluttered A0B0C0 98%
vs A1B1C1 92%). Clean null for the conflict-control contribution.

**8c. Cost:** ~50–190× protein_fast (mean 1117/1448/1236 ms vs 7.8/23.5/15.8 ms). The docs predicted 40–80 ms —
off by 15–30×.

**8d. Quality doesn't rescue it:** cluttered collide 41% vs 33% and clearance −0.0118 vs −0.0042 m — CCH-IK is
*worse* on collision than the fixed-λ baseline; higher joint-limit violations too.

**⚠️ Report claims contradicted by data:** the "94% near_singular vs 90% fixed-λ" win (`v5_research_report.md:295`)
does not exist in the CSVs; the difficulty rank-ordering (near>clutter>open) is the *opposite* of measured
(open 0.737 > cluttered 0.703 > near 0.652) and 3–6× larger. Treat both as unsupported.

**Bottom line:** V5's only defensible contribution is the **diagnostic triple** (conflict_index, lambda_final,
difficulty_score), obtained at ~100× protein_fast's cost. Conflict-controlled λ provides **no measured
success/clearance advantage** over a fixed linear schedule on UR5, and is sometimes worse. The mechanism *does*
fire (lambda_lt08% ≈ 36–48% for full V5 vs 0% for A-off) — it just doesn't buy anything.

## 9. `fixed_lambda_ik.py` (the intended clean control)

Same energy and gradients as CCH-IK, `COMPONENT_A=False` hard-coded → linear `λ=iter/max_iters`; no PCGrad, no
retreat, no D/E; keeps Component C (geometric seed). Computes `C_final` for reporting but never acts on it. Does
not return `difficulty_score`. Answer it isolates ("does conflict-control matter?"): **no** (§8).
