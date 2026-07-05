# Discrepancy & Honesty Inventory

> Every code/doc/data inconsistency surfaced by the research pass, consolidated. Grouped by area.
> Severity: **HIGH** = changes what the paper/report can claim; **MED** = misleading, fix before publishing;
> **LOW** = cosmetic/internal. Each item has a `file:line` anchor and the paper impact.

---

## A. Data / benchmarking (HIGH — these gate the paper)

| # | Issue | Evidence | Impact |
| :--: | :-- | :-- | :-- |
| A1 | **Committed master CSV is Franka-only, 7 solvers, no V5/V6.** | `backend/master_benchmark_results.csv` (21 rows = 3 scenarios × 7 solvers) | All UR5/Planar tables in `research_direction.md` and `fast_optimization.md` have **no committed data**. Re-run needed. |
| A2 | **V1 does not beat production baselines** (contradicts the README "beats simple baselines… modest collision edge" framing which implies competitiveness). | Franka: V1 72–81% vs TRAC-IK 91–95%, Multi-start 82–86% (`master_benchmark_results.csv`) | Paper must state V1 only beats *simple* baselines. |
| A3 | **V4 "competitive with TRAC-IK on speed" is false on committed data** (7–12× slower on Franka). The UR5 parity claim has no committed CSV. | V4 194–320 ms vs TRAC-IK 26–30 ms (Franka CSV); "~9–14 ms UR5" only in `fast_optimization.md:96`, `README.md:97` | Central paper claim at risk. Re-run UR5 or rescope. |
| A4 | **Self-collision edge unsupported by master CSV** — all solvers collide 65–99%; V4 ≈ or worse than simple baselines. | `master_benchmark_results.csv` `collision_pct` col; `_run_benchmark_sync` `main.py:127` | The collision-edge narrative (README headline; `research_direction.md`) is not backed by committed data. |
| A5 | **README/notes claim "94% near_singular vs 90% fixed-λ" for V5 does not exist in the data.** | No such pair in `v5_verify.csv`/`v5_verify_n100.csv`; real near_singular full-V5 vs fixed-λ = 96.7/93.3 (N=30) or 93/93 (N=100) | Drop/correct the claim. |
| A6 | **"cluttered A0B0C0 98% vs full V5 92%" is real but was framed positively** — it shows conflict-control *losing* to its own baseline. | `v5_verify_n100.csv` cluttered: A0B0C0 = 98%, A1B1C1 = 92% | Confirms the V5 NULL; frame honestly. |
| A7 | **V5 difficulty score does not rank-order scenarios as reported.** Report claims near_singular(0.204) > cluttered(0.167) > open(0.109); real values are ~0.65–0.74 and rank open > cluttered > near_singular (opposite). | `v5_research_report.md:301-308` vs `v5_verify_n100.csv` diagnostics | "Difficulty correctly rank-orders" (a headline V5 finding) is **not reproduced**. Treat as unsupported. |

---

## B. V1 — code vs docs (MED)

| # | Issue | Evidence | Impact |
| :--: | :-- | :-- | :-- |
| B1 | **Stage-4 "scoped, unlike TRAC-IK's global restart" actually escalates to a full global reseed** on its last ladder rung. | `protein_ik.py:428-431` (full `spec.random_config`) vs docstring `protein_ik.py:33-39`, `README.md:80,83` | The key differentiator is "starts scoped, escalates," not "never restarts globally." Honesty section must say this. |
| B2 | **Stage-3 search is greedy accept-if-better, NOT Metropolis** (no temperature, no probabilistic uphill accept). | `protein_ik.py:372-374` | Don't call V1's search "Metropolis/simulated annealing." (Metropolis appears in V4's `_fold_once` and V6.) |
| B3 | **`q_neutral = np.zeros(n)` is outside Franka's q4 feasible range** `[-3.0718, -0.0698]` → Stage 1 pulls q4 to its limit. | `protein_ik.py:135`; limits `kinematics.py:87-129` | The Stage-1 "neutral pose" anchor is UR5-centric; doesn't generalize. Worth a caveat. |
| B4 | `max_rescues` parameter (default 6) is **declared but never used**. | `protein_ik.py:129` | Cosmetic; rescue count is bounded by `max_iters`. |
| B5 | Two disabled feature flags never run on the benchmark path: `use_vectorial_folding=False`, `use_rotamer_bias=False`. | `protein_ik.py:130-131`, registry `_wrap_rng` passes neither | The vectorial-folding and rotamer variants are documented dead-ends, not live V1. |

## B′. V1 — documented reverted mechanisms (KEEP — these are honesty assets)

- Pure neighbor-coupling Stage 1 (no neutral anchor): cluttered **90.0%→86.0%**, reverted (`protein_ik.py:166-182`).
- Rotamer-library-biased proposals: improves mean self-distance but cluttered **90.0%→67.3%** (best variant 76.0%), disabled by default (`protein_ik.py:336-360`, `rotamer_library.py`).
- Allostery-inspired compensating step: mean self-dist −0.0074→−0.0024 but success 90.0%→88.7%, removed (`protein_ik.py:448-463`).
- Vectorial/domain-decomposition fold: overshoot up to ~5.7 rad before a fix; kept behind flag (`protein_ik.py:214-309`).

---

## C. V4 — code vs docs (MED)

| # | Issue | Evidence | Impact |
| :--: | :-- | :-- | :-- |
| C1 | **Bit-identity "0.0 diff over 9000 configs across ur5/franka/planar" is only *tested* on UR5+Planar ×500 (≈2000 comparisons, no Franka).** | Claim `solver.py:58`, `fast_optimization.md:80`, `README.md:95`; tests `test_backend.py:209-240` (parametrized UR5+Planar only) | Verify the 9000/Franka number or soften the claim. |
| C2 | **Speedup range "1.1–4.3×" vs table best 4.4×.** | `solver.py:64`, `README.md:97` vs `fast_optimization.md:89` | Pick one number. |
| C3 | **"V4" label collision:** `fast_optimization.md:85-94` calls the *old* solver "V4"; README calls the *new* barrierless solver V4. Task's "V3" is not a named version anywhere (lineup is V1/V4/V5/V6). | `fast_optimization.md` vs `README.md:11-17` | Fix naming in the paper. |
| C4 | Comment says difficulty scaling adjusts "per-replica budget" but it scales `stage2_iters` (Phase-B fold length), not `max_replicas`. | `solver.py:391-399` | Cosmetic. |

**Confirmed accurate for V4** (use freely): barrierless-first gate on `d ≥ 0.0` clean clearance; single `max_replicas=6` over both phases; tail = ~10% of targets / ~57% of wall time; micro-opt alone = 1.1–1.4×; Franka 71.7% at cap-replicas=2; Layer 1 *changes behavior* (not bit-identical).

---

## D. V5 — code vs docs (HIGH for report accuracy)

| # | Issue | Evidence | Impact |
| :--: | :-- | :-- | :-- |
| D1 | **README energy form is wrong.** README writes `E=(1-λ)·E_task + λ·E_constraints`; code has **no `(1-λ)`** — task weight is constant 1. | `README.md:107` vs `core.py:114`, `solver.py:315` | Fix the energy equation in report/README. |
| D2 | **conflict_index range is [0,2] (1−cosine), not [−1,1].** README and the solver header docstring both say [−1,1] signed. | `README.md:117`, `solver.py:37-43` vs `core.py:53`, `types.py:49` | Fix the diagnostic definition. |
| D3 | **Component count disagrees three ways:** docs say 3 (A/B/C), solver header says "Two," code defines 5 (A–E). | `v5_research_report.md:266-274`; `solver.py:30-34`; `solver.py:69-73` | Report must describe all 5 (A=λ-control, B=PCGrad surgery, C=seed, D=null-space endgame, E=monotonic no-retreat). |
| D4 | **The report's λ-retraction (§3.5, λ←λ·0.90) is dead code** because `COMPONENT_E=True` forces λ forward instead. | `solver.py:289-299` (E default True) | Report describes disabled behavior as live. |
| D5 | **Ablation "baseline" ≠ standalone `fixed_lambda_ik`.** A0B0C0 still runs Component D; only D differentiates it from the standalone baseline. | `v5_benchmark.py:70-71` patches only A/B/C | Two "baselines" share a name; clarify. |

---

## E. V6 — code vs docs (MED; report is a negative-result chapter)

| # | Issue | Evidence | Impact |
| :--: | :-- | :-- | :-- |
| E1 | **`raw_design.md` is stale on two terms** — describes the *rejected* originals: entropy = `log(manipulability)`, H-bond direction = joint axis `z_i`, Σ = random-config sampling. Code implements the *corrected* versions (free-volume `S=logΩ`, triplet-plane normal, compact-ensemble Σ). | `raw_design.md:93-171` vs `energy.py:186-345`, `landscape.py:115-135`; corrections in `raw_math.md`, `raw_audit.md` | Cite `raw_math.md`, not `raw_design.md`, for the live math. |
| E2 | **README presents V6 as "Live" with no negative result** — clean 4-term table, "thesis is solution quality." | `README.md:124-149` | README hides the V6 null; report must carry it. |
| E3 | **V6 absent from the master benchmark** — the "Live" solver has no row in the headline quality table. | `master_benchmark_results.csv` (no `raw`) | Need a committed V6 sweep if the report wants a quality table. |
| E4 | **"Franka min_self degenerate constant −0.15" is STALE/falsified** (old-radii era). Current: min −0.075/max +0.032/std 0.029/809 unique over 3000 configs. | `raw_phase1_experiment.py:134-136` (stale print) vs `raw_notes.md:264-289` (Entry 17) | Do **not** claim "degenerate constant." Correct framing = "structurally elbow-pinned / low-sensitivity to the 7th DOF." |
| E5 | Minor param drift: `κ` default 3.0 (`energy.py:228`) vs solver 2.0; `m` entropy 64 vs 16 (hot loop) vs 24 (reporter). | `energy.py`, `landscape.py:51`, `solver.py:63,123` | Cosmetic. |

**The two V6 negatives to carry in the report (both mechanistically explained):**
1. **Fold is redundant with selection** — the measured collision tie traces to multi-start branch enumeration + hard clash-free (Pauli) native selection, *not* the Langevin dynamics; the three weight configs are byte-identical on UR5 (`raw_notes.md:232-247`).
2. **Measurement boundary** — bead-origin `E_LJ` distance ≠ capsule-surface `min_self` (min-F ≠ max-clearance), and Franka self-collision is structurally elbow-pinned (elbow pair (2,4) is argmin 88% of the time; 30 IK solutions 3.2 rad apart span only 0.004 m of `min_self`; null-space ascent = +0.000). So the core "biophysical energy → better quality" claim is *untestable on this proxy*, not *false* (`raw_notes.md:264-289`).

---

## F. Collision proxy — the load-bearing measurement caveat (HIGH for report §measurement)

- Proxy = capsule (radius-inflated segment) self-distance: segment-to-segment distance minus per-link radii, non-adjacent pairs only (`kinematics.py:281-406`).
- Radii are **hand-tuned to avoid false positives at the home pose**, not from CAD: Franka cut `[0.08,0.07,…]`→`[0.05,0.04,0.025,…]` so `r[2]+r[4]=0.05 < 0.058` (home (2,4) distance). Two degenerate zero-length-segment pairs skipped explicitly (`kinematics.py:111-124, 343-354`).
- Correct claim for the paper: the proxy is a *crude but real* geometric approximation that is **dominated by one fixed structural pair (the elbow) on Franka**, so it cannot detect clearance gained via the 7th DOF's null space — corroborated independently by `sim_migration_plan.md`, which proposes a PyBullet/MuJoCo mesh oracle.
- Do **not** say the proxy returns a constant on Franka (checked false).
