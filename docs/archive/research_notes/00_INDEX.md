# ProteinIK — Research Notes (paper + technical report source material)

> Consolidated from a deep internal research pass over the codebase, docs, and result files
> (6 parallel research agents, 2026-07-06). Every claim here is traceable to `file:line`.
> These notes are the **raw material** for two planned deliverables — they are not the documents themselves.

---

## The two deliverables (locked)

1. **Research paper — V1 + V4** (positive artifact). Thesis = *the artifact*: a protein-folding-staged
   IK solver (V1) made practical by a kinetic-partitioning ensemble (V4), honestly benchmarked against a
   strong baseline field. V5/V6 appear only as a short "glimpse" subsection pointing to the report.
2. **Technical report — the whole project** (V1→V6). Houses V5/V6 in full, every negative result, all
   mechanism, and the **"staging, not physics"** saturation thesis (which needs the negatives to be central,
   so it lives here, not in the paper).

**Boundary rule:** paper = claim + evidence for V1/V4; report = complete arc + every negative + all mechanism.

---

## ⚠️ CRITICAL — read before drafting the paper

The research uncovered that the paper's intended positive claims are **not currently backed by committed data.**

The only committed results file — `backend/master_benchmark_results.csv` — contains:
- **Only `franka_panda`** (no UR5, no Planar3DOF).
- **Only 7 solvers** (Jacobian, CCD, FABRIK, TRAC-IK-style, Multi-start, V1, V4). **No V5, no V6.**
- N = 300 per cell (100 trials × seeds {1,2,3}).

Under that data:

| Intended paper claim | What the committed Franka CSV actually shows | Status |
| :-- | :-- | :-- |
| V1 beats classical baselines | V1 (72–81%) beats **simple** baselines (Jacobian/CCD/FABRIK 16–28%) by +45–55 pp | ✅ supported |
| V1 competitive with production baselines | V1 (72–81%) **trails** TRAC-IK (91–95%) and Multi-start (82–86%) everywhere | ❌ **not** supported |
| V4 competitive with TRAC-IK on **speed** | V4 mean 194–320 ms vs TRAC-IK 26–30 ms → **7–12× slower** on Franka | ❌ **not** supported by this CSV |
| "~9–14 ms, ties TRAC-IK on UR5 open_space" | Comes from `fast_optimization.md` (UR5); **no UR5 CSV committed**; only V5-diagnostic rows have V4 UR5 ≈ 7.75 ms — and **no UR5 TRAC-IK number exists anywhere** to compare | ⚠️ uncommitted / unverifiable |
| V1/V4 self-collision **edge** | Every solver collides 65–99%, negative mean clearance; V4 ≈ or worse than simple baselines | ❌ **not** supported on Franka |
| V4 leads on **success** | V4 = 98–99.7% (success leader across all Franka scenarios) | ✅ supported |

**The single most defensible positive in the committed data is: V4 is the success leader (98–99.7%), at a latency cost.**
Everything else the paper wants to say (speed parity, collision edge) currently lives in prose docs
(`fast_optimization.md`, `README.md`) whose UR5 numbers are not reproduced by any committed CSV.

### Decision required before drafting
- **Option 1 (recommended):** re-run `backend/master_benchmark.py` across **UR5 + Planar (+ V5 + V6)**, commit the
  CSVs, and let the paper's claims follow whatever the fresh numbers actually show. This is ~an afternoon of compute
  and makes every table reproducible.
- **Option 2:** scope the paper strictly to what the Franka CSV supports (V4 = success leader vs a strong field,
  honest about the latency cost; drop or heavily qualify the speed-parity and collision-edge claims).

---

## Version-at-a-glance (honest status after research)

| V | Name | What it is | Honest verdict from the data |
| :--: | :-- | :-- | :-- |
| **V1** | ProteinIK | 5-stage folding-sequenced fold (blind relax → collapse → funnel → scoped rescue → stability gate) | Beats simple baselines; **loses to production baselines** on Franka. Contribution = the *staging*, per its own docstring. |
| **V4** | ProteinIK Fast | Barrierless-first (LM) ensemble; escalates to full stochastic fold only on frustrated seeds (kinetic partitioning) | **Success leader (98–99.7%)** but slow on Franka. "Speed-competitive" is a UR5 claim without committed data. |
| **V5** | CCH-IK | Conflict-controlled homotopy; λ advances when task/constraint gradients cooperate | **NULL:** conflict-control does not beat its own fixed-λ ablation (ties/loses; cluttered 92% vs 98%). ~50–190× slower than V4. Real contribution = the *diagnostic triple*. |
| **V6** | Raw Biology | Coarse-grained Langevin folding sim on `F = E_task+E_LJ+E_HB−T·S_conf` | **NEGATIVE:** does not beat V4 on quality (tie at best, via multi-start + hard clash-free selection — *not* the fold). Core claim untestable on the capsule proxy (min-F ≠ max-clearance; Franka redundancy elbow-pinned). |

---

## File map of these notes

| File | Contents |
| :-- | :-- |
| `00_INDEX.md` | This file — plan, the critical data caveat, version status. |
| `01_v1_proteinik.md` | V1 mechanism: 5 stages, scoped rescue, stability gate, energy terms, reverted mechanisms, per-robot params. |
| `02_v4_fast.md` | V4 mechanism: barrierless-first ensemble, FK primitives, tail diagnosis, rejected tail-edits, measured latency. |
| `03_v5_cchik.md` | V5 mechanism: homotopy energy, conflict index, λ rule, retreat, A–E components, diagnostics, IFT grounding, the NULL result. |
| `04_v6_raw.md` | V6 mechanism: the 5-term free energy, Langevin dynamics, consolidation endgame, native selection, Σ/T_glass, phase-experiment numbers, the quality NULL + measurement boundary. |
| `05_results_numbers.md` | Exact numbers parsed from the 3 committed CSVs; column dictionary; V1/V4 vs baselines; V5 ablation; V6 absence. |
| `06_core_baselines_proxy.md` | Uniform interface + SolveResult, robot DH specs, the capsule collision proxy (the measurement-limitation core), scenario generators, baselines, kinematics core. |
| `07_discrepancies.md` | Consolidated inventory of every code/doc/data inconsistency the agents found, with severity and paper impact. |

---

## Recommended next actions (in order)

1. **Decide Option 1 vs 2 above** (fresh committed benchmarks vs scope-to-Franka). Everything downstream depends on it.
2. If Option 1: re-run `master_benchmark.py` for UR5 + Planar, and add V5/V6 to the sweep; commit CSVs.
3. Reconcile the doc/code drift in `07_discrepancies.md` (several are quick fixes; a few change what the paper can claim).
4. Then draft the paper outline + abstract against the *actual* committed numbers.
