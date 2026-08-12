# What Can Be Done — after two null forks

Both forks came back null. That is not a dead end — it is **information**. Two directions are now
definitively closed with evidence, which means effort can stop being spent on them and go somewhere
real. This doc lays out the honest options.

> **Alignment with the locked plan.** The project already decided (raw_notes Entry 19) on a
> **two-document split**: (1) a **research paper on V1 + V4** (the positive artifact), and (2) a
> **full technical report** that houses the V5/V6 negatives with mechanism. **These two forks are
> load-bearing evidence for document (2).** They also over-determine Entry 18's *saturation* thesis
> ("biological depth stops paying off past the architecture level"). Nothing below proposes a
> competing strategy — it slots into that plan and says what each null closes.

---

## 1. Where the project actually stands (honest scorecard)

| Component | Status | Evidence |
|---|---|---|
| **V1 (staged fold)** | ✅ Real, defensible | Beats DLS/CCD/FABRIK on success; documented honestly |
| **V4 (Fast)** | ✅ **The workhorse win** | 98–99.7% success, best accuracy (0.6–0.75 mm), collision edge; competitive with TRAC-IK |
| **V5 (CCH-IK) as a solver** | ❌ Null | Success claim washes at N=100 (cluttered baseline 98% > 92%) |
| **V5 as a difficulty diagnostic** | ❌ Null | Fork B: ρ≈0.12, p≈.45 — doesn't predict hardness |
| **Raw (V6) as a quality solver** | ❌ Null | Ties V4 only via multi-start + clash-free selection, not the fold |
| **Raw on redundant arms** | ❌ Null | Fork A: headroom shrinks with redundancy; loses to multi-start at equal wall-time |
| **Raw Σ as a diagnostic** | ❌ Null | Fork B: ρ≈0.06, p≈.7 |

The pattern is clean: **biological inspiration pays off at the shallow end (architecture: V1, V4)
and stops paying off as it goes deeper (control logic: V5; energy function: Raw).**

---

## 2. The one genuinely publishable story (recommended)

> **"From Structure to Physics: how deep can protein-folding inspiration go before it stops
> helping?"** — and the honest answer: **it stops at the architecture level.**

This is a *stronger* paper than "our bio-solver wins," because it is falsifiable, it includes a
full baseline field, and it reports negative results that most papers hide. The spectrum:

- **V1/V4** — staging + engineering: a **real, measured win** over naive baselines.
- **V5** — one principle (minimal frustration) as control logic: **honest null** — conflict-control
  doesn't beat a fixed schedule, and the conflict signal doesn't even predict difficulty.
- **Raw** — biophysics in the energy function: **honest null** — faithful, well-tested, but its
  quality edge comes from ordinary multi-start selection, not the folding; and its redundancy
  premise is empirically false.

The forks in this folder are the paper's "we tried hard to rescue the deep versions and here's the
evidence they don't rescue" section. That is what makes it credible.

**Effort:** ~days to assemble (the numbers exist). **Payoff:** a complete, honest contribution.

---

## 3. Options, ranked

### A. Execute the locked two-doc plan *(recommended, low risk)*
Per Entry 19: the **V1/V4 paper** (positive artifact) + the **technical report** (houses these fork
negatives with mechanism). Nothing new to build — the fork numbers drop straight into the report's
"what didn't work, and why" chapter. This closes the research loop honestly.
_(Caveat flagged in Entry 20/21: the only committed results CSV is Franka-only, and V4's speed/collision
claims rest on UR5 numbers not yet committed — the fresh `master_benchmark.py` sweep must land and be
committed before the paper's positives are on solid ground.)_

### B. Repurpose Raw's *engine* (not its claims) as a teaching/visualization tool *(low risk, real utility)*
Raw is a working coarse-grained folding simulator wired to a live 3D frontend. As a **science
solver** it's a null; as an **educational/demo engine** for folding funnels, glass transitions, and
hydrophobic collapse it's genuinely nice and inspectable. Reframe, don't rebuild. (Honest caveat:
not competitive with MARTINI/OpenMM as research-grade simulation.)

### C. One last arena for Raw: 3D obstacle-cluttered *external* collision *(medium risk, low expected payoff)*
Every null so far was **self-**collision. The one untested case is a 3D arm avoiding **external
obstacles**, where the null-space might move the elbow through free space. **Gate it with the Fork A
headroom test first**: if median-solution null-space clearance-climb gain is ≈0, stop immediately —
there's nothing to win. Expectations should be low given every prior arena failed.

### D. Take a *single idea* fully out of robotics *(research bet, high uncertainty)*
- **Σ as a landscape-optimizability pre-check** on non-IK problems (loss surfaces, hyperparameter
  grids). Fork B killed it for IK, but its physics assumptions were never meant for IK landscapes;
  a spin-glass / folding toy problem is its native home. This is a *new* project, not a rescue.
- **V5's conflict signal for multi-task learning** — but this is PCGrad/CAGrad territory (saturated
  field); low novelty. Not recommended.

---

## 4. What to STOP doing

- **Stop trying to make V5 or Raw beat V4 as IK solvers.** Five independent findings now say they
  don't, and *why*. Further tuning is sunk cost.
- **Stop treating Franka as Raw's arena.** Its self-collision is structurally pinned (0 headroom).
- **Don't land the worktree V4 tweaks** (`_collision_free_seed`, `_null_space_collision_resolve`) —
  both measured as duds; the null-space one is a no-op that re-confirms the pinning finding.

---

## 5. The honest one-liner

The project set out to prove that *deeper* biological grounding makes *better* IK solvers. It
proved the opposite of the ambitious version — depth stops helping past architecture — and did so
rigorously. **That negative result, cleanly demonstrated with a full baseline field, is the
contribution.** V4 is the thing that works; the spectrum is the thing that's interesting.
