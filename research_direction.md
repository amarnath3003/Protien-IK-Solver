# ProteinIK — Research Direction
## "From Structure to Physics: A Spectrum of Protein-Folding-Inspired IK Solvers"

---

## The Central Question

> **"How deeply can protein folding inspire inverse kinematics solving,
> and what does each level of depth give you?"**

---

## The Honest Architecture of What We Have

### V1 — protein_ik.py

5 stages — all using **standard IK techniques**, sequenced to mirror
the stages protein folding goes through:

```
Stage 1  →  "Secondary structure"   =  neighbor-joint local relaxation (no target)
Stage 2  →  "Hydrophobic collapse"  =  low-gain Jacobian pull toward target
Stage 3  →  "Folding funnel"        =  Metropolis search with shrinking radius
Stage 4  →  "Chaperone rescue"      =  perturb highest-energy joint, resume
Stage 5  →  "Native state check"    =  jitter + stability test before success
```

The code states it explicitly:
> *"every individual energy term and mechanism here has precedent elsewhere
> in the IK/motion-planning literature. The claimed contribution is the
> specific staged sequencing."*

**V1 is:** standard IK methods, arranged in a sequence that mirrors the
stages of protein folding. The biology is in the **architecture**, not
the energy functions.

---

### V3 (intermediate)

Refinement of V1. Added adaptive ensemble tracking and better native-state
selection. Same staged architecture. Same biological depth.

---

### V4 — protein_fast/solver.py

The code states it explicitly:
> *"V4 changes NONE of the folding behavior. It runs V3's exact staged
> fold and only swaps the per-step math for a fused, allocation-light
> primitive."*

One FK pass replaces two. Explicit cross products replace `np.cross` overhead.
Numerically identical to V3. ~5× faster.

**V4 is:** V3 with the math optimized. Not a new biological insight.
The same staged architecture running fast enough to be practical.

---

### V5 — protein_homotopy/

A completely different algorithm that takes **one biological principle**
and implements it rigorously:

- Principle: **minimal frustration** (Bryngelson & Wolynes 1987)
- Mechanism: measure gradient conflict C between task and constraint
- Schedule: λ advances exponentially fast when C is low (cooperation),
  pauses when C is high (conflict)
- Rescue: deterministic constraint retreat on conflicted joints when stuck
- Diagnostic: conflict integral = trajectory difficulty score

Biology is in the **control logic** of a gradient-based homotopy solver.
The energy function is still standard IK energy.

**Result:** 94% near-singular vs. 90% fixed-schedule baseline.
Difficulty score correctly rank-orders scenario hardness without labels.

---

### Raw (V6) — Not started

Biology moves into the **energy function itself**.

Don't mimic the stages of folding.
Don't take one principle from folding.
Rebuild the energy function from actual biophysical forces —
translated exactly, with no IK equivalent for each term.

**Result:** TBD — this is the open frontier.

---

## The Spectrum

```
V1      →  Biology in the ARCHITECTURE (staged sequence of operations)
V4      →  Same biology, optimized math (engineering, not conceptual advance)
V5      →  Biology in the CONTROL LOGIC (one principle, clean algorithm)
Raw     →  Biology in the ENERGY FUNCTION (what the solver minimizes)
```

Each level is deeper. Each level is independently useful.
They are not redundant — they answer different questions.

---

## Paper Framing

**Title:**
> *"From Structure to Physics: A Spectrum of Protein-Folding-Inspired
> Inverse Kinematics Solvers"*

**Core thesis:**
Protein folding can inspire IK at multiple levels of depth.
Each level of depth produces a distinct, measurable benefit.
We build and compare solvers at each level to show this systematically.

---

## Paper Spine

### Introduction
- Protein folding solves a high-dimensional search problem efficiently.
- Inverse kinematics is also a high-dimensional search problem.
- How far can protein folding inspire IK? We answer this systematically
  across four solvers spanning three levels of biological grounding.

---

### Section 2 — V1: Staged Architecture

**Question:** *What if we sequence standard IK techniques to mirror
the stages of protein folding?*

**Contribution:** Staging matters.
Target-blind relaxation first (Stage 1) separates coarse search from
fine refinement — mirroring how secondary structure forms before
tertiary contacts. This separation improves convergence over naive
gradient descent because it prevents the task gradient from disrupting
early chain compaction.

**Result:** X% improvement over unstaged baselines.
(Benchmark against CCD, FABRIK, Jacobian DLS — all without staging.)

---

### Section 3 — V4: Engineering the Staged Architecture

**Question:** *Same staged biology — can we make it fast enough to use?*

**Contribution:** The staged approach is now competitive with TRAC-IK
on speed. One FK pass instead of two. Explicit cross products.
~5× faster than V3 with numerically identical results.

**Honest statement:** V4 is not a new biological insight. It is the
same bio architecture made practical. This section documents the
engineering contribution separately from the algorithmic one.

**Result:** ~76ms average. 100% success on open-space UR5. Competitive
with TRAC-IK on speed while maintaining staged constraint awareness.

---

### Section 4 — V5: Single Principle, Clean Implementation

**Question:** *What if we extract ONE concept from protein folding and
implement it rigorously in its own algorithm, without V1's staged
architecture?*

**Contribution:**
- Conflict index C ∈ [0,2]: measures gradient cooperation between
  task and constraint at every iteration.
- Exponential λ schedule: advances constraints faster when C is low
  (cooperative landscape), slower when C approaches threshold.
- Conflict retreat: deterministic retreat on most-conflicted joints
  when stuck — no random perturbation.
- Difficulty score: conflict integral over trajectory correctly
  rank-orders scenario difficulty without labels.

**Result:**
- 94% near-singular vs. 90% fixed-schedule (conflict control matters
  precisely where gradient conflict is highest — near-singular targets)
- 96% open-space (no regression on easy cases)
- Difficulty score: 0.109 open → 0.167 cluttered → 0.204 near-singular
  (correct ordering, emergent from physics)

---

### Section 5 — Raw: Biophysical Energy Landscape

**Question:** *What if we rebuild the energy function itself from
actual biophysical forces — the ones that govern protein folding?*

**Contribution:**
Replace the standard IK energy (position error + joint limit penalty)
with an energy function constructed from translated protein physics.
Each term must have no existing IK equivalent — see the Raw design
document for the filter applied.

**Genuine bio contributions (no IK equivalent):**
1. Full LJ 6-12 between all link pairs — repulsion + **attraction**
   (IK solvers only repel; the attractive well creates natural spacing)
2. Directional H-bond axis coupling — distance + angle dependent
   interaction between specific joint pairs (creates robot "secondary structure")
3. Free energy F = E − T·log(w(q)) — entropy term from manipulability
   (favors well-conditioned configs by thermodynamic law, not by rule)
4. Sigma ratio Σ — landscape topology measurement before solving
   (predicts difficulty from funnel shape, not from trial-and-error)

**Solver:** Overdamped Langevin dynamics on F(q) — not simulated
annealing but the physically correct dynamics for a free energy landscape
with a thermal bath.

**Result:** TBD. Expected: lower self-collision energy at solutions,
natural joint angle distributions, sigma ratio predicts solve success.

---

### Section 6 — Unified Comparison

**Results table across all four solvers + baselines:**

| Metric | CCD/FABRIK | DLS | V4 (Fast) | V5 (Homotopy) | Raw |
|---|---|---|---|---|---|
| open_space success | ? | ? | 100% | 96% | ? |
| near_singular success | ? | ? | 100% | 94% | ? |
| cluttered success | ? | ? | 100% | 98% | ? |
| avg wall time | ? | ? | 76ms | 1700ms | ? |
| min_self_distance | ? | ? | ? | ? | ? |
| difficulty_score | N/A | N/A | N/A | 0.109–0.204 | ? |
| sigma ratio | N/A | N/A | N/A | N/A | ? |

**Key finding (hypothesis):** Each level of biological depth produces
a distinct, measurable benefit:
- V1/V4: staging improves success over unstaged baselines
- V5: conflict control improves near-singular by 4% over fixed schedule
- Raw: biophysical energy produces better solution quality (lower
  self-collision, more natural postures) even if success rate is similar

---

### Conclusion

The spectrum from structural analogy → single principle → full physics
is a productive research direction. Each level is independently useful
and addresses a different aspect of the IK problem:

- V1/V4 addresses *search efficiency* (staging separates coarse/fine)
- V5 addresses *constraint introduction timing* (conflict control)
- Raw addresses *solution quality* (biophysical energy = natural poses)

Raw points toward the next open question: can a fully biophysical energy
function produce IK solutions that are not just accurate but physically
natural — low-stress, high-manipulability, collision-free by construction?

---

## Current Status

| Solver | Status | Action Required |
|---|---|---|
| V1 | ✅ Complete | Verify benchmark numbers for paper |
| V4 | ✅ Complete | Document as V3 + speed optimization (not new algorithm) |
| V5 | ✅ Complete | Ablation table (200 trials, all 8 A/B/C combos) needed |
| Raw | ❌ Not started | Design validated → implement E_LJ first |

---

## The Open Question That Determines Raw's Value

> **"Does a biophysical energy function produce solutions that are
> qualitatively better — not just in accuracy, but in physical
> naturalness — than a solver that only minimizes position error?"**

If yes: Raw is a research contribution about *what* IK should minimize,
not just *how* to minimize it. That is a novel claim.

If no: Raw is an interesting engineering exercise but not a
research contribution beyond the others.

The experiment that answers this: compare `min_self_distance`,
`joint_limit_violations`, and joint angle distribution of Raw solutions
vs. V4 solutions. If Raw solutions have significantly higher
min_self_distance and lower joint_limit_violations — the biophysical
energy function does something V4's optimization cannot.

---

## References (for the paper)

1. Bryngelson & Wolynes (1987) — minimal frustration, spin glass theory
2. Bryngelson et al. (1995) — funnels, pathways, Langevin, sigma ratio
3. Plaxco, Simons, Baker (1998) — contact order and folding rate
4. Thirumalai & Lorimer (2001) — IAM chaperone mechanism
5. Ferreiro, Komives, Wolynes (2014) — frustration in biomolecules
6. Lennard-Jones (1924) — LJ 6-12 potential
7. Baker & Hubbard (1984) — hydrogen bonding geometry
8. Kauzmann (1959) — hydrophobic effect thermodynamics
9. Yu et al. (2020) — PCGrad gradient surgery (V5 Component B)
10. Beeson & Ames (2015) — TRAC-IK (baseline)
