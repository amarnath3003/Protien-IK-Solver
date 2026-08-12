# Context — what this figure is for

Background for whoever (or whatever) draws `fig_pipeline`. Read this first, then the
prompt. This file explains *why* each element exists so the design decisions are informed
rather than mechanical; the prompt gives the hard requirements.

---

## 1. The paper

A robotics paper for an IROS/ICRA-tier venue. Its thesis: **inverse kinematics and protein
folding are the same search problem.**

A robot arm and a protein backbone are both chains of rigid segments whose only free
variables are the rotations between neighbours — joint angles `q` for the arm, backbone
dihedral angles φ/ψ for the protein. Both search a rugged, non-convex landscape toward a
target configuration while avoiding self-overlap. The paper takes that correspondence
seriously enough to *build solvers from it*, and shows they beat the production baselines
(TRAC-IK, Multi-start).

The figure being drawn is **Figure 1** — the first figure in the paper, and the one that
shows how the winning solver actually works.

## 2. The solver being drawn: KineticFold

The paper builds two solvers. **StagedFold** runs a robot arm through the same *ordered
stages* a protein visits while folding. **KineticFold** — the one in this figure — is the
deployed solver and the headline result.

KineticFold's insight is about *scheduling*, not about the search itself.

Real proteins exhibit **kinetic partitioning**: some molecules fall straight down a smooth
funnel to their folded state with no search at all, while the rest get kinetically trapped
and fold slowly — and it is only the trapped ones that molecular chaperones like GroEL act
on. GroEL does not process every molecule; it rescues the ones that failed to fold
spontaneously.

KineticFold ports that as a **compute schedule**:

- **Phase A** is the cheap attempt everything gets. Six parallel restarts, each running a
  fast Levenberg–Marquardt polish. Most targets resolve here and are done.
- A **gate** asks whether the target is *frustrated* — whether Phase A failed to produce a
  solution that both converged and is free of self-collision.
- **Phase B** is the expensive machinery, and it fires *only* on frustrated targets. This is
  the full staged fold, with a Metropolis-annealed search and a chaperone-style rescue.

That gate is the contribution. The earlier solver's problem was never the average solve —
it was the tail: the slowest ~10 % of targets ate ~57 % of total wall time. Micro-optimising
the inner loop bought almost nothing, because the cost was not in *how* the search runs but
in *whether a target enters the expensive search at all*.

**Measured: 79 % of targets take the fast path** and never enter Phase B (two physical arms,
n = 1800). And the rate tracks difficulty — 93 % fast path on the easy open-space UR5 case,
down to 50 % on cluttered Franka. The gate is a difficulty detector, not a fixed sampling
rate.

## 3. What each element does, in plain terms

| Element | Plain meaning | Folding analog |
|---|---|---|
| **Phase A** | Six cheap parallel attempts. Take the first one that both reaches the target and doesn't self-collide. | the fraction of molecules that fold spontaneously, no chaperone needed |
| **The gate** | "Did any cheap attempt actually work *and* stay collision-free?" If yes, stop. If no, this target is hard. | kinetic partitioning — the split between fast and trapped folders |
| **Coarse collapse** | Yank the hand roughly into the target's neighbourhood. Deliberately imprecise. | hydrophobic collapse — a protein crumples into a compact blob before its fine structure forms |
| **Metropolis funnel** | The real search. Random single-joint nudges, accepted by simulated annealing: early on it will accept a worse configuration to escape a bad local minimum; as it cools it freezes into pure downhill descent. | the folding funnel |
| **Analytic rescue** | When the search stalls, find the single most conflicted joint — where the pull toward the target and the pull toward a locally smooth chain disagree most — and re-randomise a window of joints around it. Escalates from a narrow window to a full restart only as a last resort. | a chaperone unfolding a stuck region and letting it try again |
| **Select** | Among every configuration that worked, return the one furthest from self-collision. | — |
| **Stability gate** | Jiggle the answer five times. If small perturbations blow up the error, this was a knife-edge solution, not a real one — reject it. | a folded protein is a *stable* energy minimum, not merely a minimum |

## 4. What the figure must communicate at a glance

In priority order. If a design choice trades one against another, favour the higher item.

1. **Most targets exit early.** The fast path is the common case; Phase B is the exception.
   A reader who takes nothing else away should take this.
2. **The gate is the decision point.** It is the paper's contribution and should be the
   visual centre of gravity of the top band.
3. **Phase B is a detour, not the main line.** It hangs below the spine and rejoins it.
4. **Both paths converge on the same ending.** The selection and stability check are shared —
   fast-path answers get verified too. Drawing the stability check inside Phase B would be
   factually wrong.
5. **The biology is annotation, not decoration.** The italic analog labels ride on real
   algorithm boxes. That is the paper's whole argument — the correspondence is load-bearing.

## 5. Publication constraints

- **Print, two-column IEEE.** The figure spans both columns. Everything must be legible in
  black-and-white photocopy as well as colour — rely on position and label, never colour
  alone, to convey meaning.
- **Colour-blind safe.** The palette is Okabe–Ito and is fixed; do not substitute.
- **Consistent with Figures 2–4**, which are matplotlib charts in a serif font with the same
  solver-colour assignments. This figure must look like it belongs to that set.
- **Restrained.** This is a research paper, not a slide deck. No drop shadows, no gradients,
  no 3-D, no icons, no clip art, no decorative flourishes. Thin strokes, generous whitespace,
  small type, high information density.

## 6. Notation that must render correctly

These are real symbols, not ASCII approximations. Getting one wrong is a factual error in a
published paper.

`q₀` `q*` `‖Δp‖` `‖Δω‖` `d(q)` `δ` `λ` `λ₀` `φᵢ` `Tₜ` `T₀` `T_f` `rₜ` `U(−rₜ, rₜ)` `κ(J(q₀))`
`𝒩(0, σ²I)` `δqₖ` `≤` `≥` `∧` `·` `−` (minus, not hyphen) `★`

Equation numbers refer to numbered equations in the paper body and must be exact:
Eq. 7, 13, 14, 15, 17, 18, 19, 20, 21.
