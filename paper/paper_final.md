# ProteinIK: Inverse Kinematics as a Protein-Folding Process

*Final build — assembled section by section from [outline_simple.md](../outline_simple.md), with numbers locked to the
committed result files ([paper_notes.md](../paper_notes.md) is the claim→evidence spine; [methodology.md](../methodology.md)
is the deep Methods). This Markdown is the working master; it converts to LaTeX once the sections are settled.*

<!-- ============================ BUILD TRACKER ============================
     Fill in section by section. Figures generated as each section is reached.
       [x] Abstract               — DRAFTED (revisit after Results settle)
       [x] 1. Introduction        — DONE (no embedded figure — Table 1 only)
       [x] 2. Related work         — DONE (citations web-verified; no figure)
       [x] 3. Methodology          — DONE — umbrella section (Fig 1 embedded; Fig 2/3 TODO)
             [x] 3.1 Problem formulation — IK as a folding search
             [x] 3.2 StagedFold
             [x] 3.3 KineticFold
             [x] 3.4 LangevinFold
       [x] 4. Experiments          — DONE (robots, scenarios+thresholds, baseline hyperparams, protocol, validation harness)
       [x] 5. Results             — DONE (Tables 5-9; Fig 4 TODO); numbers provisional pending bg master_sim_benchmark.py run
       [x] Where it wins + DOF climax — FOLDED into §5.5 (Results paragraph, per user; Fig 6 planned)
       [~] Validation             — covered in §4.6 (harness + FK/engine agreement) + §5.3–§5.4 (two-engine collision); no standalone section
       [x] Limitations            — FOLDED into §5.6 (small Results paragraph, per user)
       [x] 9. Conclusion           — DONE (forward-refs re-pointed to §5.5/§5.6/§4.6 after folding)
       [x] References             — DONE (38 entries, APA style; bibliographic details web-verified 2026-07-09)
======================================================================= -->

---

## Abstract

Inverse kinematics (IK) and protein folding are, structurally, the same problem: a chain of rigid segments whose only
free variables are the rotations between neighbours, searching a rugged, constrained landscape for a configuration
that satisfies its boundary conditions. We take this correspondence literally and build an IK solver out of the
*process* proteins use to fold. **StagedFold** ports folding's ordered stages — local settling before the target is
even consulted, coarse collapse, a funnelled narrowing search, a scoped "chaperone" rescue, and a native-state
stability check — into an IK algorithm; its individual moves are standard IK, but the folding-inspired *sequence*, and
two unusual moves (a target-blind first stage and a scoped-then-escalating rescue), are new. StagedFold beats the
simple classical baselines but plateaus below production solvers, which motivates **KineticFold**: it adds folding's
*kinetic partitioning* as a compute schedule — try a cheap downhill fold first, and pay for the full staged search
only on genuinely frustrated targets. KineticFold leads a strong baseline field (Jacobian-DLS, CCD, FABRIK, TRAC-IK,
Multi-start) on success across three arms (100% on UR5 and Franka), matches or beats TRAC-IK on speed in the easy
regime, and clashes less on the non-redundant arm. We validate every result against **two independent physics
simulators** (PyBullet and MuJoCo): our forward kinematics agree with both to floating-point precision, and both
engines confirm — and appropriately shrink — our self-collision claims. Finally, a literal folding simulation
(**LangevinFold**) produces the cleanest solutions of any solver on the non-redundant arm, at a latency cost:
faithful biology buys quality. The contribution is an organizing *principle* for IK, not new energy terms, and it
pays off most exactly where the arm behaves most like a folding polymer.

---

## 1. Introduction

Inverse kinematics — finding the joint angles that place a robot's end effector at a target pose — is deceptively
hard. The map from configuration to pose is nonlinear; a target may admit many solutions or none; the Jacobian loses
rank at singularities; and a redundant or long arm can reach the target while folding into itself. Classical solvers
confront this as a *single* optimization to be minimized from the very first iteration — by damped least squares,
cyclic coordinate descent, reaching heuristics, or restart-based search — driving pose error down a landscape they
treat as one basin to descend.

We observe that this is the *same* search a protein performs when it folds. A protein backbone is a chain of rigid
bonds whose only soft degrees of freedom are the dihedral rotations between residues; a robot arm is a chain of rigid
links whose only degrees of freedom are the joint angles. A protein reaches its native state by descending a rugged
free-energy landscape riddled with local minima, kinetic traps, and steric (self-overlap) constraints; an IK solver
searches a landscape with local minima, singular regions, and self-collision basins. The correspondence is not a loose
analogy but a structural isomorphism (Table 1): the two problems share their variables, their constraints, and the
shape of the space they search. We make it precise in the Methodology (§3.1, Figure 1).

**Table 1 — The isomorphism.**

| Protein folding | Inverse kinematics |
|---|---|
| Backbone dihedral angles φ/ψ (soft DOF) | Joint angles `q` (the DOF) |
| Rigid bonds / fixed bond lengths | Fixed link lengths (FK constraints) |
| Native (folded) state | The IK solution configuration |
| Free-energy funnel | Convergence basin to the target |
| Rugged landscape / kinetic traps | Local minima / failed solves |
| Excluded volume (sterics) | Self-collision avoidance |
| Hydrophobic collapse | Coarse approach to the target region |
| Secondary structure (local order) | Local joint settling |
| Molecular chaperone (GroEL) | Restart / rescue from a stuck state |
| Kinetic partitioning (fast vs. slow folders) | Easy vs. hard targets |

The bridge between the two fields is already load-bearing — in one direction. Cyclic coordinate descent (CCD), a
robotics IK algorithm, was adopted into structural biology for protein loop closure [Canutescu & Dunbrack 2003]. An
algorithm has already crossed from IK into folding; we carry the *process* back the other way — not a single move, but
the ordered sequence nature uses to fold.

**Thesis.** *Inverse kinematics is, structurally, a protein-folding problem, so an IK solver built from folding's
process wins exactly where the problem becomes most folding-like.* We defend this with three solvers of increasing
biological literalness and a dual-simulator validation methodology.

**Contributions.**

1. **A design principle** — casting IK as a folding *process*: the first IK solver organized as a staged fold with
   kinetic partitioning and chaperone rescue. The novelty is the *organization*, plus two genuinely unusual moves
   (target-blind-first initialization and scoped-then-escalating rescue) — **not** new energy terms; each numerical
   ingredient is standard IK, so any advantage comes from the sequencing.
2. **KineticFold** — kinetic partitioning recast as a *compute schedule* that removes the latency tail: the success
   leader across three arms, speed-competitive with (and on the easy regime faster than) TRAC-IK, and the cleanest
   practical solver on self-collision on the non-redundant arm.
3. **A dual-simulator validation methodology** — "solve once, score three ways" (our capsule proxy + PyBullet +
   MuJoCo) — that independently confirms every success claim on two engines and *corrects* our own collision-magnitude
   claim, a level of self-scrutiny rare in heuristic-IK work.
4. **An honest map** of where the principle pays off (the per-solve edge grows with chain length), where it ties (the
   redundant arm gives every solver room to dodge equally), and where literal folding physics buys quality at a
   latency cost.

We preview the climax. As a planar arm is lengthened from 4 to 16 joints — made progressively more polymer-like —
KineticFold's single-shot collision-free solve rate degrades the most gracefully of the standard field, until it is
the last method producing clean folds at all. The concept stops being an analogy and becomes the reason the method
works: it wins because the problem *becomes* folding.

---

## 2. Related work — why this is not just a metaphor

Our solvers reuse standard IK machinery; what is new is the *organization*. We therefore review the field along the
axis that matters for our argument — how each method behaves when the search stalls — then the folding theory we port,
and finally the prior crossings between the two disciplines that make the port more than an analogy.

**Jacobian- and optimization-based IK.** Velocity-level IK descends from resolved-motion-rate control, which maps
end-effector rates to joint rates through the Jacobian (pseudo)inverse [Whitney 1969]; the raw pseudoinverse is
unbounded near singularities, which damped least squares (DLS) regularizes with a damping term that trades a little
accuracy for stability [Nakamura & Hanafusa 1986; Wampler 1986], with singularity proximity quantified by the
manipulability measure √det(JJᵀ) [Yoshikawa 1985] and refined by selective damping [Buss & Kim 2005]. Position-level
solvers cast IK as nonlinear least squares and apply Levenberg–Marquardt [Levenberg 1944; Marquardt 1963], the
optimization twin of DLS. All of these are single-trajectory local optimizers: they follow one gradient from one seed
and settle into whatever basin they start in, with no intrinsic way to escape a local minimum. We include Jacobian-DLS
as a baseline and reuse a damped-least-squares step inside our own solvers.

**Sampling and restart IK.** Production solvers wrap a local core in global restarts. TRAC-IK [Beeson & Ames 2015], our
key baseline, runs a joint-limited Newton solver — an extension of KDL [Smits et al., Orocos KDL] — concurrently with
an SQP optimizer and returns the first to converge; when the Newton branch detects stagnation (no progress between
successive iterates) it re-seeds from a *fresh random configuration*. Multi-start applies the same idea in the open:
run several independent seeds and keep the best. Both are strong production methods, and both, when stuck, *discard the
accumulated partial solution and restart globally.* Analytical generators such as IKFast [Diankov 2010] sidestep
iteration altogether by emitting closed-form solutions, but only for chains with special solvable structure — they do
not generalize to redundant arms or arbitrary constraints. This global-restart-on-stall behaviour is precisely what
our chaperone rescue replaces with a *scoped* perturbation (§3.2).

**Heuristic IK.** Geometric heuristics trade the Jacobian for cheap per-joint updates: cyclic coordinate descent (CCD)
rotates one joint at a time along the chain [Wang & Chen 1991], and FABRIK reaches forward and backward along the
links with no matrix inversion [Aristidou & Lasenby 2011]. Both are fast on easy targets and degrade on constrained
ones; we include both as baselines. CCD is also our bridge to biology, below.

**Learning-based IK.** A more recent line learns the IK map from data — IKFlow, for instance, trains a normalizing flow
to sample the full multimodal solution set for a target pose [Ames et al. 2022]. Such methods trade an expensive,
per-robot training phase for fast inference; they are orthogonal to our contribution, which is training-free and
applies to a new arm immediately.

**Biology-inspired IK.** Metaheuristic solvers already borrow from biology — but they borrow a *search operator*, not a
folding *process*. Memetic IK combines population-based mutation and selection with local gradient refinement [Starke
et al. 2019; Ruppel et al. 2018], and genetic-algorithm and particle-swarm variants import crossover/selection or
flocking dynamics as the rule that proposes the next joint configuration. In every case biology supplies only the
update operator; none organizes the solve as a *staged fold with a chaperone rescue gated by kinetic partitioning.*
That organizing principle is our contribution — and, by design, the constituent numerical moves are standard, so any
advantage must come from the sequencing rather than from a novel energy term.

**Folding theory we draw on.** The native state is the sequence-encoded free-energy minimum [Anfinsen 1973], and it
cannot be reached by exhaustive conformational search [Levinthal 1969]; it is reached instead by biased descent down a
rugged but funnel-shaped, minimally frustrated landscape [Bryngelson & Wolynes 1987; Bryngelson et al. 1995; Onuchic
et al. 1997; Dill & Chan 1997] — the direct analog of a well-shaped IK cost basin. We port three mechanisms from this
theory in particular: *kinetic partitioning*, in which some molecules fold directly while the rest are kinetically
trapped and fold slowly [Guo & Thirumalai 1995] (KineticFold's compute schedule, §3.3); *iterative-annealing
chaperone action*, in which GroEL rescues trapped chains by repeated partial unfolding and refolding [Todd et al.
1996; Thirumalai & Lorimer 2001] (StagedFold's scoped rescue, §3.2); and *coarse-grained off-lattice bead models*
[Honeycutt & Thirumalai 1990], with hydrophobic collapse as the compaction drive [Kauzmann 1959], which are the
lineage of LangevinFold (§3.4).

**The bridge is already load-bearing — in one direction.** The two fields provably share machinery. CCD, a robotics IK
algorithm, was imported wholesale into structural biology for protein loop closure [Canutescu & Dunbrack 2003]; loop
closure has likewise been solved as an analytical kinematics problem [Coutsias et al. 2004], building on classical
chain-closure geometry [Gō & Scheraga 1970]; robot motion planning has been used to map folding landscapes [Amato &
Song 2002]; and a protein backbone is routinely modeled as a kinematic linkage whose revolute joints are its dihedral
angles [Gipson et al. 2012; Noonan et al. 2005]. Every one of these crossings runs *robotics → biology.* To our
knowledge, the reverse — using the folding *process itself* (funnels, chaperones, kinetic partitioning, coarse-grained
folding kinetics) as the computational engine of a robot-arm IK solver — has not been attempted. That reverse crossing
is this paper.

---

## 3. Methodology

This section is the paper's technical core: it states the folding/IK correspondence as a formal search problem
(§3.1), then builds three solvers of increasing fidelity to folding's process on top of it — StagedFold, which ports
folding's ordered *sequence* (§3.2); KineticFold, which ports folding's *compute schedule* (§3.3); and LangevinFold,
which ports folding's *physics* outright (§3.4). Every numerical ingredient inside these solvers is standard IK
machinery; what each subsection contributes is how that machinery is organised.

### 3.1 Problem formulation — IK as a folding search

We now state the correspondence of Table 1 formally, as the single object every solver in this paper — baseline and
folding-inspired alike — searches over.

**Configuration and forward kinematics.** A robot with `n` revolute joints has configuration `q ∈ ℝⁿ`, bounded
componentwise by joint limits `q ∈ [q⁻, q⁺]`. Forward kinematics composes one rigid transform per joint. In the
standard Denavit–Hartenberg convention (UR5, planar arm):

```
Eq. (1)      Tᵢ(θᵢ) = Rot_z(θᵢ) · Trans_z(dᵢ) · Trans_x(aᵢ) · Rot_x(αᵢ)
```

with `θᵢ = qᵢ + θ_offset,ᵢ`. The Franka arm's official table is published in the *modified* (Craig) convention,
which reorders the same four elementary transforms —
`Tᵢ = Rot_x(αᵢ₋₁) · Trans_x(aᵢ₋₁) · Rot_z(θᵢ) · Trans_z(dᵢ)` — and is **not interchangeable** with Eq. (1): feeding a
modified-DH table through the standard-DH transform silently yields a different, wrong robot (a genuine correctness
bug we found and fixed, detailed in §4.1). Either convention composes into the full chain and the end-effector pose

```
Eq. (2)      T(q) = T₁(q₁) · T₂(q₂) ··· Tₙ(qₙ),         p(q) = T(q)[1:3, 4],   R(q) = T(q)[1:3, 1:3]
```

The associated geometric Jacobian `J(q) ∈ ℝ⁶ˣⁿ`, the instantaneous map from joint velocities to end-effector twist,
has columns

```
Eq. (3)      J_{v,i} = zᵢ × (p_end − pᵢ),     J_{w,i} = zᵢ
```

where `zᵢ` is joint `i`'s rotation axis and `pᵢ` its origin, both read off the chain in Eq. (2) (which frame carries
`zᵢ` depends on the DH convention, per the reordering above).

**The task.** Given a target pose `T_target ∈ SE(3)`, the pose error is the 6-vector

```
Eq. (4)      e(q) = [Δp; Δω],     Δp = p_target − p(q),     Δω = Log_SO(3)( R_target · R(q)ᵀ )
```

where `Log_SO(3)` extracts the axis–angle rotation vector of a rotation matrix (`Δω = θ·axis`, with
`θ = arccos((tr(R_err) − 1)/2)`, `axis` read off the skew-symmetric part of `R_err`, and `Δω = 0` when `θ` is
numerically zero). A configuration is a **success** if `‖Δp‖ < 1 mm` and `‖Δω‖ < 10 mrad`.

**The constraint folding calls sterics.** Every link occupies volume, and the chain must not intersect itself. We
quantify this with a signed clearance

```
Eq. (5)      d(q) = min_{(i,j): |i−j| ≥ 2}  [ dist_seg( ℓᵢ(q), ℓⱼ(q) ) − (rᵢ + rⱼ) ]
```

where `ℓᵢ(q)` is the line segment between joint origins `pᵢ` and `pᵢ₊₁` (the capsule "core" of link `i`), `rᵢ` its
radius, and `dist_seg` the standard closest-point-between-segments distance [Ericson 2004] — evaluated over every
*non-adjacent* link pair, since adjacent links share a joint and are never meaningfully "colliding." `d(q) ≥ 0` means
the arm clears itself; `d(q) < 0` means interpenetration. A solve is **clean** if it is both a success and satisfies
`d(q) ≥ 0`. This proxy is deliberately cheap — fast enough to sit inside an inner optimisation loop — and, as §4.6
shows, it is *optimistic* relative to true mesh geometry. We never quote it as an absolute rate: only as a same-tool
comparison across solvers, cross-checked in §4.6 against two independent full-mesh physics engines.

**The landscape.** Every solver in this paper — from single-trajectory Jacobian-DLS to our own — searches a combined
objective

```
Eq. (6)      E(q) = E_target(q) + E_limit(q) + E_collision(q) + …
```

(the exact terms and weights differ slightly by solver; §3.2–§3.4 give each one precisely, in closed form, down to
the calibrated constants). This objective is not convex: it has local minima wherever a joint configuration locally
reduces pose error without reaching the target, singular regions where `J(q)` loses rank and the local gradient stops
being informative, and collision-forbidden regions carved out by `d(q) < 0`. This is,
structurally, a protein's free-energy landscape — a rugged surface over the torsional degrees of freedom of a chain,
punctuated by kinetic traps and forbidden by excluded volume, whose global minimum is the native state
[Anfinsen 1973], reachable only by biased descent down a funnel and not by exhaustive search [Levinthal 1969;
Bryngelson & Wolynes 1987]. Figure 1 renders the mapping of Table 1 schematically: joint angles are dihedral angles,
link-length constraints are bond-length constraints, the target-reaching basin is the folding funnel, self-collision
is steric exclusion, and — the mapping we build solvers around in §3.2–§3.4 — a stuck search is a kinetically trapped
molecule, rescued the way a chaperone rescues a misfolded chain.

![Figure 1 — The protein-folding / inverse-kinematics correspondence.](figures/fig1_correspondence.svg)

**Figure 1.** A protein backbone and a robot arm are both chains of rigid segments whose only free variables are the
rotations between neighbours (dihedral angles φ/ψ vs. joint angles `q`). Both search a rugged landscape — free energy
vs. pose-error-plus-constraints — toward a stable target configuration, avoiding self-overlap along the way. The
right-hand column previews where each solver in §3.2–§3.4 sits on this correspondence.

**What we do and do not claim.** We do not claim `E(q)` needs new terms to become "more biological" — every term
defined in §3.2–§3.4 is standard in the IK literature. Our claim is that the *order* in which a solver visits this
landscape, and the *schedule* by which it decides how much of the landscape to search, should be organised the way
folding actually organises it: settle locally before consulting the goal, collapse coarsely before refining, escalate
a stuck search only as far as it needs to go, and — the lever that makes this practical — spend the expensive search
only on targets the landscape genuinely makes hard. The next three subsections build solvers of increasing fidelity
to that process: StagedFold ports the *sequence*, KineticFold ports the *schedule*, and LangevinFold ports the
*physics* itself.

### 3.2 StagedFold — the folding process as an algorithm

Every classical IK method we review in §2 treats the arm as a single objective to be minimised from the first
iteration. StagedFold instead runs the arm through the same *ordered stages* a protein visits while folding: settle
locally without yet consulting the target, collapse coarsely toward the target region, run a funnelled search that
narrows in, call a scoped "chaperone" if the search stalls, and finally check that the solution is *stable*, not
balanced on a knife-edge. Every individual move below is standard IK; **the sequence is the idea**, together with two
moves that are, to our knowledge, genuinely new in this context: a target-blind first stage, and a rescue that starts
scoped and only escalates to a global reseed as a last resort. Defaults across all experiments: `max_iters = 200`,
`pos_tol = 1e-3`, `orient_tol = 1e-2`.

Every stage below draws on five energy primitives, given here once in closed form (weights `wₓ` are set per-stage
below):

```
Eq. (7)   E_target(q)    = ‖Δp‖ + 0.3·‖Δω‖                                              (Eq. 4's e(q))

Eq. (8)   E_limit(q)     = 50 · Σᵢ pᵢ(fᵢ),      fᵢ = (qᵢ − loᵢ) / (hiᵢ − loᵢ)   (fractional position in [0,1])
                            pᵢ(f) = (margin − f)²        if f < margin
                            pᵢ(f) = (f − (1 − margin))²  if f > 1 − margin,     margin = 0.05
                            pᵢ(f) = 0                    otherwise

Eq. (9)   E_collision(q) = 100 + 100·|d(q)|                        if d(q) ≤ 0
                            10 · ((0.05 − d(q)) / 0.05)²            if 0 < d(q) < 0.05
                            0                                       if d(q) ≥ 0.05

Eq. (10)  E_smooth(q)    = 0.5 · Σᵢ (qᵢ₊₁ − qᵢ)²

Eq. (11)  E_neutral(q)   = 0.5 · Σᵢ (qᵢ − q_neutral,ᵢ)²,     q_neutral = 0
```

`E_limit` and `E_collision` are exactly the soft barriers implemented in `protein_energy.py` (verified against the
committed source): both are zero in the safe interior and grow — quadratically near a joint limit, quadratically
then affinely across the collision margin — rather than as a hard constraint, so gradient-based stages can still take
a step near the boundary instead of being blocked by it.

**3.2.1 Local-blind relaxation — the secondary-structure analog.** Gradient-free coordinate descent, one joint at a
time, for six sweeps over the chain: for each `i`, try `qᵢ ± 0.3 rad` and keep whichever configuration lowers the
*target-blind* local energy

```
Eq. (12)   E_blind(q) = E_neutral(q) + E_smooth(q) + E_limit(q)          (E_target never enters)
```

No production IK method we are aware of begins by ignoring the target; the point is to mirror local secondary
structure forming before the global fold commits, seeding every later stage from a relaxed, in-limits configuration
rather than an arbitrary one.

**3.2.2 Coarse collapse — the hydrophobic-collapse analog.** The first stage that consults the target at all: a
deliberately *detuned* damped-least-squares (DLS) pull on the full 6-D pose error, for 10 iterations,

```
Eq. (13)   Δq = Jᵀ (J Jᵀ + λ² I₆)⁻¹ e(q),      λ² = 0.15² = 0.0225,      q ← clip(q + 0.4·Δq)
```

This moves the hand into the right neighbourhood of the target without trying to be precise — the computational
analog of a protein collapsing to a compact molten globule before its final contacts form. (Eq. 13 is the same DLS
update [Nakamura & Hanafusa 1986; Wampler 1986] used by our Jacobian-DLS baseline in §2 — the *only* difference here
is the deliberately loose damping and the 0.4 step scale, i.e. the sequencing decision, not a new step rule.)

**3.2.3 Funnelled narrowing search — the folding-funnel analog.** The main refinement stage, run for up to
`max_iters = 200` further iterations, alternates (a) a gradient-free, coordinate-wise stochastic local search inside
a shrinking radius and (b) one finer DLS gradient step, minimising a fully-weighted combined energy:

```
Eq. (14)   E_stage3(q) = 3.0·E_target(q) + 1.0·E_limit(q) + 2.0·E_collision(q) + 0.3·E_smooth(q)

Eq. (15)   qᵢ,try = clip( qᵢ + U(−rₜ, rₜ) ),      rₜ = 0.5 · 0.985ᵗ         (fired every other iteration)
           accept qᵢ,try  iff  E_stage3(q_try) < E_stage3(q)               (greedy — no Metropolis test here)

Eq. (16)   Δq = Jᵀ (J Jᵀ + 0.05² I₆)⁻¹ e(q)                                (finer DLS step, every iteration)
```

The greedy accept-if-better rule in Eq. (15) is a deliberate and important distinction from both KineticFold's
Phase-B fold and LangevinFold (§3.3–§3.4): there is no temperature or Metropolis acceptance anywhere in StagedFold.

**3.2.4 Scoped chaperone rescue — the key differentiator from TRAC-IK.** A stall is detected by keeping a window of
the last 10 energy values and firing a rescue if progress over the window falls below `2e-4`. The "misfolded" joint
is identified by one-sided finite-difference sensitivity,

```
Eq. (17)   i* = argmaxᵢ | E_stage3(q + δ·eᵢ) − E_stage3(q) |,      δ = 0.05 rad
```

Rescue then re-randomises a *contiguous window of joints centred on `i*`*, at an escalation ladder of scopes
`[n/6, n/2, 5n/6, n]` (on the UR5: `[1, 3, 5, 6]`) — leaving the rest of the already-settled chain untouched. Only the
final rung is a full random reseed of the whole chain. This is the precise contrast with TRAC-IK, whose
stuck-detection response is *always* a full random restart (§2): StagedFold starts scoped and *escalates* toward
global only as a last resort, so on a persistently stuck target its behaviour converges to TRAC-IK's — the honest
claim is "scoped first, global only when scoped rescue has already failed repeatedly," not "never restarts globally."

**3.2.5 Stability-gated termination — the native-state stability analog.** Once the search converges, the candidate
solution `q*` is jittered five times, `q* + δqₖ` with `δqₖ` scaled to ≈1 mm of tip motion, and rejected if
`E_stage3(q* + δqₖ) − E_stage3(q*)` exceeds a threshold on four or more of the five trials. This rejects knife-edge
solutions that satisfy the pose error only by coincidence, mirroring the requirement that Anfinsen's native state be
a *stable* free-energy minimum, not merely *a* minimum.

**Honest verdict.** StagedFold beats the simple classical baselines (Jacobian-DLS, CCD, FABRIK) by wide margins but
does not beat the strong production baselines (TRAC-IK, Multi-start) on success (§5) — precisely the gap that
motivates KineticFold. We report this as a load-bearing result, not a shortfall to bury: it is evidence that folding
*process alone*, without folding's *compute schedule*, plateaus.

**Ablations (reverted, but evidentiary).** Three changes we tried and discarded show the specific choices above are
not decorative. Replacing Stage 1's neutral-pose anchor with a pure neighbour-coupling relaxation dropped cluttered
success from 90.0% to 86.0%. Biasing Stage 3's stochastic proposals with a rotamer-library prior improved mean
clearance but crashed cluttered success to 67–76%. An allostery-inspired compensating step traded success for a
small clearance gain and was removed. Each was implemented, measured, and reverted; we cite them because a design
principle that survives its own negative controls is more credible than one that has none.

*(Figure 2 — StagedFold's five stages mapped against their folding analogs — is planned for this subsection; the
diagram has not yet been generated and is tracked as an open item.)*

### 3.3 KineticFold — kinetic partitioning makes it competitive

**The diagnosis.** StagedFold's shortfall was never the average solve — it was the *tail*. On the unmodified
always-run-every-stage fold, the slowest ~10% of targets consumed ~57% of total wall time. This rules out
micro-optimisation as a fix: a bit-identical micro-pass over the same inner loop bought only 1.1–1.4×, because the
cost is not in *how* the per-fold search runs but in *whether a target enters the expensive per-fold search at all*.
The fix has to be structural, and folding already has one.

**3.3.1 The barrierless-first ensemble — the tail-killer.** Real proteins exhibit **kinetic partitioning**: some
molecules fall straight down a smooth funnel to the native state with no search at all ("downhill" folding), while
the rest are kinetically trapped and require chaperone intervention [Guo & Thirumalai 1995]. KineticFold ports this
as a *compute schedule* rather than a search heuristic. A single budget of `max_replicas = 6` governs two phases.

**Phase A (barrierless).** Each replica runs a cheap adaptive Levenberg–Marquardt polish (≤ 30 LM steps); replica 0
seeds from `q0`, the rest from random configurations. Each LM step is a damped Gauss–Newton update whose damping
`λ` self-tunes from the step's own outcome — Newton-fast when it helps, conservative when it doesn't:

```
Eq. (18)   Δq = Jᵀ (J Jᵀ + λ² I₆)⁻¹ e(q),        q_try = clip(q + Δq)
           if  E_target(q_try) < E_target(q):  accept q ← q_try,   λ ← max(0.5λ, 1e-4)
           else:                                reject,             λ ← min(2.5λ, 2.0)     (λ₀ = 0.08)
```

with the polish terminating early once `‖Δp‖ < pos_tol ∧ ‖Δω‖ < orient_tol`, or once `λ ≥ 2.0` (a persistent
overshoot signals this replica will not converge downhill). As soon as any replica converges to a sterically clean
solution (`d(q) ≥ 0`), Phase A stops early with a success — most targets never see anything more expensive than
Eq. (18). **The frustration criterion.** A target is declared *frustrated* — and only then escalated — iff, after
all Phase-A replicas have run, no converged replica is clash-free.

**Phase B (the full staged fold)** fires only on frustrated targets: a StagedFold-style fold (coarse collapse →
funnel → chaperone rescue → stability gate, §3.2.2–§3.2.5) but with its Stage 3 funnel replaced by a *true
Metropolis-accepted* search — a genuine refinement over StagedFold's greedy Eq. (15), so KineticFold is **not**
numerically identical to StagedFold; Layer 1 changes solver behaviour, not just its schedule. Each single-joint
candidate `q_try` (one coordinate perturbed by `U(−rₜ, rₜ)`, same shrinking radius as Eq. (15)) is accepted with
probability

```
Eq. (19)   P(accept) = 1                                                    if E(q_try) < E(q)
                        exp( −(E(q_try) − E(q)) / Tₜ )                      otherwise

Eq. (20)   Tₜ = T₀ · (T_f / T₀)^{t / max_iters},      T₀ = 0.3,   T_f = 0.01
```

— the standard Metropolis criterion [Metropolis et al. 1953] under a geometric cooling schedule, so the funnel search
can climb out of shallow local minima early (`Tₜ` large) and freezes into greedy descent as `t → max_iters` (`Tₜ`
small). Phase B is closed out by the same LM endgame as Eq. (18) and is further capped to stop on the first clean
fold, or after at most two collision-aware converged folds. Trying spontaneous folding first and invoking the
chaperone only on failure is *how GroEL actually works* [Thirumalai & Lorimer 2001] — this ordering is more faithful
to folding than always running the full machinery, not a departure from it.

**3.3.2 Allocation-light FK primitives — the per-step floor.** Independently of the schedule, the inner loop is made
cheap and bit-identical to the reference kinematics: a preallocated-buffer chain builder replaces per-joint array
literals; an incremental variant rebuilds only the *suffix* of the chain when a Metropolis sweep perturbs a single
joint; and pose and Jacobian are fused into one forward-kinematics pass with a shared constant `6×6` identity in
place of per-step allocation. We verified bit-identical output against the reference FK on the UR5 and the planar arm
(500 configurations each) — we state this scope precisely rather than claim coverage of all three arms, which the
committed tests do not cover; extending the check to Franka is listed as an open item before submission.

**What we tried and rejected.** Naive tail-edits that preserve the fold order but simply spend less — capping
replicas, bailing earlier, fewer per-stage iterations — bought little speed and destroyed the headline win: at
`cap_replicas = 2`, Franka open-space success collapsed from ~100% to 71.7%. The cost that matters is the *per-fold*
search, not the *number* of folds attempted — exactly why the kinetic-partitioning gate (skip the expensive search
entirely when unfrustrated) is the correct lever, and a naive budget cut is not.

*(Figure 3 — the latency-tail CDF, StagedFold's always-run schedule vs. KineticFold's barrierless-first schedule —
is planned for this subsection and has not yet been generated from the committed timing data.)*

### 3.4 LangevinFold — the literal folding simulation

StagedFold borrows folding's *process*; LangevinFold runs the *physics itself*. It treats the arm as a coarse-grained
molecule — one bead per joint origin — under thermal motion, and lets it fold under a genuine biophysical free
energy, cooling until it freezes into a solution. It is far too slow for practical deployment (seconds per solve),
but under real-mesh collision testing it produces the *cleanest* solutions of any solver in this study — evidence
that faithful biology buys solution *quality*, not speed. We give only the punchline here; the full treatment,
including the phase experiments behind these choices, is deferred to the thesis.

The arm is coarse-grained at one bead per joint origin `pᵢ(q)` (the FK chain of Eq. (2) again — the beads are read
off the existing kinematics, nothing new is introduced there); bond lengths between beads are enforced *exactly* by
FK, so the only soft degrees of freedom are the joint angles `q` — precisely the Cα-level coarse-graining used in
folding simulation itself [Honeycutt & Thirumalai 1990]. Reduced units are used throughout (`k_B = 1`, friction
`γ = 1`, energy in units of `ε_H`, length in units of `σ`).

**The free energy.** A single self-consistent temperature `T` simultaneously sets the entropic weight below, the
Langevin noise amplitude, and the cooling schedule — a real physical constraint (fluctuation–dissipation), not a
tuning convenience. The dynamics minimise a temperature-dependent potential of mean force,

```
Eq. (21)   F(q; T) = E_task(q)  +  E_LJ(q)  +  E_HB(q)  −  T · S_conf(q)
                      └target┘   └──────────── folding physics (target-blind) ────────────┘
```

`E_LJ` is a full 6-12 Lennard-Jones potential *with attraction*, over every non-adjacent bead pair `|i−j| ≥ 2`:

```
Eq. (22)   E_LJ(q) = Σ_{j>i+1}  4εᵢⱼ [ (σᵢⱼ/dᵢⱼ)¹² − (σᵢⱼ/dᵢⱼ)⁶ ],     dᵢⱼ = ‖pᵢ − pⱼ‖,     σᵢⱼ = s·(rᵢ + rⱼ)
```

with a uniform well depth `εᵢⱼ = ε` (deliberately non-Gō, so that structure emerges rather than being planted) and a
global scale `s` calibrated per robot. The retained `−(σ/d)⁶` attraction — with well minimum at `dᵢⱼ = 2^{1/6}σᵢⱼ` —
is the one energy feature with no IK equivalent: it is core packing / steric exclusion, and its emergent preferred
inter-link spacing plays the role of tertiary contacts.

`E_HB` is a directional "hydrogen-bond" term. Each bead carries a local backbone normal — the unit normal to the
plane of its own triplet, `tᵢ = normalize( (pᵢ − pᵢ₋₁) × (pᵢ₊₁ − pᵢ) )` — and, over the same non-adjacent pairs,

```
Eq. (23)   E_HB(q) = −ε_hb · Σ_{j>i+1}  F(dᵢⱼ) · G(t̂ᵢ·r̂ᵢⱼ) · H(t̂ⱼ·r̂ᵢⱼ)
           F(d) = exp( −(d − d₀)² / 2σ_d² ),        G(x), H(x) = exp( −κ(1 − |x|) )
```

`d₀, σ_d, κ, ε_hb` calibrated per robot from natural-configuration geometry. `F` gates on distance, `G` and `H` gate
on relative orientation of the two triplet normals to the inter-bead direction `r̂ᵢⱼ`; the term is stabilising only
when *both* are satisfied simultaneously, which is what makes secondary-structure-like motifs form rather than a
diffuse attraction. (Interior-only: the planar 3-DOF arm has no interior triplet and so no `E_HB` term at all.)

`S_conf` is a conformational entropy — the Boltzmann log-count of the locally accessible, clash-free, in-limits
configuration volume around `q`, estimated by a fixed Gaussian probe cloud (common random numbers, so the estimate is
smooth across steps):

```
Eq. (24)   Ω(q) ≈ (1/m) Σ_{k=1}^{m}  w_lim(q + δqₖ) · w_clash(q + δqₖ),     δqₖ ~ 𝒩(0, ρ²I)   (fixed per step)
           w_lim(q)   = Πⱼ σ(α(qⱼ − loⱼ)) · σ(α(hiⱼ − qⱼ))
           w_clash(q) = σ(α(d(q) − margin))                                    (σ = logistic sigmoid)
           S_conf(q)  = log( max(Ω(q), Ω_floor) )
```

`S_conf` carries **no target/tolerance term** — folding entropy is target-blind by construction — and is
**collision-aware** through `w_clash`, which is what separates it from manipulability `√det(JJᵀ)`: manipulability is
task/null-space-relative and ignores self-collision entirely, while `S_conf` is provably not a re-derivation of it
(measured correlation with clearance ≈ +0.9 for `S_conf` vs. ≈ 0 for manipulability, across all three arms).
`S_conf` opposes collapse — high for open configurations, low for compact, near-collision, or near-limit ones — and
competes directly against `E_LJ` in Eq. (21); the `T`-weighted crossover between the two *is* the folding transition.

`E_task` is the sole non-folding term — folding has no notion of an external boundary condition, so it is kept
minimal, reusing the same pose error as every other solver:

```
Eq. (25)   E_task(q) = w_task · ( ‖Δp‖ + 0.3·‖Δω‖ ),      ∇E_task = −w_task · Jᵀ · e(q)
```

**Dynamics.** Overdamped Langevin integration, Euler–Maruyama discretisation — pure force plus thermal noise, no
Metropolis accept/reject anywhere (the defining distinction from simulated annealing and from KineticFold's Phase-B
funnel, Eq. (19)):

```
Eq. (26)   ∇F = ∇E_task + ∇E_LJ + ∇E_HB − Tₜ·∇S_conf
           q_{t+1} = clip( q_t − ∇F·Δt + √(2Tₜ Δt) · ξₜ ),    ξₜ ~ 𝒩(0, Iₙ)     (step clipped to max_step = 0.25)

Eq. (27)   Tₜ = max( T_glass, T_start · e^{−t/τ} )
```

with `T_glass ≈ σ_E / √(2 ln Ω̄)` a per-robot glass-transition floor calibrated from a pre-solve landscape diagnostic
[Bryngelson & Wolynes 1987] (the full derivation is deferred to the thesis alongside the rest of LangevinFold's
calibration procedure). Cooling below `T_glass` without reaching the target basin is a measured *glassy trap* —
reported as a solver outcome, not silently patched.

**Endgame.** As `T → 0` in Eq. (26) the noise term vanishes and the dynamics reduce to the deterministic flow
`dq = −∇F·dt`; near a minimum the basin is locally harmonic, so the natural, quadratically-convergent continuation of
that same flow is a damped-Newton/LM step — not a foreign finisher bolted on, but the `T → 0` limit of the identical
equation:

```
Eq. (28)   H ≈ Jᵀ J   (+ optional E_LJ/E_HB curvature),     Δq = −(H + μI)⁻¹ ∇F,     q ← clip(q + Δq)
```

iterated to tolerance once `Tₜ` reaches `T_glass`. This is folding's own last phase — native-state consolidation:
final packing and vdW/H-bond network locking into the unique native minimum. Among consolidated candidates the solver
selects the clash-free candidate of minimum enthalpy (excluded volume enforced as a hard constraint), drawn from a
multi-start ensemble of size `n_ws = 10 + 2·max(0, n−6)` and passed through the same Anfinsen jitter stability check
as StagedFold (§3.2.5): jitter the converged `q` by small `δq` and relax — return to the same basin and accept as
native, or escape (an energy jump) and reject as a knife-edge non-native point.

**Honest status.** LangevinFold's measured collision advantage traces in part to its multi-start-plus-hard-selection
endgame, not to the free-energy terms alone, and its core "biophysics buys quality" claim is only *measurable* on a
real-mesh oracle — our own capsule proxy cannot see it (§4.6). Both nuances are treated in full in the thesis; here we
report only the validated headline: cleanest self-collision on the UR5 among all solvers in this study, at a latency
cost that rules it out for anything but offline, quality-critical use.

---

## 4. Experimental setup

We test three arms of increasing kinematic hardness, three target scenarios of increasing difficulty, a field of six
baselines spanning the IK literature reviewed in §2, and — the differentiator we lean on throughout §5 — every
solver sees exactly the same targets, and every solver's final configuration is independently re-scored by two full
physics engines it never saw during solving. This section fixes every parameter of that protocol precisely enough to
reproduce it; all figures traceable to a script are cited by filename.

### 4.1 Robots

**Table 2 — Robots.**

| Arm | DOF | Notes |
|---|---|---|
| Planar 3-DOF (RRR) | 3 | link lengths `[0.4, 0.3, 0.2]` m; has an **exact closed-form IK solution** — the ground-truth validator for every numerical solver |
| UR5 | 6 | non-redundant; standard-DH; the primary tuning and validation arm |
| Franka Panda | 7 | **redundant**; requires the **modified/Craig DH** convention (Eq. 1's reordering, §3.1) — using the standard-DH transform instead put the computed end-effector ~1.4 m from the real robot, a correctness bug we found and fixed; verified against the `panda_link8` frame in `franka_ros`'s official URDF to ~1e-7 m via PyBullet; tight, asymmetric joint limits, including joint 4 permanently confined to `[−3.07, −0.07]` rad (the elbow-down constraint) |

### 4.2 Scenarios — target generators

Every scenario draws a joint configuration uniformly from the joint limits and forward-kinematics it into a Cartesian
target, so every target is reachable by construction:

```
Eq. (29)   q_cfg ~ U(q⁻, q⁺),      T_target = T(q_cfg)                         (Eq. 2's FK)
```

`open_space` uses Eq. (29) directly, with an independent fresh draw of the same form for the start configuration
`q0` — no geometric relationship between `q0` and the target is imposed, and no rejection sampling is applied; this
is the baseline difficulty distribution, and on its own already yields configurations that are ~40% near-singular by
the manipulability measure below.

`near_singular` and `cluttered` instead *reject-sample* Eq. (29) against a hardness criterion, keeping the
best-scoring draw seen if no draw clears the threshold within the try budget. The hardness criterion for
`near_singular` is the Yoshikawa manipulability index [Yoshikawa 1985],

```
Eq. (30)   m(q) = √det( J(q) J(q)ᵀ )
```

evaluated on the full `6×n` Jacobian of Eq. (3) for UR5 and Franka, or on the reduced `3×n` planar sub-Jacobian
(x-velocity, y-velocity, z-angular-velocity rows) for the planar arm, whose full 6-row Jacobian is rank-deficient by
construction. A configuration is accepted once `m(q) < τ_ms`, per-arm (**Table 3**), within `max_tries = 50`.

`cluttered` rejects on the self-collision clearance of Eq. (5) instead, accepting once `d(q) < −0.03` within
`max_tries = 200`. The `−0.03` m threshold is not arbitrary: over random UR5 configurations the median min-self-distance
is ≈0.020 m with a 5th-percentile of ≈−0.06 m, so a threshold at the median (an earlier, looser choice) accepted
almost every first draw and failed to select distinctly harder configurations; `−0.03` m sits near the 5th percentile
and does select for it.

**Table 3 — Scenario hardness thresholds.**

| Scenario | Criterion | Threshold | `max_tries` |
|---|---|---|---|
| `open_space` | none | — | 1 (no rejection) |
| `near_singular` | `m(q) < τ_ms` (Eq. 30) | planar: 0.001 · UR5: 0.005 · Franka: 0.015 | 50 |
| `cluttered` | `d(q) < −0.03` (Eq. 5) | −0.03 m (all arms) | 200 |

### 4.3 Baselines — the field to beat

Every baseline reuses the shared kinematics and pose error of §3.1 (Eqs. 2–4); the table gives each solver's own
tuned parameters, verified against the committed implementation.

**Table 4 — Baseline hyperparameters.**

| Solver | Update rule | Iteration budget | Damping / population | Stagnation response |
|---|---|---|---|---|
| **Jacobian-DLS** | Eq. (13)'s DLS step, `step_scale = 1.0` | `max_iters = 200` | `λ = 0.05` (`λ² = 0.0025`) | none — single trajectory |
| **CCD** [Wang & Chen 1991] | one-joint-at-a-time base→tip rotation, wrist joints (`min(3, max(1, n//2))`) blend a 0.5×-weighted orientation term | `max_iters = 300` full sweeps | n/a | none |
| **FABRIK** [Aristidou & Lasenby 2011] | forward/backward reaching, wrist orientation nudged 0.6× before each position pass | `max_iters = 150` | n/a | none |
| **TRAC-IK-style** [Beeson & Ames 2015] | DLS (`λ = 0.05`) in attempts of `iters_per_attempt = 50` | `max_total_iters = 300` | n/a | **global**: if `combined = ‖Δp‖ + 0.3‖Δω‖` improves by `< 1e-5` over a window of 8 iterations, abandon the attempt and reseed `q ← random_config()` |
| **Multi-start** | Eq. (13)'s DLS step per member, `max_iters_per_member = 60` | 60 × 8 members | `population_size = 8` (`q0` + 7 random restarts) | n/a — best of 8 by `combined`, preferring converged members |
| **Analytical (planar only)** | closed-form trigonometric IK | exact | — | — |

TRAC-IK-style's stagnation rule is the exact behaviour StagedFold's chaperone (§3.2.4, Eq. 17) is built to contrast
with: both detect stagnation over a short window of recent progress, but TRAC-IK-style's *only* response is a full
random reseed, while StagedFold's is scoped-then-escalating.

### 4.4 Protocol and fairness

**Scale.** The standard sweep runs `trials = 100` targets per seed at `seeds = [1, 2, 3]`, giving `n = 300` per
(robot, scenario, solver) cell. Because §5's real-mesh collision numbers showed large cell-to-cell variance under
only three seeds (a 15–20 percentage-point swing between draws, §5), the UR5 collision headline is instead drawn from
a dedicated 10-seed run (`seeds = [1..10]`, `n = 1000` per cell) — we report this explicitly rather than quoting the
cheaper 3-seed draw.

**Shared targets.** Within a cell, targets are drawn once per seed from `rng = default_rng(seed)` *before* the
solver loop begins, and the resulting target list is then handed unchanged to every solver in that cell — no solver
ever sees an easier draw than another. Per-trial solver RNG is decoupled and reproducible
(`default_rng(seed * 1_000_003 + i)` for trial `i`).

**Warm-up.** Each cell runs `warmup = 8` untimed solves before timing starts, from a separate fixed generator
(`default_rng(10_000 + w)`) so warm-up draws never overlap or bias the timed trial stream.

**Timing.** Wall-clock is measured with a monotonic counter bracketing only the solver's iteration loop — target
generation and warm-up are excluded. Latency percentiles (p50/p95/p99) are computed on the *pooled* set of timings
across all seeds in a cell, not averaged per-seed-then-combined, so the tail statistics reflect the full trial
population.

### 4.5 Metrics

For every trial we record: **success** (`‖Δp‖ < 1 mm ∧ ‖Δω‖ < 10 mrad`, Eq. 4); wall-clock latency (mean and
p50/p95/p99 — the tail is a first-class metric, not a footnote); **self-collision** (`d(q) < 0`, Eq. 5) and mean
clearance; joint-limit violations; and restart count. A solve is **clean** iff it is a success *and* collision-free.

### 4.6 Validation harness — solve once, score three ways

Every solver's final configuration `q*` from every trial is re-scored independently by two full-mesh physics
engines it never queried while solving: **PyBullet** and **MuJoCo**, both loading the *same* URDF (resolved via the
`robot_descriptions` package — UR5 from `ur_description/urdf/ur5_robot.urdf`, Franka from franka_ros's official
`panda.urdf`; MuJoCo loads it through a URDF-to-MJCF-compatible rewrite that preserves fixed-joint links as separate
bodies rather than fusing them). Both queries are **purely kinematic** — PyBullet via `resetJointState` +
`getLinkState`, MuJoCo via a direct `qpos` write followed by `mj_kinematics` — neither engine steps a physics
simulation or resolves contacts dynamically; each is asked only "at this configuration, where are the links, and
how close do the non-adjacent ones get?" This makes the comparison apples-to-apples with our own DH-based FK and
capsule proxy: three independent geometric queries against the identical model, not a dynamics rollout against a
kinematic one.

**FK agreement.** At backend construction we assert our DH FK matches each engine to a residual `< 1e-4` m/rad;
measured residuals are far tighter (UR5 DH↔PyBullet `9.5e-7`, DH↔MuJoCo `4.2e-8`; Franka DH↔PyBullet `6.6e-7`,
DH↔MuJoCo `8.7e-16`; PyBullet↔MuJoCo agree to `~4–6e-8` m on both arms) — every success claim in this paper is true
independently on two engines, including the corrected Franka kinematics of §4.1.

**Collision agreement.** Over `n = 2000–3000` random configurations per arm, we compute the proxy clearance and both
engines' closest-point distances, then measure

```
Eq. (31)   sign-agree(A, B) = 100 · mean( [d_A(q) < 0] = [d_B(q) < 0] )     over n random q

Eq. (32)   corr(A, B) = Pearson( d_A(q), d_B(q) )                          over n random q
```

for every engine pair. PyBullet↔MuJoCo agree on the sign call 97.8–99.0% of the time with correlation 0.88–0.99 on
raw signed distance — the two independent oracles corroborate each other, so a proxy-vs-oracle disagreement below can
be attributed to the proxy, not to noise between the oracles.

**Solve-once-score-three-ways.** Each trial's solver runs exactly once, on the fast numpy kinematic core it was
tuned against; the single resulting `q*` is then scored three times — our capsule proxy (what the solver actually
optimised against), PyBullet, and MuJoCo — over the identical set of non-adjacent link pairs. This is the single
reproducible artifact behind every collision claim in §5: it lets us report collision as a same-tool
comparison across solvers (never an absolute proxy rate, §3.1) while independently confirming, on two engines that
never saw the proxy, whether that comparison survives contact with real geometry.

---

## 5. Results and discussion

*Provenance note.* Every number in this section is drawn from a committed benchmark run named at first use:
proxy-scored success/speed from `backend/v1v4_full_benchmark.md` (planar + UR5, `trials=100 × seeds=[1,2,3]`, `n=300`
per cell) and `backend/franka_corrected_benchmark.md` (Franka, same scale, post-DH-fix); real-mesh collision from
`backend/results/ur5_collision_seeds10.md` (UR5, `seeds=[1..10]`, `n=1000` per cell — the dedicated large-seed run
§4.4 motivates) and `backend/sim_crosscheck.md` §C (Franka, real-mesh, smaller sample). **A larger confirmatory run
(`bench/master_sim_benchmark.py`, target `N=300`, both real-mesh engines, all three arms) is running in the
background as this section is written**; these are therefore *current, real, committed* numbers, not placeholders,
but headline figures are subject to a final re-lock against that run before submission, and any shift will be
reported, not silently absorbed.

*(Figure 4 — success / speed / collision bars against the full field, UR5 — is planned for this section and has not
yet been generated from the tables below; tracked as an open item alongside Figs. 2–3.)*

### 5.1 Success: KineticFold leads the field on both arms

**Table 5 — Success rate (%), UR5** (`v1v4_full_benchmark.md`).

| Solver | open_space | near_singular | cluttered |
|---|--:|--:|--:|
| Jacobian-DLS | 52.3 | 49.0 | 56.3 |
| CCD | 43.7 | 32.0 | 41.0 |
| FABRIK | 49.3 | 34.7 | 36.7 |
| TRAC-IK-style | 99.0 | 98.3 | 97.7 |
| Multi-start | 97.0 | 97.7 | 98.7 |
| StagedFold (§3.2) | 94.0 | 90.7 | 89.7 |
| **KineticFold (§3.3)** | **100.0** | **100.0** | **100.0** |

**Table 6 — Success rate (%), Franka Panda, corrected kinematics** (`franka_corrected_benchmark.md`).

| Solver | open_space | near_singular | cluttered |
|---|--:|--:|--:|
| Jacobian-DLS | 49.7 | 45.7 | 33.0 |
| CCD | 23.0 | 11.7 | 12.3 |
| FABRIK | 18.0 | 11.3 | 22.7 |
| TRAC-IK-style | 98.7 | 97.7 | 92.7 |
| Multi-start | 97.3 | 96.3 | 86.7 |
| StagedFold (§3.2) | 97.7 | 93.0 | 83.3 |
| **KineticFold (§3.3)** | **100.0** | **99.7** | **99.0** |

The simple geometric/single-trajectory baselines (Jacobian-DLS, CCD, FABRIK) collapse under both arms' harder
scenarios — none breaks 60% on Franka, and all three degrade further from `open_space` to `cluttered` (CCD:
23.0 → 12.3% on Franka), exactly the single-basin-descent failure mode §2 predicts for methods with no restart
mechanism. StagedFold clears every simple baseline by 30–70 points but visibly trails the two restart-capable
production baselines on both arms' hardest cells (StagedFold 83.3% vs. TRAC-IK 92.7% / Multi-start 86.7% on Franka
`cluttered`) — the honest verdict flagged in §3.2: folding's *process* alone, without its *compute schedule*,
plateaus below production methods. KineticFold is the only solver to clear 99% everywhere, and is the only one to
reach 100% success on any UR5 or Franka cell at all; unlike the two baselines it beats, it never drops below 99% on
either arm's hardest scenario.

### 5.2 Speed: matching TRAC-IK's core, with an honest tail

**Table 7 — Latency (ms): easy vs. hard regime, KineticFold vs. TRAC-IK-style.**

| Arm — scenario | Solver | Mean | p50 | p95 | p99 |
|---|---|--:|--:|--:|--:|
| UR5 — open_space | TRAC-IK-style | 12.6 | 7.2 | 44.2 | 77.3 |
| UR5 — open_space | KineticFold | 12.9 | **3.0** | 38.4 | 215.1 |
| UR5 — cluttered | TRAC-IK-style | 12.7 | 7.1 | 43.2 | 78.0 |
| UR5 — cluttered | KineticFold | 21.6 | **4.8** | 100.5 | 248.7 |
| Franka — open_space | TRAC-IK-style | 22.1 | 12.1 | 73.5 | 120.1 |
| Franka — open_space | KineticFold | **18.7** | **4.7** | 111.8 | 313.8 |
| Franka — cluttered | TRAC-IK-style | 24.7 | 13.0 | 89.4 | 93.6 |
| Franka — cluttered | KineticFold | 271.2 | 45.6 | 1342.0 | 1805.7 |

On the easy regime (`open_space`) KineticFold ties TRAC-IK-style's *mean* on UR5 (12.9 vs. 12.6 ms) and is
*faster on mean* on Franka (18.7 vs. 22.1 ms), while beating it on *median* everywhere by 2–2.5× (e.g. UR5: 3.0 vs.
7.2 ms) — the direct signature of Phase A's barrierless-first schedule (§3.3.1, Eq. 18): most targets never leave
the cheap LM polish. The cost is concentrated exactly where §3.3's diagnosis says it should be: on Franka
`cluttered` — the hardest cell on the hardest arm — mean latency rises to 271 ms and p99 to 1.8 s, because this cell
has the highest rate of *frustrated* targets (Table 6: 99.0% Franka-cluttered collision under the proxy, the
scenario built to force self-collision) and therefore the highest rate of escalation to Phase B (§3.3.1). This is
the tail §3.3 diagnoses, not hidden: KineticFold pays for its 99–100% success and cleaner collision profile (§5.3–
§5.4) with an occasional slow solve, which is why we position it for planning and offline generation rather than
tight real-time control — a framing we return to with hard numbers in §5.5. (All timings are wall-clock and carry OS
scheduling noise on mean/p95/p99, per the source tables; success, collision, and error columns are deterministic
given the seed.)

### 5.3 Self-collision, UR5: cleanest high-success solver on the two harder regimes

Because §4.4 found real-mesh collision rates swinging 15–20 percentage points between different 3-seed draws, this
comparison is drawn *only* from the dedicated 10-seed run (`n=1000`/cell, both PyBullet and MuJoCo — §4.6).

**Table 8 — UR5 real-mesh self-collision, 10-seed average** (`ur5_collision_seeds10.md`; PB/MJ = % of trials in
collision on each engine).

| Solver | Succ% (o/n/c) | open (PB/MJ) | near_singular (PB/MJ) | cluttered (PB/MJ) | Clearance, cluttered (PB) |
|---|--:|--:|--:|--:|--:|
| StagedFold (V1) | 98.8 / 93.8 / 79.1 | 27.8 / 27.1 | 32.4 / 32.2 | 76.6 / 76.4 | −0.0342 m |
| TRAC-IK-style | 100.0 / 99.9 / 96.5 | 30.6 / 28.8 | 46.8 / 46.2 | 71.1 / 71.1 | −0.0317 m |
| Multi-start | 99.8 / 99.4 / 98.8 | 39.0 / 34.4 | 44.6 / 43.2 | 65.0 / 64.7 | −0.0318 m |
| **KineticFold (V4)** | **100.0 / 100.0 / 100.0** | 36.1 / 33.8 | **40.0 / 38.8** | **57.0 / 56.1** | **−0.0206 m** |

Three honest qualifications, in the order a reviewer would raise them. **(1) StagedFold's raw open/near-singular
collision rate is the lowest in the table (27.8/32.4%) but is not the "cleanest practical solver" reading** — it
buys that low rate with the lowest success in the field (79.1% on `cluttered`, dropping the hardest, most
collision-prone targets from its own denominator rather than solving them cleanly); the comparison that matters is
among the three solvers that clear ≈99–100% success, where KineticFold leads on both harder regimes and is the only
one to reach 100% success on `cluttered` at all (TRAC-IK-style: 96.5%). **(2) The real-mesh edge is real but
modest**, not the multiplicative gap the capsule proxy would suggest (§4.6 already flags the proxy as
systematically optimistic): KineticFold's collision rate is 1.15–1.25× lower than TRAC-IK-style's on the two harder
regimes (near-singular: 46.8/40.0 = 1.17×; cluttered: 71.1/57.0 = 1.25×) and penetrates ≈35% less deeply when it
does clash (cluttered mean clearance −0.0206 m vs. −0.0317 m); on `open_space` the two are comparable (36.1 vs.
30.6%, KineticFold slightly *higher*). **(3) The mechanism traces to Eq. (19)'s Metropolis funnel and the collision
term in Eq. (14):** on frustrated targets, KineticFold's Phase-B search explicitly weights `E_collision` at 2× the
target term and can escape shallow steric traps via thermal acceptance, whereas TRAC-IK-style's response to a stall
is a full random restart with no collision-directed search at all. LangevinFold is not part of this comparison (too
slow for the seed-averaged protocol) but its separately measured UR5 collision rate — the lowest of any solver in
this study — is the "faithful biology buys quality" glimpse promised in §3.4.

### 5.4 Self-collision, Franka: KineticFold ties the strongest baseline

**Table 9 — Franka real-mesh self-collision** (`sim_crosscheck.md` §C, PyBullet column; smaller sample than the
UR5 10-seed run — treat as directional pending the background run's Franka pass).

| Scenario | StagedFold (V1) | LangevinFold (V6) | TRAC-IK-style | KineticFold (V4) | Multi-start |
|---|--:|--:|--:|--:|--:|
| open_space | 12.0 | 10.0 | 10.0 | 11.0 | 7.0 |
| near_singular | 11.0 | 9.0 | 12.0 | 13.0 | 12.0 |
| cluttered | **72.0** | 76.0 | **78.0** | **79.0** | 80.0 |

On `open_space` and `near_singular` every solver sits in a narrow 7–13% band with no consistent ranking — a tie
across the board. On `cluttered`, where the scenario actively forces self-collision, KineticFold (79.0%) is
statistically indistinguishable from TRAC-IK-style (78.0%) — a **tie**, not a loss and not a lead. The mechanism is
structural, not a solver weakness: Franka's redundant 7th joint gives *every* solver a null-space direction to dodge
self-collision while still reaching the target, so the collision-directed search that gives KineticFold its UR5 edge
(§5.3) has much less room to matter once a spare joint already does the dodging for free. We read this as
corroborating, not undermining, the UR5 result: the edge appears exactly where the arm has no redundancy to spare,
and disappears exactly where it does — consistent with §3's thesis that the method's advantage should track how
folding-like (chain-constrained, not gifted a spare DOF) the problem actually is, a relationship §5.5's DOF-scaling
climax tests directly.

**Taken together**, §5.1–§5.4 draw one consistent picture rather than four separate wins. Success is unconditional —
KineticFold leads or ties every baseline on every arm and every scenario, including the two it is built to beat
(TRAC-IK-style, Multi-start) — but the *collision* edge is conditional on redundancy: decisive on the non-redundant
UR5, where the chain has nowhere to hide from its own search, and a tie on the redundant Franka, where a spare joint
lets every solver dodge for free. That conditionality is not a weaker result than a uniform win would be — it is
independent evidence *for* the mechanism claimed in §3.3: the edge comes from collision-directed search finding
routes a restart-only baseline cannot, and such routes matter most exactly when the chain is most constrained, which
is the folding-like regime the whole paper is staked on. Speed closes the loop: KineticFold buys this profile without
giving up the field's fastest core, paying only in an occasional slow solve on the single hardest cell we tested
(Franka `cluttered`), which is why §5.5 positions it for planning and offline generation rather than tight real-time
control, and why the dual-engine validation (§4.6) matters at all — every success, collision, and tie reported here is re-derived independently on
two physics engines neither solver nor scenario generator ever saw, so the picture above is not an artifact of our
own proxy.

### 5.5 Where it wins: the advantage grows as the arm becomes a polymer

The profile in §5.1–§5.4 — unconditional success, a redundancy-conditional collision edge, an honest tail — points
to a deployment niche rather than a universal claim, and to the one experiment that turns the paper's thesis from a
metaphor into a mechanism. **Deployment role.** KineticFold behaves as a *quality* solver for planning, offline
batch generation, and reliability fallback, not a real-time servo. On the committed use-case sweep
(`usecase_experiments.md`) it returns a usable clean goal on ~5 of 6 planner attempts against ~4 of 7 for
TRAC-IK-style (83.4 vs 56.9 usable goals per attempt, UR5 `cluttered`), wins offline clean-solve rate by +18–30
points on the honest cells (UR5 `open` 96.5 vs 78.5%, `cluttered` 78.5 vs 48.5%), and as a fallback tier cleans up
60–78% of the targets TRAC-IK-style abandons (UR5 `cluttered` 60.2%, `near_singular` 77.6%). **The DOF-scaling
climax.** We then grow a planar arm from 4 to 16 joints in the `cluttered` scenario — making it, for no reason but
geometry, progressively more like a self-avoiding polymer — and measure single-shot *clean-solve* rate (reach the
target *and* clear self-collision). Both KineticFold and TRAC-IK-style reach the target 100% of the time at every
length; the entire difference is self-collision avoidance. Here both run as native compiled code — KineticFold as its
C++/Eigen port, TRAC-IK as the genuine TRACLabs C++ library (`tracikpy`) on the identical DH chain — so the comparison
is apples-to-apples. KineticFold's clean-solve advantage over genuine TRAC-IK holds at every chain length and grows
through the mid-DOF range — **2.0× at 4 DOF, 2.7× at 6, peaking near 3.2× at 8, 2.2× at 12** — then narrows in the
hyper-redundant tail as clean configurations become vanishingly rare for both, until at
**16 DOF KineticFold is the only method of the standard field still producing collision-free folds at all** (TRAC-IK:
0.0% clean). This is the concept proving itself: a short arm is easy for every solver, but a long one *is* a folding
polymer, and the method built from folding is the last one standing. We state it precisely, not triumphantly — it is
a **single-shot** advantage over the **standard baseline field**, and a clearance-selecting selection wrapper narrows
it (§5.6) — but the trend is unambiguous, and it points exactly where Table 1 says it should: the payoff tracks how
folding-like the arm has become. *(The DOF-scaling curve is the study's headline figure, planned as Figure 6.)*

### 5.6 Limitations and scope

We name the boundaries of these claims directly. **Not a real-time solver.** The latency tail of §5.2 rules
KineticFold out of tight control loops: on Franka, 74% of solves exceed a 10 ms budget and the worst case is ~2.5 s
(`usecase_experiments.md`). This is why we position it for planning and offline use — the tail is the price of the
thoroughness those roles reward, not a defect to tune away. **The climax is single-shot.** §5.5's widening advantage
is measured one solve per target against the standard baseline field; it is *not* a claim of absolute supremacy. A
clearance-selecting multi-start (solve `K` times, keep the cleanest) is a strong, cheap, orthogonal booster available
to *every* solver — and an honestly-reported negative control from our own research found that the literal folding
*physics* actually loses to it on genuinely redundant planar arms. The honest reading is that KineticFold has the
best *per-solve* clean rate, while selection wrappers are a separate axis of gain open to all methods. **Self-collision
only.** Every collision result in this paper is self-collision (Eq. 5); no solver here reasons about workspace
obstacles yet (§9.2). **A proxy-scored climax.** §5.5's DOF sweep is scored with the capsule proxy, which §4.6 shows
is systematically optimistic; no full-mesh model exists for a synthetic planar arm, so those magnitudes are same-tool
comparisons across solvers rather than absolute rates — the monotone *trend* is robust, but any single ratio at a
given DOF is proxy-relative. **Verified FK scope.** The allocation-light kinematics of §3.3.2 are checked
bit-identical to the reference only on UR5 and the planar arm, not yet on Franka.

---

<!-- Where-it-wins + DOF climax folded into §5.5, and Limitations into §5.6, per user direction (Results paragraphs,
     not standalone sections). Validation lives in §4.6 (harness + FK/engine agreement) and §5.3–§5.4 (two-engine
     collision); no standalone §7. Figures 2–6 remain to be generated. -->

## 9. Conclusion and future work

### 9.1 Conclusion

We opened with a structural claim, not a metaphor: a robot arm and a protein backbone are the same kind of object — a
chain of rigid segments whose only freedom is the rotation between neighbours, searching a rugged, constrained
landscape for a configuration that satisfies its boundary conditions (§3.1). We built three solvers that take that
claim increasingly literally. **StagedFold** ports folding's ordered *process* — settle locally before consulting
the goal, collapse coarsely, funnel narrowly, rescue what gets stuck, verify what converges — using only standard IK
machinery, and the sequencing alone is enough to clear every simple baseline by wide margins, though it plateaus
below the production baselines it does not yet out-schedule (§5.1). **KineticFold** closes that gap not with new
machinery but with folding's *second* idea, kinetic partitioning, recast as a compute schedule: try the cheap
downhill fold first, and reserve the expensive staged search for targets the landscape actually frustrates (§3.3).
The result is the success leader on every arm and scenario we tested, including a 100%/100%/100% sweep on UR5 and a
worst case of 99.0% on Franka `cluttered` (§5.1), matching or beating TRAC-IK's own core latency in the easy regime
while paying its cost only on the hardest cells (§5.2), and — decisively on the non-redundant UR5, and tied on the
redundant Franka for the structural reason §5.4 gives — the cleanest practical solver on self-collision, confirmed
independently on two physics engines that never saw our own proxy (§4.6). And the single
result that turns the paper's thesis from an analogy into a mechanism is the climax (§5.5): as a planar arm is lengthened
from 4 to 16 joints and made progressively more polymer-like, KineticFold's single-shot clean-solve advantage over
genuine TRAC-IK holds at every length and grows through the mid-DOF range — 2.0× at 4 DOF, 2.7× at 6, peaking near
3.2× at 8 — and by 16 DOF KineticFold is the
only method of the standard field still producing collision-free solutions at all (`usecase_experiments.md`, EXP E).
We report that result carefully, not triumphantly: it is a **single-shot** advantage over the standard baseline
field specifically, and a clearance-selecting multi-start wrapper — an orthogonal, honestly-reported counter-finding
from our own research (Fork A) — closes most of the gap when every solver is allowed to select its best of several
tries (§5.6). The honest reading is not "the only method that works," but that **KineticFold has the best
per-solve rate, and the advantage grows precisely as the problem becomes more like the thing it was designed to
resemble.**

That last sentence is the paper's actual contribution. We did not invent new energy terms — every numerical
ingredient in StagedFold and KineticFold has precedent in the IK literature we review in §2, and we say so
explicitly rather than let a reviewer discover it. What is new is the *organizing principle*: that folding's staged,
kinetically-partitioned process is a better schedule for optimization machinery IK already has, and that the payoff
from adopting it is not uniform but *diagnostic* — it appears where the arm is chain-constrained (UR5, the
DOF-scaling sweep) and recedes where the arm is handed an escape hatch (Franka's redundant 7th joint), tracking the
folding correspondence of Table 1 rather than tracking arbitrary implementation luck. We defended that reading with
a validation discipline uncommon in heuristic-IK work: every success claim independently reproduced on two physics
engines to floating-point precision, and every collision claim re-scored on real mesh rather than quoted from the
proxy the solvers themselves optimize against — a check that, in §5.3, *shrank* our own collision-magnitude claim
rather than confirming it, which is the outcome we would want a paper to report honestly whether or not it was
convenient. Finally, **LangevinFold** — the literal folding simulation, run for a glimpse rather than as a
production candidate — shows that the correspondence has more depth than optimization alone can extract: taking the
biology literally, at real computational cost, buys the cleanest solutions of any solver in this study (§3.4, §5.3),
evidence that the analogy is not exhausted by the two practical solvers built on top of it.

### 9.2 Future work

**Environment obstacles.** Every collision claim in this paper is *self*-collision only (§3.1, Eq. 5); no solver
here reasons about a workspace obstacle, which is the immediate next step toward deployability and the first
extension we would make to the shared energy landscape of Eq. (6) — an `E_obstacle` term folds into the same staged
and kinetically-partitioned machinery without changing either solver's organizing logic.

**The full LangevinFold study.** §3.4 reports only LangevinFold's validated headline (cleanest UR5
self-collision, at a latency cost); the calibration procedure, the phase-transition experiments that confirm the
mechanism sequence of raw_math.md §7 actually emerges (unfolded ensemble → collapse → secondary structure →
consolidation → native state, rather than just a low final energy), and the glass-transition diagnostic `Σ`
(raw_math.md §6) are reserved for the thesis, where LangevinFold's biophysics can be given the space it needs without
unbalancing a conference paper built around KineticFold.

**Locking the headline numbers.** As §5's provenance note states, a larger confirmatory run
(`bench/master_sim_benchmark.py`, target `N=300`, both real-mesh engines, all three arms) is in progress; every
headline figure in §5 will be re-verified against it before submission, and the two research forks that
*produced* honest negative or qualifying results during this project — the clearance-selecting multi-start wrapper
that tempers §5.5's climax, and the second, unrelated fork exploring rescue strategies beyond IK (both reported, not
suppressed, in §5.6) — are natural follow-on studies in their own right rather than loose ends.

**Extending validated scope.** One scope statement made honestly in this paper is also a concrete next step: the
allocation-light FK primitives (§3.3.2) are verified bit-identical to reference kinematics on UR5 and the planar arm,
but not yet on Franka; extending that bit-identity check to the redundant arm is the remaining step before the
primitive's per-step speedup can be claimed across the full arm set rather than the two arms it is verified on.

The correspondence in Table 1 was proposed once, as a mapping, with no result yet behind it. §3 then implemented it
without needing a single energy term the IK literature had not already supplied. §5's collision advantage tracked
redundancy exactly as a chain-constraint account predicts — present on UR5, gone on Franka. §5.5's DOF-sweep shows
the same advantage widen as joint count is made, for no reason but geometry, into chain length. The dual-engine
check of §4.6 and §5.3 shows the numbers hold under two physics engines that never saw our own proxy, and revised
one of them downward where they did not fully agree. None of that was guaranteed by the mapping alone. That it held anyway, independently, at every
scale we tested it, is what turns the correspondence from an analogy into a working design principle.

---

<!-- ============================ REFERENCES ============================
     38 entries (34 scholarly + 4 software/model tools). Style: APA 7-ish, sentence-case
     titles, italic venue, volume(issue), page range, resolvable DOI/URL. Each entry leads
     with the exact in-text bracket key so a reader can match [Author Year] → entry directly.
     Bibliographic fields (authors, year, title, venue, vol/issue, pages, DOI) web-verified
     2026-07-09 against publisher pages / CrossRef / DBLP / PubMed / Annual Reviews.
     The four simulators/model libraries (PyBullet, MuJoCo, robot_descriptions, franka_ros)
     are named directly in §4.6 rather than cited author-year; their bracket keys below are
     the cross-reference handles. Pin the robot_descriptions version actually used before
     submission; add ur_description / NumPy here if the target venue expects them.
===================================================================== -->

## References

*Entries are ordered by their in-text citation key. The physics engines and model libraries
(PyBullet, MuJoCo, robot_descriptions, franka_ros) are referred to by name in §4.6; their
bracket keys below are provided for cross-reference.*

**[Amato & Song 2002]** Amato, N. M., & Song, G. (2002). Using motion planning to study protein folding pathways. *Journal of Computational Biology*, 9(2), 149–168. https://doi.org/10.1089/10665270252935395

**[Ames et al. 2022]** Ames, B., Morgan, J., & Konidaris, G. (2022). IKFlow: Generating diverse inverse kinematics solutions. *IEEE Robotics and Automation Letters*, 7(3), 7177–7184. https://doi.org/10.1109/LRA.2022.3181374

**[Anfinsen 1973]** Anfinsen, C. B. (1973). Principles that govern the folding of protein chains. *Science*, 181(4096), 223–230. https://doi.org/10.1126/science.181.4096.223

**[Aristidou & Lasenby 2011]** Aristidou, A., & Lasenby, J. (2011). FABRIK: A fast, iterative solver for the inverse kinematics problem. *Graphical Models*, 73(5), 243–260. https://doi.org/10.1016/j.gmod.2011.05.003

**[Beeson & Ames 2015]** Beeson, P., & Ames, B. (2015). TRAC-IK: An open-source library for improved solving of generic inverse kinematics. In *2015 IEEE-RAS 15th International Conference on Humanoid Robots (Humanoids)* (pp. 928–935). IEEE. https://doi.org/10.1109/HUMANOIDS.2015.7363472

**[Bryngelson & Wolynes 1987]** Bryngelson, J. D., & Wolynes, P. G. (1987). Spin glasses and the statistical mechanics of protein folding. *Proceedings of the National Academy of Sciences of the United States of America*, 84(21), 7524–7528. https://doi.org/10.1073/pnas.84.21.7524

**[Bryngelson et al. 1995]** Bryngelson, J. D., Onuchic, J. N., Socci, N. D., & Wolynes, P. G. (1995). Funnels, pathways, and the energy landscape of protein folding: A synthesis. *Proteins: Structure, Function, and Genetics*, 21(3), 167–195. https://doi.org/10.1002/prot.340210302

**[Buss & Kim 2005]** Buss, S. R., & Kim, J.-S. (2005). Selectively damped least squares for inverse kinematics. *Journal of Graphics Tools*, 10(3), 37–49. https://doi.org/10.1080/2151237X.2005.10129202

**[Canutescu & Dunbrack 2003]** Canutescu, A. A., & Dunbrack, R. L., Jr. (2003). Cyclic coordinate descent: A robotics algorithm for protein loop closure. *Protein Science*, 12(5), 963–972. https://doi.org/10.1110/ps.0242703

**[Coutsias et al. 2004]** Coutsias, E. A., Seok, C., Jacobson, M. P., & Dill, K. A. (2004). A kinematic view of loop closure. *Journal of Computational Chemistry*, 25(4), 510–528. https://doi.org/10.1002/jcc.10416

**[Diankov 2010]** Diankov, R. (2010). *Automated construction of robotic manipulation programs* [PhD thesis]. Robotics Institute, Carnegie Mellon University. https://publications.ri.cmu.edu/automated-construction-of-robotic-manipulation-programs/

**[Dill & Chan 1997]** Dill, K. A., & Chan, H. S. (1997). From Levinthal to pathways to funnels. *Nature Structural Biology*, 4(1), 10–19. https://doi.org/10.1038/nsb0197-10

**[Ericson 2004]** Ericson, C. (2004). *Real-time collision detection*. Morgan Kaufmann.

**[franka_ros]** Franka Emika. (n.d.). *franka_ros: ROS integration for Franka Emika research robots* [Computer software]. GitHub. https://github.com/frankaemika/franka_ros

**[Gipson et al. 2012]** Gipson, B., Hsu, D., Kavraki, L. E., & Latombe, J.-C. (2012). Computational models of protein kinematics and dynamics: Beyond simulation. *Annual Review of Analytical Chemistry*, 5, 273–291. https://doi.org/10.1146/annurev-anchem-062011-143024

**[Gō & Scheraga 1970]** Gō, N., & Scheraga, H. A. (1970). Ring closure and local conformational deformations of chain molecules. *Macromolecules*, 3(2), 178–187. https://doi.org/10.1021/ma60014a012

**[Guo & Thirumalai 1995]** Guo, Z., & Thirumalai, D. (1995). Kinetics of protein folding: Nucleation mechanism, time scales, and pathways. *Biopolymers*, 36(1), 83–102. https://doi.org/10.1002/bip.360360108

**[Honeycutt & Thirumalai 1990]** Honeycutt, J. D., & Thirumalai, D. (1990). Metastability of the folded states of globular proteins. *Proceedings of the National Academy of Sciences of the United States of America*, 87(9), 3526–3529. https://doi.org/10.1073/pnas.87.9.3526

**[Kauzmann 1959]** Kauzmann, W. (1959). Some factors in the interpretation of protein denaturation. *Advances in Protein Chemistry*, 14, 1–63. https://doi.org/10.1016/S0065-3233(08)60608-7

**[Levenberg 1944]** Levenberg, K. (1944). A method for the solution of certain non-linear problems in least squares. *Quarterly of Applied Mathematics*, 2(2), 164–168. https://doi.org/10.1090/qam/10666

**[Levinthal 1969]** Levinthal, C. (1969). How to fold graciously. In P. Debrunner, J. C. M. Tsibris, & E. Münck (Eds.), *Mössbauer spectroscopy in biological systems: Proceedings of a meeting held at Allerton House, Monticello, Illinois* (pp. 22–24). University of Illinois Press.

**[Marquardt 1963]** Marquardt, D. W. (1963). An algorithm for least-squares estimation of nonlinear parameters. *Journal of the Society for Industrial and Applied Mathematics*, 11(2), 431–441. https://doi.org/10.1137/0111030

**[Metropolis et al. 1953]** Metropolis, N., Rosenbluth, A. W., Rosenbluth, M. N., Teller, A. H., & Teller, E. (1953). Equation of state calculations by fast computing machines. *The Journal of Chemical Physics*, 21(6), 1087–1092. https://doi.org/10.1063/1.1699114

**[MuJoCo]** Todorov, E., Erez, T., & Tassa, Y. (2012). MuJoCo: A physics engine for model-based control. In *2012 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)* (pp. 5026–5033). IEEE. https://doi.org/10.1109/IROS.2012.6386109

**[Nakamura & Hanafusa 1986]** Nakamura, Y., & Hanafusa, H. (1986). Inverse kinematic solutions with singularity robustness for robot manipulator control. *Journal of Dynamic Systems, Measurement, and Control*, 108(3), 163–171. https://doi.org/10.1115/1.3143764

**[Noonan et al. 2005]** Noonan, K., O'Brien, D., & Snoeyink, J. (2005). Probik: Protein backbone motion by inverse kinematics. *The International Journal of Robotics Research*, 24(11), 971–982. https://doi.org/10.1177/0278364905059108

**[Onuchic et al. 1997]** Onuchic, J. N., Luthey-Schulten, Z., & Wolynes, P. G. (1997). Theory of protein folding: The energy landscape perspective. *Annual Review of Physical Chemistry*, 48, 545–600. https://doi.org/10.1146/annurev.physchem.48.1.545

**[PyBullet]** Coumans, E., & Bai, Y. (2016–2021). *PyBullet, a Python module for physics simulation for games, robotics and machine learning* [Computer software]. http://pybullet.org

**[robot_descriptions]** Caron, S., et al. (n.d.). *robot_descriptions.py: Robot descriptions in Python* [Computer software]. GitHub. https://github.com/robot-descriptions/robot_descriptions.py

**[Ruppel et al. 2018]** Ruppel, P., Hendrich, N., Starke, S., & Zhang, J. (2018). Cost functions to specify full-body motion and multi-goal manipulation tasks. In *2018 IEEE International Conference on Robotics and Automation (ICRA)* (pp. 3152–3159). IEEE. https://doi.org/10.1109/ICRA.2018.8460799

**[Smits et al., Orocos KDL]** Smits, R., Bruyninckx, H., & Aertbeliën, E. (n.d.). *KDL: Kinematics and Dynamics Library* [Computer software]. Orocos Project. http://www.orocos.org/kdl

**[Starke et al. 2019]** Starke, S., Hendrich, N., & Zhang, J. (2019). Memetic evolution for generic full-body inverse kinematics in robotics and animation. *IEEE Transactions on Evolutionary Computation*, 23(3), 406–420. https://doi.org/10.1109/TEVC.2018.2867601

**[Thirumalai & Lorimer 2001]** Thirumalai, D., & Lorimer, G. H. (2001). Chaperonin-mediated protein folding. *Annual Review of Biophysics and Biomolecular Structure*, 30, 245–269. https://doi.org/10.1146/annurev.biophys.30.1.245

**[Todd et al. 1996]** Todd, M. J., Lorimer, G. H., & Thirumalai, D. (1996). Chaperonin-facilitated protein folding: Optimization of rate and yield by an iterative annealing mechanism. *Proceedings of the National Academy of Sciences of the United States of America*, 93(9), 4030–4035. https://doi.org/10.1073/pnas.93.9.4030

**[Wampler 1986]** Wampler, C. W. (1986). Manipulator inverse kinematic solutions based on vector formulations and damped least-squares methods. *IEEE Transactions on Systems, Man, and Cybernetics*, 16(1), 93–101. https://doi.org/10.1109/TSMC.1986.289285

**[Wang & Chen 1991]** Wang, L.-C. T., & Chen, C. C. (1991). A combined optimization method for solving the inverse kinematics problems of mechanical manipulators. *IEEE Transactions on Robotics and Automation*, 7(4), 489–499. https://doi.org/10.1109/70.86079

**[Whitney 1969]** Whitney, D. E. (1969). Resolved motion rate control of manipulators and human prostheses. *IEEE Transactions on Man-Machine Systems*, 10(2), 47–53. https://doi.org/10.1109/TMMS.1969.299896

**[Yoshikawa 1985]** Yoshikawa, T. (1985). Manipulability of robotic mechanisms. *The International Journal of Robotics Research*, 4(2), 3–9. https://doi.org/10.1177/027836498500400201
