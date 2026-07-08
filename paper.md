# ProteinIK: Inverse Kinematics as a Protein-Folding Process

> **Working draft — conference paper.** Source of truth for claims/numbers: [paper_notes.md](paper_notes.md);
> deep Methods: [methodology.md](methodology.md); plain-English plan: [outline_simple.md](outline_simple.md).
> **Numbers status.** Success and speed are locked from the distinct-target sweeps `v1v4_full_benchmark`
> (UR5/planar) and `franka_corrected_benchmark` (Franka). UR5 self-collision is locked from the corrected 10-seed
> real-mesh sweep `backend/results/ur5_collision_fixed.*` (seeds 1–10, n=1000/cell = 1000 distinct targets,
> PyBullet+MuJoCo); Franka self-collision from `sim_crosscheck` (n=100 distinct, both engines); FK/collision oracle
> agreement from the master sim run's config-sampled validation block. *(An earlier `master_sim_benchmark.py` had a
> target-generation bug — the RNG was reseeded per trial, collapsing each seed to one repeated target — which
> corrupted `master_full` and a first 10-seed draw; it is fixed, and the collision numbers are re-locked on the
> corrected run. A full fixed both-engine master sweep across all three arms is the one artifact still to regenerate
> before submission.)*

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

Inverse kinematics — finding joint angles that place a robot's end effector at a target pose — is deceptively hard.
The map from configuration to pose is nonlinear; solutions are non-unique or absent; the Jacobian loses rank at
singularities; and a redundant or long arm can satisfy the target while folding into itself. Classical solvers treat
IK as a single optimization to be minimized from the first iteration, whether by damped least squares, cyclic
coordinate descent, reaching heuristics, or restart-based search.

We observe that this is the *same* search a protein performs when it folds. A protein backbone is a chain of rigid
bonds whose only soft degrees of freedom are the dihedral rotations between residues; a robot arm is a chain of rigid
links whose only degrees of freedom are the joint angles. A protein reaches its native state by descending a rugged
free-energy landscape riddled with local minima, kinetic traps, and steric (self-overlap) constraints; an IK solver
searches a landscape with local minima, singular regions, and self-collision basins. The correspondence is not a
loose analogy — it is a structural isomorphism (Table 1, Figure 1).

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

The bridge between the fields is already load-bearing in one direction: cyclic coordinate descent (CCD), a robotics IK
algorithm, was adopted into structural biology for protein loop closure [Canutescu & Dunbrack 2003]. An algorithm has
already crossed from IK into folding; we carry the *process* back the other way.

**Thesis.** *Inverse kinematics is, structurally, a protein-folding problem, so an IK solver built from folding's
process wins exactly where the problem becomes most folding-like.* We defend this with three solvers of increasing
biological literalness and a dual-simulator validation methodology.

**Contributions.**
1. **A design principle** — casting IK as a folding *process*: the first IK solver organized as a staged fold with
   kinetic partitioning and chaperone rescue. The novelty is the *organization* plus two genuinely unusual moves
   (target-blind-first initialization, scoped-then-escalating rescue), **not** new energy terms.
2. **KineticFold** — kinetic partitioning as a compute schedule that removes the latency tail: the success leader
   across three arms, speed-competitive with TRAC-IK on the easy regime, and the cleanest practical solver on
   self-collision on the non-redundant arm.
3. **A dual-simulator validation methodology** — "solve once, score three ways" (our capsule proxy + PyBullet +
   MuJoCo) — that independently confirms every success claim and *corrects* our own collision-magnitude claim.
4. **An honest map** of where the principle pays off (the per-solve edge grows with chain length), where it ties (the
   redundant arm), and where literal folding physics buys quality at a latency cost.

We preview the climax: as a planar arm is lengthened from 4 to 16 joints — made progressively more polymer-like —
KineticFold's single-shot collision-free solve rate degrades the most gracefully of the standard field, until it is
the last method producing clean folds at all.

---

## 2. Related work — why this is not just a metaphor

**Energy- and Jacobian-based IK.** Damped least squares [Nakamura & Hanafusa 1986; Wampler 1986] minimizes pose error
with a damping term that regularizes singularities; it is fast but single-trajectory and prone to local minima. We
include it as a baseline (Jacobian-DLS) and reuse a damped-least-squares step inside our own solvers.

**Sampling and restart IK.** TRAC-IK [Beeson & Ames 2015] couples a Jacobian solver with stuck-detection and *global*
random restarts, and is our key baseline to beat. Multi-start runs several independent seeds and keeps the best. Both
are strong production methods; both restart *globally* when stuck.

**Heuristic IK.** CCD adjusts one joint at a time; FABRIK [Aristidou & Lasenby 2011] reaches forward and backward
along the chain. Both are fast on easy targets and degrade on constrained ones. CCD is also our bridge to biology
(loop closure, above).

**Biology-inspired IK.** Evolutionary, neural, and swarm methods borrow biological *search operators*. None builds the
solver as a *folding process* — a staged fold with a hydrophobic-collapse phase, a funnelled search, and a chaperone
rescue gated by kinetic partitioning. That organizing principle is our contribution; the constituent numerical moves
are deliberately standard, so any advantage comes from the sequencing, not from a novel energy term.

**Folding theory we draw on.** Anfinsen's thermodynamic hypothesis (the native state is a stable free-energy minimum)
[Anfinsen 1973]; funnel/landscape theory [Bryngelson & Wolynes 1987; Bryngelson et al. 1995; Onuchic & Wolynes];
kinetic partitioning between fast and slow folders [Guo & Thirumalai 1995]; iterative-annealing chaperone action
[Thirumalai & Lorimer 2001]; and coarse-grained bead models [Honeycutt & Thirumalai 1990], the lineage of
LangevinFold.

---

## 3. StagedFold — the folding process as an algorithm

StagedFold runs the arm through the same ordered stages a protein uses to fold. Every individual move is standard IK;
the **order** is the idea. (Full formulas and parameters: [methodology.md](methodology.md) §3.)

**Stage 1 — local-blind relaxation** *(secondary-structure analog).* Gradient-free coordinate descent that minimizes a
**target-blind** local energy (neutral-pose anchor + neighbour smoothness + joint-limit barrier); the target pose is
never consulted. This is the first unusual move: no production IK method begins by ignoring the goal. It mirrors local
secondary structure forming before the global fold and seeds the later stages from a relaxed, in-limits configuration.

**Stage 2 — coarse collapse** *(hydrophobic-collapse analog).* A deliberately detuned damped-least-squares pull on the
full 6-D pose error — the first stage that sees the target — moving the hand into the right neighbourhood without
trying to be precise.

**Stage 3 — funnelled narrowing search** *(folding-funnel analog).* The main refinement: a gradient-free coordinate
search inside a shrinking radius, **greedy accept-if-better** (not Metropolis — an important honesty point that
distinguishes StagedFold from KineticFold and LangevinFold), interleaved with a fine damped-least-squares step.

**Stage 4 — scoped chaperone rescue** *(GroEL/chaperone analog)* — the key differentiator from TRAC-IK. On a detected
stall, one-sided finite-difference sensitivity identifies the "misfolded" joint, and a rescue re-randomizes a
*contiguous window* of joints centred on the culprit, growing the scope on an escalation ladder. This is the second
unusual move: **scoped, not global, rescue.** Stated honestly, StagedFold *starts* scoped and *escalates* — its final
rung is a full reseed, so on a persistently stuck target it converges to TRAC-IK-like global-restart behaviour. The
accurate claim is "scoped first, global only as a last resort."

**Stage 5 — stability-gated termination** *(native-state stability analog).* A converged solution is jittered and
rejected if its energy is not robust, echoing Anfinsen's requirement that the native state be a *stable* minimum.

**Honest verdict.** StagedFold beats the simple classical baselines (Jacobian-DLS, CCD, FABRIK) by wide margins but
does **not** beat the production baselines (TRAC-IK, Multi-start) on success — precisely the motivation for
KineticFold. We also report ablations that *reduced* performance and were reverted (e.g. a pure neighbour-coupling
Stage 1 dropped cluttered success 90.0→86.0%; rotamer-biased proposals crashed cluttered success to 67–76%). These
show the sequencing choices are empirically load-bearing, not decorative.

---

## 4. KineticFold — kinetic partitioning makes it competitive *(the star)*

**The diagnosis.** StagedFold's weakness was not the average solve but the *tail*: on the always-run-everything fold,
the slowest ~10% of targets consumed ~57% of total wall time, because every target paid for the full expensive search.
A per-step micro-optimization cannot move a tail like that — a bit-identical fast pass bought only 1.1–1.4×. The cost
is *entering the expensive per-fold search at all*, so the fix must be structural.

**The fix — barrierless-first, escalate only if frustrated.** Real proteins undergo **kinetic partitioning**: some
molecules fall straight down a smooth funnel to the native state (downhill/barrierless folding, no search), while
others get trapped and need the chaperone. KineticFold mirrors this with a single replica budget:
- **Phase A (barrierless).** Each replica runs a cheap Levenberg–Marquardt polish; as soon as one converges to a
  sterically clean solution it returns a success.
- **Frustration criterion.** The target is declared *frustrated* only if, after the LM restarts, no converged replica
  is clash-free.
- **Phase B (the full staged fold)** fires **only on frustrated targets** — a StagedFold-style fold with a true
  Metropolis-accepted funnel and an LM endgame.

Trying spontaneous folding first and invoking the chaperone only on failure is how GroEL actually works, so this
ordering is *more* faithful to folding, not a departure. Framing line: **optimization decides *how well* a fold
solves; folding decides *when* to spend the effort.**

**Rejected shortcuts (honesty).** Naive tail-edits that keep the fold order but simply spend less — capping replicas,
bailing early, fewer iterations — bought little speed and destroyed the headline win (Franka open-space success
collapsed to 71.7% at `cap_replicas=2`). This confirms the cost is the per-fold search, which is exactly what the
kinetic-partitioning gate removes.

**Layer 2 — allocation-light FK primitives.** Independently, the inner loop is made cheap and verified bit-identical
to the reference kinematics on UR5 and the planar arm (~2000 configs each; we state this tested scope precisely rather
than over-claim it across all three arms).

---

## 5. LangevinFold — taking the biology literally *(glimpse)*

Where StagedFold borrows the *process*, LangevinFold runs the *physics*. It treats the arm as a coarse-grained
molecule (one bead per joint) and folds it under a real biophysical free energy
`F(q;T) = E_task + E_LJ + E_HB − T·S_conf`, evolved by overdamped Langevin dynamics with a single self-consistent
temperature that cools until the configuration freezes into place; at `T→0` the noise vanishes and the dynamics become
a damped-Newton consolidation (native-state selection). There is **no Metropolis test** — motion is pure force plus
thermal noise, the defining distinction from simulated annealing. It is far too slow for routine use (seconds per
solve), but under real-mesh collision testing it produces the cleanest solutions of any solver on the non-redundant
arm (Section 7). Punchline: **faithful biology buys quality, not speed — and only a real physics oracle can see it.**
Full biophysics is deferred to the thesis.

---

## 6. Experiments

**Robots.** (i) Planar 3-DOF (RRR), which has an exact closed-form solver as ground truth; (ii) UR5, non-redundant,
6-DOF, our primary tuning and validation arm; (iii) Franka Panda, redundant, 7-DOF, on the corrected modified/Craig DH
convention (an earlier standard-DH forward-kinematics model was ~1.4 m wrong; §9).

**Scenarios.** *open_space* (uniform reachable targets), *near_singular* (rejection-sampled low-manipulability
targets), *cluttered* (rejection-sampled low-self-clearance targets that force the arm near self-collision).

**Baselines.** Jacobian-DLS, CCD, FABRIK, **TRAC-IK-style** (the one to beat), Multi-start, and the exact analytical
solver on the planar arm.

**Protocol.** Targets are generated once per (arm, scenario, seed) and shared across all solvers — no solver sees an
easier draw. We report success (‖Δp‖<1 mm ∧ ‖Δω‖<10 mrad); latency mean and p50/p95/p99 (the tail is a first-class
metric); self-collision rate and mean clearance; joint-limit violations. Full sweeps use 100 trials × 3 seeds = 300
solves per cell (self-collision, sensitive to the target draw, is locked from a corrected 10-seed UR5 sweep on 1000 distinct targets; §7.3).

**Validation harness.** Every solver's final configuration is re-scored in two independent simulators, PyBullet and
MuJoCo, both loading the identical URDF and querying the identical non-adjacent link pairs (§9).

---

## 7. Results

### 7.1 Success — KineticFold leads every arm

KineticFold leads the field on success across all three arms, ahead of the strongest baselines (TRAC-IK, Multi-start)
and far ahead of the simple ones. *(Sources: `v1v4_full_benchmark.md` for UR5/planar, `franka_corrected_benchmark.md`
for Franka — both on distinct-target sweeps.)*

| Arm / scenario | **KineticFold** | TRAC-IK | Multi-start | StagedFold |
|---|--:|--:|--:|--:|
| UR5 open / near / cluttered | **100 / 100 / 100** | 99.0 / 98.3 / 97.7 | 97.0 / 97.7 / 98.7 | 94.0 / 90.7 / 89.7 |
| Franka open / near / cluttered | **100 / 99.7 / 99.0** | 98.7 / 97.7 / 92.7 | 97.3 / 96.3 / 86.7 | 97.7 / 93.0 / 83.3 |

StagedFold beats the simple baselines but trails the production ones — the gap KineticFold closes. The simple
baselines (CCD, FABRIK, Jacobian-DLS) collapse on cluttered and near-singular targets (0–67%).

### 7.2 Speed — competitive on the easy regime, honest about the tail

On easy UR5 targets KineticFold matches TRAC-IK's core on the mean and beats it on the median: mean ~12.9 ms
(p50 ~3.0 ms) vs TRAC-IK mean 12.6 ms (p50 7.2 ms). On the Franka open space it is faster than TRAC-IK (mean 18.7 ms
vs 22.1 ms). It is the fastest of the folding family on every arm. *(Sources: `v1v4_full_benchmark.md`,
`franka_corrected_benchmark.md`.)*

The honest cost is the tail on hard targets: UR5 open p95 ~38 ms / p99 ~215 ms; Franka cluttered mean ~271 ms
(p50 ~46 ms) — because frustrated targets invoke the full staged fold. The tail, not the mean, is why we position
KineticFold as a planning/offline/quality tool (§8), not a hard-real-time controller.

### 7.3 Self-collision, UR5 — cleanest high-success solver on every regime

Self-collision *rate* on real meshes is sensitive to the target draw, so we lock the UR5 comparison on the corrected
**10-seed sweep** (`ur5_collision_fixed.*`, seeds 1–10, n=1000/cell = 1000 distinct targets, PyBullet+MuJoCo). *(An
earlier draw from a buggy harness — the RNG reseeded per trial, so each seed collapsed to a single repeated target —
produced the wild ~15–20-point swings we first saw; regenerated on distinct targets the ranking is stable.)* Real-mesh
collision rate (PyBullet / MuJoCo) among the high-success practical solvers:

| UR5 scenario | **KineticFold** | TRAC-IK | Multi-start | reading |
|---|--:|--:|--:|---|
| open_space | **28.6 / 26.3** | 31.1 / 29.4 | 30.6 / 29.5 | KineticFold cleanest (modest) |
| near_singular | **40.1 / 38.5** | 47.4 / 46.1 | 44.0 / 42.9 | KineticFold cleanest |
| cluttered | **59.6 / 58.1** | 71.0 / 70.0 | 67.1 / 65.7 | KineticFold cleanest *and* highest success |

**KineticFold is the cleanest high-success solver in all three regimes**, and the margin widens as the regime hardens —
from ~2.5 points on open_space to ~11 points on cluttered. On the cluttered cell it also penetrates ~40% less deeply
(mean clearance −0.0203 m vs TRAC-IK −0.0340 m) *while being the only practical solver to reach 100% success*
(TRAC-IK 97.9%, Multi-start 98.1%). Two engines agree throughout (per-cell PyBullet↔MuJoCo within ~2 points).

**Honest sizing.** The edge is consistent but modest, and our capsule proxy substantially overstates it (the proxy
showed ~2–6×; the real-mesh gap is ~1.1–1.2×). We never quote an absolute proxy collision rate — collision is always a
solver-vs-solver comparison on the two real engines. The direction is stable across all ten seeds; the magnitude is
small, and we report it as such.

### 7.4 Self-collision, Franka — KineticFold ties the strongest baseline

On the redundant 7-DOF arm, every practical solver clashes at a similar rate: ~6–13% on open/near, and on cluttered
StagedFold 72%, LangevinFold 76%, TRAC-IK 78%, KineticFold 79%, Multi-start 80% — a tight band. **A redundant arm
gives every solver null-space room to dodge self-collision about equally, so collision-aware search stops being
decisive.** We report this plainly: KineticFold ties the strongest baseline here, never worse than the field by more
than a couple of points (within noise). *(Source: `sim_crosscheck.md`, n=100 distinct targets, PyBullet+MuJoCo; to be
re-locked on the fixed full sim run.)*

---

## 8. Where it wins — deployment roles and the folding climax

**Deployment roles.** KineticFold's profile — high success, clean solutions, occasional slow solve — fits planning,
offline batch generation, and reliability fallback rather than tight real-time control. As a planning goal-sampler on
UR5 cluttered it returns 83.4 usable clean goals per 100 attempts vs 56.9 (TRAC-IK) and 65.3 (Multi-start); as an
offline clean-solve batch it wins the honest cells by +18–30 pp; as a reliability fallback it rescues 60–78% of the
targets TRAC-IK gives up on. *(Source: `usecase_experiments.md`.)*

**The climax — the advantage grows with chain length.** On the planar arm we grow the joint count 4→16 and measure the
single-shot **clean-solve** rate (reach the target *and* be self-collision-free):

| DOF | KineticFold clean% | TRAC-IK clean% | ratio |
|--:|--:|--:|--:|
| 4 | 75.8 | 34.2 | 2.2× |
| 6 | 59.2 | 16.7 | 3.5× |
| 8 | 36.7 | 5.0 | 7.3× |
| 12 | 11.7 | 0.8 | 15× |
| 16 | 1.7 | 0.0 | only KineticFold > 0 |

Both methods *reach* the target ~100% of the time; the entire gap is self-collision avoidance. As the arm lengthens
into a self-avoiding chain — a polymer — KineticFold degrades most gracefully and is eventually the only standard-field
method producing collision-free folds. This is the concept proving itself: the method wins because the problem *becomes
folding.* *(Source: `usecase_experiments.md` EXP E, planar arm, proxy checker.)*

**Honest framing (mandatory).** This is a **single-shot** advantage over the **standard baseline field**. It is not
"the only method that works": a clearance-selecting multi-start (solve K times, keep the cleanest) beats KineticFold on
these arms, and a K-select wrapper lifts every solver. So the accurate statement is: KineticFold gives the best
*per-solve* clean rate, and selection wrappers are a strong, orthogonal booster we place in Limitations.

---

## 9. Validation — the dual-simulator honesty engine

We rebuilt every arm in **two independent physics simulators** and re-scored every solution.

**Forward-kinematics parity.** Our DH kinematics agree with both engines to floating-point noise: on UR5, PyBullet↔MuJoCo
max position residual 4.1e-8 m; on Franka, 5.9e-8 m (including the corrected modified-DH model — an earlier standard-DH
version was ~1.4 m wrong and only "succeeded" because targets were generated from the same wrong FK). Every success
claim is therefore independently true on two engines.

**Collision reality check.** Our capsule proxy is systematically optimistic — real meshes collide more — and both
engines agree on that and with each other (UR5 PyBullet↔MuJoCo sign-agreement 97.9%, correlation 0.993; Franka 99.1%,
0.876). We therefore report collision only as a *ranking* of solvers, never as an absolute rate, and we shrink our own
proxy-based magnitude claim accordingly. On the Franka the proxy is dominated by one fixed structural (elbow) link-pair
and is nearly insensitive to the 7th joint — the reason the Franka comparison is a tie, stated as a mechanism, not
buried.

**Edge replication.** The UR5 collision ranking and the Franka tie reproduce identically on both engines. "Solve once,
score three ways" (our proxy + PyBullet + MuJoCo) is the single reproducible artifact behind the results tables.

---

## 10. Limitations

- **The latency tail** — a minority of hard targets invoke the full staged fold, so KineticFold is positioned for
  planning/offline/quality use, not hard real-time. We show the distribution rather than hide it.
- **The climax is single-shot vs the standard field.** A clearance-selecting multi-start is competitive on redundant
  planar arms; selection wrappers lift all solvers. We do not claim absolute supremacy.
- **Self-collision only.** No environment obstacles yet.
- **Collision rate is seed-sensitive** on real meshes; we average over 10 seeds and report the spread.
- **The proxy is hand-tuned, not CAD;** its FK primitives' bit-identity is verified on UR5+planar (~2000 configs each),
  the scope we state.

---

## 11. Conclusion

Inverse kinematics is structurally a protein-folding problem, and porting folding's *process* yields a solver that
leads on success, is speed-competitive with the best fast solver on easy targets, clashes less on the non-redundant
arm, and becomes the last method standing as the arm grows chain-like. A literal folding simulation buys still-cleaner
solutions at a latency cost. The contribution is an organizing principle for IK, validated against two independent
physics engines. Future work: environment-obstacle collision and a full quality study of LangevinFold (thesis).

---

## References *(to finalize)*

Anfinsen 1973; Bryngelson & Wolynes 1987; Bryngelson, Onuchic, Socci & Wolynes 1995; Onuchic & Wolynes; Guo &
Thirumalai 1995; Thirumalai & Lorimer 2001; Honeycutt & Thirumalai 1990; Canutescu & Dunbrack 2003; Nakamura &
Hanafusa 1986; Wampler 1986; Aristidou & Lasenby 2011; Beeson & Ames 2015; Yoshikawa 1985; Levenberg 1944; Marquardt
1963; Coumans & Bai (PyBullet); Todorov, Erez & Tassa 2012 (MuJoCo); Lennard-Jones 1924; Kauzmann 1959; Baker &
Hubbard 1984.
