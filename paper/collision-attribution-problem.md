# The collision-attribution problem

**Status:** open, blocking submission
**Scope:** §1 (contribution claim), §3.3 (method description), §5.3 (mechanism), §5.7 (limitations), §6 (framing)
**Cost:** blocking control — ~10 min compute, 1–2 h harness, then 1 h to a full day of rewriting depending on outcome.
Widened to the full ablation this points to (§7): ~half a day of harness, under an hour of compute.
**Last verified:** 2026-08-12 against `master_10seed_fast(cpp).csv`, `solver.py`, `multi_start.py`, `genuine_solvers.py`

---

## TL;DR

The paper says KineticFold collides less because **Phase B's Metropolis funnel searches for collision-free
configurations**. That is the sentence which makes the folding correspondence look load-bearing.

The real mechanism is simpler and is not folding-specific: KineticFold **tries up to six times, refuses to
return a self-colliding answer, and finally returns the candidate with the largest clearance**. No baseline
does any of this — they optimise pose error only and return the first or best-by-pose answer.

So the comparison is not "folding schedule vs. restart schedule." It is **"a solver that optimises for
clearance vs. solvers that do not."** A reviewer will say: *of course it wins — it is the only one trying.*

The paper already knows. §5.7 names the exact missing control and does not run it.

**This is fixable, and the fixed version is still a real paper.** But it needs one experiment before the
wording can be settled.

And it points at something larger. The reason nobody caught this is that **the paper has no ablation study** —
`find . -iname "*ablation*"` returns nothing, and the four ablations it describes in prose all remove
components that were *rejected*, never components that *shipped*. Ten shipped components carry headline
numbers; zero have been isolated. §7 specifies the full sweep, of which the experiment above is Group A.

---

## 1. What the paper currently claims

Three sites, each a different strength of the same claim.

### 1.1 The contribution claim — §1, line 84

> "every ingredient is standard IK, so any advantage derives from **the sequencing** rather than from a new
> energy function."

This is the paper's core intellectual claim: nothing new was invented, only reordered. It is what makes
"folding as a compute schedule" a contribution rather than a metaphor.

### 1.2 The mechanism claim — §5.3, line 714

> "the mechanism traces to **Eq. (19)'s Metropolis funnel** and the collision term in Eq. (14): on frustrated
> targets, KineticFold's Phase-B search weights `E_collision` heavily (coefficient 2.0 in Eq. 14, against 3.0
> on the target term) and can escape shallow steric traps via thermal acceptance, whereas TRAC-IK's response
> to a stall is a full random restart with no collision-directed search."

This attributes the result specifically to **Phase B**, and specifically to the **thermal/Metropolis**
machinery — the most distinctively folding-flavoured component in the whole method.

### 1.3 The admission the paper already makes — §5.7

> "A clearance-selecting Multi-start (solve K times, keep the cleanest) is competitive on redundant planar
> arms, and such selection wrappers lift all solvers."

The paper names the confound, scopes it to §5.4 and the planar arm, and never applies it to §5.3 — where the
headline collision result lives.

---

## 2. What the code actually does

Three lines decide the returned configuration. All verified in both the Python reference and the C++ port.

### 2.1 Phase A refuses clashing answers

`backend/app/solvers/protein_fast/solver.py:427-440`

```python
for r in range(max_replicas):                       # up to 6 attempts
    seed = q0.copy() if r == 0 else spec.random_config(rng)
    q_lm, e_lm, conv, lm_steps = _lm_polish_fast(...)
    if conv:
        d = self_collision_min_distance(spec, q_lm)
        converged_candidates.append((d, q_lm.copy()))
        if d >= 0.0:
            break                                   # <-- stops ONLY if clash-free
```

A converged, on-target, self-**colliding** solution does not end the loop. The solver keeps going.

### 2.2 Both phases feed one clearance-ranked pool

`solver.py:475-478`

```python
if converged_candidates:
    _, q_best = max(converged_candidates, key=lambda c: c[0])   # c[0] is clearance d
    global_best_q = q_best
```

The returned answer is the **largest-clearance** candidate across everything both phases produced.

### 2.3 No baseline has either behaviour

| solver | selection rule | source |
|---|---|---|
| **KineticFold** | reject clashing; return max clearance | `solver.py:439`, `:476` |
| Multi-start | `min(pool, key=combined pose error)` | `multi_start.py:90` |
| TRAC-IK | `solve_type=Speed` — returns the **first** solution found | `genuine_solvers.py:289` |
| Jacobian-DLS, CCD, FABRIK | single trajectory, no selection at all | — |

**Not one baseline ever evaluates collision.** Clearance appears in exactly one solver's decision rule.

### 2.4 How often the rejection actually fires

From the Phase-A replay harness (`scratchpad/gate_rate.py`, n = 300/cell), mean replicas consumed:

| cell | mean replicas | reading |
|---|--:|---|
| UR5 `open_space` | **2.39** | ~1.4 converged answers rejected per solve, for clashing |
| UR5 `near_singular` | 3.10 | ~2.1 rejected |
| UR5 `cluttered` | 3.29 | ~2.3 rejected |
| Franka `open_space` | 2.82 | ~1.8 rejected |
| Franka `cluttered` | 4.30 | ~3.3 rejected |

On UR5 `open_space`, TRAC-IK returns its **first** answer. KineticFold returns, on average, its **2.4th** —
having thrown away the earlier ones *specifically because they collided*. That alone predicts a collision gap,
with no funnel, no temperature, and no energy function involved.

---

## 3. The evidence

There are two independent arguments. The first is decisive everywhere; the second is decisive on one cell and
inconclusive elsewhere. **Both must be stated honestly — overselling the second is how this becomes a worse
problem than it already is.**

### 3.1 Argument A — the objective mismatch (decisive, universal)

KineticFold's returned answer is chosen to maximise clearance. Every baseline's is chosen to minimise pose
error. The paper then ranks them **on clearance**.

This does not require any arithmetic. It is a design fact, visible in four source files, and it holds in every
cell of every arm. It means §5.3's comparison does not isolate the schedule — it isolates the objective.

### 3.2 Argument B — the arithmetic bound (decisive on UR5 `open_space` only)

If Phase B is the mechanism, then removing Phase B should remove the advantage. We can bound Phase A's
standalone performance without running anything.

Let `f` be the fast-path fraction, `a` Phase A's collision rate on it, `b` Phase B's rate on the rest:

```
KF_overall = f·a + (1−f)·b        with b ≥ 0    ⟹   a ≤ KF_overall / f
TRAC_overall = f·t₁ + (1−f)·t₂    with t₂ ≤ 100 ⟹   t₁ ≥ (TRAC_overall − 100(1−f)) / f
```

Phase A provably beats TRAC-IK on the identical subset when `TRAC_overall − KF_overall > 100(1−f)`.

| cell | fast-path % | KF coll% | TRAC coll% | gap | threshold 100(1−f) | verdict |
|---|--:|--:|--:|--:|--:|---|
| **UR5 `open_space`** | **93.0** | **26.2** | **35.3** | **9.1** | **7.0** | **PROVEN** |
| UR5 `near_singular` | 79.7 | 40.4 | 49.9 | 9.5 | 20.3 | inconclusive |
| UR5 `cluttered` | 76.3 | 56.4 | 74.2 | 17.8 | 23.7 | inconclusive |
| Franka `open_space` | 87.7 | 7.3 | 7.8 | 0.5 | 12.3 | inconclusive |
| Franka `near_singular` | 88.3 | 11.2 | 9.1 | −2.1 | 11.7 | inconclusive |
| Franka `cluttered` | 50.0 | 82.4 | 77.1 | −5.3 | 50.0 | inconclusive |

On UR5 `open_space`:

```
Phase A's own collision rate       a  ≤ 28.2%
TRAC-IK on the identical subset    t₁ ≥ 30.4%
```

**Phase A alone is cleaner than TRAC-IK, on the 93% of targets Eq. (19) never touches.** Phase B cannot be
what produces the margin there.

### 3.3 What Argument B does *not* show

It is a worst-case bound, so "inconclusive" means exactly that — not refuted, not confirmed. On the five other
cells, Phase B may well contribute materially. The bound is a **counterexample to a general mechanism claim**,
not a proof that Phase B never matters.

§5.3 says KineticFold "has the lowest real-mesh collision … on all three scenarios and both engines" and
offers one mechanism for all of it. One cell where that mechanism provably cannot be the cause is enough to
break the attribution as stated. It is not enough to claim Phase B is useless.

---

## 4. Why this is dangerous

### 4.1 The reviewer's path is short

1. §5.3 claims a collision advantage and attributes it to Phase B.
2. §3.3 says Phase B fires on a minority of targets — the paper states 79% take the fast path.
3. The reviewer asks the obvious question: *what is the fast path doing that TRAC-IK is not?*
4. The repository is public. `solver.py:439` and `:476` answer it in ten seconds.
5. §5.7 confirms the reviewer's suspicion **in the paper's own words**.

### 4.2 It undercuts the contribution, not just one paragraph

The paper's thesis is that **reordering standard machinery** buys the result. If the result comes from adding
a clearance criterion, then the contribution is "add a collision term and select on it" — which is neither
new nor folding-related, and which §2 does not even review (see the separate finding on missing
collision-aware IK literature).

### 4.3 Half-seeing it is worse than not seeing it

Naming the confound in §5.7, scoping it to a section where it is less damaging, and omitting it where the
headline sits reads as deliberate. It almost certainly is not — it is what happens when sections are written
weeks apart. But that is not how it will be read.

### 4.4 It has a precedent inside this project

The same attribution error was already found and corrected for LangevinFold: its low collision rate came from
multi-start plus a clash-free filter, not from the folding physics it claimed. That is the identical mechanism
now driving KineticFold's number. The correction was applied to one solver and not the other.

---

## 5. The three possible worlds

The experiment in §6 distinguishes these. **All three leave you with a publishable paper.** Only the wording
changes.

### World 1 — the schedule matters (best case)

A clearance-selecting Multi-start closes only part of the UR5 gap; KineticFold keeps a clear margin.

**Claim becomes:** the clearance criterion is necessary but not sufficient; the barrierless-first schedule is
what makes it *affordable* (six cheap polishes instead of six full solves) and what finds clean solutions the
restart wrapper cannot. Both the speed and the collision result stand, with an honest decomposition.

### World 2 — the wrapper explains most of it (most likely)

Multi-start + clearance selection recovers most of KineticFold's UR5 advantage.

**Claim becomes:** clearance selection is the mechanism; KineticFold's contribution is that its schedule makes
that selection nearly free — sub-millisecond, against a wrapper that costs K full solves. Plus the Franka
success lead and the latency, which are untouched by any of this.

This is a **narrower but fully defensible** paper, and it comes with a genuinely interesting number: the price
of clean solutions.

### World 3 — Phase B is doing nothing measurable

KineticFold with Phase B disabled matches full KineticFold everywhere.

**Claim becomes:** the barrierless phase plus clearance selection is the whole method; Phase B is insurance
for the frustrated tail. Report where it earns its cost (Franka `cluttered`, planar) and where it does not.

Even here you keep: the success lead on Franka, the latency result, the DOF scaling result (monotone 2.0→10.4×
after the n=1000/n=5000 re-run), and the dual-engine validation.

---

## 6. The experiment

### 6.1 Design — a 2×2 on schedule × selection

The confound is that KineticFold varies **two** things at once versus the baselines. Separate them:

|  | **no clearance selection** | **clearance selection** |
|---|---|---|
| **single-shot** | TRAC-IK, Jacobian-DLS *(have)* | — |
| **multi-restart** | Multi-start *(have)* | **`multi_start_clear`** ← new |
| **KineticFold schedule** | **`protein_fast_noselect`** ← new | KineticFold *(have)* |

Plus one decomposition run:

- **`protein_fast_nob`** — KineticFold with Phase B disabled, to split Phase A's contribution from Phase B's.

**Three new solver configurations. Everything else already exists in the benchmark.**

### 6.2 Implementation

All three are thin wrappers. **Do not modify the shipped solvers** — that invalidates every committed number.

**`multi_start_clear`** — copy `app/solvers/multi_start.py`, change the selection line only:

```python
# multi_start.py:90 currently
best = min(pool, key=lambda r: r[0])                    # r[0] = combined pose error

# the variant
conv = [r for r in pool if r[2]]                        # r[2] = success flag
if conv:
    best = max(conv, key=lambda r: self_collision_min_distance(spec, r[1]))
else:
    best = min(pool, key=lambda r: r[0])
```

Set `population_size = 6` to match `max_replicas = 6`. This is the honest head-to-head: same restart budget,
same selection rule, different search core.

**`protein_fast_noselect`** — copy `solve_protein_fast`, make two changes:

- line 439: `if d >= 0.0: break` → `break` (accept the first converged answer regardless of clearance)
- line 476: `max(converged_candidates, key=lambda c: c[0])` → take the first converged candidate

**`protein_fast_nob`** — copy `solve_protein_fast`, force the gate closed: `if False:` at line 448.

Register all three in the bench solver registry (`get_solvers_for_robot` / `SOLVER_DISPLAY_NAMES`, imported at
`bench/master_sim_benchmark.py:86`).

### 6.3 Run

Same protocol as the committed collision sweep, so the numbers are directly comparable:

```bash
# in WSL, from backend/  (see memory: native-bench WSL workflow)
python bench/master_sim_benchmark.py \
    --robots ur5 franka_panda planar3dof \
    --seeds 1 2 3 4 5 6 7 8 9 10 \
    --trials 100 \
    --solvers protein_fast trac_ik_style multi_start \
              multi_start_clear protein_fast_noselect protein_fast_nob \
    --out results/collision_ablation
```

`n = 1000`/cell, matching `master_10seed_fast(cpp).csv`. Expected runtime is minutes — the full DOF sweep at
this scale took ~6 minutes.

**Note:** the C++ port will not have these variants. Run the ablation in Python across *all* arms of the
comparison, so it is internally consistent; do not mix a Python variant against a C++ baseline. The provenance
note in both benchmark files states success and collision are statistically identical between the two.

### 6.4 What to record

Per cell, per solver: `success_pct`, `pb_collision_pct`, `mj_collision_pct`, `pb_mean_clearance_m`, `mean_ms`.

Then compute two decompositions on UR5 (where the advantage lives):

```
selection effect  =  collision(multi_start) − collision(multi_start_clear)
schedule effect   =  collision(multi_start_clear) − collision(protein_fast)
phase B effect    =  collision(protein_fast_nob) − collision(protein_fast)
```

### 6.5 Reading the result

| observation | world | what §5.3 becomes |
|---|---|---|
| schedule effect large (≳5 pts) | 1 | keep a schedule claim, add the selection decomposition |
| selection effect large, schedule effect small (≲2 pts) | 2 | selection is the mechanism; schedule makes it cheap |
| phase B effect ≈ 0 everywhere | 3 | Phase A + selection is the method; Phase B is tail insurance |

---

## 7. Why this must become a full ablation study

The 2×2 in §6 is the **minimum** control — enough to settle the collision attribution and unblock submission.
It is not enough to defend the paper's actual contribution. This section argues for widening it, and specifies
what to.

### 7.1 The paper has no ablation study

`find . -iname "*ablation*"` returns nothing. There is no committed, reproducible ablation experiment in the
project. What the paper calls ablations are four prose claims sourced from design notes:

| site | claim | source | n / seeds / CI |
|---|---|---|---|
| §3.2 (~366) | Stage 1 neutral-pose anchor → neighbour-coupling: ≈4 pts cluttered success | research notes, `docs/METHODOLOGY.md` | none stated |
| §3.2 (~368) | rotamer-library prior: better clearance, −14 to −23 pts success | research notes | none stated |
| §3.2 (~369) | allostery-inspired compensating step: −1 pt success, small clearance gain | research notes | none stated |
| §3.3.2 (~460) | `cap_replicas = 2` → Franka open-space success 71.7% | `docs/design/kineticfold-barrierless-first.md` | none stated |

The last one additionally compares across runs: its own contemporaneous baseline was **96.7%**, but the paper
sets it against "the ≈100% the default schedule reaches (Section 5.1)" — a different sweep, different `n`,
different language, ~3000× different mean latency.

### 7.2 All four ablate things that were *rejected*, not things that *shipped*

This is the structural problem, and it is worse than the missing artifacts.

- rotamer prior — **rejected** (cost 14–23 points)
- allostery step — **rejected** (traded a point of success)
- Stage 1 anchor swap — a **losing variant**
- `cap_replicas = 2` — a **degradation**, not a component removal

Every one documents the search history: *here are things we tried that did not work.* None isolates the
contribution: *here is what each shipped component contributes.*

That is the difference between a lab notebook and an ablation. A reviewer will name it.

### 7.3 Ten shipped components, zero ablated

| # | component | where | plausibly buys | ablated? |
|---|---|---|---|---|
| 1 | Phase A's clash-free accept | `solver.py:439` | **the collision result** | no |
| 2 | max-clearance select across both phases | `solver.py:476` | **the collision result** | no |
| 3 | Phase B at all | `solver.py:448-473` | success on frustrated targets | no |
| 4 | the frustration gate | `solver.py:448` | **the latency result** | no¹ |
| 5 | Metropolis funnel vs. greedy (★1) | Eq. 19 vs. Eq. 15 | **the folding claim itself** | no |
| 6 | analytic rescue vs. finite-difference (★2) | Eq. 21 vs. Eq. 17 | cost per rescue | no |
| 7 | adaptive LM damping | Eq. 18 | Phase A quality and speed | no |
| 8 | contact-order factor `δ` | §3.3.1 | coarse-collapse length | no |
| 9 | stability gate | `solver.py:481` | **the reported success rate** | no |
| 10 | `max_replicas = 6` | `:427`, `:454` | the speed/quality trade | partial¹ |

¹ `cap_replicas = 2` degrades the budget; it does not remove the gate or sweep the budget.

Four of these — 1, 2, 5, 9 — sit directly under headline numbers.

### 7.4 Why this is now urgent, not nice-to-have

**(a) The contribution *is* the composition.** §1 says every ingredient is standard IK and the novelty is the
arrangement. A paper whose contribution is an arrangement must show each element's share, or the claim is not
merely unproven — it is untestable.

**(b) The collision attribution is a symptom, not the disease.** It arose because nobody measured which
component produced which number. The same gap exists elsewhere and nobody has looked: is the latency win the
*gate*, or just that Phase A is cheap and would be cheap without a gate? Is the Franka success lead the
*schedule*, or six restarts? Component 9 alone can move a reported success rate and has never been isolated.

**(c) Reviewers generalise from one catch.** Someone who finds one unmeasured attribution assumes the rest are
unmeasured too — correctly, in this case. One table removes the entire class of objection at once.

**(d) Component 5 is the thesis.** The Metropolis funnel is the most distinctively folding-flavoured thing in
the method. If a greedy variant matches it, the folding machinery is decorative. **That already happened once
in this project** — LangevinFold's advantage turned out to be multi-start plus a clash-free filter, not
physics. Finding out from a reviewer instead of from your own sweep is the bad ending.

### 7.5 The full design

Same harness, same protocol, one variant per row. Group A is blocking; B and C are pre-submission.

**Group A — the collision mechanism** *(the §6 experiment; settles the blocking question)*

| variant | change | isolates |
|---|---|---|
| `protein_fast_noselect` | accept first converged; drop max-clearance | components 1 + 2 |
| `protein_fast_nob` | force the gate closed | component 3 |
| `multi_start_clear` | Multi-start + clearance selection, K = 6 | selection alone, on a different core |

**Group B — the schedule**

| variant | change | isolates |
|---|---|---|
| `protein_fast_nogate` | always run Phase B | component 4 — what the gate actually saves |
| `protein_fast_r{1,3,6,12}` | sweep `max_replicas` | component 10 — the budget curve, replacing one degraded point |

**Group C — the folding substitutions** *(the thesis)*

| variant | change | isolates |
|---|---|---|
| `protein_fast_greedy` | Phase B Stage 3 uses greedy Eq. 15, no Metropolis | **component 5 — is the funnel doing anything?** |
| `protein_fast_fdrescue` | Eq. 17 finite-difference instead of Eq. 21 | component 6 — cost only |
| `protein_fast_nodelta` | `δ = 1`, fixed coarse-collapse length | component 8 — the contact-order port |
| `protein_fast_nostab` | no stability gate | component 9 — how much reported success it removes |

Group C is the only group that tests folding-specific machinery. It is also the only group that can *hurt* —
which is exactly why it should be run privately, before a reviewer runs it publicly.

### 7.6 Cost

| | configs | build | run |
|---|--:|---|---|
| Group A | 3 new | 1–2 h | ~10 min |
| Group B | 5 new | ~2 h | ~20 min |
| Group C | 4 new | ~3 h | ~15 min |
| **total** | **12 new** | **~half a day** | **under an hour** |

Every variant is a copied function with a one-to-three-line change. Compute is not the constraint; wrapper
writing is. Doing all twelve in one sitting costs roughly twice Group A alone, because the harness, the
registry plumbing and the run script are shared.

### 7.7 What it becomes in the paper

One table — one row per component, columns for success on the hardest cell, real-mesh collision, mean latency,
and the delta from full KineticFold. It replaces §3.2's three-sentence prose list and §3.3.2's `cap_replicas`
paragraph with a single committed artifact, and it gives §1's "the novelty is the arrangement" claim something
to stand on.

That table is also the most effective available answer to *"this is multi-start with extra steps"* — the
objection this entire document is about. Right now the paper has no reply to it. With the table, the reply is
a number per component.

---

## 8. The rewrite

### 8.1 Required regardless of outcome

**§3.3 — state the merge in the body.** The clearance-selecting merge currently exists **only** in Figure 1's
caption and nowhere in the text. This is the single most important missing sentence in the paper: it is the
step that produces the headline result. Add at the close of §3.3.1:

> Both phases feed one pool of converged candidates; the returned configuration is the one with the largest
> self-clearance `d(q)`, and it must clear the stability gate of Section 3.2.5.

Cheap now, and every other edit below depends on it existing.

**§5.7 — promote the caveat.** Move the clearance-selecting-Multi-start sentence from a §5.4-scoped remark to
one that governs §5.3 as well, and replace "is competitive on redundant planar arms" with whatever the
experiment measures.

**§1 line 84 — drop the causal clause.** "so any advantage derives from the sequencing" claims more than any
outcome supports. Replace with a statement of fact:

> every numerical ingredient is standard IK: what is new is the order and the schedule, not the energy function.

### 8.2 §5.3's mechanism paragraph — the version for World 1 or 2

Replace line 714's single-mechanism sentence with a decomposition by fraction:

> Two mechanisms produce it. Phase A stops only on a converged replica that is clash-free, and both phases feed
> a single largest-clearance select (Section 3.3.1), so every target — including the 93% that never leave the
> fast path on `open_space` — receives a collision-aware choice among its own converged candidates. On the
> frustrated fraction that escalates (7.0% of UR5 `open_space`, 23.7% of `cluttered`), Eq. (19)'s Metropolis
> funnel and Eq. (14)'s collision term add a collision-directed search. TRAC-IK's response to a stall is a
> random reseed with neither.

Then one sentence carrying the ablation result, e.g.:

> A Multi-start given the identical restart budget and the identical clearance selection reaches X%, leaving
> Y points to the schedule.

### 8.3 If World 3

Restructure §3.3 so Phase B is presented as tail insurance rather than the method's core, and move the
Metropolis-funnel discussion out of §5.3's mechanism paragraph into §5.4, where the long-chain results are the
place Phase B demonstrably earns its cost.

### 8.4 §6 — remove the overclaim

Delete "rather than implementation luck" from the conclusion. Whatever the ablation returns, that phrase claims
a level of mechanism isolation the paper does not have.

---

## 9. What is not at risk

Worth keeping in view, because the finding list is long and this problem is narrow:

- **The success results.** Franka lead (98.3% vs 94.7%/93.7%), the UR5/planar tie, the two-tier structure —
  none depends on collision or on the selection rule.
- **The latency results.** Fastest on both physical arms, mean 0.1–0.7 ms, worst p99 4.7 ms. Untouched.
- **The DOF-scaling result.** Now stronger after the n=1000/n=5000 re-run: monotone 2.0→10.4× vs TRAC-IK,
  1.7→4.5× vs Multi-start, every cell significant.
- **The validation harness.** Dual-engine rescoring, sub-micron FK agreement, native parity. Above venue median.
- **~150 numbers**, all independently re-derived and reconciled.
- **All ~45 algorithm constants in §3**, matching both implementations exactly.

The problem is one paragraph's causal claim and the contribution sentence that rests on it. It is not the
paper's data, and it is not the paper's engineering.

---

## 10. Checklist

**Before the experiment (30 min, do now — these are needed either way)**

- [ ] Add the clearance-merge sentence to §3.3.1's body (§8.1)
- [ ] Drop "so any advantage derives from the sequencing" at line 84 (§8.1)
- [ ] Delete "rather than implementation luck" from §6 (§8.4)

**Build (1–2 h)**

- [ ] `multi_start_clear` — copy `multi_start.py`, swap the selection rule, `population_size = 6`
- [ ] `protein_fast_noselect` — copy the solver, remove the clash-free break and the max-clearance select
- [ ] `protein_fast_nob` — copy the solver, force the gate closed
- [ ] Register all three; confirm none of the shipped solvers changed (`git diff` on `app/solvers/`)

**Run (~10 min)**

- [ ] 10 seeds × 100 trials, three arms, six solvers → `results/collision_ablation`
- [ ] Commit the CSV, MD and manifest

**Read (30 min)**

- [ ] Compute the three effects in §6.4
- [ ] Determine which world you are in (§6.5)

**Rewrite (1 h in World 1/2, up to a day in World 3)**

- [ ] §5.3 mechanism paragraph, with the measured decomposition
- [ ] §5.7 caveat promoted from §5.4-scope to results-wide
- [ ] §1 and §6 framing aligned to whatever §5.3 now says
- [ ] Re-run `scratchpad/sweep56.py` to confirm no number or cross-reference broke

**Widen to the full ablation — before submission, not before the rewrite (§7)**

- [ ] Group B: `protein_fast_nogate`, `protein_fast_r{1,3,6,12}` — what the gate saves, and the budget curve
- [ ] Group C: `protein_fast_greedy`, `_fdrescue`, `_nodelta`, `_nostab` — the folding substitutions
- [ ] **Run `protein_fast_greedy` first of Group C.** It tests whether the Metropolis funnel does anything.
      If it comes back flat, that changes the paper, and you want to know before a reviewer does.
- [ ] Emit a `phase_b_fired_pct` column so §3.3's gate rates get a committed source (see Appendix)
- [ ] Replace §3.2's three-sentence prose ablation list and §3.3.2's `cap_replicas` paragraph with the table
- [ ] Fix the cross-run comparison at §3.3.2: `cap_replicas = 2` is 71.7% against **96.7%** in its own sweep,
      not against §5.1's ≈100% from a different one

---

## Appendix — measured values used above

**Real-mesh collision, PyBullet, `master_10seed_fast(cpp).csv`, n = 1000/cell**

| cell | KineticFold | TRAC-IK | Multi-start |
|---|--:|--:|--:|
| UR5 `open_space` | 26.2 | 35.3 | 35.8 |
| UR5 `near_singular` | 40.4 | 49.9 | 47.6 |
| UR5 `cluttered` | 56.4 | 74.2 | 74.7 |
| Franka `open_space` | 7.3 | 7.8 | 6.1 |
| Franka `near_singular` | 11.2 | 9.1 | 9.6 |
| Franka `cluttered` | 82.4 | 77.1 | 77.0 |

**Phase-A/Phase-B split** (`scratchpad/gate_rate.py`, Phase-A replay, n = 300/cell) — escalation %, and mean
replicas consumed:

| cell | escalate % | mean replicas |
|---|--:|--:|
| UR5 `open_space` | 7.0 | 2.39 |
| UR5 `near_singular` | 20.3 | 3.10 |
| UR5 `cluttered` | 23.7 | 3.29 |
| Franka `open_space` | 12.3 | 2.82 |
| Franka `near_singular` | 11.7 | 2.78 |
| Franka `cluttered` | 50.0 | 4.30 |
| planar `open_space` | 9.3 | 2.54 |
| planar `near_singular` | 53.0 | 4.18 |
| planar `cluttered` | 63.7 | 4.85 |

These escalation rates are themselves **uncommitted** — they exist only in this harness and in
`figures/fig_pipeline_SPEC.md`, while the paper quotes them at line 385. Emitting a `phase_b_fired_pct`
column during the ablation run closes that gap at zero extra cost, and gives §3.3 a citable source.
