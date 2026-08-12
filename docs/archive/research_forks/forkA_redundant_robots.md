# Fork A — Redundant-Robot Arena

**Question:** Does the ProteinIK physics (V6 "Raw" folding energy; V5 conflict-gated homotopy)
do anything *useful* on a **genuinely** redundant robot — unlike the Franka Panda (7-DOF),
whose self-collision we proved is structurally pinned (its 1 spare DOF gives ~0.000 null-space
clearance headroom)?

**Method.** Planar N-link RRR arms defined inline as `RobotSpec` (task is 3-DOF → redundancy
= n−3, so a 20-link arm has **17 redundant DOF**). Mirrored the Franka null-space diagnostic:
for a fixed target, gather many distinct IK solutions and measure (a) joint-space diversity,
(b) `min_self` spread, (c) projected null-space gradient-ascent clearance gain. Then Raw vs V4
vs plain multi-start+clash-free selection on collision-stressing targets. All seeded.

---

## Result 1 — Headroom vs DOF: it **SHRINKS**, it does not grow

10 targets/DOF, 30 solutions/target, reach=1.0, radius=0.02:

| n | redund | joint-div (rad) | **min_self spread** | best-of-30 clearance | **null-space climb gain (median)** |
|---|--------|-----------------|---------------------|----------------------|-------------------------------------|
| 6  | 3  | 6.54  | **0.1127** | +0.073 | **+0.0122** |
| 9  | 6  | 7.93  | **0.0567** | +0.017 | **+0.0000** |
| 12 | 9  | 9.13  | **0.0225** | −0.018 | **+0.0000** |
| 20 | 17 | 11.51 | **0.0033** | −0.037 | **+0.0000** |

Joint-space diversity **grows** (solutions are genuinely distinct), but the clearance spread
**collapses** and null-space climb gain is **+0.000 for all n ≥ 9**. By n=20 even the *best* of
30 diverse solutions still clashes (−0.037 ≈ links crossing).

**Confound ruled out** (re-run under constant per-link geometry and under thin links r=0.008):
all three geometries agree — headroom shrinks monotonically with redundancy; clash-free-solution
fraction goes 22% → 9% → 3% → **0%** as n goes 6 → 20.

**Why:** a planar chain is **space-filling**. Adding links adds *joint-space* redundancy, not
*clearance* redundancy — the clash-free fraction of configuration space collapses combinatorially
as the chain lengthens, and the null-space clearance gradient is too rugged for projected ascent
to move. This is a *different* pathology than Franka (too few spare DOF), reaching the **same
null** from the opposite end.

---

## Result 2 — Raw vs baselines: the physics does **NOT** win

Collision-stressing curled/compact targets. `ms_cf` = plain multi-start (DLS from 40 seeds, keep
converged, pick most clash-free). `win%` = among jointly-solved targets, fraction with strictly
higher `min_self`.

| n | solver | succ% | clash-free% | mean min_self | win% vs ms_cf |
|---|--------|-------|-------------|---------------|---------------|
| 6  | raw       | 100 | 76  | +0.0349 | 28 |
|    | fast (V4) | 100 | 84  | +0.0375 | 20 |
|    | **ms_cf** | 100 | **100** | **+0.0635** | — |
| 9  | raw       | 100 | 45  | −0.0053 | 35 |
|    | fast (V4) | 100 | 25  | −0.0201 | 10 |
|    | **ms_cf** | 100 | **60**  | **+0.0106** | — |
| 12 | raw       | 100 | 7   | −0.0247 | 47 |
|    | fast (V4) | 100 | 0   | −0.0367 | 27 |
|    | **ms_cf** | 100 | **13**  | **−0.0220** | — |
| 20 | raw / fast / ms_cf | 100 | 0 | ≈ −0.04 | infeasible regime |

Two clean results:
1. **Raw beats its own sibling V4/Fast** on clearance (win 52/60/67% at n=6/9/12) — collision-aware
   native-state selection is real *within the family*.
2. **Raw loses to plain multi-start+clash-free selection** at every DOF — `ms_cf` has the highest
   clash-free rate and mean clearance everywhere.

**Fairness check** (matched restart budget K = 10+2(n−6) to Raw's warm-start count): at **equal
restart count** the physics and dumb restart-sampling are a **statistical wash** (Raw marginally
better mean, same clash-free rate) — but Raw pays **10–100× the wall-time** (Raw ≈ 2.0 s at n=6,
**19.6 s at n=12, 129.9 s at n=20** per solve; baselines sub-second). Spend the same wall-time on
more restarts (K=40, still far cheaper than one Raw solve) and the trivial baseline **decisively**
wins.

---

## Verdict: **NULL**

There is **no measured advantage** for the ProteinIK folding physics on genuinely redundant planar
arms. (1) The premise fails — clash-free headroom *shrinks* with redundancy. (2) Where headroom
exists (low n), the physics doesn't exploit it better than brute force; at matched restart budget
it's a wash, at matched wall-time it loses. Raw's only genuine win is intramural (over V4).

> **Bottom line: the folding physics is a 10–100× more expensive way to lose to a five-line
> multi-start loop, on exactly the redundant robots where it was supposed to shine.**

Combined with the earlier Franka finding, **both ends of the redundancy spectrum give a null**:
too-few spare DOF (Franka) *and* space-filling high-DOF (planar-20) both lack exploitable
clash-free headroom.

---

## What would need to be true instead (if ever revisited)

- **A robot whose redundancy is *clearance* redundancy, not space-filling** — a 3D arm in an
  obstacle-cluttered workspace where the null-space demonstrably moves the elbow through free
  space. Gate it with the headroom test first (Result 1): if median-solution null-space climb gain
  is ~0, stop — there is nothing to win.
- **A metric the physics beats restart-sampling at.** Multi-start+clash-free selection is the true
  baseline (not V4), and it is embarrassingly strong and cheap. Beating it needs configs that dense
  random restarts *miss* — e.g. narrow clash-free corridors requiring correlated multi-joint moves
  i.i.d. seeding can't hit. No evidence of that here.
- **A win at equal wall-time, not equal restart count.** Given the 10–100× slowdown, "equal budget"
  gives the baseline ~40–4000× more restarts. Not close.

_Scripts: `scratchpad/forkA/{a0_sanity, a1_headroom, a1b_headroom_geom, a2_compare, a3_fairness}.py`
(seeded, reproducible)._
