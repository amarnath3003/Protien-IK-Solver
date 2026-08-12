# Sim-Oracle Findings (Phase 2) — what PyBullet says about the solvers

Independent re-scoring of every solver's `q_final` inside PyBullet (real FK + real
mesh self-collision), on the **same** target distributions as `master_benchmark.py`.
Raw tables: [`sim_oracle_ur5.md`](sim_oracle_ur5.md). Runner: `bench/sim_benchmark.py`;
oracle: `app/sim/pybullet_backend.py`. UR5, n=150/cell (50 trials × seeds 1,2,3).

> Scope note: UR5 first (the arm Phase-1 validated cleanest). Franka results are
> generated separately. `planar3dof` has no standard URDF (validated analytically).

---

## 1. The corrected kinematics are real — FK parity holds end to end

`our_succ` and `sim_succ` **agree on 100% of solves** across all solvers and
scenarios (one single V4 near-singular solve sat exactly on the tolerance
boundary: 100.0 vs 99.3). Our self-reported position error equals the simulator's
to three decimals (`our_pos mm` == `sim_pos mm` in every row). An independent,
widely-trusted simulator confirms: when we call a UR5 solve successful, it *is*
successful on the real model. The Phase-1 parity result now holds through the full
benchmark pipeline, not just in isolation.

**This is the load-bearing validation for every UR5 success-rate claim in the paper.**

---

## 2. The capsule collision proxy is systematically optimistic

Real mesh collision (`sim_col`) is **higher than our capsule proxy (`our_col`) in
every single cell** — often 2–3×. Examples (UR5):

| Scenario | Solver | our_col% (proxy) | sim_col% (real) |
|---|---|--:|--:|
| open_space | ProteinIK Fast (V4) | 2.7 | 29.3 |
| open_space | TRAC-IK | 17.3 | 32.7 |
| cluttered | ProteinIK Fast (V4) | 17.3 | 56.0 |
| cluttered | TRAC-IK | 44.0 | 64.7 |

The capsule segments-with-radii model misses real link-mesh overlaps (the UR5's
wrist cluster is geometrically tight). **Absolute collision rates from the proxy
cannot be trusted** — the real numbers are far higher. Any paper sentence quoting a
proxy collision *rate* must be re-stated against the sim.

---

## 3. …but V4's collision *edge* over the baselines is real (just smaller)

The paper's headline is not the absolute rate — it's whether ProteinIK's
collision-aware search actually clashes *less than the alternatives*. Under real
meshes, it does. Among the solvers that actually succeed (>90%), **V4 has the
lowest real-collision rate of any *practical* solver in all three scenarios:**

| Scenario | V4 sim_col% | TRAC sim_col% | Multi sim_col% | native-IK sim_col% |
|---|--:|--:|--:|--:|
| open_space | **29.3** | 32.7 | 34.7 | 42.0 |
| near_singular | **39.3** | 48.7 | 44.7 | 54.0 |
| cluttered | **56.0** | 64.7 | 65.3 | 71.3 |

And on mean clearance V4 stays clearer (e.g. cluttered: V4 −0.0165 m vs TRAC
−0.0302 m — it penetrates about half as deep).

**Honesty correction the sim forces:** the *magnitude* of V4's edge was inflated by
the proxy. On the proxy V4 looked ~2.5–6× cleaner than TRAC (open_space 2.7 vs
17.3); on real meshes it's ~1.1–1.25× (open_space 29.3 vs 32.7). The direction of
the claim survives; the size of it must be cut substantially. PyBullet's own IK —
which has *no* collision awareness — is the worst on collision in every scenario,
which is the clean reference the edge is measured against.

---

## 4. New result the proxy hid: V6 (raw biology) is the real-collision champion

On the capsule proxy, V4 and V6 were indistinguishable on collision (open_space
2.7 vs 2.7; cluttered 17.3 vs 16.0). **Under real meshes V6 pulls clearly ahead of
everything** while keeping ~99–100% success:

| Scenario | V6 sim_col% | V4 sim_col% | best baseline sim_col% |
|---|--:|--:|--:|
| open_space | **14.0** | 29.3 | 32.7 (TRAC) |
| near_singular | **26.0** | 39.3 | 44.7 (Multi) |
| cluttered | **48.7** | 56.0 | 64.7 (TRAC) |

The biophysically-faithful energy terms (V6 "raw biology") produce meaningfully
better real-collision avoidance than V4 — a difference the approximate proxy was
blind to. This **partially reopens the V6 "biophysics → better solution quality"
thesis** the technical report was ready to close: on a real collision oracle, V6 is
the quality leader. Its cost is prohibitive latency (~3–6 s/solve vs V4's ~20 ms),
so it stays a "quality-at-any-cost / offline" tool — but the quality claim is now
*measured*, not just argued.

---

## 5. What this means for the deliverables

- **Paper (V1 + V4):** keep the collision-edge claim, but re-state it on sim
  numbers and **shrink the magnitude** — V4 is the cleanest *fast, high-success*
  solver under real meshes (beats TRAC-IK / Multi-start / native-IK on collision in
  every UR5 scenario), not the near-collision-free result the proxy implied.
- **Technical report (V5/V6):** V6 gets a genuine positive to report — best
  real-collision avoidance on UR5 — reframing it from "biophysics didn't help" to
  "biophysics helped *quality* but not *speed*, and only a real collision oracle
  could see it."
- **Phase 3 is essentially answered for UR5:** the collision edge is real, the
  proxy exaggerates it, and V6 > V4 > baselines on real collision. Remaining work:
  confirm the same ordering on Franka (the redundant arm) and cross-check with a
  second simulator (MuJoCo, Phase 4).

---

## 6. Franka (7-DOF, redundant) — the collision edge does NOT replicate

Same oracle, on the corrected modified-DH Panda (n=150/cell for the fast solvers;
raw table [`sim_oracle_franka_fast.md`](sim_oracle_franka_fast.md)). Two results:

**(a) The corrected Panda kinematics validate end to end.** `our_succ` == `sim_succ`
with agree% = 100 in every cell (one V4 cluttered boundary blip, 98.7→98.0), and
`our_pos` == `sim_pos` to 3 dp. An independent simulator confirms the Entry-33
modified-DH fix through the full benchmark — the single most important thing to
establish before any Franka number re-enters the paper.

**(b) V4's collision edge over TRAC-IK is UR5-specific — it disappears on Franka.**
Real-collision rates for the high-success solvers:

| Scenario | V4 | TRAC | Multi | V1 | native-IK |
|---|--:|--:|--:|--:|--:|
| open_space | 8.7 | 7.3 | 6.0 | 11.3 | 28.0 |
| near_singular | 10.7 | 11.3 | 10.7 | 8.7 | 24.0 |
| cluttered | 78.7 | 78.0 | 78.0 | **72.0** | 85.3 |

In cluttered, V4 (78.7%) is a dead heat with TRAC (78.0) and Multi (78.0) — V1 is
actually the cleanest of the ProteinIK family (72.0). The only robust signal is
that *every* collision-aware solver beats PyBullet's zero-awareness native IK
(open_space 6–11% vs 28%; cluttered 72–79% vs 85%).

---

## 7. Driving the PyBullet collision rate DOWN (clean-solve)

The question that matters for deployment: can we make our solutions actually
collide *less in PyBullet* (not relabel our proxy)? Yes. Two experiments settle
how, and how far.

**(a) The proxy cannot guide it — you need the real signal.** Config-level, the
capsule proxy has a **~20% false-clear rate** on UR5 (it reports clearance ≥ 0
while the real meshes interpenetrate; `collision_parity.md`). So selecting a
solution by *best proxy clearance* does **not** lower real collision — it slightly
*raised* it (open_space 32%→35%). Only selecting by the **real** PyBullet signal
helps. There is no cheating the proxy.

**(b) Real-collision-certified selection halves it, cheaply.** `app/sim/clean_solve.py`
generates K candidate solutions from diverse start configs (which reach different
IK branches), scores each with PyBullet, and returns the cleanest. PyBullet is used
only at the boundary (K queries), never in the solver loop. V4, UR5:

| Scenario | single-shot col% | clean-solve col% (floor) | reduction |
|---|--:|--:|--:|
| open_space | 31.7 | **11.7** | 2.7× |
| near_singular | 41.7 | **23.3** | 1.8× |
| cluttered | 66.7 | **50.0** | 1.3× |

It **saturates at K≈8–16** (K=36 no better). The residual floor is largely
*physical*: for those targets none of 36 diverse branches is collision-free, i.e.
the tight pose forces a clash — not a solver failure. So clean-solve reaches the
reachable floor.

**Honest caveats:** (i) the selection wrapper is **not V4-exclusive** — under the
same K-select, TRAC-IK ties V4 (both ~13% open_space); it *equalizes* rather than
widening V4's edge. V4's exclusive advantage remains the **single-shot** rate
(collides less per solve: 32 vs TRAC 38 open, 67 vs 80 cluttered). (ii) It costs
~K× solves + K collision queries, so it is an **offline / planning-grade** mode —
which is exactly where the ProteinIK family is deployed. (iii) Demonstrated on UR5;
Franka's proxy is too weak to even seed candidate diversity well (deferred).

**Bottom line:** we can genuinely cut the real collision rate roughly in half by
letting PyBullet certify the cleanest of several candidate solves — a real,
mergeable capability — as long as we're honest that it's an offline booster that
lifts every solver, and that V4's *unique* edge is the single-shot number.

---

**V6 doesn't rescue it either.** V6 was the UR5 real-collision champion, so the
obvious question is whether it wins on Franka. It does not (n=60): open_space
sim_col 10.0% (pack 6–11), near_singular 6.7% (best here), but **cluttered 86.7% —
*worse* than the pack's 78%**, at lower success (91.7%). So *both* protein variants'
collision advantage is UR5-specific; on the redundant arm neither V4 nor V6 has an
edge, and V6's biophysical search actually hurts in dense clutter.

**Mechanism-honest reading:** on the **redundant 7-DOF** Panda the null space lets
*every* solver — including TRAC-IK's random restarts — dodge self-collision about
equally, so the collision-energy search (V4) and biophysical energy (V6) stop being
decisive. On the **non-redundant 6-DOF UR5** there is no such freedom, and the
collision-aware search is what separates V4 and V6 from the pack. **The collision
edge is a UR5 / non-redundant-arm result, and should be stated as such** — it does
not generalize to the redundant arm. (This also finally *measures* Franka collision,
which the old degenerate capsule proxy could not: with corrected kinematics + real
meshes the rate spreads a real 6%→87% across solvers.)

---

## 8. Phase 3 complete — *where* the proxy is wrong (per-link-pair mechanism)

Config-level, the capsule proxy is systematically optimistic (`collision_parity.md`,
n=3000/arm): UR5 real-collision **36.5%** vs proxy **16.9%** (**20.2% false-clear** —
proxy says clear, meshes interpenetrate); Franka real **9.9%** vs proxy **0.5%**
(9.5% false-clear). No safety margin `δ ≤ 0.15 m` drives the residual risk under 1%
on either arm — the proxy can't be rescued by a threshold, it must be re-geometried
(and even a mesh-faithful fast model can't do it well enough — the scrapped V7,
`raw_notes.md` Entry 37).

**The new, mechanistic result:** attributing every real collision to the link *pair*
whose meshes are closest shows the proxy's optimism is **not diffuse — it localizes
to one link pair per arm:**

| Arm | pair behind the proxy's false-clears | share |
|---|---|--:|
| UR5 | `forearm_link ↔ wrist_2_link` (+ `forearm ↔ wrist_3` 16%) | **73%** |
| Franka | `panda_link5 ↔ panda_link7` | **73%** |

Both are the **tight forearm↔wrist cluster** — exactly where the proxy's thin
capsule, laid on the joint-axis segment, cannot represent the bulky, offset wrist
link mesh, so it reports clearance that isn't there. This is the geometric reason the
proxy exaggerated V4's collision edge, and it names the single pair a future
CAD-derived collision model would have to fix first. It also explains Franka's weak
proxy correlation (0.41 near-boundary): a single blind pair dominates the metric.

**Bottom line for the paper:** quote collision as a *comparison* (V4 vs baselines),
never as an absolute proxy rate; the real rate is 2–20× higher and its error is
concentrated on the wrist cluster.

---

## 9. Phase 4 — MuJoCo second oracle: three-way agreement

_(populated from the authoritative `sim_crosscheck.md` sweep — see next section)_
