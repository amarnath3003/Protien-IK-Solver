# ProteinIK — Paper Notes (the SPINE)

> **Purpose.** This is the single source of truth for writing the paper — the thesis, the argument flow, every
> claim with the exact number that backs it, the figures, the honesty guardrails, and the citations. We write the
> paper *from* this file and iterate section contents on top of it.
>
> **Companion files:** [outline_simple.md](outline_simple.md) (plain-English section plan) ·
> [methodology.md](methodology.md) (deep technical Methods). **Superseded:** `research_direction.md` is an
> *outdated* pre-audit framing (wrong "V3" label, "V4 = numerically identical," V5 "94% vs 90%") — **do not cite it.**
>
> **Venue:** full conference paper first → then the thesis absorbs everything.
> **Framing:** concept-forward — *"IK is structurally a protein-folding problem"* — backed by data.
> **Numbers status:** figures below are the current **committed** values (UR5/planar proxy from
> `v1v4_full_benchmark`, corrected Franka from `franka_corrected_benchmark`, real-mesh collision from
> `sim_crosscheck` n=100 both engines). **Final headline numbers get re-locked on the high-end-PC full run**
> (`bench/master_sim_benchmark.py`, N=300, both engines) before submission.

---

## 1. Thesis

**One sentence:**
> Inverse kinematics is, structurally, a protein-folding problem — the same chain searching the same kind of rugged,
> constrained landscape — so we build an IK solver from folding's *process*, and it wins exactly where the problem
> becomes most folding-like.

**Expanded:** A robot arm and a protein backbone are both kinematic chains whose only free variables are the
rotations between rigid segments. "Solve IK" and "fold" are the same search — for a configuration that satisfies
boundary conditions on a landscape full of local minima, singular/frustrated regions, and self-overlap. We port
folding's *staged process* into an IK solver (StagedFold), make it competitive using folding's *kinetic
partitioning* as a compute schedule (KineticFold), and show a literal folding simulation (LangevinFold) buys
solution quality. The contribution is the **organizing principle**, not new energy terms — and it is validated
against two independent physics simulators.

---

## 2. Narrative through-line (the beats)

1. **Isomorphism** — IK *is* folding (correspondence table). The bridge is real: CCD, a robotics IK algorithm, was
   already borrowed *into* protein loop closure — we cross it back.
2. **StagedFold** — port the folding *process*; honest that the parts are standard IK, the *sequence* is new.
   Beats simple baselines, plateaus below production ones → motivates the next step.
3. **KineticFold** (the star) — folding's kinetic partitioning schedules the optimization: cheap downhill fold
   first, expensive staged search only on frustrated targets. Now competitive: success leader, speed-matched,
   cleaner per solve.
4. **Climax** — as the arm lengthens into a polymer, KineticFold's single-shot clean-solve rate degrades most
   gracefully of the standard field: the concept proves itself where the problem becomes folding.
5. **Validation** — two independent simulators confirm the kinematics and the collision *ranking*, and shrink our
   own over-optimistic proxy claim. Honesty as a feature.
6. **Glimpse** — LangevinFold: take the physics literally, get the cleanest solutions (quality, not speed).

---

## 3. The isomorphism (conceptual core → Figure 1)

| Protein folding | Inverse kinematics |
|---|---|
| Backbone dihedral angles φ/ψ (soft DOF) | Joint angles `q` (the DOF) |
| Rigid bonds / fixed bond lengths | Fixed link lengths (FK constraints) |
| Native (folded) state | The IK solution configuration |
| Free-energy funnel | Convergence basin to the target |
| Rugged landscape / kinetic traps | Local minima / failed solves |
| Excluded volume (sterics) | Self-collision avoidance |
| Hydrophobic collapse | Coarse approach to the target region |
| Secondary structure (local order) | Local joint settling (StagedFold Stage 1) |
| Molecular chaperone (GroEL) | Restart / rescue from a stuck state |
| Kinetic partitioning (fast vs. slow folders) | Easy vs. hard targets (KineticFold's schedule) |

---

## 4. Contributions (precise)

1. **A design principle:** casting IK as a folding *process* — the first IK solver built as a staged fold + kinetic
   partitioning + chaperone rescue. (Novelty = the organization + two unusual moves: target-blind-first init and
   scoped-then-escalating rescue. **Not** new energy terms.)
2. **KineticFold:** kinetic partitioning as a compute schedule that kills the latency tail — success leader across
   three arms, speed-matched to TRAC-IK on the easy regime, cleanest single-shot self-collision among practical
   solvers on the non-redundant arm.
3. **A dual-simulator validation methodology:** "solve once, score three ways" (our proxy + PyBullet + MuJoCo),
   which *independently confirms* every success claim and *corrects* our own collision-magnitude claim — rare in
   heuristic-IK papers.
4. **An honest map of where the principle pays off and where it doesn't:** the per-solve edge grows with chain
   length vs the standard field (climax); it ties on the redundant arm; and (glimpse) literal folding physics buys
   quality at a latency cost.

---

## 5. Claim → Evidence map (the heart — every number sourced)

> Legend: **proxy** = our capsule checker; **real** = PyBullet≈MuJoCo (`sim_crosscheck.md`, n=100).
> success/speed are deterministic-per-seed and engine-independent.

### C1 — The concept is new (not a metaphor)
- Biology-inspired IK exists (evolutionary/neural/swarm) but none is built as a folding *process*.
- **Anchor:** CCD (our baseline) was adopted for protein loop closure (Canutescu & Dunbrack 2003) → the fields
  provably share machinery; we transfer the folding *process* the other way.

### C2 — KineticFold is the success leader (all arms)
| Arm | KineticFold | TRAC-IK | Multi-start | best simple baseline |
|---|--:|--:|--:|--:|
| UR5 open/near/clut | **100 / 100 / 100** | 99 / 98.3 / 97.7 | 97 / 97.7 / 98.7 | ≤56 |
| Franka open/near/clut | **100 / 99.7 / 99** | 98.7 / 97.7 / 92.7 | 97.3 / 96.3 / 86.7 | ≤50 |
| StagedFold (V1) | beats simple only | — | — | — |
- Sources: `v1v4_full_benchmark.md` (UR5), `franka_corrected_benchmark.md` (Franka). StagedFold UR5 94/90.7/89.7,
  Franka 97.7/93/83.3 → **beats simple baselines, trails production** (state honestly).

### C3 — Speed: matches TRAC-IK's core on the easy regime; honest tail
- UR5 open: KineticFold **mean ~12.9 ms, p50 ~3.0 ms** vs TRAC-IK **mean 12.6, p50 7.2** → mean-tie, better median.
  (Quick re-run: 9.4 ms.) Fastest of the folding family across all arms.
- Corrected Franka open: KineticFold **18.7 ms** *faster* than TRAC-IK 22.1 ms.
- **Tail (state it):** hard targets inflate p95/p99 (UR5 open p95 38 / p99 215 ms; Franka cluttered mean 271, p50 46)
  — the reason it's a planning/offline tool. Source: `v1v4_full_benchmark.md`, `franka_corrected_benchmark.md`.

### C4 — Self-collision, UR5 (real mesh): RE-LOCKED on 10 seeds (2026-07-09) — V4 cleanest on the hard regimes
> **The seed problem is RESOLVED by averaging, NOT by deleting seeds** (user asked to drop worst seeds — declined as
> cherry-picking; ran the honest 10-seed average instead). Committed run: `backend/results/ur5_collision_seeds10.md`
> (seeds **1–10**, n=1000/cell, PyBullet+MuJoCo, practical solvers). The old [1,2,3] full-run draw was anomalous on
> BOTH the open cell (TRAC's 15% was a lucky-clean outlier → 31% over 10 seeds) AND the cluttered cell (V4 71% → 57%
> over 10 seeds). **Averaging vindicated V4 on the two harder regimes:**
| UR5 scenario | **KineticFold (V4)** | TRAC-IK | Multi-start | note |
|---|--:|--:|--:|---|
| open_space | 36.1 / 33.8 | 30.6 / 28.8 | 39.0 / 34.4 | comparable (all ~100%) |
| near_singular | **40.0 / 38.8** | 46.8 / 46.2 | 44.6 / 43.2 | V4 cleanest |
| cluttered | **57.0 / 56.1** | 71.1 / 71.1 | 65.0 / 64.7 | V4 cleanest **& only 100%-success** (TRAC 96.5) |
> (numbers = PyBullet / MuJoCo real-mesh collision %; two engines agree within ~2 pp everywhere.)
> **Honest paper claim (locked):** "KineticFold is the cleanest high-success practical solver on the two harder UR5
> regimes (near-singular, cluttered), penetrating ~35% less deeply on cluttered (clr −0.0206 vs TRAC −0.0317 m) while
> being the only practical solver to reach 100% success; on easy open_space it is comparable to TRAC-IK."
> **Caveats the paper MUST respect:**
> 1. **Seed variance is large** — rate swings ~15–20 pp between 3-seed draws → quote from the 10-seed run, never [1,2,3].
> 2. **Edge is MODEST** — real-mesh gap ~1.15–1.25× on near/cluttered, a wash on open; NOT the 2–6× the proxy shows.
>    Mechanism: V4 crushes the *proxy* but its false-clear rate is high (proxy-clean ≠ real-clean; proxy blind in the
>    wrist cluster). Direction real, magnitude small.
> 3. **V1 (StagedFold) shows the lowest raw rate on open/near** (27.8/32.4%) but at lower success (98.8/93.8%) and high
>    position error — so it is NOT the "cleanest practical" winner; compare among the ~99–100%-success solvers only.
- Note V6/LangevinFold was NOT in the 10-seed run (slow); its UR5 collision (open 0.0, near 0.3, cluttered 73.7 from
  `master_full`) stands — V6 is the open/near champion but WORSE than V4 on cluttered. So V4, not V6, leads cluttered.
- **Superseded:** the old n=100 `sim_crosscheck` C4 table (12/31/56 etc.) — use the 10-seed numbers above.

### C5 — Self-collision, Franka (real mesh): KineticFold **ties** the strongest baseline
| Scenario | rates (all solvers) | reading |
|---|---|---|
| open / near | ~6–13% across the board | tie |
| cluttered | StagedFold 72, Langevin 76, TRAC 78, **KineticFold 79**, Multi 80 | **KineticFold ties TRAC-IK** |
- **Mechanism (state it):** a redundant 7-DOF arm gives *every* solver null-space room to dodge self-collision
  about equally, so collision-aware search stops being decisive. Never worse than the best baseline; just not a
  lead. Source: `sim_crosscheck.md`.

### C6 — The climax: single-shot clean-solve degrades most gracefully as the arm lengthens
- Planar arm, grow DOF 4→16, cluttered (proxy). **Clean-solve rate** (reach target AND self-collision-free):
| DOF | KineticFold clean% | TRAC-IK clean% | ratio |
|--:|--:|--:|--:|
| 4 | 75.8 | 34.2 | 2.2× |
| 6 | 59.2 | 16.7 | 3.5× |
| 8 | 36.7 | 5.0 | 7.3× |
| 12 | 11.7 | 0.8 | 15× |
| 16 | 1.7 | 0.0 | only one > 0 |
- Both **reach** the target 100%; the entire gap is self-collision avoidance. Source: `usecase_experiments.md` EXP E.
- **⚠️ HONEST FRAMING (mandatory):** this is a **single-shot** advantage over the **standard baseline field**
  (TRAC-IK, error-selected Multi-start). It is **not** "the only method that works": a *clearance-selecting*
  multi-start (solve K times, keep the cleanest) **beats KineticFold** on these arms (`forkA_redundant_robots.md`),
  and a K-select wrapper equalizes all solvers (`sim_oracle_findings.md §7`). So: **KineticFold provides the best
  per-solve rate; selection wrappers are a strong, orthogonal booster that lifts everyone.** State the climax as
  single-shot vs standard field, and cite the selection-wrapper nuance in Limitations. (Do NOT claim absolute
  supremacy — our own Fork A refutes it.)

### C7 — Deployment roles (where the profile fits)
- **NOT real-time control:** Franka p99 ~928 ms, max ~2.5 s, 74% of solves >10 ms → planning/offline tool
  (frame gracefully, not as a disclaimer). Source: `usecase_experiments.md` EXP A.
- **Planning goal-sampler (UR5 cluttered):** usable clean goals/attempt **83.4 vs TRAC 56.9 / Multi 65.3.** (EXP B)
- **Offline batch clean-solve:** wins honest cells by **+18–30 pp** (UR5 open 96.5 vs TRAC 78.5; cluttered 78.5 vs
  48.5). (EXP C)
- **Reliability fallback:** rescues **60–78%** of the targets TRAC-IK punts (UR5 cluttered 60.2, near 77.6). (EXP D)

### C8 — Validation (the honesty engine)
- **FK three-way agreement:** DH ≡ PyBullet ≡ MuJoCo to float noise (UR5 PB↔MJ 4.1e-8 m; Franka 5.9e-8 m) → every
  success claim independently true on two engines, including the corrected Franka kinematics.
- **Proxy is optimistic — engine-independently:** UR5 proxy 18.1% vs PB 38.3% ≈ MJ 36.1% (sign-agree 97.8%,
  corr 0.991); Franka proxy 0.6% vs PB 9.2% ≈ MJ 8.2%.
- **Ranking replicates:** the UR5 collision ordering and the Franka tie are identical on both engines (col-call
  agreement 97–100%). Source: `sim_crosscheck.md` A/B/C.

### C9 — LangevinFold glimpse (quality champion, slow)
- Lowest UR5 real self-collision of all solvers (12 / 31 / 51%), ~99–100% success, at **~2–3.5 s/solve**. Only a
  real mesh oracle reveals it (proxy showed it tied KineticFold). → "faithful biology buys quality, not speed."
  Source: `sim_crosscheck.md`, `sim_oracle_findings.md §4`.

---

## 6. Section-by-section spine (write from this)

**S1 Introduction** — IK is hard (multiplicity, singularities, self-collision, local traps) → same as folding →
correspondence (Fig 1) → thesis (folding *process* as design principle) → preview headline + forward-ref the climax.

**S2 Related work / "not just a metaphor"** — energy-based IK (DLS), sampling/restart IK (TRAC-IK, Multi-start),
heuristics (CCD, FABRIK), biology-inspired IK (evolutionary/neural/swarm). Position: none build the solver as a
folding *process*. The **CCD↔loop-closure** anchor makes the bridge credible. State novelty = organization, not terms.

**S3 StagedFold** — the 5 stages with folding analogs (Fig 2 mapping table). Two unusual moves: target-blind-first,
scoped-then-escalating rescue. Honest verdict (beats simple, not production) + reverted-mechanism ablations as
evidence the choices matter. Detail → `methodology.md §3`.

**S4 KineticFold (star)** — tail diagnosis (~10% of targets, ~57% of time) → kinetic partitioning (barrierless-first,
escalate only if frustrated) → the GroEL-gating faithfulness argument → Layer-2 FK primitives. Rejected tail-edits
(Franka 71.7% at cap=2). Fig 3 (tail collapse). Detail → `methodology.md §4`.

**S5 LangevinFold (glimpse)** — literal folding sim; the quality result (C9); defer full biophysics to thesis.

**S6 Experiments** — 3 arms, 3 scenarios, strong field, shared targets, metrics incl. the tail; the validation
harness. Detail → `methodology.md §6`.

**S7 Results** — success (C2) → speed (C3, with the tail) → UR5 collision (C4) → Franka tie (C5). Led by UR5, Fig 4.

**S8 Where it wins + climax** — deployment roles (C7) → the DOF-scaling climax (C6) with the honest single-shot
framing → the plain "the problem becomes folding, our method is the graceful one" line. Fig 6 (the money figure).

**S9 Validation** — C8 (FK parity, proxy optimism, ranking replication). Fig 5. The "we corrected our own claim"
narrative.

**S10 Limitations** — the tail (planning/offline positioning); the climax is single-shot vs the standard field and a
clearance-selecting wrapper is competitive (the Fork A honesty); self-collision only (no env obstacles yet); proxy is
hand-tuned, not CAD; bit-identity tested on UR5+Planar (state scope).

**S11 Conclusion** — thesis landed; future = env obstacles, full LangevinFold quality study (thesis).

---

## 7. Figures

| # | Content | Data source |
|---|---|---|
| 1 | Protein ↔ robot-arm correspondence | conceptual |
| 2 | StagedFold's 5 stages ‖ folding steps | conceptual + code |
| 3 | Latency tail: old fold vs KineticFold (CDF) | benchmark timing |
| 4 | Success / speed / collision vs the field (UR5) | `v1v4_full` + `sim_crosscheck` |
| 5 | Proxy-vs-real collision + three-way FK/collision agreement | `sim_crosscheck` A/B |
| **6** | **Single-shot clean-solve vs joint count (4→16 DOF)** | `usecase_experiments` EXP E |

---

## 8. Honesty guardrails (must-follow when drafting)

- **Never quote an absolute proxy collision rate.** Collision is always a *solver-vs-solver comparison* on **real**
  (sim) numbers; note the proxy is 2–20× optimistic and its error concentrates on the wrist link-pair.
- **Climax is single-shot vs the standard field.** A clearance-selecting multi-start beats KineticFold on redundant
  planar arms (Fork A) — cite it; don't claim absolute "only tool."
- **Franka = "ties TRAC-IK"** (never worse), not "no edge / null." Explain the null-space mechanism.
- **StagedFold rescue "starts scoped, escalates to a global reseed"** — not "never restarts globally."
- **StagedFold Stage-3 search is greedy accept-if-better, NOT Metropolis.** Metropolis lives in KineticFold's Phase-B
  fold and in LangevinFold. (research_direction.md is wrong here.)
- **KineticFold is NOT numerically identical to StagedFold** — Layer 1 changes behavior. (research_direction.md's
  "V4 = numerically identical to V3" is stale/false; there is no "V3.")
- **Bit-identity of the FK primitives** is tested on UR5+Planar (~2000 configs), not "9000 across all three arms" —
  state the tested scope or extend the test to Franka first.
- **StagedFold beats only the simple baselines**, not TRAC-IK/Multi-start — say so plainly.
- **Franka proxy = "structurally elbow-pinned / insensitive to the 7th DOF,"** never "degenerate constant."
- **Corrected Franka only** — every pre-fix (standard-DH) Franka number is stale; use `franka_corrected_*` /
  `sim_crosscheck`.
- **Real-time positioning is graceful** — "built for planning/offline/quality," not a blunt "fails at real-time."

---

## 9. Related work + citations (harvest / to confirm)

- **Folding theory:** Anfinsen 1973 (native state); Bryngelson & Wolynes 1987 (minimal frustration); Bryngelson,
  Onuchic, Socci, Wolynes 1995 (funnels/landscape); Onuchic & Wolynes (funnel theory).
- **Kinetic partitioning / chaperone:** Guo & Thirumalai 1995 (kinetic partitioning); Thirumalai & Lorimer 2001
  (GroEL / iterative annealing); Honeycutt & Thirumalai 1990 (coarse-grained bead model — LangevinFold lineage).
- **The bridge anchor:** Canutescu & Dunbrack 2003 (CCD for protein loop closure).
- **IK baselines:** Nakamura & Hanafusa 1986 / Wampler 1986 (DLS); Aristidou & Lasenby 2011 (FABRIK); Beeson &
  Ames 2015 (TRAC-IK); Yoshikawa 1985 (manipulability); Levenberg 1944 / Marquardt 1963 (LM).
- **Simulators:** Coumans & Bai (PyBullet); Todorov, Erez, Tassa 2012 (MuJoCo).
- **LangevinFold physics:** Lennard-Jones 1924 (6-12); Kauzmann 1959 (hydrophobic effect); Baker & Hubbard 1984
  (H-bond geometry).
- *(V5-specific refs — PCGrad (Yu 2020), IFT/homotopy (Allgower & Georg) — not needed; V5 is dropped from the paper.)*

---

## 10. Open items before submission

1. **Lock final numbers** with the high-end-PC full run (`bench/master_sim_benchmark.py`, N=300, both engines) — the
   spine's figures are current-committed and may shift a point or two.
2. **Decide the Fork A / clearance-selection comparison** — recommend: include it honestly in Limitations (strong),
   optionally add a "collision-aware selection wrapper" as a discussion baseline.
3. **Bit-identity test** — extend to Franka or state the UR5+Planar scope in the text.
4. **Figures** — generate Fig 3 (tail CDF) and Fig 6 (DOF-scaling) from committed CSVs.
5. **Env-obstacle collision** — name as future work (current collision = self-collision only).
6. **o1/o2 speed-tuning are NOT merged to main — base V4 is the paper's KineticFold (VERIFIED 2026-07-08).**
   `main`'s `protein_fast/solver.py` is base V4; o1/o2 live only in the `v4opt` worktree. **o2 real-mesh test done
   this session** (isolated `protein_fast_o2` = base V4 + o2 IAM partial-unfold on corrected DH; base untouched;
   `backend/results/o2_test3.*`, n=150 [seeds 1,2,3], PyBullet+MuJoCo): **success identical (100% all cells); o2
   modestly faster + cuts the cluttered p99 tail ~17–25% (UR5 344→259 ms, Franka 355→296 ms); and — the key finding —
   the ~1–2 pp collision erosion o2 showed on the CAPSULE PROXY does NOT reproduce on real meshes** (o2≈base within
   n=150 noise: UR5 cluttered o2 68.7% vs base 71.3%, UR5 near 23.3 vs 22.0, Franka cluttered 85.3=85.3). So the
   original objection to o2 (proxy collision cost) was itself a proxy artifact. **⇒ merging o2 is now better-supported,
   but:** differences are within noise, so confirm on the full N=300 both-engine run; and merging would require
   re-running every headline number on o2. Recommendation for the paper: **keep base V4 as the validated star**
   (numbers already done on it), report o2 as the faithful speed-tuned variant with the honest sim note above.
   *(Fork A is a redundant-arm **experiment**, not a solver variant — nothing from it is or was meant to be in V4.)*
</content>
