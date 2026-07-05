# Raw (V6) — Running Notes / Thought Process

> Living document. Updated after **every** prompt to track our evolving thinking on Raw.
> Companion docs: `raw_design.md` (term filter), `raw_audit.md` (faithfulness×rawness
> verdict), `raw_math.md` (locked math spec), `research_direction.md` (paper spine).

---

## 0. Governing principle (locked)

**Exactly follow the protein-folding mechanism.** Rawness is a *consequence*, not a separate
filter — no IK solver replicates folding, so a faithful replica is automatically novel.
Where a folding-faithful term happens to coincide with an existing IK technique, that is a
signal the term is *unfaithful* (wrong quantity), not that folding should be abandoned — fix
the quantity. Scope of "exact": the **coarse-grained Cα level** (a real, simulated level), not
all-atom. The single non-folding term is `E_task` (folding is target-blind), kept minimal.

---

## 1. What Raw is (the thesis)

Not "IK dressed as biology" (V1–V5). A literal coarse-grained Cα-bead, implicit-solvent
**folding simulation** whose polymer is a robot arm (Honeycutt–Thirumalai / Enciso–Rey /
Clementi–Onuchic lineage). Joint origins `pᵢ` = beads · links = rigid bonds (FK-enforced) ·
joint angles `q` = torsions (only soft DOF).

Spectrum: V1 = biology in architecture · V4 = optimized math · V5 = one principle (minimal
frustration) · **Raw = biology in the energy function itself.**

---

## 2. Locked decisions

| Decision | Choice | Rationale |
|---|---|---|
| Governing rule | exactly follow folding; rawness follows | user directive |
| Dynamics | overdamped Langevin; **endgame = its `T→0` limit** (native-state consolidation = LM/Newton), NOT a foreign finisher | resolves the "pure Langevin can't hit tolerance" tension faithfully |
| Build sequence | bio + math first, then code | user directive |
| Beads | joint origins from FK chain | Cα correspondence |
| LJ ε | **uniform** (non-Gō) | structure must emerge, not be planted |
| Temperature | **single self-consistent `T`** across entropy weight + noise + cooling | faithful (FDT) + the defining feature |
| Entropy | **`S=log Ω`, target-blind, clash-aware** (NOT manipulability) | the real hydrophobic entropy; faithful AND raw |
| `E_native` for Σ | cheap warm-start proxy, stated openly | the one IK-specific circularity |

---

## 3. The free energy (what Langevin minimizes) — see `raw_math.md` for full forms

```
F(q;T) = E_task + E_LJ + E_HB − T·S_conf(q)
```
- **E_LJ**: full 6-12 with attraction, non-adjacent beads, uniform ε. Role: packing/contacts.
- **E_HB**: directional; vector = **triplet-plane normal** of `(p_{i−1},pᵢ,p_{i+1})` (NOT joint
  axis). Role: secondary structure.
- **S_conf** = `log Ω(q)`, Ω = clash-free, in-limits accessible micro-volume, **target-blind**
  (MC estimate, common random numbers). Role: hydrophobic collapse / clash+singularity avoidance
  by thermodynamics.
- **Dynamics**: `q←clip(q − ∇F·Δt + √(2TΔt)ξ)`, cool `T_t=max(T_glass, T_start e^{−t/τ})`.
- **Last step (§4b)**: at `T→0`, noise off → damped-Newton/LM = native-state consolidation;
  then jitter-and-relax stability gate (Anfinsen native check).
- **Σ** (pre-solve): `σ_E/ΔE = 1/Z`; `<1` funnel, `>1` glassy.

---

## 4. Rawness audit — verdict (evidence in `raw_audit.md`)

| Term | Faithful? | Raw? |
|---|---|---|
| LJ attraction | ✅ | RAW ✓ |
| Directional H-bond (triplet-plane normal) | ✅ | RAW ✓ |
| `S=log Ω` entropy (target-blind, clash-aware) | ✅ | **RAW ✓** (resolved — was the failing term) |
| Σ landscape topology | ✅ | RAW ✓ (E_native proxy caveat) |
| Langevin + `T→0` consolidation endgame | ✅ | RAW ✓ (endgame is the dynamics' limit, not a bolt-on) |

All five terms now pass once the §5 corrections are applied.

---

## 5. Corrections — status

1. **H-bond direction → triplet-plane normal** (not `zᵢ`). ✅ locked into `raw_math.md`.
2. **Entropy → `S=log Ω`, target-blind, clash-aware** (not manipulability). ✅ locked.
3. **Σ native reference = warm-start proxy**, stated openly. ✅ locked.
4. **Keep single self-consistent `T`.** ✅
5. **Endgame = `T→0` consolidation (LM/Newton) + stability gate.** ✅ locked (`raw_math.md §4b`).

---

## 6. Open questions

- Per-robot calibration of `s, ε, ε_hb, d₀, σ_d, ρ, m, margin, T_start, τ, Δt, w_task` from geometry.
- How strongly may `E_task` tilt the landscape before Raw is "just IK"? (the experiment)
- Entropy MC variance vs. step cost — cheap enough for many Langevin steps?
- Success metric = quality (min_self_distance, naturalness) over success-rate — confirmed thesis.

---

## 7. Phase status

- **Phase 1 — `E_LJ` + analytic force — ✅ DONE & validated.**
  - Code: `backend/app/solvers/protein_raw/energy.py` (`lj_energy`, `lj_energy_and_grad`,
    analytic force from one FK pass), exported via the package `__init__`.
  - Tests: `backend/tests/test_raw_energy.py` — 13 pass; analytic force = central-FD to <1e-4 on
    all 3 robots, both modes; well shape (−ε at 2^(1/6)σ) verified. Full suite 76/76, no regressions.
  - Experiment: `backend/raw_phase1_experiment.py`. **Attraction proven to do real work:**
    UR5 spacing std 0.54→0.28, in-well 61% vs 12% (repulsion-only); Planar `E_LJ`→ **negative**
    (chain bound in wells), 83% in-well vs 0%. Franka: force verified (E descends 2e8→1e4) but
    constrained geometry frustrates collapse. ⚠️ *(The "capsule `min_self` is a degenerate constant
    −0.15" claim here is **STALE / corrected in Entry 17** — that was the old-radii era; current
    Franka `min_self` varies, std 0.029. The real issue is structural elbow-pinning, not degeneracy.)*
- **Phase 2 — directional H-bond `E_HB` — ✅ DONE & validated.**
  - Code: `energy.py` — `hbond_energy`, `hbond_energy_and_grad` (FD force per §3.2), backbone
    normal = **triplet-plane normal** of `(p_{i−1},pᵢ,p_{i+1})` (corrected, not joint axis);
    interior beads only (endpoints/collinear excluded); distance-only ablation flag.
  - Tests: `tests/test_raw_hbond.py` — 9 pass (distance/angle gates, triplet-normal right-angle
    & collinear, planar has no pairs, directionality reduces magnitude, FD gradient is a descent
    direction). Full suite 85/85.
  - Experiment: `raw_phase2_experiment.py`. **Two-condition gate proven:** ideal H-bond
    (d₀ + aligned) is **55× stronger** than perpendicular, **4× stronger** than off-distance —
    requires distance AND orientation, exactly like a real H-bond.
  - **Honest finding:** gradient descent on `E_HB` alone only modestly orients contacts
    (UR5 align 0.51→0.53; Franka ~flat — its normals sit perpendicular to contacts). Robot arms
    are **short polymers (6–10 beads)** and the angular gate is **flat when misaligned**, so
    *emergent* secondary structure is a **Langevin-stage** property (thermal escape), not a GD one.
    The term is correct and ready; emergence will be shown once the solver exists.
- **Phase 3 — configurational entropy `S = log Ω` — ✅ DONE & validated.**
  - Research confirmed (both prongs): conformational entropy is the standard *destabilizing*
    (unfolded-favoring) term and is estimated by the polymer **free-volume** MC method; IK uses
    *clearance*/*C-free* but never a local accessible-volume entropy → no IK equivalent.
  - Code: `energy.py` — `config_entropy`, `config_entropy_and_grad` (FD force, **common random
    numbers** stencil). Soft (sigmoid) feasibility = in-limits × clash-free; **target-blind**,
    **collision-aware**; Gaussian local cloud.
  - Tests: `tests/test_raw_entropy.py` — 6 pass (S≤0, deterministic, target-blind signature,
    lower near limits, collision-aware ranking, FD gradient is an ascent direction). Suite 91/91.
  - Experiment: `raw_phase3_experiment.py`. **Rawness proven empirically:** corr(clearance, S_conf)
    = **+0.90 / +0.65 / +0.91** (UR5/Planar/Franka) vs corr(clearance, manipulability)
    **+0.08 / +0.21 / −0.27** — S tracks self-collision, manipulability is blind → S ≠
    manipulability. Entropy ascent **opens** configs (UR5 clearance −0.085→+0.020, limit-margin
    0.42→0.90) — the unfolding drive that competes with LJ collapse.
  - Corrected `raw_math.md §3.3`: soft/differentiable estimator; entropy **opposes** collapse
    (chain conformational entropy), avoids clashes/limits — NOT singularities.
- **Phase 4 — Σ ratio + `T_glass` (landscape topology) — ✅ DONE & validated.**
  - Code: `landscape.py` — `RawParams.calibrate` (per-robot scales), `bio_energy` (capped LJ +
    H-bond), `warm_start` (DLS native proxy), `sigma_ratio` (BW **compact ensemble** of warm-start
    solutions; balanced task+bio; native = best), `_sigma_from_energies`,
    `configurational_entropy_scale` (S₀), `glass_temperature`. Added backward-compatible `e_cap`
    to `lj_energy` (random configs sit in the 1e13 r⁻¹² wall).
  - Tests: `tests/test_raw_landscape.py` — 13 pass (Σ measure validated on controlled inputs:
    funnel < glass, Σ>1 achievable, Σ<1 achievable; sigma_ratio positive/finite; T_glass formula
    & monotonic; S₀>0; DLS warm-start reduces error). Suite 104/104.
  - Experiment: `raw_phase4_experiment.py`. **Honest result:** Σ spans the BW threshold
    meaningfully (UR5 ~0.77–0.87 funnelled; Franka/Planar ~1.0 glassy); correlation with
    collision-blind DLS difficulty is modest/mixed (−0.24/+0.16/−0.12) — Σ is a **collision-aware**
    landscape measure, **complementary** to V5's conflict (per `raw_design.md`), NOT a strong
    oracle. Reported transparently. Operational use: sets `T_glass` (cooling target).
  - Corrected `raw_math.md §6`: compact-ensemble Σ (not random sampling), balanced potential,
    LJ cap, honest scope.
- **Discovery (honest):** the assumed scenario difficulty order (open<cluttered<near_singular)
  does **not** hold under DLS (UR5: cluttered easiest, open hardest) — so Σ is validated against
  measured difficulty, not an assumed label order.
- **Phase 5 — the Langevin folding solver — ✅ DONE & registered (V6 LIVE).**
  - Code: `solver.py` — `solve_protein_raw` assembling `F = E_task + E_LJ + E_HB − T·S_conf`;
    overdamped Langevin (Euler–Maruyama) cooling `T_start→T_glass`; per-term gradient balancing +
    step/LJ-force clipping; `T→0` damped-Newton consolidation endgame (`_consolidate`) + Anfinsen
    stability gate (`_stable_native`); multi-start warm-start seeds (redundant arms get more) for
    the reaching boundary condition; records folding phases.
  - New `SolveResult` fields (defaults 0.0, backward-compatible): `sigma_ratio`, `free_energy`,
    `t_glass`.
  - Registered: backend `registry.py` (`protein_raw`), frontend `solverMeta.js` (entry, compat ×3,
    order, phase labels). README V6 → **Live** with the term table.
  - Tests: `tests/test_raw_solver.py` — 4 pass (valid result+diagnostics+phase trace, reaches
    easy targets, registered/served, API path). Full suite **108/108**, no regressions.
  - **Measured:** UR5 10/10, Planar 9/10, Franka 9/10 on open_space; ~2.6s / 0.9s / 4.6s per solve
    (slowest of the family, by design — quality over speed).
- **Status: all 6 phases complete. Raw (V6) is implemented, tested, live, and honest.**
  Remaining polish (optional): full benchmark sweep vs V4/V5 for the paper's quality table
  (min_self_distance / naturalness), analytic gradients for E_HB & S_conf (speed), per-robot
  weight calibration.

---

## Changelog (per prompt)

- **Entry 1** — Built full project context (backend core, solvers, registry, frontend, scenarios).
- **Entry 2** — User reframed Raw as exact folding replica; deep folding-physics review.
- **Entry 3** — Build plan; user chose pure Langevin + bio/math-first.
- **Entry 4** — Biology→math derivation (CG bead-chain, F(q), forces, PMF, Σ, Langevin).
- **Entry 5** — Goal: research deeply + check if *actually raw*. Literature audit (8+ sources).
  Verdict: 3 raw terms; entropy term FAILED the filter. Wrote `raw_audit.md`; created this log.
- **Entry 6** — User principle: "exactly follow folding." Resolved entropy → `S=log Ω` target-blind
  clash-aware (faithful = raw); locked H-bond triplet-plane normal. Wrote `raw_math.md`.
- **Entry 7** — Last-step reconciliation: IK's LM/Newton endgame = the `T→0` limit of the same
  Langevin = native-state consolidation; + Anfinsen stability gate. Added `raw_math.md §4b`.
  "No finisher" decision updated honestly (endgame is the dynamics' physical endpoint).
- **Entry 8** — **Phase 1 implemented properly:** `E_LJ` + analytic force (`energy.py`), 13 tests
  (analytic=FD <1e-4, well shape), 76/76 suite. Experiment proves the attractive well creates
  emergent preferred spacing (UR5/Planar strong; Planar binds to negative E). ⚠️ *(The "Franka
  `min_self` degenerate constant −0.15" finding is STALE — corrected in Entry 17.)* See §7.
- **Entry 9** — **Phase 2 implemented properly:** directional `E_HB` + FD force with the
  triplet-plane-normal vector (`energy.py`), 9 tests, 85/85 suite. Two-condition gate proven
  (ideal 55× > perpendicular). Honest finding: emergent secondary structure needs the Langevin
  thermal search, not GD on short arms — term is correct and ready. See §7.
- **Entry 10** — **Phase 3 researched + implemented:** configurational entropy `S=log Ω`
  (`energy.py`, FD force, CRN stencil), 6 tests, 91/91 suite. Research confirmed faithfulness
  (free-volume MC, opposing conformational entropy) + rawness (no IK accessible-volume entropy).
  Experiment refutes the audit's worry empirically: corr(clearance, S_conf)≈+0.9 vs
  manipulability≈0 → S ≠ manipulability. Corrected `raw_math.md §3.3`. See §7.
- **Entry 11** — **Phase 4 implemented:** Σ ratio + `T_glass` (`landscape.py`, `RawParams`),
  13 tests, 104/104 suite. Σ over BW compact ensemble; validated the measure on controlled
  funnel/glass inputs. **Honest:** Σ correlates only modestly with collision-blind DLS difficulty
  (complementary, collision-aware measure, not an oracle); assumed scenario order doesn't even
  hold. `e_cap` added to `lj_energy`. Corrected `raw_math.md §6`. See §7.
- **Entry 12** — **Phase 5 implemented + registered (V6 LIVE):** `solve_protein_raw`
  (`solver.py`) — Langevin on `F`, cooling to `T_glass`, `T→0` consolidation + stability gate,
  multi-start seeds. Added `SolveResult.{sigma_ratio,free_energy,t_glass}`; registered backend +
  frontend; README V6→Live. 4 solver tests, 108/108 suite. Measured ~9-10/10 open_space across
  arms, ~2.6/0.9/4.6 s (slowest by design). Found+fixed a best_q tracking bug. **All 6 phases done.**
- **Entry 14** — **Fair-shot quality sweep (UR5 cluttered, N=24; `raw_quality_sweep.py`).**
  Verdict: **Raw does NOT beat V4 on collision quality** (V4 +0.0061 min_self / 0% win-baseline;
  Raw −0.016…−0.023 / 27-32% win). Two mechanistic root causes, both important:
  (1) **weights are inert** — Raw selected the returned solution by task error only, so the fold's
  LJ/H-bond/entropy shaped the trajectory but not the OUTPUT. Fixed the architecture: native =
  **min free energy** among target-reaching candidates (`solver.py`, faithful Anfinsen). But
  calibration STILL inert + slightly worse, because (2) the candidate pool is dominated by the
  shared multi-start DLS seeds, AND — the deeper one — **Raw's `E_LJ` uses joint-ORIGIN distances
  while `min_self` uses capsule SURFACE distances** (the audit's flagged geometry mismatch): min-F
  ≠ max-measured-clearance, so optimizing F doesn't optimize the scored metric. Plus UR5 is
  **non-redundant** (no null-space to exploit; collision = discrete-basin choice). Conclusion: on
  the current proxy + non-redundant arms, Raw's physics can't win, for understood structural
  reasons. Raw's true best case (redundant Franka + an energy-consistent collision oracle) needs
  the sim migration. **Honest null, well explained.**
- **Entry 15** — **Fixed the null → Raw now clash-free & on par with V4** (UR5 cluttered, N=24).
  Root cause found by instrumenting the native-state candidate pool (`raw_diag.py`, since removed):
  the pool routinely **contained** a clean branch (e.g. +0.0196) but Raw returned a *colliding* one
  (−0.10, worse than every seed) because selection ranked by **bead-based enthalpy/free energy**,
  which is **decoupled from capsule clearance** — a capsule collides mid-span while its origin beads
  stay far apart, so low E_LJ ≠ clash-free. Fix (`solver.py` endgame): **excluded volume is a HARD
  steric constraint (Pauli), not a soft energy** — native state = the **clash-free** converged
  candidate (min_self>0) with **min enthalpy** (T→0, entropy term gone) as tiebreak; least-bad
  clearance only if none clash-free. Plus widened the molten-globule ensemble (`n_ws` 4→10 seeds)
  so a clean branch is usually present. **Result: Raw −0.0228 → +0.0039 min_self (clash-free),
  success 92%→100%, win-vs-V4 32%→42%; V4 +0.0061.** Now a tie, not a loss. 109/109 suite green;
  timing 2.1/0.68/3.2 s (UR5/Planar/Franka), not degraded. **Honest limitation:** the three weight
  configs stay byte-identical — on non-redundant UR5 the continuous Langevin *fold* still adds
  nothing over multi-start + clash-free selection; the win's active ingredients (excluded-volume
  native selection + multi-start ensemble) are biophysically principled but not the dynamics. The
  fold's real arena remains redundant arms + an energy-consistent collision oracle (sim migration).
- **Entry 16** — **Full-project gather / state snapshot (no code change).** Re-read every doc +
  result set to decide "what next." Current state: all 6 versions live & registered; **108/108
  suite green**; frontend registered ×3. Result corpus: (a) master benchmark = Franka, 10 solvers,
  N=300/cell — V4 wins success (98–99.7%) + accuracy (0.6–0.7 mm) but slow mean (194–320 ms);
  TRAC-IK fast (26 ms) but 91–95% & high collide%; (b) `v5_verify_n100` = UR5, N=100 — **honest
  wash for V5's headline: conflict-control (A) does NOT beat fixed-λ; cluttered A0B0C0 98% vs full
  V5 92%.** V5's real contribution is the *diagnostic* (difficulty score, conflict index), not a
  success win, at ~100× protein_fast's cost; (c) Raw quality = honest null → tied V4 via
  excluded-volume native selection (Entries 14–15). **Two strategic blockers identified, both point
  to the same unlock:** (1) capsule `min_self` is degenerate on Franka (−0.15 const) — the one
  redundant arm where Raw should win can't be measured; (2) Raw's bead-origin `E_LJ` ≠ capsule
  surface `min_self` (min-F ≠ max-clearance). ⇒ the central research question ("does biophysical
  energy give better *quality*?") **cannot be answered on the current proxy.** `sim_migration_plan.md`
  (PyBullet→MuJoCo oracle, ~1 wk) is the gating next step. Also uncommitted: **both worktree agents
  independently wrote the same V4 change** (`_collision_free_seed`) — a quick win to measure & land.
  **[SUPERSEDED by Entry 17 — both premises in this entry are now falsified by data.]**
- **Entry 17** — **Diagnostic session: the two premises Entry 16 rested on are BOTH FALSIFIED with
  data** (scripts in scratchpad; no code change).
  1. **"Franka `min_self` = degenerate constant −0.15" is STALE.** 3000 random Franka configs:
     min −0.075 / max +0.032 / **std 0.029 / 809 unique values** — comparable to UR5 (std 0.025).
     −0.15 was the *old* link-radii era; radii were already cut (0.08→0.05…, see `franka_panda_spec`
     docstring). The phase-1 "degenerate" print only fires on `std<1e-9`, no longer true. **NOT
     degenerate.** (Fix#1 = correct the record, done here + README.)
  2. **The "Raw's arena = redundant Franka null-space" thesis is DEAD — self-collision is
     STRUCTURALLY PINNED.** Elbow pair (2,4) is the argmin **88%** of the time (clearance set by q4).
     For a fixed target, 30 distinct IK solutions (**3.2 rad** mean pairwise) have `min_self` spread
     of only **0.004**, and **projected null-space ascent on `min_self` = +0.000 gain**. UR5
     (non-redundant) has **0.057** spread — but from *discrete branches* (multi-start), not null
     space. ⇒ **the 7th DOF buys ZERO clearance headroom under the capsule proxy.** Likely cause:
     thin-capsule (r≈15–50 mm) → collision is elbow-dominated. A real mesh (sim) is the only fair
     redundancy test, but under the metric we score on there is **no Franka win to unlock.**
  3. **Both worktree V4 variants measured (UR5+Franka cluttered, N=40, identical targets) → DUDS.**
     `aa1e92` (`_collision_free_seed`): UR5 +0.0110→+0.0096 (worse tail), Franka succ 100→98% (lost
     one), clash-free unchanged. `ae33b6` (`_null_space_collision_resolve`, 58 lines): UR5
     byte-identical, Franka −0.0418→−0.0417 = **no-op**, independently re-confirming (2). Headline:
     **0% clash-free on Franka cluttered for ALL builds** (near-collision targets + pinned geometry).
     **Land neither.**
  4. **Fix#2** (bead-origin `E_LJ` vs capsule-surface `min_self`) is real but already handled at
     *selection* (Entry 15); given (2) it cannot manufacture a Franka win. Deprioritized.
     **Upshot: the v5/raw decision is now evidence-based — the phantom "redundant Franka" hope that
     kept Raw's thesis alive is gone. Raw's measured quality edge (UR5 cluttered tie) comes from
     multi-start branch enumeration + hard clash-free selection, NOT the Langevin fold.**
- **Entry 18** — **Paper-framing pivot: V6 reclassified as a documented negative result (no code
  change).** User confirmed V5 & Raw "don't truly work" as positive results → paper leads with
  **V1/V4 (positive)** and treats **V5/V6 as negative/side results.** Key strategic move: our
  *original* spectrum thesis in `research_direction.md` ("deeper biological grounding → more
  measurable benefit") is **falsified by our own data** — benefit *saturates* at the shallow
  architectural level. Recommended reframe → **"Staging, not physics":** bio-inspiration transfers
  to IK at the *search-architecture/sequencing* level (metric-agnostic), but deeper *physics*
  transplants (V5 conflict-homotopy, V6 Langevin energy) give diminishing/negative returns under
  standard IK metrics. V6's role in the paper is TWO separable findings, kept distinct:
  (1) **redundancy** — the measured collision "win" traces to multi-start branch enumeration + hard
  clash-free (Pauli) selection, NOT the Langevin dynamics (fold shapes trajectory, not output;
  Entries 14–15); (2) **measurement-boundary** — the central "biophysical energy → better *quality*"
  claim is **not demonstrable** on the capsule proxy (min-F ≠ max-clearance) and the only redundant
  arm is structurally elbow-pinned (Entry 17), so the honest verdict is *"untestable under this
  proxy,"* not *"useless"* → fair test needs the mesh-collision oracle (`sim_migration_plan.md`).
  Both negatives are *mechanistically explained* → publishable as negative results, not
  embarrassments; they are the load-bearing evidence for the saturation claim, not an appendix.
  Retire the "From Structure to Physics: A Spectrum" title (now oversells). Open fork surfaced to
  user: **(A)** V1/V4 solver paper + compact "what didn't work" coda, vs **(B)** depth-vs-benefit as
  the central thesis with V5/V6 negatives as main evidence — recommended **(B)**. Awaiting choice.
- **Entry 19** — **Deliverables split LOCKED — TWO documents (no code change).** User chose: (1) a
  **research paper on V1 + V4** (positive artifact: folding-staged fold + kinetic-partitioning
  ensemble; honestly competitive w/ TRAC-IK on the easy regime + collision edge vs a strong field),
  with **V5/V6 only as a short "glimpse" subsection** pointing to (2) a **full technical report of
  the whole project** that goes deep on V5/V6 and houses all negatives + mechanism. **Consequence
  named:** the paper's thesis is now the *artifact*, NOT the "staging-not-physics" depth-vs-benefit
  inversion — that inversion needs the negatives central, so it lives in the REPORT, not the paper.
  V6 therefore appears twice: one honest paragraph in the paper (glimpse), a full chapter in the
  report (5-term energy, phase experiments, the two negatives — fold-redundant-with-selection +
  measurement-boundary, pinned redundancy, sim-migration). **Boundary rule to prevent redundancy:**
  paper = claim + evidence for V1/V4; report = complete arc + every negative + all mechanism. Report
  is ~80% assembly of existing docs (research_direction / fast_optimization / raw_* / v5 reports /
  this changelog) under a coherent spine. Recommended draft order: paper first (forces the crisp
  claim). Awaiting user: paper format/venue + which doc to draft first.
- **Entry 20** — **Deep-research pass COMPLETE → `research_notes/` written (6 parallel agents).**
  Consolidated exact mechanism + numbers + `file:line` for V1, V4, V5, V6, results-CSVs, and shared
  core/baselines/collision-proxy. **CRITICAL data finding that gates the V1/V4 paper:** the only
  committed results file `backend/master_benchmark_results.csv` has **ONLY Franka Panda**, 7 solvers
  (no UR5, no Planar, no V5/V6), N=300. Under that data — (a) **V1 does NOT beat production baselines**
  (V1 72–81% vs TRAC-IK 91–95%, Multi-start 82–86%; V1 only beats the simple Jacobian/CCD/FABRIK at
  16–28%); (b) **V4 leads success (98–99.7%) but is 7–12× SLOWER than TRAC-IK** (194–320 ms vs
  26–30 ms) — the "competitive on speed / ~9–14 ms" claim is `fast_optimization.md` **UR5** numbers
  that are **not in any committed CSV**; (c) **the self-collision edge is UNSUPPORTED by the master
  CSV** — every solver collides 65–99%, V4 ≈ or worse than simple baselines. ⇒ the paper's two
  intended positives (speed-competitive + collision edge) rest on UR5 numbers with no committed
  result file. **ACTION before drafting: re-run master benchmark on UR5+Planar (+V5/V6), commit the
  CSV — OR scope the paper's claims to what the Franka data actually supports.** Also logged a stack
  of doc/code drift (V5 README energy has a spurious `(1−λ)`; conflict range is [0,2] not [−1,1];
  V4 bit-identity only *tested* on UR5+Planar×500, not "9000 configs/3 arms"; V1 Stage-4 escalates to
  a full global reseed; V1 Stage-3 is greedy accept-if-better, NOT Metropolis; V1 `q_neutral=0` is
  outside Franka's q4 range). Full inventory → `research_notes/07_discrepancies.md`.
- **Entry 21** — **Paper claim set LOCKED by user + fresh full benchmark launched (V1/V4 only).** Honest
  claims for the paper: **V1 beats ONLY the simple baselines** (Jacobian/CCD/FABRIK); **V4 is on par with
  TRAC-IK on speed with an EDGE on success rate + self-collision rate** — that is the headline. User also
  wants a **V4 speed micro-opt before writing** (after benchmarks). Smoke + early bg results confirm the
  story is **robot-dependent**: UR5 open_space (N=2, warm) V4 = **3.2 ms vs TRAC-IK 4.5 ms** both 100%;
  Planar open_space (N=300) V4 = **100% / 4.0% collide** vs TRAC-IK **99.3% / 9.0%** at 12.3 vs 6.7 ms —
  V4's edge on success + collision is real; the committed **Franka-only** CSV was V4's WORST arm (194–320 ms),
  which is why the earlier data looked bad. Launched deep full sweep `master_benchmark.py` N=300
  (trials=100 × seeds{1,2,3}) across **planar3dof+ur5+franka × {open,near_singular,cluttered}**, solvers =
  6 baselines + V1 + V4 + analytical(planar), **EXCLUDING V5/V6** → `backend/v1v4_full_benchmark.{csv,md}`
  (bg task bjzoe38yy). Env: python 3.13.14 / numpy 2.5.0 (no `py` launcher — use `python`). Also wrote full
  research notes `research_notes/` (00–07). Next: review full numbers with user → then V4 speed opt.
- **Entry 22** — **Two research forks run to rescue V5/Raw *beyond* IK → BOTH NULL** (new subfolder
  `research_forks/`; scripts in `scratchpad/forkA,forkB/`; no tracked code changed). User asked "what
  else can the novel tech do." Ran two falsifiable forks in parallel (2 subagents):
  **Fork A — redundant-robot arena:** does the folding physics do real work on a *genuinely*
  redundant arm (high-DOF planar), where Franka's 1 spare DOF couldn't? **NULL.** Clearance headroom
  *shrinks* with redundancy (planar chains are space-filling: clash-free-solution fraction 22%→0% as
  n=6→20; null-space climb gain +0.000 for all n≥9). Raw beats V4 intramurally but **loses to plain
  multi-start+clash-free selection at every DOF**, and at equal wall-time loses decisively (Raw
  10–100× slower: 2.0 s→19.6 s→129.9 s at n=6/12/20). Both ends of the redundancy spectrum
  (Franka too-few-DOF; planar-20 space-filling) hit the same null.
  **Fork B — difficulty diagnostics:** do V5's conflict-integral and Raw's Σ predict per-instance
  hardness label-free? **NULL** (UR5, n=42; Franka never finished). Spearman vs measured difficulty
  (K=40-restart failure fraction + iters): V5 difficulty ρ=+0.12 (p=.45), V5 conflict +0.05, Raw Σ
  +0.06 (p=.7) — all indistinguishable from 0, within-scenario ≈0 too; **manipulability baseline
  (ρ=−0.27) beats both** at ⅕ the compute. V5-difficulty ⟂ Σ (ρ=−0.015, genuinely orthogonal) but
  both predict nothing → hollow complementarity; V5's own two diagnostics redundant (+0.65).
  **Significance:** closes the **last two escape hatches** for V5/V6 (redundant-arm arena +
  difficulty-instrument reframe). Consistent with & reinforces Entry 18's *saturation* thesis and
  Entry 19's plan — these forks are **load-bearing negative evidence for the technical report**, not
  the paper. Docs: `research_forks/{README,forkA_redundant_robots,forkB_difficulty_diagnostics,
  what_can_be_done}.md`. Net project verdict unchanged and now over-determined: **V1/V4 are the
  positives; V5/V6 are mechanistically-explained negatives.**
- **Entry 23** — **Full 3-arm benchmark COMPLETE** (`backend/v1v4_full_benchmark.{csv,md}`, N=300, V5/V6
  excluded). Confirms the claim set. **Success: V4 beats TRAC-IK in ALL 9 cells** (98–100% vs 91–99%).
  **Collision edge REAL on planar+UR5**, marginal on Franka (proxy elbow-pinned, everyone 65–99%): UR5 open
  collide 3.0% vs 17.3%; UR5 cluttered 19.0% vs 45.7% with V4 clearance **+0.0077 (clash-free)** vs TRAC
  **−0.0127 (colliding)**; planar cluttered V4 **+0.0196** vs −0.0056. **Speed: V4 median (p50) is ~2×
  FASTER than TRAC-IK on UR5** (3.0/4.2/4.8 vs 7.2/9.8/7.1 ms); the MEAN is on-par-to-1.7×-slower due to a
  TAIL (p95 38–151, p99 215–278). **Franka is the problem arm** — V4 7–10× slower even at p50 (122/109/192
  ms). **V1 confirmed beats ONLY simple baselines** (UR5 open 94% vs Jac/CCD/FABRIK 44–52%; trails TRAC 99%
  + Multi 97%). ⇒ **V4 speed-opt target = the LATENCY TAIL (p95/p99), esp. Franka** — matches
  `fast_optimization.md`'s "tail, not per-step" diagnosis; must NOT lose 98–100% success or the collision
  edge (the trap that collapsed Franka to 71.7% when replicas were capped).
- **Entry 24** — **Franka-collision diagnosis + V4 speed-opt fork** (branch `v4-speed-opt` @
  `C:/Users/subik/v4opt`). **Franka's high collision rate = OUR capsule proxy, NOT the arm, NOT V4.**
  Evidence: EVERY solver collides 65–99% on Franka incl. TRAC-IK (75% open / 99% cluttered, negative mean
  clearance) — a production solver doesn't really self-collide on ¾ of random poses; the same proxy
  discriminates fine on UR5 (V4 3% vs TRAC 17%). Root: elbow pair (2,4) is argmin 88%, clearance set by
  fixed `a[3]=0.0825`+q4; radii hand-tuned 0.08→0.05 (not CAD). The SAME defect also makes V4 slow on
  Franka (fast-path exit needs clash-free `d≥0`, structurally unreachable → V4 burns full budget hunting).
  **Decision: do NOT "fix" Franka for the paper** (retuning a metric to get prettier numbers is dishonest) —
  scope the collision-edge claim to planar+UR5, report Franka as a measurement limitation; the real fix is
  the mesh oracle (`sim_migration_plan.md`). **V4-opt experiment:** new registered solver
  `protein_fast_opt` = **warm-start Phase B from Phase A's best converged (clashing) candidate + shortened
  coarse collapse** (the tail fold escapes the clash instead of re-reaching the target); random-seed folds
  kept for collision diversity. **Quick N=40 (ur5+franka): opt faster mean in all 6 cells, tail cut** (ur5
  near p95 118→47, cluttered 43→20; franka cluttered mean 226→159, −30%), **success identical**, collision
  within N=40 noise (slight near-singular wiggle to confirm). Launched **N=300 all-3-arm validation**
  (bg `b2kj1xkxr` → `backend/opt_full.{csv,md}`). Gate to land: opt must keep base's success AND collision
  while cutting latency. Only report a "+" to the user if it clears that gate.
- **Entry 25** — **Faithfulness principle for V4 opt + N=300 verdict on warm-start.** User's key question:
  V1→V4 was math-opt or biologically-faithful, and V4→V4-new must be faithful (not a gimmick). Framing
  logged: **V1→V4 = TWO layers, honestly split** — Layer 1 (barrierless-first / kinetic partitioning +
  GroEL-gated rescue) is biologically FAITHFUL and delivered the real wins (behavior-changing, 1.1–4.3×);
  Layer 2 (allocation-light FK) is pure ENGINEERING, bit-identical, labeled as such (alone only 1.1–1.4×).
  **Principle for V4→V4-new: two legitimate lanes — (A) faithful mechanism change, or (B) bit-identical
  engineering speedup — NEVER a behavior-changing trick that trades away success/collision** (that's the
  rejected naive tail-edit). **N=300 verdict on the warm-start opt (planar+ur5 done, the collision-relevant
  arms):** real SPEED win — p95 tail cut 25–40% (ur5 near 155→51, ur5 cluttered 101→43, planar near 231→173,
  cluttered 240→159), faster mean in ~all cells, **success identical**. BUT **collision is consistently
  ~1–2 pp WORSE in ALL cells** (ur5 open 3.0→3.3, near 9.7→12.0, cluttered 19→21; planar open 4→5,
  cluttered 49.7→51.3; franka open 72→73) — 6/6 same direction ⇒ **real erosion, not noise** (p≈0.016).
  Cause = warm-starting Phase B from the trapped/clashing intermediate shrinks the multi-basin chaperone
  search, which IS the biological source of the collision edge. So warm-start-in-place = real speed but
  **drifts toward a trick; fails the "keep the wins" bar → do NOT land as-is.** Two honest paths instead:
  **(B)** bit-identical speedup of the fold's hot loop (self-collision distance recomputed per Metropolis
  candidate = the dominant cost) — provably can't change success/collision; **(A)** redo warm-start as a
  proper iterative-annealing **partial unfold** (stochastic kick up the funnel + re-anneal, as GroEL/IAM
  actually works) to preserve re-exploration/diversity. Recommend (B) first (safe), then (A) as the
  paper-worthy faithful mechanism.
- **Entry 26** — **N=300 3-way verdict (base V4 vs o1 warm-start vs o2 IAM partial-unfold)** →
  `backend/opt2_full.{csv,md}`. **Both warm-start variants trade ~1–2 pp collision for ~20–35% speed,
  and the FAITHFUL o2 did NOT fix the erosion** — o2's collision ≈ o1's, worse-or-equal to base in every
  cell, never better (ur5 near 9.7→12.0→12.3; ur5 cluttered 19→21→20; planar open 4→5→5.7). Success
  identical all three (100% planar/ur5; 99.7/98.0/98.3 franka). Speed: o1/o2 both ~20–35% faster + big
  tail cuts (o1 generally fastest; franka cluttered base 248→o1 176→o2 197). **Root cause confirmed
  FUNDAMENTAL:** the speed idea (skip from-scratch Phase-B restarts) and the collision edge (multiple
  independent from-scratch routes) are the SAME lever pulled opposite ways — any warm-start cuts basin
  diversity. Profile (base V4 franka cluttered): cost is SPREAD (pose_error ~17%, FK prims ~20%,
  self-collision ~15%, joint_limit ~5%) → a bit-identical micro-opt would net only ~1.1–1.3×. Gave user a
  4-option menu: (1) keep base V4 (collision pristine) + report o1/o2 as honest speed/quality tradeoff;
  (2) ship warm-start as a disclosed "fast mode"; (3) collision-SAFE lane = bit-identical + Phase-A bail
  (smaller, zero cost); (4) **sim migration (real mesh collision) = highest value** — measures collision
  honestly on all arms, tests if V4's edge is real or a proxy artifact. Recommended (1)+(4). Base V4
  untouched; variants live only in fork `v4-speed-opt`. **Awaiting user decision.**
- **Entry 27** — **Story arc LOCKED + pivot to sim migration (user continuing in a NEW chat).**
  Paper story arc confirmed: **V1 (origin) → V4 base (star) → o2 (ONE light honest paragraph: speed/quality
  tradeoff, faithful IAM couldn't beat it) → V5/V6 minimal.** V4-opt decision: **do NOT land o1/o2** as the
  default — collision edge > 25% speed; base V4 stays the paper's star (fork `v4-speed-opt` kept for the
  record). **Next active task = the sim migration.** Updated `sim_migration_plan.md` with a
  "Kickoff — START HERE (2026-07-06)" section tying it to our findings: the collision edge is measured only
  vs our capsule proxy, which is **degenerate/elbow-pinned on Franka**, so real mesh collision
  (PyBullet → MuJoCo) is the paper's **most important open validation** — is V4's edge real or a proxy
  artifact (Phase 3, the headline)? First step = **Phase-1 UR5 FK-parity test in PyBullet**. For cross-chat
  continuity, wrote project memory `proteinik-paper-and-sim-migration` + added it to `MEMORY.md` so a fresh
  session picks up the state. Base V4 untouched; nothing landed.
- **Entry 28** — **Reconsidering the V4 + o2 merge** (user: "the collision diff is only a little — is it
  better to merge?"). Correct, and stronger than "little": the ~1–2 pp erosion is only vs BASE V4 — **o2
  keeps essentially the FULL collision edge over the actual competitor TRAC-IK** (ur5 cluttered o2 20.0% vs
  TRAC 45.7%; ur5 near 12.3 vs 27.0; ur5 open 3.3 vs 17.3 — o2 collides at ~½ TRAC's rate; planar cluttered
  51.3 vs 68.3). o2 is also biologically faithful (IAM), success-identical, ~25% faster + tails cut. ⇒
  **merging V4+o2 is DEFENSIBLE and honest, not a gimmick.** ONE caveat: we'd be finalizing a 1–2 pp call
  against the capsule proxy we're about to replace with real mesh collision — so ideally **decide POST-SIM**
  (does the edge survive real collision?). If moving now, merge **o2 (faithful), not o1**. Recommendation:
  hold ~1 wk for the sim if willing (free certainty); else merging o2 now is acceptable. **Not landed —
  user deciding (likely in the new chat).** [Supersedes Entry 27's "do NOT land o1/o2" as the firm default:
  o2 is now a live merge candidate, pending the post-sim vs merge-now choice.]
- **Entry 29** — **Decision LOCKED: build the sim FIRST, then decide the o2 merge on real-collision data.**
  User committed to the sequencing (sim → then the merge call). **No merge now; base V4 stays until the sim
  adjudicates** whether the collision edge is real under mesh collision. Next chat = start the sim migration
  at **Phase 1 (UR5 FK-parity in PyBullet)**. Continuity in place: `sim_migration_plan.md` Kickoff section +
  project memory `proteinik-paper-and-sim-migration` + `research_notes/` + this log.
- **Entry 13** — **Benchmark (10 solvers × 3 arms × {open,cluttered}, N=6-10).** Honest verdict:
  the protein family's collision edge over collision-blind baselines is REAL (UR5 cluttered:
  protein −0.010…+0.013 min_self / 20-40% collide, vs baselines −0.03…−0.05 / 60-80%). But **Raw
  (V6) does NOT beat V1/V4 on quality** — on UR5 cluttered, V4 edges Raw on collision (−0.0036/30%
  vs −0.0097/40%) AND is ~200× faster. This answers `research_direction.md`'s open question
  ("biophysical energy > V4 quality?") as **not clearly** — staging captures most of the benefit.
  Caveat: judged by the capsule proxy, which is **degenerate on Franka** (can't measure Raw's
  best case = redundant null-space collision-min). The quality verdict rests on a weak oracle.
