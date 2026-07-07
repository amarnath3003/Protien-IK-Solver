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
- **Entry 30** — **"Where does Protein IK excel?" — niche positioning for the paper (from the N=300 sweep).**
  Framing: every solver in the reference table owns ONE axis (analytical=closed-form speed, DLS=real-time
  servo, FABRIK/CCD=cheapness, whole-body opt=many constraints). The axis **V4 uniquely owns = solution
  QUALITY UNDER DIFFICULTY** — the *simultaneous* triple of (a) near-guaranteed success (only solver
  98–100% in all 9 cells, incl. near-singular where TRAC dips to 91–98 and DLS/CCD collapse to 32–49%),
  (b) tightest accuracy (0.25–0.75 mm / sub-mrad vs TRAC 0.7–1.8 mm / up-to-4 mrad), and (c) self-collision
  freeness (ur5 open 3% vs TRAC 17.3%; ur5 cluttered 19% vs 45.7%; planar cluttered clash-free +0.020 vs
  TRAC penetrating −0.006). No other single solver hits all three at once. **Cost that defines the
  anti-niche:** heavy latency tail (fast p50 ~2–5 ms but tail-inflated mean; Franka 7–12× slower than
  TRAC) → NOT a hard-real-time/servo/embedded solver. **Excel niche (the table's missing row):** offline /
  planning-time IK (posture/grasp DBs, roadmap & waypoint seeding for cluttered redundant arms); reliability
  FALLBACK tier behind a fast production solver (solves the hard residual TRAC punts); cluttered redundant
  workspaces where self-collision is the bottleneck. One-liner: *high-reliability, collision-aware QUALITY
  solver — niche = offline/planning-time/fallback IK for redundant arms in clutter, where a clean accurate
  almost-always-successful solution beats bounded latency.* Caveats reiterated: (1) collision edge currently
  rests on planar+UR5 — Franka is the elbow-pinned capsule proxy the **sim migration** will adjudicate;
  (2) this is a **V4** claim, not the family (V1 beats only simple baselines; V5/V6 net-negative).
- **Entry 13** — **Benchmark (10 solvers × 3 arms × {open,cluttered}, N=6-10).** Honest verdict:
  the protein family's collision edge over collision-blind baselines is REAL (UR5 cluttered:
  protein −0.010…+0.013 min_self / 20-40% collide, vs baselines −0.03…−0.05 / 60-80%). But **Raw
  (V6) does NOT beat V1/V4 on quality** — on UR5 cluttered, V4 edges Raw on collision (−0.0036/30%
  vs −0.0097/40%) AND is ~200× faster. This answers `research_direction.md`'s open question
  ("biophysical energy > V4 quality?") as **not clearly** — staging captures most of the benefit.
  Caveat: judged by the capsule proxy, which is **degenerate on Franka** (can't measure Raw's
  best case = redundant null-space collision-min). The quality verdict rests on a weak oracle.
- **Entry 31** — **Sim migration Phase 0 + 1 IMPLEMENTED (PyBullet FK-parity oracle) — and it already
  caught a real model bug.** Built `backend/app/sim/` (`__init__`, `models.py`, `parity.py`, `README.md`) +
  `tests/test_sim_parity.py`. **Env reality:** PyBullet ships NO Windows wheel for any Python (checked
  cp37–cp312); on Win it always compiles from source (needs MSVC), and core backend is Py3.13. Resolution
  (user chose "install MSVC, keep PyBullet first"): installed VS2022 C++ Build Tools via winget, made a
  **separate Py3.12 venv `backend/.venv-sim`** (git-ignored) with `pybullet` (compiled from source ~5min) +
  `robot_descriptions` + numpy + pytest. Core `requirements.txt` stays pure-pip (pybullet gated behind a
  `python_version<'3.13'` marker + comment). Models pinned via `robot_descriptions`: UR5 = ur5_robot.urdf,
  Panda = panda.urdf. **Phase-0 validation PASSES:** all 7 Panda joint limits match the URDF exactly, incl.
  the prime-suspect always-negative **joint4 [-3.0718,-0.0698]**; UR5 limits match except the intentionally
  wider elbow (flagged benign). **Phase-1 FK parity (10k random configs, DH `end_effector_pose` vs PyBullet
  `getLinkState` idx4/5 = URDF link frame; detects BOTH base- and tool-side constant offsets):**
  • **UR5 = VALIDATED.** DH EE == URDF `tool0` up to a CONSTANT base offset of exactly **Rz(180°)** (the UR
  `base` vs `base_link` flip); structural residual **< 8e-7 m/rad**. Our UR5 DH IS the real robot → sim is a
  pure oracle for UR5; Phase 2 is easy there.
  • **Panda = REAL MODEL BUG, root-caused.** Current standard-DH FK is off by **~1.4 m**. `franka_panda_spec`
  actually holds the Franka **modified-DH (Craig)** table, but `forward_kinematics_chain` applies the
  **standard-DH** transform. Proof: same params through a modified-DH FK match `panda_link8` to **~6e-8 with
  IDENTITY base+tool offset** (guarded by `test_panda_modified_dh_reconciles`). ⇒ current Panda kinematics do
  **not** correspond to a real Panda; solvers "succeed" only because targets are generated by the same wrong
  FK — exactly the model-parity bug the migration was built to find. Broken FK guarded by `xfail(strict)`.
  **PAPER IMPACT:** every Franka benchmark (V4/TRAC/etc.) used this FK; fixing it (add `dh_convention` to
  RobotSpec + a modified-DH branch, or derive spec from URDF — must also re-derive the Jacobian's frame→axis
  mapping) **changes all Franka numbers**. NOT applied unilaterally — staged for a user decision (fix &
  re-run Franka / derive-from-URDF / drop Franka from sim-validated claims). Tests: 7 pass + 1 xfail in
  `.venv-sim`; core suite 108 pass + 1 skip under 3.13 (sim test skips without the optional deps). Combined
  with the known Franka **capsule-proxy degeneracy**, Franka is now doubly-suspect; **UR5 is the clean arm**
  to carry the paper's sim-validated collision claims in Phase 2/3.
- **Entry 32** — **USE-CASE experiments run (empirical "where does V4 excel", not reasoning).** User
  pushed back on prose ("do experiments... find where it truly excels"). Built
  `backend/usecase_experiments.py` (+ `.md`), 5 experiments each keyed to a real IK deployment ROLE and its
  deciding metric, reusing app kinematics/solvers. Full run 1386 s, N=200/cell (E: 120), seeds. Results
  (`usecase_results.json`): **(A) real-time servo — V4 LOSES**, Franka 74% of solves >10 ms, **max 2.5 s**,
  p99 928 ms (TRAC always <3 ms) → empirically disqualified from control loops, tail is unbounded.
  **(B) planner goal-sampler — V4 wins** usable-clean-goals/attempt 83% vs TRAC 57% vs Multi 65% (diversity
  a tie ~3.1 rad). **(C) offline clean-solve rate — V4 wins every honest cell** by +18–30 pp (ur5 open
  96.5 vs 78.5; ur5 cluttered 78.5 vs 48.5; planar cluttered 50.5 vs 30.5); Franka ~1% for ALL solvers
  (capsule-proxy artifact, excluded). **(D) fallback tier — V4 rescues 60–78%** of targets TRAC punts on
  honest arms (ur5 near_singular 77.6%, cluttered 60.2%); Franka rescue 1% = proxy. **(E) hyper-redundant
  folding (planar 4→16 DOF, cluttered) = THE headline:** both solvers ALWAYS reach the pose (solved 100%);
  the entire difference is self-collision. V4 CLEAN vs TRAC CLEAN: 4-DOF 75.8/34.2, 6 59.2/16.7, 8
  36.7/5.0, 12 11.7/0.8, 16 1.7/0.0 → edge widens 2×→15× and **past 12-DOF V4 is the ONLY solver producing
  collision-free folds.** Interpretation: the more the task resembles folding (long chain avoiding
  self-intersection), the more V4 is the only viable tool — the mechanism-honest niche, now measured.
  Caveats: self-collision-only (no env obstacles yet); Franka = proxy (sim migration replaces it); the
  latency tail IS the reason the niche is planning/offline/fallback, not a bug. This empirically converts
  Entry 30's reasoning-only claims into data. Paper-usable now (lead with UR5 + EXP E DOF-scaling curve).
- **Entry 33** — **Panda model bug (Entry 31) FIXED — user chose "rebuild Panda truest, like UR5".** Added
  `dh_convention` to `RobotSpec` + a **modified-DH (Craig)** branch in `forward_kinematics_chain` /
  `end_effector_pose`; set `franka_panda_spec(dh_convention="modified")`. The tricky part: the standard-DH
  "joint axis = chain[i] z" assumption is baked into MANY places, not just FK. Added one source-of-truth
  accessor `joint_axis_frames(spec, chain)` (modified DH: joint i axis = chain[i+1], not chain[i]) and routed
  ALL of them through it: `geometric_jacobian`, ccd, fabrik (×2), fixed_lambda `_fast_pose_jac_fl`, V5
  `_fast_pose_jac`, protein_raw `lj_energy_and_grad`. V4 (protein_fast) additionally reimplements FK
  (`_fast_chain`/`_incremental_chain`) with the DH local matrix inline — added a modified-DH branch to both.
  **Verified:** Panda FK parity → **VALIDATED** (matches URDF panda_link8 to ~6e-8, identity offset); Jacobian
  vs finite-diff clean on all 3 arms (~4e-8); every bespoke solver FK/Jacobian now **bit-identical (0.0)** to
  core for ur5/panda/planar; core suite **108 pass, 1 skip** (the 2 protein_raw analytic-grad Panda failures
  from the naive fix are gone); solvers converge on the corrected Panda (V4 8/8 @0.16mm, TRAC 8/8, V1 8/8).
  UR5 + planar FK unchanged (still standard DH). **⚠️ ALL prior Franka benchmark numbers are now STALE** (old
  standard-DH FK) and must be **re-run** before any Franka claim enters the paper/report; UR5 + planar numbers
  unaffected. Both arms now pass the parity pytest (`test_sim_parity.py`: 8 pass in `.venv-sim`). **NEXT:**
  re-run Franka benchmarks, then Phase 2 (PyBullet evaluation oracle) + Phase 3 (real-collision headline).

- **Entry 34** — **Franka bench re-run kicked off (background) + Phase 2 (PyBullet evaluation oracle) started.**
  Goal reset to "continue the migration to real sim software; run the Franka bench in the background." (1)
  Launched the full corrected-kinematics Franka sweep in the background: `master_benchmark.py --robots
  franka_panda --out franka_corrected_benchmark` (N=300/cell = trials 100 × seeds[1,2,3], all 10 solvers, 3
  scenarios) — replaces the STALE standard-DH Franka numbers flagged in Entry 33. (2) Building Phase 2 per
  plan §5: `app/sim/pybullet_backend.py` (SimBackend: load/fk/self_collision/set_config/reachable_target/
  native_ik) + `bench/sim_benchmark.py` (generates targets via the SAME `generate_target` scenarios as the
  master bench so cells are comparable, runs each solver on RobotSpec, then RE-SCORES `q_final` against sim FK
  + sim real-collision, plus a PyBullet native-IK baseline column). Key design: solver stays on our fast DH
  core; sim only touches the boundaries (target frame + scoring). Frame offset from Phase-1 parity (UR5 base
  Rz180°, Panda identity) is applied as a constant C so sim-frame scoring is consistent (`T_sim = C @ T_dh`).
  Env: entire sim bench runs in `.venv-sim` (Py3.12) — solvers need only numpy (no scipy), confirmed
  importable there. NEXT: finish backend + runner, smoke on UR5 (clean arm), then full UR5+Panda oracle report.

- **Entry 35** — **Phase 2 (PyBullet evaluation oracle) BUILT + UR5 report in — the collision edge is REAL
  but the proxy exaggerates it; V6 is the real-collision champion.** Built `app/sim/pybullet_backend.py`
  (`PyBulletBackend`: fk / self_collision via real-mesh `getClosestPoints` / set_config / reachable_target /
  native_ik / score; constant Phase-1 frame offset C re-measured + asserted <1e-4 at load = self-checking)
  and `bench/sim_benchmark.py` (mirrors master_benchmark's SAME scenario target distributions, runs each
  solver on the fast DH core, RE-SCORES q_final in PyBullet: real FK + real mesh collision, + a PyBullet
  native-IK baseline column). 8 Phase-2 pytests pass in `.venv-sim` (frame round-trip, self-consistency,
  native-IK reaches targets, offset constant). **UR5 sim-oracle report (n=150/cell, 4050 solves,
  `sim_oracle_ur5.md` + interpretation `sim_oracle_findings.md`):**
  • **(1) FK parity holds END TO END** — our_succ==sim_succ agree% = 100 on every solver/scenario (one V4
  near-singular boundary blip 100→99.3); our_pos mm == sim_pos mm to 3 dp. An independent sim confirms every
  UR5 success claim. The Phase-1 result now holds through the whole benchmark pipeline.
  • **(2) The capsule proxy is OPTIMISTIC everywhere** — sim_col > our_col in every cell, often 2-3×
  (V4 open_space 2.7%→29.3%; V4 cluttered 17.3%→56.0%). Absolute proxy collision RATES are untrustworthy;
  real rates are far higher (UR5 wrist meshes are tight). Any paper sentence quoting a proxy collision rate
  must be re-stated vs the sim.
  • **(3) V4's collision EDGE survives, but MUCH smaller.** Among high-success (>90%) practical solvers, V4
  has the LOWEST sim_col in all 3 scenarios (open 29.3 vs TRAC 32.7; near_sing 39.3 vs 48.7; cluttered 56.0
  vs 64.7) and stays clearer (cluttered mean clearance V4 -0.0165 vs TRAC -0.0302, ~½ the penetration).
  But the RATIO collapsed: proxy said V4 ~2.5-6× cleaner than TRAC, real meshes say ~1.1-1.25×. Direction of
  the claim survives; magnitude must be cut hard. PyBullet native-IK (zero collision awareness) is worst on
  collision every scenario (42/54/71%) = the clean reference the edge is measured against.
  • **(4) NEW result the proxy HID: V6 (raw biology) is the real-collision CHAMPION.** On the proxy V4≈V6
  (open 2.7/2.7, cluttered 17.3/16.0). Under real meshes V6 pulls clearly ahead with ~99-100% success:
  sim_col open 14.0 / near_sing 26.0 / cluttered 48.7 — lowest of ANY solver in all 3. The biophysical
  energy terms genuinely improve real-collision avoidance, invisible to the approximate proxy. **Partially
  REOPENS the V6 "biophysics→quality" thesis the tech report was ready to close** — V6 is the quality leader
  on a real oracle; cost is latency (~3-6 s/solve). Reframe V6 from "didn't help" to "helped QUALITY not
  SPEED, and only a real collision oracle could see it."
  ⇒ Phase 3 (real-collision headline) is essentially ANSWERED for UR5: edge is real, proxy exaggerates,
  V6>V4>baselines on real collision. Remaining: confirm ordering on Franka (redundant arm) + MuJoCo
  cross-check (Phase 4).
  **BUG CAUGHT:** first Franka master re-run COMPLETED its compute then CRASHED on a `print()` — the `λ` in
  "Fixed-λ Homotopy" can't encode to Windows cp1252 stdout under Py3.13, and results are only written AFTER
  all cells, so the whole sweep was lost (tee masked it as exit 0). Fixed durably: both `master_benchmark.py`
  and `bench/sim_benchmark.py` now `sys.stdout.reconfigure(encoding="utf-8")` at start of main(). Franka
  master re-running with the fix; Franka sim-oracle report next (solo, to avoid 2×V6 contention).

- **Entry 36** — **"Can we bring the PyBullet collision % DOWN?" — YES. Built `solve_clean` (real-collision-
  certified candidate selection).** User clarified the ask: we've migrated to PyBullet (the real authority);
  don't relabel our proxy — make the solver actually collide less IN PyBullet. Route taken (in-migration:
  PyBullet is the collision judge, used only at the boundary, never in the solver loop):
  • **Diagnosis (`bench/collision_parity.py`, `collision_parity.md`):** config-level, the capsule proxy has
    a **~20% false-clear** on UR5 (says clear ≥0 while real meshes interpenetrate), corr 0.84 near-boundary;
    Franka corr 0.42 (proxy ~blind, deferred). ⇒ a solver optimizing the proxy CANNOT see the real collisions.
  • **Uniform radius inflation FAILS** (`bench/calibrate_radii.py`): ROC is discontinuous — false-clear
    stuck ~17-20% up to Δr=9mm, then at 10mm snaps to 0% but false-alarm explodes to 62.5% (a dominant
    always-close wrist pair). No usable operating point; the proxy geometry (thin capsules) is wrong, not
    just its scale. (This was the WRONG direction anyway — "updating our sim values"; user corrected it.)
  • **Proxy-selection FAILS, real-selection WORKS:** picking the best-*proxy*-clearance candidate slightly
    *raised* real collision (32→35%); picking best-*PyBullet*-clearance halved it. No cheating the proxy.
  • **`app/sim/clean_solve.py` (`solve_clean`)** — generate K candidates from diverse start configs (→
    different IK branches: elbow up/down, wrist flips), score each by PyBullet real mesh collision, return the
    cleanest. **V4, UR5, K=16, n=120, success stays 100%:** open_space **32.5→5.8%** (5.6×; clearance
    +0.0002→+0.0093), near_singular **42.0→24.2%** (clearance -0.0073→+0.0011, flips clear), cluttered
    **60.0→40.8%** (penetration halved). Saturates at K≈8-16 (K=36 no better); residual floor is largely
    PHYSICAL (tight targets with no collision-free IK branch). Cost ~238-486 ms/solve (K× solves + K queries)
    ⇒ OFFLINE/planning-grade — exactly V4's niche. `bench/clean_benchmark.py` + `clean_ur5.md`; 2 pytests pass.
  • **HONEST caveats (must carry into any claim):** (i) the wrapper is **NOT V4-exclusive** — under the same
    K-select TRAC-IK TIES V4 (~13% open); it *equalizes*, doesn't widen V4's edge. V4's exclusive advantage
    stays the **single-shot** rate (per-solve: 32 vs TRAC 38 open, 67 vs 80 cluttered). (ii) To make V4
    *intrinsically* clean single-shot (& widen the edge, since TRAC has NO collision term) needs a fast
    mesh-FAITHFUL collision model (fit capsules/spheres to URDF link meshes) it can optimize in-loop — bigger
    change, NOT done, offered. (iii) UR5 only; Franka proxy too weak (deferred, per user "later fix v6 dense").
  **MERGE STATUS:** feature integrated in code (app/sim/clean_solve.py + bench + tests + findings §7); NOT
  git-committed (standing rule). "merge v4" ⇒ this is an offline clean-solve MODE usable with V4 (base V4
  solver unchanged), honestly an all-solver booster, not an intrinsic V4 upgrade.

- **Entry 37** — **V7 attempt (V4 + fast mesh-faithful collision model) — TRIED, SCRAPPED with decisive
  evidence.** Goal: genuinely lower V4's REAL (PyBullet) collision and WIDEN the V4-TRAC gap by giving V4's
  collision-aware machinery (clean-fold test lines 437/470, candidate selection line 476, collision energy)
  an ACCURATE collision signal instead of the optimistic thin-capsule proxy — V4-specific because TRAC/CCD/
  native have NO collision term to benefit. Built `app/sim/mesh_collision.py`: extract each URDF link's real
  collision-mesh verts (PyBullet `getMeshData`, LOCAL frame), fit medial-axis spheres (centerline + true
  perpendicular-thickness radius; k-means-on-surface first tried but bulged elongated links → wrist over-
  collision), and compute link world frames by numpy URDF-FK (NOT the DH chain — UR5's DH intermediate frames
  do NOT rigidly coincide with URDF link frames, only the EE does; DH-frame mapping gave 1.5-2m residuals).
  URDF-FK validated to **7.9e-8 m** vs PyBullet. **BUT the model fails the fidelity gate:** vs PyBullet real
  collision on UR5, best corr **0.75 — WORSE than the old proxy's 0.83**; no operating point has both low
  false-clear and low false-alarm (scale 1.0 → false-clear 0 but false-alarm 61%/everything flags; scale 0.85
  → false-clear 20%/under-reports). **DECISIVE TEST (ranking V4's K candidates by each signal, measure REAL
  collision of the pick):** open_space single 31.7 / proxy 35.0 / **faithful 38.3** / real 13.3; cluttered
  single 66.7 / proxy 63.3 / **faithful 65.0** / real 51.7. The faithful model ranks NO BETTER than the proxy
  (≈ single-shot), nowhere near the real oracle. ⇒ a fast sphere/capsule approximation cannot reproduce
  PyBullet mesh collision with enough fidelity to guide V4 better than the proxy. **Fundamental barrier: to
  match PyBullet collision you basically need PyBullet collision (too slow for V4's inner loop)** — which is
  exactly WHY the only thing that genuinely lowers real collision is the boundary oracle (`clean_solve`,
  Entry 36), and why that's general/not-V4-specific. **DECISION: SCRAP V7.** V4's honest position stands:
  reliable + modest single-shot collision edge (32 vs TRAC 38 open, 67 vs 80 cluttered), and the collision
  edge is fidelity-limited by any fast model — itself an honest paper finding. `mesh_collision.py` kept as a
  shelved experimental artifact (FK + mesh extraction are correct/reusable). Untested last variant: faithful
  model inside the FOLD energy (not just ranking) — very unlikely to help given the model can't even rank;
  offered to user. **STRATEGIC (user Q): benchmark in PyBullet-primary = right; keep our DH-FK as a fast
  validated cross-check, drop our collision proxy for the metric, add MuJoCo (Phase 4) as a 2nd oracle.**

- **Entry 38** — **Phase 3 completed FULLY + Phase 4 (MuJoCo second oracle) built.** Goal pivot (user): stop
  trying to lower V4's PyBullet collision (V7 scrapped, Entry 37) — KEEP the honest modest PyBullet results —
  and finish Phases 3 & 4 so the sim-migration is done. 
  **PHASE 3 (proxy vs real mesh) — made paper-grade with a per-link-pair MECHANISM.** collision_parity.py now
  attributes each real collision (and each dangerous false-clear) to the link PAIR whose meshes are closest
  (added PyBulletBackend.self_collision_detail → argmin pair). Result (n=3000): UR5 real 36.5% vs proxy 16.9%,
  false-clear 20.2%; **73% of false-clears are forearm_link|wrist_2_link** (+16% forearm|wrist_3) — the proxy's
  optimism is NOT diffuse, it's localized to the tight forearm↔wrist cluster where a thin joint-axis capsule
  can't represent the bulky link mesh. Franka: real 9.9% vs proxy 0.5%, **73% of false-clears = panda_link5|
  panda_link7** (same wrist-cluster mechanism; explains the weak corr 0.51). calibrated δ = none≤0.15m for both
  → no safety margin rescues the proxy (consistent w/ V7 scrap: you can't cheaply approximate mesh collision).
  **PHASE 4 (MuJoCo) — the decisive design choice: load the IDENTICAL URDF PyBullet uses** (classic UR5
  ur5_robot.urdf + franka_ros panda.urdf via robot_descriptions), NOT Menagerie ur5e/panda — so it's a pure
  engine-vs-engine comparison on the exact model our DH validated against. Built app/sim/mujoco_backend.py
  mirroring PyBulletBackend (fk/self_collision/set_config/score + compute_parity self-check). Key hurdles solved:
  (1) MuJoCo can't resolve package:// URIs & chokes on .dae visuals → _mujoco_urdf preprocessor rewrites mesh
  paths absolute, strips <visual>, injects <mujoco><compiler discardvisual balanceinertia fusestatic=false/>.
  (2) fusestatic=false keeps fixed-joint links (tool0/panda_link8) as bodies so the EE frame exists for parity.
  (3) read link frames from data.xmat (rotation matrix) NOT xquat → sidesteps MuJoCo's wxyz vs our xyzw (plan
  risk #2). (4) mj_geomDistance (MuJoCo 3.x) = exact analog of PyBullet getClosestPoints (signed, neg=penetrate).
  **THREE-WAY FK AGREEMENT (self-checked at construction):** MuJoCo independently reproduces PyBullet's parity —
  UR5 same base Rz(180°) offset, residual 8.9e-12m/4.7e-8rad; Franka identity offset, residual 8.5e-16m. DH ≡
  PyBullet ≡ MuJoCo to float noise → an independent 2nd engine re-confirms the corrected modified-DH Panda.
  **COLLISION CROSS-CHECK — fairness bug found & fixed:** MuJoCo first reported 56% vs PB 36% on UR5. Root cause:
  PB's collision set is "{URDF root -1} + revolute children"; UR5's root is the geomless 'world' so PB never
  checks base_link's mesh, but my MJ used base_link (5 phantom pairs). Franka matches (both use panda_link0,
  geom-bearing). Fix: MuJoCoBackend takes collision_link_names; crosscheck passes PB's EXACT names → identical
  meaningful pairs (UR5 10⊆16 [6 geomless no-ops], Franka 21=21). After fix: UR5 proxy 18% ≪ PB 38.3% ≈ MJ 36.1%,
  **sign-agree 97.8%, corr 0.991**; Franka PB 8.6% ≈ MJ 8.1%. Both engines agree the proxy is optimistic → Phase-3
  finding is ENGINE-INDEPENDENT. **SOLVER-EDGE REPLICATION (the money result):** scoring each solver's q_final in
  BOTH engines, the V4<TRAC<Multi<V1 collision ordering on UR5 holds identically on MuJoCo (open_space V4 31/TRAC
  34/Multi 35/V1 40 on PB; MJ tracks within 1%, col-call agree 99%). The paper's comparative claim survives a 2nd
  independent simulator. Files: app/sim/mujoco_backend.py, bench/sim_crosscheck.py → sim_crosscheck.md/.csv.
  Absolute rates are engine-dependent (convex-hull treatment) but ORDERING is not — the honest framing.
