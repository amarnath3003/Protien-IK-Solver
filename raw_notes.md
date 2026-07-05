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
    constrained geometry frustrates collapse, and its capsule `min_self` is a **degenerate
    constant −0.15** (proxy issue, flagged — echoes `sim_migration_plan.md` Phase 3).
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
  emergent preferred spacing (UR5/Planar strong; Planar binds to negative E). Found Franka's
  capsule `min_self` is a degenerate constant −0.15. See §7 for full numbers.
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
- **Entry 13** — **Benchmark (10 solvers × 3 arms × {open,cluttered}, N=6-10).** Honest verdict:
  the protein family's collision edge over collision-blind baselines is REAL (UR5 cluttered:
  protein −0.010…+0.013 min_self / 20-40% collide, vs baselines −0.03…−0.05 / 60-80%). But **Raw
  (V6) does NOT beat V1/V4 on quality** — on UR5 cluttered, V4 edges Raw on collision (−0.0036/30%
  vs −0.0097/40%) AND is ~200× faster. This answers `research_direction.md`'s open question
  ("biophysical energy > V4 quality?") as **not clearly** — staging captures most of the benefit.
  Caveat: judged by the capsule proxy, which is **degenerate on Franka** (can't measure Raw's
  best case = redundant null-space collision-min). The quality verdict rests on a weak oracle.
