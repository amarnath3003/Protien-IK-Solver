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
