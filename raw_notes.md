# Raw (V6) — Running Notes / Thought Process

> Living document. Updated after **every** prompt to track our evolving thinking on Raw.
> Companion docs: `raw_design.md` (term filter), `raw_audit.md` (faithfulness×rawness
> verdict), `research_direction.md` (paper spine). Math derivation lives here until promoted
> to `raw_math.md`.

---

## 1. What Raw is (the thesis)

Raw is **not** "another IK trick dressed as biology" (that was V1–V5). It is a literal,
ground-up replica of real protein folding physics, mapped onto a robot arm and then *solved
as physics*.

- V1 = biology in the **architecture** (renamed staged IK).
- V4 = same biology, **optimized math**.
- V5 = **one** folding principle (minimal frustration → conflict-controlled λ).
- **Raw = biology in the energy function itself** — rebuild what the solver minimizes from
  actual biophysical forces, each with **no existing IK equivalent**.

Correct reference class (verified science): a **coarse-grained (one-bead-per-residue),
off-lattice, implicit-solvent folding simulation** — Honeycutt–Thirumalai / Clementi–Onuchic
/ Enciso–Rey lineage — whose polymer happens to be a manipulator.

Mapping: joint origins `pᵢ` = Cα beads · links = rigid virtual bonds · joint angles `q` =
backbone torsions (the only soft DOF). Bonds/angles are enforced *exactly* by FK (stiffer
than MD, not looser).

---

## 2. Locked decisions

| Decision | Choice | Rationale |
|---|---|---|
| Solver dynamics | **Pure overdamped Langevin, NO LM/Metropolis finisher** | User: "exact deep-down replica." Native state reached by physics or not at all. |
| Build sequence | **Bio + math first, then code** | User directive. Get the physics provably right before implementing. |
| Beads | joint origins from FK chain | Cα-bead correspondence. |
| LJ ε | **uniform** (non-Gō) | Gō biases toward a known native = "tracking the answer" (rejected). Structure must *emerge*. |
| Temperature | **single self-consistent `T`** across entropy weight + Langevin noise + cooling | Faithful (fluctuation-dissipation) AND Raw's strongest distinguishing feature. |
| `E_native` for Σ | cheap warm-start proxy (DLS/geometric seed), stated openly | The one IK-specific circularity; absent in the protein case. |

---

## 3. Current free energy (the thing Raw minimizes via Langevin)

```
F(q; T) = E_task + E_LJ + E_hbond  −  T · S(q)
```

- **E_LJ** (full 6-12, with attraction), non-adjacent bead pairs:
  `Σ 4εᵢⱼ[(σᵢⱼ/dᵢⱼ)¹² − (σᵢⱼ/dᵢⱼ)⁶]`, `dᵢⱼ=‖pᵢ−pⱼ‖`, `σᵢⱼ=rᵢ+rⱼ` (global scale TBD), uniform ε.
- **E_hbond** (directional): `−ε_hb · exp(−(dᵢⱼ−d₀)²/2σ_d²) · (angular factor)`.
  ⚠️ direction vector must be the **normal to the triplet plane** `(p_{i−1},pᵢ,p_{i+1})`,
  NOT the joint axis `zᵢ` (see §5).
- **S(q)** entropy: currently `log w(q)=½log det(JJᵀ)` (manipulability) — ⚠️ **flagged as not
  raw**, see §4/§5.
- **Dynamics:** `q_{t+1}=clip(q_t − ∇F·Δt + √(2TΔt)·ξ)`, `T_t=max(T_glass, T_start·e^{−t/τ})`.
- **Σ (pre-solve difficulty):** `Σ=σ_E/ΔE = 1/Z`; `Σ<1` funnelled, `Σ>1` glassy.
  `T_glass≈σ_E/√(2 ln N)`.

Forces are FK-chain quantities (analytic for E_task via Jacobian; FD for bio terms first).

---

## 4. Rawness audit — current verdict (see `raw_audit.md` for evidence)

| Term | Faithful? | Raw (no IK equivalent)? |
|---|---|---|
| LJ attraction (emergent inter-link) | ✅ | **RAW ✓** |
| Directional H-bond | ✅ (after vector fix) | **RAW ✓** |
| `−T·log w` entropy | ✅ (hydrophobic PMF) | **NOT RAW ✗** — = manipulability-max singularity avoidance; "manipulability-as-entropy" already exists in robotics |
| Σ landscape topology | ✅ | **RAW ✓** (with `E_native` proxy caveat) |
| Langevin on PMF | ✅ | **BORDERLINE** — raw only via free-energy framing |

**Net:** Raw is substantially raw — 3 clearly-raw terms — but the **entropy term fails the
project's own filter**. It must be fixed or it reintroduces the V1 move Raw exists to avoid.

---

## 5. Corrections queued before coding

1. **H-bond direction** → triplet-plane normal of `(p_{i−1},pᵢ,p_{i+1})`, not `zᵢ`. *(faithfulness)*
2. **Entropy term** → replace bare manipulability with a **collision-aware local
   accessible-volume entropy** `S(q)=log Ω(q)` (Ω = measure of `δq` keeping EE in-tolerance
   AND collision-free). Genuine Boltzmann `S=k log Ω`, collision-aware (manipulability isn't),
   no standard IK name. *(rawness — preferred fix, "Option A")*. Alt: reframe claim around the
   thermodynamic folding **transition** (Option B), or drop the term (Option C).
3. **Σ native reference** → state warm-start proxy openly.
4. **Keep** single self-consistent `T`.

---

## 6. Open questions

- Calibration of `σ` (LJ scale), `ε_hb`, `d₀`, `T_start`, `τ` per robot — from geometry, how?
- How strongly may `E_task` tilt the landscape before Raw is "just IK again"? (the experiment)
- Does the accessible-volume entropy (Option A) stay cheap enough for many Langevin steps?
- Success metric: quality (min_self_distance, joint naturalness) over success-rate — confirmed thesis.

---

## 7. Next step

Write `raw_math.md` (formal derivation incl. the §5 corrections), then Phase 1: code `E_LJ`
+ its force and run the attractive-well-vs-repulsion-only experiment.

---

## Changelog (per prompt)

- **Entry 1** — Built full project context (read backend core, solvers, registry, frontend
  meta, scenarios). Established Raw touchpoints.
- **Entry 2** — User reframed Raw as exact folding replica. Did deep protein-folding physics
  review (forces, thermodynamics, funnel, kinetics, chaperones).
- **Entry 3** — Proposed build plan; user chose **pure Langevin** + **bio/math-first**.
- **Entry 4** — Produced the biology→math derivation (CG bead-chain framing, F(q), forces,
  PMF/entropy, Σ, Langevin SDE).
- **Entry 5** — Goal set: research deeply + check if *actually raw*. Literature audit
  (8+ sources). Verdict: 3 raw terms, entropy term FAILS filter. Wrote `raw_audit.md`.
  Created this notes file; will update each prompt going forward.
