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

## 7. Next step

`raw_math.md` is written and folding-faithful (incl. §4b last step). Next: **Phase 1** — code
`E_LJ` + its analytic force and run the attractive-well-vs-repulsion-only experiment
(measure emergent inter-link spacing and `min_self_distance`).

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
