# Raw (V6) — Bio/Math Faithfulness & Rawness Audit

**Purpose:** before implementing Raw, verify two things against the real literature,
per-term:

1. **Faithfulness** — does it match *actual* protein-folding physics?
2. **Rawness** — does it have **no existing IK equivalent**? (This is `raw_design.md`'s
   own filter. It is the bar V1/V4/V5 deliberately did *not* clear.)

A term that is faithful but has an IK equivalent is **not raw** — it is biology-flavoured
conventional robotics, which is exactly what the Raw filter was written to exclude.

---

## Part 1 — The biology/math, checked against sources

### 1.1 Raw's correct reference class: coarse-grained Cα bead folding

Raw is not an analogy to folding; it is a **coarse-grained (one-bead-per-residue),
off-lattice, implicit-solvent folding simulation** whose polymer is a robot arm. This is a
real, standard level of folding simulation (Honeycutt–Thirumalai 1990; Clementi/Onuchic
structure-based models; Enciso–Rey anisotropic Cα model). The arm's joint origins are the
Cα beads; the links are rigid virtual bonds; the joint angles are the backbone torsions.
**This framing is verified as legitimate science**, which is what licenses "raw down to
atoms."

### 1.2 Lennard-Jones with attraction — exact form confirmed

The anisotropic Cα model uses, between non-bonded beads:

```
U_LJ(i,j) = ε_H · S1 · [ (σ/r_ij)^12 − S2·(σ/r_ij)^6 ]
```

with **S2 = −1 for attractive pairs, repulsive otherwise**. So "the attraction is the new
part" is not a slogan — it is literally the sign toggle that distinguishes an attractive
bead pair from a steric-only one. Raw's full 6-12 with the `−(σ/r)^6` term retained is
faithful.

### 1.3 Directional hydrogen bond — faithful form, with a CORRECTION

The real anisotropic H-bond (Enciso–Rey) is:

```
U_HB = ε_HB · F(r_ij − r_HB) · G(|t_i · r̂_ij| − 1) · H(|t_j · r̂_ij| − 1)
```

- `F` = Gaussian in distance, centred at `r_HB` (≈1.35 helix / 1.25 sheet in reduced units).
- `G,H` = exponential angular factors.
- **`t_i` is the unit vector NORMAL to the plane of the consecutive bead triplet
  `(i−1, i, i+1)`** — i.e. the *local backbone geometry*, not an externally given axis.

> **Correction for Raw:** `raw_design.md` defines the H-bond direction from the **joint
> rotation axis `z_i`**. The faithful CG model instead uses the **normal to the plane of
> three consecutive joint origins** `(p_{i−1}, p_i, p_{i+1})`. To be an exact replica, Raw's
> H-bond directional vector should be that triplet-plane normal. Using `z_i` is a shortcut
> that breaks faithfulness (it couples to the actuator axis, not the chain's local shape).

### 1.4 Hydrophobic effect as a potential of mean force — basis for the entropy term

The hydrophobic interaction is a **solvent PMF**: you integrate out water and what remains
on the solute coordinates is an effective, temperature-dependent free energy whose dominant
part is `−TΔS` (solvent reorganisation). Implicit-solvent simulations run dynamics directly
on this PMF. So running Langevin on a free energy `F = E − T·S` (rather than a bare
potential) is a **faithful, standard** move — *provided* `S` is a real entropy and the same
`T` appears in the PMF, the noise, and the schedule. (Verified: implicit-solvent PMFs are
explicitly temperature-dependent; the contact-minimum depth changes with T.)

### 1.5 Frustration / foldability / glass temperature — Σ is faithful

Bryngelson–Wolynes foldability is quantified by the **energy gap vs. roughness**:

```
Z = (⟨E⟩_random − E_native) / σ_E         (folding Z-score; large = good folder)
T_f / T_g > 1                              (good folder: folds before it glasses)
T_g ≈ √( σ_E² / (2 k_B S_0) )              (REM glass temperature; S_0 = config. entropy)
```

`raw_design.md`'s `Σ = σ_E / ΔE` is **exactly `1/Z`** (with `ΔE = ⟨E⟩_random − E_native`).
`Σ < 1 ⇔ Z > 1 ⇔` funnelled. This is a faithful (reciprocal) restatement of the standard
foldability criterion, and `T_glass = σ_E/√(2 ln N)` is the REM formula with
`S_0 = ln N_accessible`. **Faithful.**

> **One IK-specific caveat:** `E_native` is *known* for a protein (it is the global
> minimum) but *unknown* in IK (it is the thing we are solving for). Σ therefore needs a
> proxy native energy — proposed: a cheap warm-start (a few DLS/geometric-seed steps),
> computed once up front. This is the single circularity in the scheme and must be stated
> honestly; it does not exist in the protein case.

### 1.6 Overdamped Langevin — the correct dynamics

Constant-temperature Langevin (reduced units, `T* = ε_H/k_B`) is the standard folding
integrator. The fluctuation–dissipation theorem ties the noise amplitude to `T`. This is
the physics, not an algorithm — distinct from Metropolis MC (accept/reject) and from
gradient descent (no noise). **Faithful.**

---

## Part 2 — Rawness verdict, per term

The real question the goal asks. Evidence = IK/robotics literature search.

| # | Raw term | Existing IK equivalent? | Verdict |
|---|---|---|---|
| 1 | **LJ attraction between links** (emergent inter-link spacing) | LJ appears in robotics only for **swarms / obstacle fields**, and virtual-spring "attraction" pulls toward a **fixed posture setpoint** — neither is a pairwise attractive *well between an arm's own links* producing emergent spacing. | **RAW ✓** |
| 2 | **Directional H-bond axis coupling** (distance + orientation between joint pairs) | No IK term couples specific joint pairs by **distance AND relative orientation**. The Jacobian encodes influence, not preferred geometry. | **RAW ✓** (once the direction vector is fixed per §1.3) |
| 3 | **Free energy `−T·log w(q)`** (manipulability as entropy) | **YES.** (a) Manipulability maximization *is* the textbook singularity-avoidance subtask in null-space redundancy resolution. (b) There is prior work **explicitly framing manipulability as an "entropy"** performance measure. The term's *effect* — steer toward well-conditioned configs, away from singular ones — is standard robotics, and `raw_design.md` itself already **rejected** "compactness = redundancy resolution." | **NOT RAW ✗** |
| 4 | **Σ ratio** (landscape-topology difficulty *before* solving) | IK difficulty is measured as **point** quantities (manipulability, condition number, distance-to-singularity) or **learned** from data — not as a sampled **funnel/roughness** statistic over the configuration ensemble. No precedent found. | **RAW ✓** (with the `E_native`-proxy caveat, §1.5) |
| 5 | **Overdamped Langevin on the PMF** | Stochastic trajectory optimization (STOMP) and simulated annealing exist; `raw_design.md` already conceded "Langevin alone ≈ SA." Distinctness rests entirely on it being **force-based continuous dynamics on a free energy** with a single self-consistent `T`. | **BORDERLINE** — raw only via the free-energy/PMF framing, not as "a stochastic solver." |

---

## Part 3 — So is Raw *actually* raw?

**Mostly yes — and far more than V1/V4/V5 — but not uniformly, and one term fails the
project's own filter.**

- **Genuinely raw (no IK equivalent):** LJ attraction, directional H-bond, Σ landscape
  topology. These three are real contributions with no precedent in the IK literature
  surveyed.
- **Borderline:** Langevin-on-PMF — defensible as physics, but only the *free-energy*
  framing separates it from existing stochastic IK.
- **Fails the filter:** **the `−T·log w(q)` entropy term.** Its behaviour is identical to
  manipulability-based singularity avoidance, and "manipulability-as-entropy" already exists
  in robotics. It is *faithful* to the hydrophobic-PMF idea but **not raw** by the
  project's definition. Keeping it as-is would reintroduce exactly the V1-style move
  (rename an existing IK technique in biological language) that Raw exists to avoid.

### What would make term #3 actually raw

The differentiator must be something with **no static-manipulability equivalent**:

- **Option A — real configurational entropy, not the Yoshikawa proxy.** Define
  `S(q) = log Ω(q)` where `Ω(q)` is the *local accessible micro-volume*: the measure of
  joint-perturbations `δq` that keep the end-effector within tolerance AND collision-free.
  This is a genuine Boltzmann `S = k log Ω`, includes the collision constraint (manipulability
  does not), and is **not** the manipulability index — it is the volume of the feasible
  null-space *cell*, which has no standard IK name.
- **Option B — make the thermodynamic coupling itself the measured claim.** The novelty is
  not "avoid singularities" but "a single bath `T` drives entropy weight, thermal noise, and
  cooling self-consistently, producing a folding transition." Then the contribution is the
  *transition* (unfolded→folded as `T` crosses `T_f`), demonstrated as an order parameter,
  not the singularity avoidance. Manipulability-max has no temperature and no transition.
- **Option C — drop it.** Keep Raw to the three clearly-raw terms (LJ + H-bond + Σ) and the
  Langevin dynamics, and do not claim an entropy term at all.

**Recommendation:** Option A. It keeps the hydrophobic/free-energy story, is provably
distinct from manipulability (it is collision-aware accessible volume, not a Jacobian
determinant), and turns the weakest link into a second genuinely-raw contribution.

---

## Part 4 — Net corrections before coding

1. **H-bond direction:** use the **triplet-plane normal** of `(p_{i−1}, p_i, p_{i+1})`, not
   the joint axis `z_i`. (Faithfulness fix.)
2. **Entropy term:** replace the bare manipulability `−T log √det(JJᵀ)` with a
   **collision-aware local accessible-volume entropy** `S(q)=log Ω(q)` (Option A), or
   reframe the claim around the thermodynamic transition (Option B). (Rawness fix.)
3. **Σ native reference:** state the warm-start `E_native` proxy openly as the one
   IK-specific departure from the protein case.
4. **Single self-consistent `T`** across entropy weight / Langevin noise / cooling — keep
   this; it is both faithful and the strongest distinguishing feature.

---

## Sources

Bio / folding physics:
- Enciso & Rey, anisotropic Cα H-bond model — https://pmc.ncbi.nlm.nih.gov/articles/PMC3474853/
- Hydrogen-bond models for folding/aggregation simulation — https://arxiv.org/pdf/1208.2177
- Coarse-grained models for protein folding & aggregation — https://link.springer.com/protocol/10.1007/978-1-62703-017-5_22
- Frustration in biomolecules (Ferreiro, Komives, Wolynes) — https://arxiv.org/pdf/1312.0867
- Glass transition temperature of protein energy landscapes — https://arxiv.org/pdf/cond-mat/0001195
- Enthalpy–entropy of hydrophobic PMF (implicit solvent) — https://pmc.ncbi.nlm.nih.gov/articles/PMC2538449/
- Free energy/enthalpy/entropy from implicit-solvent simulation — https://www.frontiersin.org/articles/10.3389/fmolb.2018.00011/full

IK / robotics (rawness checks):
- Application of the Lennard-Jones potential in modelling robot motion — https://www.researchgate.net/publication/337994457
- Entropy-based approach to manipulator performance measures — https://www.researchgate.net/publication/365272120
- Manipulability maximization in constrained IK of surgical robots — https://arxiv.org/pdf/2406.10013
- Virtual spring-damper hypothesis for redundant arms (Arimoto) — https://www.researchgate.net/publication/224635110
- Self-motion control of redundant manipulators (singularity-avoidance subtask) — https://www.researchgate.net/publication/297734751
