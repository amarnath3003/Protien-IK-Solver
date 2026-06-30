# Raw (V6) — Mechanism & Math (folding-faithful spec)

**Governing principle (locked):** *exactly follow the protein-folding mechanism.* Rawness is
a consequence, not a separate filter — no IK solver replicates folding, so a faithful replica
is automatically novel.

**Scope of "exact":** exact at the **coarse-grained Cα level** — the level at which folding is
routinely and legitimately simulated (Honeycutt–Thirumalai; Enciso–Rey; Clementi–Onuchic).
Not all-atom. Every term below traces to a specific folding interaction. **One honest
exception:** `E_task` (the external EE target) has no folding analog — folding is target-blind.
It is the imposed boundary condition (templated folding) and is kept minimal so it does not
distort the physical landscape.

---

## 1. The arm as a coarse-grained chain

| Folding object | Raw realization |
|---|---|
| Cα beads | joint origins `pᵢ` from the FK chain |
| rigid virtual bonds (fixed length) | links — enforced **exactly** by FK (no soft bond term) |
| backbone torsions φ,ψ (only soft DOF) | joint angles `q` |
| local backbone frame | triplet geometry of `(p_{i−1}, pᵢ, p_{i+1})` |

Bead positions and the joint axes come straight from `forward_kinematics_chain`. Nothing here
is added; it is read off the existing kinematics.

---

## 2. Reduced units and the single temperature

Reduced units (`k_B = 1`, friction `γ = 1`, energy in units of `ε_H`, length in units of `σ`).
A **single temperature `T`** governs three things simultaneously — this self-consistency is
both the real physics (fluctuation–dissipation) and Raw's defining feature:

1. the weight of the entropic term in the free energy `F(q;T)`,
2. the amplitude of the Langevin thermal noise,
3. the cooling schedule.

---

## 3. The free energy (a temperature-dependent PMF)

```
F(q; T) = E_task  +  E_LJ  +  E_HB  −  T · S_conf(q)
          └─target─┘  └────── folding physics (target-blind) ──────┘
```

Running Langevin on `F` (not a bare potential) is exactly the **implicit-solvent / PMF**
approach: the integrated-out degrees of freedom (solvent ↔ null-space volume) reappear as the
explicit entropic term `−T·S_conf`.

### 3.1 `E_LJ` — van der Waals (Pauli repulsion + London attraction)

All non-adjacent bead pairs `|i−j| ≥ 2`:

```
E_LJ = Σ_{j>i+1} 4 εᵢⱼ [ (σᵢⱼ/dᵢⱼ)¹² − (σᵢⱼ/dᵢⱼ)⁶ ],   dᵢⱼ = ‖pᵢ − pⱼ‖
```

- `σᵢⱼ = s·(rᵢ + rⱼ)`, global scale `s` calibrated per robot; **uniform `ε`** (non-Gō — structure
  must emerge, not be planted).
- The retained `−(σ/d)⁶` attraction (well minimum at `dᵢⱼ = 2^{1/6}σᵢⱼ`) is the part with no IK
  equivalent. **Folding role:** core packing / steric exclusion; emergent preferred inter-link
  spacing = "tertiary contacts."

**Analytic force:**
```
∂E_LJ/∂q_k = Σ_pairs (dE/dd)(∂dᵢⱼ/∂q_k)
dE/dd      = (24εᵢⱼ/dᵢⱼ)[ (σᵢⱼ/dᵢⱼ)⁶ − 2(σᵢⱼ/dᵢⱼ)¹² ]
∂dᵢⱼ/∂q_k  = ûᵢⱼ·(∂pᵢ/∂q_k − ∂pⱼ/∂q_k),  ûᵢⱼ=(pᵢ−pⱼ)/dᵢⱼ,  ∂pₘ/∂q_k = z_k×(pₘ−p_k) if k<m else 0
```

### 3.2 `E_HB` — directional hydrogen bond (corrected to the faithful vector)

Each bead carries a **local backbone normal** — the unit normal to the plane of its triplet:

```
tᵢ = normalize( (pᵢ − p_{i−1}) × (p_{i+1} − pᵢ) )        # NOT the joint axis z_i
```

Over non-adjacent pairs:

```
E_HB = −ε_hb Σ_{j>i+1} F(dᵢⱼ) · G(t̂ᵢ·r̂ᵢⱼ) · H(t̂ⱼ·r̂ᵢⱼ)
F(d)   = exp(−(d − d₀)²/2σ_d²)                            # Gaussian in distance
G,H    = exp(−κ(1 − |·|))   (or cos²)                     # angular gates → directionality
```

`d₀, σ_d, ε_hb` calibrated per robot from natural-configuration geometry (as backbone geometry
sets real H-bond geometry). **Folding role:** secondary structure — stabilizing **only** when
distance *and* relative orientation are both satisfied; this is what makes helices/sheets form.
Force by finite difference initially (analytic later).

### 3.3 `S_conf` — configurational entropy (the hydrophobic / free-energy term, corrected)

In a coarse-grained chain the favourable **solvent** entropy (the hydrophobic *collapse* drive)
is already folded into the attractive LJ contacts (§3.1). What remains as an explicit entropy is
the **chain conformational entropy** — the local accessible-conformation volume, Boltzmann
`S = k_B log Ω`. The faithful, **target-blind** estimator (the standard polymer free-volume
method — sample a local cloud, count the excluded-volume-respecting samples):

```
Ω(q) ≈ (1/m) Σ_{k=1..m}  w_lim(q+δq_k) · w_clash(q+δq_k)          # soft feasibility ∈ [0,1]
δq_k ~ 𝒩(0, ρ²I)              # FIXED cloud per step (common random numbers)
w_lim   = Π_j σ(α(q_j−lo_j))·σ(α(hi_j−q_j))     w_clash = σ(α(min_self_dist(·) − margin))
S_conf(q) = log( max(Ω(q), Ω_floor) )           # σ = sigmoid; α→∞ recovers the hard indicator
```

- **No target/tolerance condition** — folding entropy ignores any external goal. This is exactly
  what separates it from manipulability, which is task/null-space-relative.
- **Collision-aware** — counts only clash-free accessible volume (the excluded-volume entropy);
  manipulability `√det(JJᵀ)` ignores self-collision entirely. *Measured:* corr(clearance, S_conf)
  ≈ **+0.9** across all three arms, vs corr(clearance, manipulability) ≈ **0** — they are
  genuinely different quantities.
- **Folding role:** the **chain conformational entropy** — high for open/extended configs, low
  for compact / near-collision / near-limit ones. It **opposes** collapse (favours the unfolded
  ensemble) and competes with `E_LJ`. It thermodynamically avoids **clashing and near-limit
  (low-freedom)** configurations — it does **not** target singularities (that was the
  manipulability artifact removed for rawness).
- Soft (differentiable) relaxation of the hard accessible-volume indicator; gradient `∇S_conf`
  by FD with **common random numbers** (reuse the same `δq_k` cloud) so the estimate is smooth.
  `ρ, m, margin, α, Ω_floor` are calibration params.

As `T→0` the `−T·S_conf` term vanishes (enthalpy wins → folded/packed); at high `T` it dominates
(→ open, high-freedom, unfolded). That crossover **is** the folding transition.

### 3.4 `E_task` — the external boundary condition (only non-folding term)

```
E_task = w_task · ( ‖p_err‖ + 0.3·‖o_err‖ )       # reuse pose_error;  ∇E_task = −w_task·Jᵀ·err
```

Honesty: folding has no `E_task`. `w_task` is kept as small as still reaches the target, so the
solution is shaped by physics and only *pinned* by the task.

---

## 4. Dynamics — overdamped Langevin

Euler–Maruyama, pure physics (no Metropolis accept/reject; the precise endgame in §4b is the
**zero-temperature limit** of this same equation, not a separate solver):

```
∇F   = ∇E_task + ∇E_LJ + ∇E_HB − T_t·∇S_conf
q_{t+1} = clip( q_t − ∇F·Δt + √(2 T_t Δt)·ξ_t ),   ξ_t ~ 𝒩(0, Iₙ)
```

The native state is whatever the dynamics settle into. Cooling below `T_glass` without reaching
the target basin is a **measured glassy trap**, reported — not patched.

---

## 4b. Native-state consolidation — the last step (IK technique ↔ folding analogy)

The endgame is drawn **from IK** but justified **as folding**, so it is precise *and* faithful.

**Problem:** stochastic Langevin is diffusive near the minimum — it will not snap to the
`1e-3 m / 1e-2 rad` task tolerance. IK's answer is a Levenberg–Marquardt / damped-Newton endgame.

**Why that is faithful (not a bolt-on):** it is the **`T → 0` limit of the very same dynamics.**
`dq = −∇F·dt + √(2T)·dW`; as cooling drives `T → 0` the noise vanishes → deterministic flow
`dq = −∇F·dt`. Near the native minimum the basin is locally harmonic, so the natural,
quadratically-convergent form of that flow is the damped-Newton/LM step. This *is* the last phase
of real folding — **native-state consolidation**: final side-chain packing and vdW + H-bond
network locking, settling the chain into the unique native minimum.

Continue the §4 loop once `T_t` reaches `T_glass`, with noise off (the `T→0` continuation):
```
H  ≈ Jᵀ J            # task curvature near the basin (+ optional E_LJ/E_HB curvature)
dq = −(H + μI)⁻¹ ∇F  # damped Newton/LM;  μ = LM damping = staying overdamped (no overshoot)
q  ← clip(q + dq)    # iterate to tolerance
```

**Then the kinetic native-state stability gate** (Anfinsen: native = a *stable* global minimum;
this is V1's Stage 5, here derived from physics): jitter the converged `q` by small `δq` and
relax. Returns to the same basin → accept as native. Energy jumps and it escapes → it was a
knife-edge non-native point → reject and continue / re-anneal.

| Folding last phase | IK technique (inspiration) | Faithful because |
|---|---|---|
| native-state consolidation (downhill packing → unique min) | LM / damped-Newton endgame | `T→0` limit of the same overdamped Langevin |
| native-state stability (stable min, not transient) | convergence + perturbation-robustness check | jitter-and-relax confirms a true native basin |

"Exact replica" is preserved: no foreign solver is introduced — only the temperature is taken to
its physical endpoint.

---

## 5. Temperature schedule & glass transition

```
T_start > T_f   (begin unfolded);   T_t = max( T_glass, T_start · e^{−t/τ} )
T_glass ≈ σ_E / √(2 ln Ω̄)            # REM (Bryngelson); Ω̄ = effective accessible-state count
```

---

## 6. Σ — pre-solve foldability (landscape topology)

```
sample M random configs q_m;  E_m = E_LJ(q_m) + E_HB(q_m)
σ_E = std(E_m);  ΔE = mean(E_m) − E_native_proxy;  Σ = σ_E/ΔE  ( = 1/Z )
Σ < 1 → funnelled (good folder);  Σ > 1 → glassy
```

`E_native_proxy` = energy at a cheap warm-start (a few DLS/geometric-seed steps), computed once
up front. **This is the one IK-specific departure** (a protein knows its native energy; we do
not). Stated openly. Σ is reported as a novel difficulty diagnostic, measured *before* solving.

---

## 7. Mechanism mapping (what to verify — faithfulness is kinetic, not just energetic)

| Folding step | Raw realization | Emergent signature to confirm |
|---|---|---|
| Unfolded ensemble | high-`T` Langevin, `−T·S_conf` dominates | high spread, open configs |
| Hydrophobic collapse | `T` cools through `T_f`; `E_LJ` overtakes `−T·S` | compaction (link spread ↓) |
| Secondary structure | `E_HB` engages | H-bond count ↑, preferred motifs appear |
| Tertiary consolidation | low-`T` packing in LJ wells | `min_self_distance` settles high |
| Native state | global min of `F` under `E_task` | target reached, low-stress posture |
| Glassy trap | cooled below `T_glass`, not solved | correlates with `Σ > 1` |

Raw is faithful only if this **sequence emerges** — not just if the final energy is low.

---

## 8. Calibration table (fit once per robot, from geometry)

`s` (LJ scale) · `ε` · `ε_hb, d₀, σ_d, κ` (H-bond) · `ρ, m, margin` (entropy MC) ·
`T_start, τ, Δt` (dynamics) · `w_task`. To be set from each `RobotSpec`'s link radii / lengths
and natural-configuration statistics; recorded here when fixed.

---

## 9. Open items

- Analytic gradients for `E_HB` and `S_conf` (FD first).
- Entropy MC variance vs. step cost (`m`, `ρ`) — keep cheap enough for many Langevin steps.
- Per-robot calibration procedure (§8).
- Planar3dof: `min_self_dist` has only one non-adjacent pair; entropy still well-defined.
