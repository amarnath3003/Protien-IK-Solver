# Raw Solver (V6) — Corrected Design
## Filter: every element must have NO existing IK equivalent

---

## What was rejected from the previous design (and why)

| Rejected | Why |
|---|---|
| Ramachandran/Fourier wells | = soft joint limits. Already in constrained IK. |
| Compactness penalty | = null-space redundancy resolution. Standard robotics. |
| Gō-model target attraction | = tracking best iterate. Every solver does this. |
| Electrostatic joint coupling | = Jacobian column dot product. Already implicit in J^T J. |
| "Langevin dynamics" alone | = simulated annealing + continuous noise. Already exists. |

These are rejected. They are V1's problem at the energy-function level.

---

## What genuinely has no IK equivalent

### 1. Full pairwise LJ with ATTRACTION (not just repulsion)

Every IK self-collision model does one thing: repel when links are too close.
That is the (r^-12) term only. It's a hard wall.

In actual molecular physics, the van der Waals force has TWO components:
- Repulsion at r < σ (Pauli exclusion — atoms can't overlap)
- **ATTRACTION at r ≈ 1.1σ (London dispersion — induced dipole)**

The attraction term (r^-6) is what makes atoms STICK together at the right
distance. Proteins hold their shape because atoms sit in the attractive well,
not because they are merely pushed apart.

**No IK solver includes the attractive well. They only repel.**

For Raw: every pair of non-adjacent links has a full LJ potential:
```
E_LJ(d_ij) = 4ε [ (σ/d_ij)^12 - (σ/d_ij)^6 ]
```
The subtraction of the r^-6 term is what's new. At the right link-link
distance, the arm is ATTRACTED to hold that configuration — not because
we programmed it, but because the physics says so. This creates natural
"secondary structure" in the arm — preferred inter-link spacings that
emerge from the potential, not from hand-coded rules.

**Citation:** Lennard-Jones, J. E. (1924). On the determination of molecular
fields. Proc. Royal Society A.

---

### 2. Hydrogen Bond Analog — Directional Axis Coupling

H-bonds in proteins: specific, directional interaction between N-H (donor)
and C=O (acceptor). Two requirements: distance < 3.5Å AND angle > 120°.
This angle requirement is what makes H-bonds directional — and it's what
creates secondary structure (alpha helices, beta sheets). A helix forms
because residues i and i+4 have their N-H and C=O groups pointing at the
right distance AND angle toward each other.

**No IK concept captures directional, distance+angle coupling between
specific joint pairs. The Jacobian captures influence, not preferred geometry.**

For Raw: each joint i has a rotation axis z_i and origin p_i. Pairs of
joints (i, j) with fixed sequence separation k = j-i have a directed
interaction when their axes and lever arms satisfy the H-bond geometry:

```
d_ij = ||p_j - p_i||           (distance between joint origins)
θ_ij = arccos(z_i · z_j)       (angle between rotation axes)
φ_ij = arccos(z_i · (p_j - p_i) / d_ij)  (axis-to-lever angle)

E_hbond(i,j) = -ε_hb · exp(-(d_ij - d0)²/2σd²) · cos²(φ_ij - φ0)
```

This is stabilizing ONLY when:
- The inter-joint distance d_ij is near the preferred value d0
- The axis-to-lever angle φ_ij is near the preferred value φ0

These preferred values (d0, φ0) are determined from the robot's geometry
at natural configurations — analogous to how H-bond geometry is determined
by backbone bond lengths and angles.

**What this produces:** Certain joint-pair configurations become energetically
favored — not because we said "prefer elbow-up," but because the axis
geometry satisfies the directional coupling condition. This is robot
"secondary structure" emerging from physics.

**Citation:** Baker, E.N. & Hubbard, R.E. (1984). Hydrogen bonding in
globular proteins. Progress in Biophysics and Molecular Biology.

---

### 3. Free Energy = E - T·S(q), where S(q) = log(manipulability)

This is the only genuinely novel translation of the hydrophobic effect.

The hydrophobic effect is NOT a direct force between atoms. It is an
ENTROPIC effect mediated by the solvent. Nonpolar groups force water into
ordered low-entropy arrangements. The system lowers free energy by
clustering nonpolar groups → releasing water → increasing entropy.

The key equation: **ΔG = ΔH - TΔS**. The -TΔS term drives compaction.

For a robot arm, there is no solvent. But there IS configuration-space
entropy — the volume of joint-angle space that achieves similar task
performance. This is exactly the manipulability measure:

```
w(q) = sqrt(det(J(q) J(q)^T))
```

High w(q) = the arm has many nearby configurations that reach similar
poses = high configuration-space entropy.
Low w(q) = near-singular = few nearby reachable configurations = low entropy.

The free energy of configuration q is therefore:
```
F(q) = E_task(q) + E_LJ(q) + E_hbond(q)  -  T · log(w(q))
```

The -T·log(w(q)) term is the hydrophobic analog. It drives the solver
AWAY from singular configurations (low entropy) toward well-conditioned
configurations (high entropy) — not because we added a singularity avoidance
rule, but because free energy minimization naturally avoids low-entropy states.

**This does not exist in IK.** No IK solver minimizes free energy.
They minimize energy. The entropy term is structurally new.

**Citation:** Kauzmann, W. (1959). Some factors in the interpretation of
protein denaturation. Advances in Protein Chemistry.
Ben-Naim, A. (1980). Hydrophobic Interactions. Springer.

---

### 4. Sigma Ratio — Landscape Topology Measurement

This is the measurement framework that makes Raw a research contribution,
not just a different solver.

Bryngelson & Wolynes 1995 define the foldability of a protein sequence by:
```
Σ = σ_E / ΔE

where:
  σ_E = standard deviation of energy over random configurations
  ΔE  = (mean energy over random configs) - (energy at native state)
```

**Σ < 1** → landscape is funneled → fast folding
**Σ > 1** → landscape is glassy → kinetically trapped

This is computable before any folding attempt. It predicts whether a
sequence will fold efficiently from the landscape topology alone.

For Raw, this becomes the first IK difficulty predictor based on
landscape topology (not empirical trial):

```python
def sigma_ratio(spec, T_target, n_samples=500):
    """Measure funnel quality of E_raw landscape for this target."""
    rng = np.random.default_rng()
    energies = []
    for _ in range(n_samples):
        q = spec.random_config(rng)
        energies.append(E_raw(spec, q, T_target))
    
    E_native = E_raw(spec, q_best_known, T_target)  # best known solution
    sigma_E = np.std(energies)
    delta_E = np.mean(energies) - E_native
    return sigma_E / max(delta_E, 1e-8)  # Σ
```

If Σ < 1: target is in a well-funneled region → solve proceeds confidently.
If Σ > 1: target is in a glassy region → trigger more restarts early.

**This does not exist anywhere in IK literature.** No solver measures
landscape topology to predict difficulty. V5's difficulty_score measures
conflict DURING solve. Sigma ratio measures landscape topology BEFORE solving.
These are complementary.

**Citation:** Bryngelson, J.D. et al. (1995). Funnels, pathways, and the
energy landscape of protein folding: a synthesis. Proteins, 21(3), 167-195.

---

### 5. The Solver: Overdamped Langevin on F(q)

Not Langevin as a search algorithm. Langevin as the CORRECT PHYSICS
for a system with a free energy landscape and a thermal bath.

In protein folding, the chain doesn't run gradient descent. It is
physically connected to a thermal bath and moves according to the
forces it feels plus thermal fluctuations. This is Langevin dynamics.

For Raw, the same physics applies. The arm's configuration moves according
to the gradient of F(q) (the full free energy) plus thermal noise at
temperature T(t):

```python
def langevin_step(spec, q, T, dt):
    grad_F = gradient_of_free_energy(spec, q, T_target, T)
    noise = np.sqrt(2 * T * dt) * rng.standard_normal(spec.n_joints)
    q_new = spec.clip(q - grad_F * dt + noise)
    return q_new
```

The temperature cools according to the glass transition temperature
estimate from REM theory (Bryngelson 1987):
```
T_glass = σ_E / sqrt(2 * log(N_accessible))
T(t) = T_start * exp(-t / τ)  stopping at T_glass
```

Cooling below T_glass without solving = the system is kinetically trapped.
Trigger: compute Σ ratio again at this point. If Σ > 1 at T_glass,
the landscape is genuinely glassy for this target → need rescue.

**The difference from simulated annealing:** SA uses Metropolis criterion
(probabilistic accept/reject). Langevin uses continuous force-based dynamics.
The physical interpretation is different. The behavior near barriers is
different. Langevin is the correct physics for a system coupled to a
thermal bath. SA is an algorithm that happens to work similarly.

---

## What Raw Does NOT Include

- Soft joint limits (Fourier wells) → that's just constrained optimization
- Compactness penalty → that's redundancy resolution  
- Best-iterate tracking → every solver does this
- Jacobian column coupling → already in J^T J
- Random restarts → standard practice, not bio

---

## The Raw Solver in One Statement

Raw minimizes the FREE ENERGY of the robot's configuration:
```
F(q) = E_task + E_LJ_pairwise + E_hbond_directional - T · log(w(q))
```
using overdamped Langevin dynamics at a temperature that cools to the
glass transition temperature estimated from the sigma ratio of the
landscape.

Before solving, it measures Σ to predict difficulty.
During solving, it measures F(q) and tracks the funnel descent.
After solving, it reports Σ, mean F along trajectory, and whether
the solve resembled a funneled descent (Σ < 1) or a glassy search (Σ > 1).

**The four contributions with no IK equivalent:**
1. LJ attraction between links (not just repulsion)
2. Directional axis-coupling H-bond energy
3. Free energy = E - T·log(manipulability) (entropy term)
4. Sigma ratio landscape topology measurement

Everything else is excluded.
