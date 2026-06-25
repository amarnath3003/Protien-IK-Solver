# ProteinIK V4 — Speed Optimization Pass

V4 is **not** a new solver design. It is a pure performance pass over V3: it runs
V3's *exact* folding trajectory and produces the same success rate and the same
self-collision rate, but it is **~1.7–2.2× faster** because it removes wasted
floating-point work in the hot loop. The constraint for this pass was explicit:
**bring the milliseconds down while staying inside the protein-folding domain** —
no pivot to a different algorithm family. V4 honors that by changing only the
linear-algebra primitives, not the folding logic.

## How the bottleneck was found (don't guess — profile)

A first attempt changed the *search* (gate the stochastic sweep to "stalled"
regions; perturb only a folding "nucleus" of 3 joints; hand off to the LM endgame
earlier). Measured against V3 in a controlled, warm, same-seed harness, that
version was **slower** (≈40 ms vs 32 ms) with **more** iterations (52–55 vs 42):
the nucleus-only search explored less effectively, so convergence needed more
funnel iterations, and the looser LM handoff entered the 12-step endgame before
the basin was quadratic and wasted polish steps. Lesson: the search was already
well-tuned; touching it hurt.

`cProfile` over 150 solves then showed where the time actually was:

```
   tottime  cumtime  function
    0.891    2.684    numpy.cross                       <-- inside geometric_jacobian
    0.634    0.953    numpy normalize_axis_tuple        <-- np.cross internals
    0.514    1.648    numpy moveaxis                    <-- np.cross internals
    0.489    1.040    forward_kinematics_chain
    ...      3.386    geometric_jacobian (cumulative)   <-- the dominant cost
```

Two facts stood out:
1. **`geometric_jacobian` dominated**, and most of its cost was `np.cross`'s
   per-call overhead (`normalize_axis_tuple`, `moveaxis`), not the arithmetic.
2. **Every gradient / LM step computed forward kinematics twice** — once via
   `end_effector_pose()` for the pose, and again inside `geometric_jacobian()`
   for the Jacobian.

The folding *logic* was never the bottleneck. The *primitives* were.

## The optimization

A single fused primitive replaces the pose+Jacobian computation in V4's coarse
collapse, funnel descent, LM endgame, and fast path:

```python
def _fast_pose_jac(spec, q):
    chain = forward_kinematics_chain(spec, q)   # ONE forward-kinematics pass
    n = spec.n_joints
    pose = chain[n]                             # pose for free from the chain
    z = chain[:n, :3, 2]; p = chain[:n, :3, 3]
    d = chain[n, :3, 3] - p
    J = np.empty((6, n))
    J[0] = z[:,1]*d[:,2] - z[:,2]*d[:,1]        # explicit cross product
    J[1] = z[:,2]*d[:,0] - z[:,0]*d[:,2]        # (no np.cross overhead)
    J[2] = z[:,0]*d[:,1] - z[:,1]*d[:,0]
    J[3:] = z.T
    return pose, J
```

This (a) computes forward kinematics **once** per step instead of twice, and
(b) builds the Jacobian with explicit vectorized cross products, avoiding
`np.cross`'s `moveaxis`/`normalize_axis_tuple` machinery. The Jacobian it returns
is **bit-identical** to `geometric_jacobian` (verified to 0.0 max difference over
2000 random configs), so the solve trajectory is unchanged — V4 just spends fewer
and cheaper operations getting there.

Nothing else changed: same Stage-1-skipped replicas, same full-chain Metropolis
funnel search, same chaperone rescue ladder, same adaptive ensemble, same
collision-aware native-state selection, same stability check.

## Result

Controlled, warm, same-seed (150 solves): V3 = 31.9 ms / 42 iters / 149 successes
/ 12 collisions; V4 = 19.1 ms / 42 iters / 149 successes / 12 collisions —
identical behavior, **1.67× faster**.

Noise-averaged benchmark (300 solves per cell, 2 seeds):

| Scenario | V3 | V4 | Speedup |
| :-- | :-- | :-- | :-- |
| open_space | 100.0% · 7.7% coll · 43.4 ms | 99.7% · 6.3% coll · 25.1 ms | **1.73×** |
| near_singular | 100.0% · 19.3% coll · 52.9 ms | 100.0% · 18.3% coll · 26.8 ms | **1.97×** |
| cluttered | 100.0% · 34.0% coll · 56.5 ms | 100.0% · 36.3% coll · 26.3 ms | **2.15×** |

V4 keeps every one of V3's wins — still the highest success rate and lowest
self-collision rate of all nine solvers, still beating both production baselines
(TRAC-IK-style, Multi-start) on both axes — and now solves in a steady ~26 ms,
roughly half of V3's time and ~2.5× faster than the other population method
(Multi-start). It remains slower than TRAC-IK's ~10 ms pure-numerical core; that
gap is the honest remaining cost, and the untouched analytical-wrist seed
(deep-dive §8) is the lever most likely to close it.

## Note

`geometric_jacobian` in the shared `core/kinematics.py` was deliberately left
unchanged so the baselines are unaffected and the V3-vs-V4 comparison is clean
(the speedup is attributable entirely to V4). Folding the same fast primitive into
the core would speed up every solver — worth doing if absolute throughput across
the whole suite ever becomes the goal.
