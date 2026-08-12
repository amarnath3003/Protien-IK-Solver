# V4 — ProteinIK Fast (barrierless-first / kinetic-partitioning ensemble)

> Source: `backend/app/solvers/protein_fast/solver.py` (505 lines), `__init__.py`,
> `fast_optimization.md`, `backend/tests/test_backend.py:207-269`, `README.md:87-99`.

## 1. The barrierless-first ensemble (kinetic partitioning)

Biological framing (docstring `solver.py:16-42`): folding's **kinetic partitioning** splits a population into a
fast fraction that descends a smooth funnel directly to the native state (barrierless / "downhill", no search)
and a slow fraction that is kinetically trapped and needs an activated search with chaperone (GroEL/IAM) rescue.

Two-phase schedule in `solve_protein_fast` (`solver.py:357-504`):

- **Phase A — barrierless folding restarts** (`solver.py:411-430`). Each replica first runs a cheap
  Levenberg–Marquardt fold `_lm_polish_fast(…, 30, …)` (max 30 LM steps). `seed=q0` for `r==0`, else
  `spec.random_config(rng)`.
- **Phase B — stochastic funnel folding** (`solver.py:432-463`). The full staged fold `_fold_once(...)`
  (Metropolis funnel + chaperone rescue + collision energy) fires **only if Phase A produced no sterically
  clean solution.**

**Escalation ("frustration") criterion** — `_have_clean()` (`solver.py:408-409`): `any(d>=0.0 for d,_ in
converged_candidates)`. A candidate enters the pool only when its LM fold *converged*, at which point its
clearance `d=self_collision_min_distance(spec,q_lm)` is measured. Phase A breaks early **only on a clean fold**
(`d>=0.0`, `solver.py:429-430`). Escalation to Phase B = `if not _have_clean()`. So a landscape is "frustrated"
iff, after up to `max_replicas` LM restarts, **no converged LM candidate is clash-free**. Collision-awareness
subtlety: LM minimizes pose error only (collision-blind); a converged-but-clashing LM fold is a fallback that
does *not* short-circuit Phase B (`solver.py:32-34, 438-442`).

**`max_replicas=6`** (`solver.py:369`) is the single budget bounding both loops. Phase B extra caps
(`solver.py:445`): stop as soon as a clean fold appears, or after ≤2 *collision-aware* converged folds.
(Note: Phase-A collision-blind LM successes are excluded from `phase_b_converged`.)

Minor: `solver.py:391` comment says difficulty scaling adjusts "per-replica budget," but it scales
`stage2_iters` (Phase-B fold length via `s2`), not `max_replicas`.

## 2. FK / per-step primitives (allocation-light floor)

- **`_fast_chain`** (`solver.py:101-124`): all `n` DH local transforms via vectorized assignment into a
  preallocated `(n,4,4)` buffer (no per-joint `np.array` literal), chained with `np.matmul(out=)`. Returns
  `(chain, L)` where `L[i]` is joint i's local transform, cached for reuse.
- **`_incremental_chain`** (`solver.py:127-148`): for a config differing at only joint `i`, copies frames `0..i`,
  rebuilds the single changed local, propagates the **suffix only**. Exploits that the Metropolis sweep perturbs
  one joint at a time.
- **`_fast_pose_jac`** (`solver.py:171-187`): one FK pass → fused `(EE pose, geometric Jacobian)`.
- **`_energy_from_chain`** (`solver.py:151-168`), **`_I6`** (`solver.py:87`): shared constant 6×6 identity reused
  across all DLS/LM solves.

**Bit-identity claim:** "0.0 max diff over 9000 configs across ur5/franka/planar" (`solver.py:56-59`,
`fast_optimization.md:79-81`, `README.md:95`). **⚠️ The committed tests only cover UR5+Planar ×500 each**
(`test_backend.py:209-240`, `assert np.max(np.abs(ref-fast))==0.0`) — ≈2000 comparisons, **no Franka, not 9000.**
Verify or soften.

## 3. Tail diagnosis (cost is the tail, not per-step)

Docstring `solver.py:8-14`; `fast_optimization.md:12-32`. Old Fast (UR5 open_space): mean 76 ms, p50 11.4,
p95 392 — p50 **tied** TRAC-IK (~11 vs ~10 ms), but the slowest **~10%** of solves ate **~57%** of wall time.
A per-step micro-opt cannot move that tail — **confirmed: bit-identical micro-pass = only 1.1–1.4×.**

## 4. Rejected alternatives (honesty / negatives)

`fast_optimization.md:107-124`:
1. **Bit-identical micro-opts only:** 1.1–1.4×, cannot touch the tail. Kept as Layer 2 but insufficient alone.
2. **Naive tail-edits (cap replicas / earlier bail / fewer iters), order intact:** every speed gain **lost the
   headline win.** ur5 near_singular cap-replicas=3: 100→98.3% for barely-119 ms; **franka open_space
   cap-replicas=2: success collapses to 71.7%** (mean 200 ms). "The cost is the *per-fold* Metropolis search,
   not the *number* of folds." Rejected.

## 5. Relationship to the earlier fold

There is **no "V3"** in the codebase — lineup is V1/V4/V5/V6. V4's `_fold_once` (`solver.py:225-354`) is
essentially the V1 fold (coarse collapse → Metropolis funnel + LM endgame → scoped chaperone rescue → jitter
stability gate), preserved — but now fires **only in Phase B**. **V4 is NOT numerically identical** to the
earlier fold: Layer 1 (barrierless-first schedule) *changes behavior* — on unfrustrated landscapes the full
stochastic fold is skipped entirely. Only Layer 2 (FK primitives) is bit-identical. (`solver.py:61-64`;
README.md:96-97 confirmed accurate.) **⚠️ Label collision:** `fast_optimization.md:85-94` calls the *old* solver
"V4"; README calls the *new* barrierless one V4.

## 6. Measured performance (with sources — note UR5 numbers are NOT in a committed CSV)

| Claim | Value | Conditions | Source |
| :-- | :-- | :-- | :-- |
| Mean/tail latency cut | 1.1–4.3× (table best 4.4×) | UR5/Franka/Planar | `solver.py:64`, `fast_optimization.md:89` |
| Consolidated UR5 latency | ~9–14 ms mean, p50 ~3 ms | UR5 open_space | `fast_optimization.md:96`, `README.md:97` |
| Old Fast tail | mean 76 / p50 11.4 / p95 392 ms | UR5 open_space | `fast_optimization.md:21` |
| TRAC-IK ref | mean 16 / p50 10.8 / p95 50 ms | UR5 open_space | `fast_optimization.md:22` |

Full `fast_optimization.md:83-94` table (120 trials/cell, old→Fast): ur5 open 61→14 ms (4.4×), ur5 near
100→66, ur5 cluttered 93→48, planar open 55→26, planar cluttered 101→60, franka open 458→355.
**⚠️ None of these UR5/Planar latencies appear in `master_benchmark_results.csv` (Franka-only).** See
`05_results_numbers.md` for what the committed data actually shows (V4 = 194–320 ms on Franka, 7–12× slower
than TRAC-IK). Reproduce: `python backend/master_benchmark.py`.
