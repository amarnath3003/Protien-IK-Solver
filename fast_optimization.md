# ProteinIK Fast — Barrierless-First Optimization

How "ProteinIK Fast" was made *actually* fast — the fastest of the protein
lineup and, on the easy regime, competitive with (or faster than) TRAC-IK —
without leaving the protein-folding domain.

Solver: [`backend/app/solvers/protein_fast/solver.py`](backend/app/solvers/protein_fast/solver.py).
Reproduce every number below with `python backend/master_benchmark.py`.

---

## 1. Diagnosis — the problem was the tail, not the per-step cost

The earlier "Fast" was a *bit-identical* micro-optimization pass over the staged
fold (fused FK+Jacobian, fewer allocations). Profiling the result showed the
median was already fine — **its p50 tied TRAC-IK** (~11 ms vs ~10 ms on UR5) —
but the **mean was wrecked by a tail**:

| UR5 open_space | mean | p50 | p95 |
| :-- | --: | --: | --: |
| ProteinIK Fast (old) | 76 ms | 11.4 | 392 |
| TRAC-IK | 16 ms | 10.8 | 50 |

The slowest ~10% of solves consumed ~57% of all wall time. Those are the targets
where the barrierless fast-path missed and the solver fell into a full ensemble
of stochastic Metropolis folds. **A per-step micro-opt cannot move a tail like
that** — confirmed: the bit-identical pass bought only 1.1–1.4× (and on the
7-DOF Franka, the per-candidate allocation overhead of an over-clever
incremental scheme even went *negative* before being measured correctly).

So the lever is the tail: stop firing the expensive machinery when it isn't
needed.

## 2. The design — barrierless-first kinetic partitioning

Two changes, both inside the folding domain.

### (a) Barrierless-first ensemble — the tail killer

This is the **kinetic partitioning mechanism** of protein folding (Thirumalai et
al.): a folding population splits into

- a **fast-folding fraction** that descends a smooth, minimally-frustrated funnel
  directly to the native state — *barrierless / "downhill" folding* (Muñoz,
  Eaton), no search required; and
- a **slow fraction** that is kinetically trapped and reaches the native state
  only via an activated search with **chaperone (GroEL / Iterative Annealing
  Mechanism)** rescue.

So each replica **first attempts a cheap barrierless (Levenberg–Marquardt) fold**
from its seed. Only a seed whose landscape is *frustrated* (LM fails to reach a
sterically clean native state) escalates to the full stochastic Metropolis funnel
+ chaperone rescue. The cheap downhill path resolves the bulk of targets in
~TRAC-IK time; the expensive protein machinery fires only where frustration
demands it — exactly where the success and self-collision wins live.

Gating the chaperone-bearing fold behind "spontaneous folding failed" is **how
GroEL actually operates** — it rescues trapped substrates, not every molecule —
so this order is *more* biologically faithful than always running the full
machinery, not less.

A **single budget** (`max_replicas`) governs both phases — one folding-attempt
budget, applied barrierless-first. No new magic constant, scales with the
ensemble.

**Honesty.** Real folding is massively parallel (all copies fold at once); the
sequential "try barrierless, escalate on failure" here is the *computational
rendering* of that parallel partition. The mechanisms (downhill vs. activated
folding, chaperone-as-rescue, minimal frustration) are faithful; the scheduling
is engineering. And collision-aware native-state selection is preserved: a
converged-but-clashing barrierless fold is kept only as a fallback and does **not**
short-circuit the collision-seeking folds.

### (b) Allocation-light FK primitives — the per-step floor

`_fast_chain` builds all DH locals with vectorized assignment into a preallocated
buffer (no per-joint `np.array` literal) + `np.matmul(out=)`; `_incremental_chain`
rebuilds only the suffix when the Metropolis sweep perturbs one joint; `_I6` is a
shared constant identity. All verified **bit-identical** to
`core.forward_kinematics_chain` (0.0 max diff over 9000 configs across all three
arms; locked in by `tests/test_backend.py`).

## 3. Results (120 trials/cell, warm)

`protein_fast` (barrierless) vs the prior staged-fold Fast (V4) and TRAC-IK:

| Robot · scenario | Succ% (Fast) | Collide% (V4 → Fast) | Mean ms (V4 → Fast) | Speedup |
| :-- | --: | :-- | :-- | --: |
| ur5 · open_space | 100 | 4.2 → **2.5** | 61 → **14** | **4.4×** |
| ur5 · near_singular | 100 | 19.2 → **12.5** | 100 → **66** | 1.5× |
| ur5 · cluttered | 100 | 30.8 → **30.0** | 93 → **48** | 1.9× |
| planar · open_space | 100 | 6.7 → 6.7 | 55 → **26** | 2.1× |
| planar · cluttered | 100 | 55.8 → **49.2** | 101 → **60** | 1.7× |
| franka · open_space | 96.7 | (target-imposed) | 458 → **355** | 1.3× |

On UR5 open_space the consolidated solver runs ~**9–14 ms mean, p50 ~3 ms** —
matching or beating TRAC-IK's pure-numerical core while keeping 100% success and a
*lower* self-collision rate. Success and collision are held at or above the prior
Fast everywhere; the solver is strictly faster.

The speedup tracks the size of the fast-folding partition: large on smooth
landscapes (UR5/planar open_space), smaller on rugged high-DOF distributions
(Franka near_singular/cluttered) where most seeds are genuinely frustrated and
correctly escalate. That is the honest, expected behavior of kinetic
partitioning.

## 4. What was tried and rejected (negative results, kept on purpose)

- **Bit-identical micro-opts only.** Allocation-light FK + cached identity +
  incremental sweep: 1.1–1.4×, and they cannot touch the tail. Kept as the
  per-step floor under (b); insufficient alone.

- **Tail-edits (cap replicas / earlier bail / fewer iters), order unchanged.**
  Tested by sweeping the solver's own budget params. Every variant that gained
  speed **lost the headline win**:

  | | succ% | mean ms |
  | :-- | --: | --: |
  | ur5 near_singular, baseline | 100.0 | 124 |
  | ur5 near_singular, cap replicas=3 | 98.3 | 119 (barely) |
  | **franka open_space, cap replicas=2** | **71.7** | 200 |

  The cost is the *per-fold* Metropolis search, not the *number* of folds, so
  capping just discards hard-case successes. Rejected.

## 5. Reproduce

```bash
cd backend
python master_benchmark.py                  # full: 3 arms × 3 scenarios × all solvers, multi-seed
python master_benchmark.py --skip-slow       # drop the ~1s homotopy pair for a fast pass
python -m pytest tests/ -q                    # 63 tests incl. bit-identical FK + barrierless checks
```
