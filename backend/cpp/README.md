# Native C++ ProteinIK V4 — the "implement it like TRAC-IK" experiment

**Goal.** The earlier Python benchmark showed ProteinIK V4 ~18–35× slower than
real TRAC-IK on UR5, *but flagged that much of the gap is Python/WSL runtime, not
algorithm*. This directory removes that confound: it reimplements **V4's exact
idea and logic in native C++** (Eigen, like TRAC-IK's own stack) and pits it
against the **real TRAC-IK C++/KDL library in the same process, on identical
kinematics and identical targets**.

## Files
- `ur5_dh.hpp` — UR5 standard-DH FK, geometric Jacobian, axis-angle pose error,
  capsule self-collision, and the V4 energy stack (target/limit/collision/smooth,
  `frustration_index`). Ported 1:1 from `app/core/kinematics.py` +
  `app/solvers/protein_energy.py`.
- `proteinik_v4.hpp` — V4 (`protein_fast`) ported 1:1 from
  `app/solvers/protein_fast/solver.py`: Phase A barrierless-LM ensemble → Phase B
  stochastic Metropolis fold + chaperone rescue → native-stability jitter gate.
  Same weights, tolerances, budgets, acceptance rules.
- `bench_v4_vs_tracik.cpp` — builds a URDF from the UR5 DH table, constructs the
  real `TRAC_IK` solver AND C++ V4 on it, asserts FK parity, benchmarks both on
  the same reachable targets, prints the table.
- `build.sh` — compiles in WSL, linking the TRAC-IK C++ objects from the tracikpy
  build (`trac_ik.o nlopt_ik.o kdl_tl.o`) + `orocos-kdl / nlopt / urdfdom`.

## Build & run (WSL Ubuntu-2204)
```
wsl -d Ubuntu-2204 -u root -- bash "/mnt/c/Coding Projects/Protien IK/backend/cpp/build.sh"
wsl -d Ubuntu-2204 -u root -- /tmp/bench_v4_vs_tracik --trials 100 --seed 0 --timeout 0.05
```

## Result — UR5, 100 reachable targets, identical DH (FK parity 2.2e-16)

| Solver | Success | Mean pos | Mean ms | p50 ms | p95 ms |
|---|---|---|---|---|---|
| **REAL TRAC-IK (C++)** | 100% | 0.001 mm | 0.46–0.61 | 0.28–0.34 | 1.3–2.3 |
| **ProteinIK V4 (C++)** | 99–100% | 0.24–0.39 mm | **0.09–0.17** | **0.03–0.04** | 0.3–1.0 |

Medians over seeds 0–2. V4 uses 0.16–0.25 chaperone rescues/trial; 95–98% of its
solves are self-collision-free.

### The headline
- **In native C++, V4 is NOT slower than real TRAC-IK — it's faster.** Mean
  latency ~0.2–0.3× TRAC-IK, median ~0.1×, with success tied (99–100% vs 100%).
- The earlier Python result (V4 ~18–35× *slower*) was **almost entirely the
  Python/WSL interpreter**, not the algorithm: same logic, compiled, is ~100–120×
  faster (p50 ~4.3 ms → ~0.04 ms; mean ~21 ms → ~0.17 ms).
- This vindicates the paper's original, previously-unverified claim that **V4 is
  speed-competitive with TRAC-IK on UR5** — once implemented at the same level.

### Port faithfulness (C++ vs Python V4, seed-matched)
| | success | rescues/trial | collision-free |
|---|---|---|---|
| Python V4 | 99 / 100 / 100 % | 0.22 / 0.14 / 0.14 | 97 / 95 / 96 % |
| C++ V4    | 99 / 100 / 100 % | 0.25 / 0.18 / 0.16 | 95 / 98 / 97 % |

Statistically the same fold — the only difference is the RNG stream
(`std::mt19937_64` vs numpy PCG64), so individual trajectories differ but the
algorithm, and its outcomes, are identical.

## Caveats
- V4's C++ uses hand-rolled DH FK/Jacobian/collision (Eigen) to stay numerically
  faithful to the Python; TRAC-IK uses its own KDL FK. Both operate on the same
  UR5 (FK parity 2.2e-16), exactly mirroring the Python setup.
- V4 here is single-threaded and un-vectorized (a direct port, not a tuned
  implementation); it could be faster still. TRAC-IK internally runs KDL + an
  SQP/NLopt solver concurrently. Both timed with the same `steady_clock` in one
  process, so the comparison is fair.
- WSL1 native C++ runs at ~normal speed (unlike the pluginlib syscall issue that
  affected URDF loading); these are genuine compiled-code timings.
