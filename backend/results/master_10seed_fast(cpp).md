# Master Sim Benchmark — every solver × arm × scenario, scored in PyBullet + MuJoCo

- **100** trials/seed × seeds **[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]** (n=1000 per cell)  |  warm-up 8 untimed solves/cell
- Arms: ur5, franka_panda  |  Scenarios: open_space, near_singular, cluttered
- Engines: PyBullet + MuJoCo  |  generated 2026-07-14 21:30:16

Each solver runs **once** on our DH `RobotSpec` core; that `q_final` is then scored by two independent real-mesh oracles — **PB** = PyBullet, **MJ** = MuJoCo (identical URDF & non-adjacent link pairs). The capsule proxy the solvers optimize against is **not** used to evaluate them here — only real-mesh collision counts (planar3dof has no URDF, so it carries success/speed only). `PyBullet native IK` is the sim's own solver on the identical targets. Timing is wall-clock (OS noise on mean/p95/p99); success/collision/error columns are deterministic given the seed.

## Oracle validation — DH ≡ PyBullet ≡ MuJoCo

### A. Forward-kinematics agreement

| Arm | n | DH↔PB resid | DH↔MJ resid | PB↔MJ max pos | max orient |
|:--|--:|--:|--:|--:|--:|
| ur5 | 2000 | 9.5e-07 (base) | 5.2e-08 (base) | 4.11e-08 m | 5.93e-07 rad |
| franka_panda | 2000 | 6.6e-07 (tool) | 6.7e-08 (tool) | 5.90e-08 m | 4.13e-07 rad |

### B. Self-collision agreement — PyBullet vs MuJoCo

| Arm | n | PB col% | MJ col% | PB↔MJ sign-agree% | PB↔MJ corr |
|:--|--:|--:|--:|--:|--:|
| ur5 | 2000 | 39.1 | 37.0 | 97.9 | 0.992 |
| franka_panda | 2000 | 9.2 | 8.3 | 99.1 | 0.866 |

## Verdict — lowest real-mesh-collision solver per cell (among ≥90% success)

| Arm | Scenario | **PyBullet** | **MuJoCo** |
|:--|:--|:--|:--|
| ur5 | open_space | ProteinIK Raw Biology (V6) | ProteinIK Raw Biology (V6) |
| ur5 | near_singular | ProteinIK Raw Biology (V6) | ProteinIK Raw Biology (V6) |
| ur5 | cluttered | ProteinIK Raw Biology (V6) | ProteinIK Raw Biology (V6) |
| franka_panda | open_space | Multi-start (real RTB ik_LM restarts) | Multi-start (real RTB ik_LM restarts) |
| franka_panda | near_singular | ProteinIK Raw Biology (V6) | ProteinIK Raw Biology (V6) |
| franka_panda | cluttered | Multi-start (real RTB ik_LM restarts) | Multi-start (real RTB ik_LM restarts) |

## ur5 — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM restarts) | 100.0 | 100.0 | 100.0 | 0.6 | 0.7 | 0.9 | 32 | 35.8 | 33.7 | -0.0042 | -0.0031 | 0.001 | 0.00 |
| ProteinIK Fast (V4) | 99.8 | 99.8 | 99.8 | 0.3 | 0.3 | 2.0 | 58 | 26.2 | 24.6 | 0.0017 | 0.0020 | 0.305 | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 99.8 | 99.8 | 99.8 | 0.1 | 0.2 | 1.5 | 55 | 26.7 | 25.2 | 0.0012 | 0.0016 | 0.306 | 0.01 |
| ProteinIK Fast (V4 real-calib) | 99.8 | 99.7 | 99.7 | 0.6 | 2.0 | 3.5 | 242 | 23.0 | 21.4 | 0.0026 | 0.0030 | 0.336 | 0.01 |
| ProteinIK Raw Biology (V6) | 99.6 | 99.6 | 99.6 | 14.2 | 14.8 | 15.0 | 100 | 13.1 | 12.1 | 0.0057 | 0.0058 | 0.184 | 0.01 |
| TRAC-IK (real C++ TRACLabs) | 99.5 | 99.5 | 99.5 | 0.6 | 1.9 | 4.2 | 1 | 35.3 | 33.1 | -0.0035 | -0.0027 | 5.097 | 0.00 |
| ProteinIK (V1) | 96.0 | 96.0 | 96.0 | 0.1 | 0.6 | 0.7 | 45 | 32.9 | 31.0 | -0.0017 | -0.0009 | 14.019 | 0.02 |
| Jacobian DLS (real RTB LM, single-shot) | 69.9 | 69.9 | 69.9 | 0.6 | 0.9 | 1.0 | 38 | 39.9 | 37.5 | -0.0055 | -0.0045 | 254.184 | 0.00 |
| FABRIK (in-repo; no genuine upstream) | 47.5 | 47.4 | 47.5 | 93.3 | 153.7 | 157.4 | 95 | 40.7 | 38.9 | -0.0064 | -0.0056 | 49.547 | 0.49 |
| CCD (in-repo; no genuine upstream) | 45.1 | 45.1 | 45.1 | 142.1 | 228.9 | 232.9 | 191 | 36.9 | 35.0 | -0.0057 | -0.0047 | 36.639 | 0.55 |
| PyBullet native IK |   –   | 72.4 | 72.4 | 4.0 | 7.1 | 7.5 |   –   | 43.0 | 40.8 | -0.0068 | -0.0059 | 11.094 |   –   |

## ur5 — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM restarts) | 100.0 | 100.0 | 100.0 | 0.6 | 0.8 | 1.0 | 36 | 47.6 | 46.4 | -0.0153 | -0.0144 | 0.001 | 0.00 |
| ProteinIK Fast (V4) | 99.8 | 99.8 | 99.8 | 0.2 | 1.1 | 2.9 | 82 | 40.4 | 38.6 | -0.0083 | -0.0077 | 0.330 | 0.01 |
| ProteinIK Fast (V4 real-calib) | 99.9 | 99.8 | 99.8 | 0.7 | 2.4 | 3.7 | 258 | 38.6 | 37.0 | -0.0073 | -0.0067 | 0.411 | 0.01 |
| ProteinIK Fast (V4+o2 IAM) | 99.8 | 99.8 | 99.8 | 0.2 | 0.6 | 2.2 | 75 | 40.7 | 39.0 | -0.0086 | -0.0081 | 0.342 | 0.01 |
| ProteinIK Raw Biology (V6) | 99.3 | 99.3 | 99.3 | 14.6 | 14.9 | 15.1 | 100 | 27.9 | 27.2 | -0.0028 | -0.0024 | 0.539 | 0.02 |
| TRAC-IK (real C++ TRACLabs) | 99.3 | 99.3 | 99.3 | 0.6 | 1.7 | 4.4 | 1 | 49.9 | 47.6 | -0.0168 | -0.0159 | 7.312 | 0.00 |
| ProteinIK (V1) | 88.1 | 88.1 | 88.1 | 0.2 | 0.7 | 0.7 | 73 | 43.4 | 41.7 | -0.0114 | -0.0105 | 35.863 | 0.06 |
| Jacobian DLS (real RTB LM, single-shot) | 70.2 | 70.2 | 70.2 | 0.6 | 0.9 | 1.1 | 41 | 49.5 | 47.7 | -0.0160 | -0.0150 | 236.018 | 0.00 |
| FABRIK (in-repo; no genuine upstream) | 36.2 | 36.2 | 36.2 | 104.9 | 147.6 | 149.3 | 111 | 54.4 | 53.0 | -0.0181 | -0.0172 | 48.182 | 0.44 |
| CCD (in-repo; no genuine upstream) | 35.7 | 35.7 | 35.7 | 162.8 | 233.5 | 236.9 | 220 | 51.2 | 49.3 | -0.0191 | -0.0182 | 31.329 | 0.56 |
| PyBullet native IK |   –   | 45.8 | 45.8 | 6.1 | 7.0 | 7.1 |   –   | 52.8 | 51.3 | -0.0178 | -0.0168 | 11.886 |   –   |

## ur5 — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM restarts) | 100.0 | 100.0 | 100.0 | 0.6 | 0.7 | 0.8 | 28 | 74.7 | 73.6 | -0.0367 | -0.0353 | 0.001 | 0.00 |
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 0.2 | 1.3 | 1.9 | 93 | 56.4 | 54.9 | -0.0189 | -0.0182 | 0.343 | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 0.1 | 0.4 | 1.5 | 80 | 57.1 | 55.8 | -0.0201 | -0.0195 | 0.340 | 0.00 |
| ProteinIK Raw Biology (V6) | 100.0 | 100.0 | 100.0 | 14.0 | 14.7 | 15.3 | 100 | 48.6 | 46.8 | -0.0141 | -0.0137 | 0.318 | 0.01 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 0.4 | 0.8 | 1.4 | 1 | 74.2 | 72.1 | -0.0372 | -0.0360 | 0.002 | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 99.9 | 99.9 | 0.5 | 1.6 | 2.5 | 208 | 57.2 | 55.1 | -0.0189 | -0.0180 | 0.353 | 0.01 |
| ProteinIK (V1) | 87.4 | 87.4 | 87.4 | 0.2 | 0.7 | 0.7 | 70 | 64.6 | 62.9 | -0.0267 | -0.0257 | 25.558 | 0.05 |
| Jacobian DLS (real RTB LM, single-shot) | 75.8 | 75.8 | 75.8 | 0.6 | 0.7 | 0.9 | 26 | 63.5 | 62.6 | -0.0294 | -0.0283 | 145.013 | 0.00 |
| CCD (in-repo; no genuine upstream) | 41.3 | 41.3 | 41.3 | 154.9 | 229.2 | 230.4 | 206 | 73.6 | 71.4 | -0.0401 | -0.0389 | 18.705 | 0.50 |
| FABRIK (in-repo; no genuine upstream) | 39.2 | 39.2 | 39.2 | 100.1 | 148.3 | 150.0 | 106 | 71.4 | 70.0 | -0.0365 | -0.0354 | 33.186 | 0.43 |
| PyBullet native IK |   –   | 54.0 | 54.0 | 5.5 | 6.8 | 7.0 |   –   | 74.4 | 73.3 | -0.0377 | -0.0363 | 5.843 |   –   |

## franka_panda — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 0.1 | 0.6 | 1.8 | 74 | 7.3 | 6.7 | 0.0150 | 0.0136 | 0.357 | 0.25 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 0.1 | 0.6 | 1.9 | 74 | 7.3 | 6.7 | 0.0149 | 0.0136 | 0.358 | 0.25 |
| ProteinIK Fast (V4 real-calib) | 99.9 | 99.8 | 99.8 | 0.1 | 0.7 | 2.1 | 76 | 7.2 | 6.6 | 0.0150 | 0.0137 | 0.364 | 0.25 |
| ProteinIK Raw Biology (V6) | 99.1 | 99.1 | 99.1 | 21.9 | 22.8 | 24.2 | 125 | 6.7 | 5.9 | 0.0158 | 0.0143 | 0.226 | 0.29 |
| TRAC-IK (real C++ TRACLabs) | 99.1 | 99.1 | 99.1 | 0.8 | 2.3 | 4.9 | 1 | 7.8 | 6.9 | 0.0149 | 0.0133 | 7.511 | 0.07 |
| Multi-start (real RTB ik_LM restarts) | 99.0 | 99.0 | 99.0 | 0.9 | 1.2 | 3.1 | 79 | 6.1 | 5.2 | 0.0160 | 0.0148 | 7.937 | 0.00 |
| ProteinIK (V1) | 97.3 | 97.3 | 97.3 | 0.2 | 0.5 | 0.7 | 55 | 7.5 | 6.6 | 0.0146 | 0.0131 | 5.830 | 0.12 |
| Jacobian DLS (real RTB LM, single-shot) | 28.4 | 28.4 | 28.4 | 0.8 | 1.1 | 1.3 | 34 | 9.6 | 9.2 | 0.0144 | 0.0132 | 564.707 | 0.00 |
| CCD (in-repo; no genuine upstream) | 27.0 | 27.0 | 27.0 | 216.6 | 281.2 | 288.0 | 242 | 18.3 | 17.5 | 0.0102 | 0.0096 | 25.334 | 1.10 |
| FABRIK (in-repo; no genuine upstream) | 21.1 | 21.1 | 21.1 | 164.6 | 197.3 | 200.1 | 127 | 18.5 | 18.3 | 0.0105 | 0.0098 | 53.311 | 1.30 |
| PyBullet native IK |   –   | 80.8 | 80.8 | 4.4 | 9.3 | 9.4 |   –   | 29.5 | 28.9 | -0.0028 | -0.0034 | 12.327 |   –   |

## franka_panda — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 99.9 | 99.8 | 99.8 | 0.2 | 0.8 | 2.6 | 77 | 11.2 | 10.3 | 0.0139 | 0.0128 | 0.395 | 0.21 |
| ProteinIK Fast (V4+o2 IAM) | 99.9 | 99.8 | 99.8 | 0.2 | 0.7 | 2.3 | 77 | 11.2 | 10.3 | 0.0139 | 0.0128 | 0.395 | 0.21 |
| ProteinIK Fast (V4 real-calib) | 99.7 | 99.6 | 99.6 | 0.2 | 0.7 | 2.7 | 80 | 11.0 | 10.1 | 0.0140 | 0.0128 | 0.439 | 0.21 |
| ProteinIK Raw Biology (V6) | 98.6 | 98.6 | 98.6 | 22.3 | 22.8 | 23.5 | 125 | 7.9 | 7.1 | 0.0150 | 0.0136 | 0.613 | 0.20 |
| TRAC-IK (real C++ TRACLabs) | 98.3 | 98.3 | 98.3 | 1.1 | 3.3 | 5.1 | 1 | 9.1 | 8.5 | 0.0147 | 0.0140 | 13.669 | 0.06 |
| Multi-start (real RTB ik_LM restarts) | 97.7 | 97.7 | 97.7 | 1.0 | 1.4 | 4.2 | 106 | 9.6 | 8.5 | 0.0146 | 0.0141 | 20.274 | 0.00 |
| ProteinIK (V1) | 92.7 | 92.7 | 92.7 | 0.2 | 0.8 | 0.8 | 65 | 9.7 | 9.1 | 0.0142 | 0.0131 | 15.650 | 0.17 |
| Jacobian DLS (real RTB LM, single-shot) | 27.5 | 27.5 | 27.5 | 0.8 | 1.1 | 1.2 | 35 | 10.0 | 9.3 | 0.0145 | 0.0131 | 594.687 | 0.00 |
| CCD (in-repo; no genuine upstream) | 15.5 | 15.5 | 15.5 | 246.2 | 286.7 | 309.3 | 273 | 17.2 | 16.7 | 0.0112 | 0.0109 | 24.855 | 1.07 |
| FABRIK (in-repo; no genuine upstream) | 12.7 | 12.7 | 12.7 | 175.8 | 199.2 | 206.7 | 139 | 19.3 | 18.8 | 0.0101 | 0.0099 | 51.406 | 1.20 |
| PyBullet native IK |   –   | 62.3 | 62.3 | 7.2 | 9.4 | 9.6 |   –   | 23.2 | 22.9 | 0.0015 | 0.0011 | 16.750 |   –   |

## franka_panda — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4+o2 IAM) | 98.4 | 98.4 | 98.4 | 0.5 | 2.9 | 4.6 | 201 | 82.3 | 82.1 | -0.0468 | -0.0457 | 0.575 | 0.29 |
| ProteinIK Fast (V4) | 98.4 | 98.2 | 98.2 | 0.7 | 3.3 | 5.2 | 218 | 82.4 | 82.2 | -0.0468 | -0.0456 | 0.582 | 0.29 |
| ProteinIK Fast (V4 real-calib) | 97.9 | 97.6 | 97.6 | 0.8 | 3.5 | 5.0 | 274 | 81.4 | 81.1 | -0.0456 | -0.0446 | 0.630 | 0.29 |
| ProteinIK Raw Biology (V6) | 96.5 | 96.5 | 96.5 | 20.9 | 21.4 | 22.0 | 125 | 80.1 | 79.4 | -0.0439 | -0.0429 | 0.783 | 0.44 |
| Multi-start (real RTB ik_LM restarts) | 93.2 | 93.2 | 93.2 | 1.2 | 4.3 | 4.9 | 211 | 77.0 | 76.3 | -0.0445 | -0.0434 | 49.971 | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 92.5 | 92.5 | 92.5 | 1.3 | 5.1 | 5.1 | 1 | 77.1 | 76.8 | -0.0441 | -0.0432 | 50.507 | 0.06 |
| ProteinIK (V1) | 84.7 | 84.7 | 84.7 | 0.3 | 0.8 | 0.8 | 85 | 77.3 | 76.4 | -0.0429 | -0.0422 | 38.960 | 0.35 |
| FABRIK (in-repo; no genuine upstream) | 20.5 | 20.5 | 20.5 | 161.8 | 202.0 | 206.3 | 125 | 81.1 | 80.7 | -0.0454 | -0.0442 | 50.258 | 1.58 |
| Jacobian DLS (real RTB LM, single-shot) | 19.7 | 19.7 | 19.7 | 0.7 | 0.8 | 1.2 | 21 | 23.1 | 22.4 | 0.0031 | 0.0025 | 594.187 | 0.00 |
| CCD (in-repo; no genuine upstream) | 15.0 | 15.0 | 15.0 | 245.2 | 292.3 | 297.4 | 264 | 83.9 | 83.3 | -0.0511 | -0.0497 | 21.743 | 1.68 |
| PyBullet native IK |   –   | 94.6 | 94.6 | 3.1 | 9.1 | 9.9 |   –   | 87.6 | 87.3 | -0.0654 | -0.0639 | 3.588 |   –   |

---

### Provenance — every solver runs as NATIVE compiled code

This is the **10-seed collision benchmark** (`master_10seed_fast.md`) re-run **entirely
in the native system** — 10 seeds × 100 trials (n=1000 per cell), ur5 + franka_panda.
Every solver is either a genuine imported library or a native-C++ port — none is
interpreted Python, so the speed columns are apples-to-apples.

| Solver | Native implementation |
|:--|:--|
| **TRAC-IK** | REAL TRAC-IK — TRACLabs C++/KDL/NLopt via `tracikpy` |
| **Jacobian DLS** | REAL Robotics Toolbox (Corke) Levenberg–Marquardt, single-shot |
| **Multi-start** | REAL Robotics Toolbox `ik_LM` with native random restarts |
| **ProteinIK V1 / V4 / V4+o2 / V4-calib / V6** | **native C++/Eigen ports** (backend/cpp/, `pik_native`) of the project's own solvers — same logic/weights/tolerances, FK & energy parity to ≤1e-11, success/collision statistically identical to the Python (only the RNG stream differs) |
| PyBullet native IK | REAL PyBullet `calculateInverseKinematics` |
| CCD, FABRIK | in-repo algorithm — no genuine DH-native upstream exists |

Homotopy (CCH-IK) and Fixed-λ are excluded from this benchmark. Numbers differ from
`master_10seed_fast.md` **by design**: native compiled solvers, not Python — e.g. ProteinIK V4
now runs sub-millisecond and competes with TRAC-IK on speed as well as quality.
