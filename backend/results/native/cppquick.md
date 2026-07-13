# Master Sim Benchmark — every solver × arm × scenario, scored in PyBullet + MuJoCo

- **5** trials/seed × seeds **[1]** (n=5 per cell)  |  warm-up 2 untimed solves/cell
- Arms: planar3dof, ur5, franka_panda  |  Scenarios: open_space, near_singular, cluttered
- Engines: PyBullet + MuJoCo  |  generated 2026-07-13 19:54:09

Each solver runs **once** on our DH `RobotSpec` core; that `q_final` is then scored by two independent real-mesh oracles — **PB** = PyBullet, **MJ** = MuJoCo (identical URDF & non-adjacent link pairs). The capsule proxy the solvers optimize against is **not** used to evaluate them here — only real-mesh collision counts (planar3dof has no URDF, so it carries success/speed only). `PyBullet native IK` is the sim's own solver on the identical targets. Timing is wall-clock (OS noise on mean/p95/p99); success/collision/error columns are deterministic given the seed.

## Oracle validation — DH ≡ PyBullet ≡ MuJoCo

### A. Forward-kinematics agreement

| Arm | n | DH↔PB resid | DH↔MJ resid | PB↔MJ max pos | max orient |
|:--|--:|--:|--:|--:|--:|
| ur5 | 200 | 9.5e-07 (base) | 5.2e-08 (base) | 3.80e-08 m | 3.13e-07 rad |
| franka_panda | 200 | 6.6e-07 (tool) | 6.7e-08 (tool) | 5.24e-08 m | 3.71e-07 rad |

### B. Self-collision agreement — PyBullet vs MuJoCo

| Arm | n | PB col% | MJ col% | PB↔MJ sign-agree% | PB↔MJ corr |
|:--|--:|--:|--:|--:|--:|
| ur5 | 200 | 37.0 | 36.5 | 99.5 | 0.992 |
| franka_panda | 200 | 11.5 | 10.5 | 99.0 | 0.865 |

## Verdict — lowest real-mesh-collision solver per cell (among ≥90% success)

| Arm | Scenario | **PyBullet** | **MuJoCo** |
|:--|:--|:--|:--|
| planar3dof | open_space | — | — |
| planar3dof | near_singular | — | — |
| planar3dof | cluttered | — | — |
| ur5 | open_space | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| ur5 | near_singular | ProteinIK Fast (V4 real-calib) | ProteinIK (V1) |
| ur5 | cluttered | ProteinIK Fast (V4 real-calib) | ProteinIK Fast (V4 real-calib) |
| franka_panda | open_space | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| franka_panda | near_singular | ProteinIK Raw Biology (V6) | ProteinIK (V1) |
| franka_panda | cluttered | ProteinIK Fast (V4 real-calib) | ProteinIK Fast (V4 real-calib) |

## planar3dof — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Analytical IK (Planar 3-DOF, exact) | 100.0 |   –   |   –   | 0.1 | 0.1 | 0.1 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4) | 100.0 |   –   |   –   | 0.0 | 0.1 | 0.1 | 38 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 |   –   |   –   | 0.0 | 0.1 | 0.1 | 38 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 |   –   |   –   | 0.0 | 0.1 | 0.1 | 38 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK (V1) | 100.0 |   –   |   –   | 0.0 | 0.1 | 0.1 | 28 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Raw Biology (V6) | 100.0 |   –   |   –   | 3.6 | 3.7 | 3.7 | 100 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 |   –   |   –   | 0.2 | 0.2 | 0.2 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |

## planar3dof — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Analytical IK (Planar 3-DOF, exact) | 100.0 |   –   |   –   | 0.1 | 0.1 | 0.1 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4) | 100.0 |   –   |   –   | 0.0 | 0.0 | 0.0 | 36 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 |   –   |   –   | 0.0 | 0.1 | 0.1 | 36 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 |   –   |   –   | 0.0 | 0.0 | 0.0 | 36 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK (V1) | 100.0 |   –   |   –   | 0.1 | 0.2 | 0.2 | 72 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Raw Biology (V6) | 100.0 |   –   |   –   | 3.6 | 3.7 | 3.7 | 100 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 |   –   |   –   | 0.2 | 0.2 | 0.2 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |

## planar3dof — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Analytical IK (Planar 3-DOF, exact) | 100.0 |   –   |   –   | 0.1 | 0.1 | 0.1 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4) | 100.0 |   –   |   –   | 0.4 | 0.6 | 0.6 | 277 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 |   –   |   –   | 0.4 | 0.6 | 0.6 | 277 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 |   –   |   –   | 0.2 | 0.6 | 0.7 | 155 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 |   –   |   –   | 0.2 | 0.2 | 0.2 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK (V1) | 80.0 |   –   |   –   | 0.1 | 0.3 | 0.3 | 75 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Raw Biology (V6) | 80.0 |   –   |   –   | 3.8 | 3.9 | 3.9 | 100 |   –   |   –   |   –   |   –   |   –   | 0.20 |

## ur5 — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 0.0 | 0.0 | 0.1 | 13 | 0.0 | 0.0 | 0.0109 | 0.0129 | 0.140 | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 0.6 | 1.3 | 1.4 | 266 | 0.0 | 0.0 | 0.0106 | 0.0113 | 0.447 | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 0.0 | 0.1 | 0.1 | 13 | 0.0 | 0.0 | 0.0109 | 0.0129 | 0.140 | 0.00 |
| ProteinIK (V1) | 100.0 | 100.0 | 100.0 | 0.0 | 0.1 | 0.1 | 20 | 20.0 | 20.0 | 0.0019 | 0.0039 | 0.252 | 0.00 |
| ProteinIK Raw Biology (V6) | 100.0 | 100.0 | 100.0 | 13.7 | 13.8 | 13.8 | 100 | 0.0 | 0.0 | 0.0104 | 0.0110 | 0.000 | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 0.3 | 0.4 | 0.4 | 1 | 0.0 | 0.0 | 0.0114 | 0.0106 | 0.001 | 0.00 |
| PyBullet native IK |   –   | 100.0 | 100.0 | 1.4 | 2.4 | 2.5 |   –   | 0.0 | 0.0 | 0.0086 | 0.0105 | 0.009 |   –   |

## ur5 — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 0.1 | 0.1 | 0.1 | 20 | 60.0 | 60.0 | -0.0144 | -0.0124 | 0.341 | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 0.3 | 0.5 | 0.6 | 168 | 40.0 | 40.0 | 0.0006 | 0.0026 | 0.264 | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 0.0 | 0.1 | 0.1 | 20 | 60.0 | 60.0 | -0.0144 | -0.0124 | 0.341 | 0.00 |
| ProteinIK (V1) | 100.0 | 100.0 | 100.0 | 0.2 | 0.6 | 0.7 | 62 | 40.0 | 20.0 | -0.0092 | -0.0097 | 0.734 | 0.00 |
| ProteinIK Raw Biology (V6) | 100.0 | 100.0 | 100.0 | 14.1 | 14.3 | 14.3 | 100 | 40.0 | 40.0 | 0.0007 | 0.0026 | 0.284 | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 0.8 | 1.1 | 1.1 | 1 | 80.0 | 80.0 | -0.0224 | -0.0204 | 0.003 | 0.00 |
| PyBullet native IK |   –   | 40.0 | 40.0 | 6.1 | 7.0 | 7.0 |   –   | 60.0 | 60.0 | -0.0251 | -0.0231 | 6.131 |   –   |

## ur5 — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 0.2 | 0.6 | 0.7 | 98 | 80.0 | 80.0 | -0.0162 | -0.0142 | 0.239 | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 0.3 | 0.6 | 0.7 | 181 | 60.0 | 60.0 | -0.0120 | -0.0128 | 0.336 | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 0.2 | 0.6 | 0.7 | 98 | 80.0 | 80.0 | -0.0162 | -0.0142 | 0.239 | 0.00 |
| ProteinIK (V1) | 100.0 | 100.0 | 100.0 | 0.2 | 0.5 | 0.5 | 56 | 60.0 | 60.0 | -0.0242 | -0.0247 | 0.619 | 0.00 |
| ProteinIK Raw Biology (V6) | 100.0 | 100.0 | 100.0 | 13.7 | 14.0 | 14.1 | 100 | 60.0 | 60.0 | -0.0122 | -0.0130 | 0.368 | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 0.3 | 0.4 | 0.4 | 1 | 60.0 | 60.0 | -0.0292 | -0.0273 | 0.002 | 0.00 |
| PyBullet native IK |   –   | 60.0 | 60.0 | 3.7 | 6.6 | 6.6 |   –   | 60.0 | 60.0 | -0.0280 | -0.0260 | 5.724 |   –   |

## franka_panda — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 0.1 | 0.2 | 0.3 | 77 | 20.0 | 20.0 | -0.0003 | -0.0019 | 0.484 | 0.20 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 0.3 | 1.0 | 1.1 | 118 | 20.0 | 20.0 | -0.0003 | -0.0019 | 0.484 | 0.20 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 0.1 | 0.2 | 0.3 | 77 | 20.0 | 20.0 | -0.0003 | -0.0019 | 0.484 | 0.20 |
| ProteinIK (V1) | 100.0 | 100.0 | 100.0 | 0.1 | 0.3 | 0.3 | 44 | 40.0 | 40.0 | -0.0113 | -0.0093 | 0.514 | 0.20 |
| ProteinIK Raw Biology (V6) | 100.0 | 100.0 | 100.0 | 21.4 | 21.8 | 21.8 | 125 | 20.0 | 20.0 | -0.0020 | -0.0044 | 0.015 | 0.20 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 0.7 | 1.6 | 1.7 | 1 | 20.0 | 20.0 | -0.0013 | 0.0007 | 0.002 | 0.00 |
| PyBullet native IK |   –   | 100.0 | 100.0 | 2.7 | 7.4 | 8.6 |   –   | 40.0 | 40.0 | -0.0158 | -0.0138 | 0.012 |   –   |

## franka_panda — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 0.2 | 0.6 | 0.7 | 141 | 20.0 | 20.0 | 0.0138 | 0.0122 | 0.328 | 0.40 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 0.2 | 0.6 | 0.7 | 141 | 20.0 | 20.0 | 0.0138 | 0.0122 | 0.328 | 0.40 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 0.2 | 0.6 | 0.7 | 141 | 20.0 | 20.0 | 0.0138 | 0.0122 | 0.328 | 0.40 |
| ProteinIK (V1) | 100.0 | 100.0 | 100.0 | 0.2 | 0.4 | 0.4 | 61 | 20.0 | 0.0 | 0.0154 | 0.0174 | 0.536 | 0.00 |
| ProteinIK Raw Biology (V6) | 100.0 | 100.0 | 100.0 | 20.5 | 20.7 | 20.7 | 125 | 0.0 | 0.0 | 0.0172 | 0.0192 | 0.198 | 0.20 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 0.7 | 1.6 | 1.9 | 1 | 0.0 | 0.0 | 0.0196 | 0.0216 | 0.001 | 0.00 |
| PyBullet native IK |   –   | 80.0 | 80.0 | 5.2 | 8.7 | 8.7 |   –   | 0.0 | 0.0 | 0.0181 | 0.0201 | 30.100 |   –   |

## franka_panda — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 0.8 | 2.6 | 2.9 | 224 | 80.0 | 80.0 | -0.0597 | -0.0578 | 0.379 | 0.60 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 0.5 | 1.2 | 1.2 | 198 | 60.0 | 60.0 | -0.0444 | -0.0424 | 0.350 | 0.40 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 0.8 | 2.7 | 3.1 | 224 | 80.0 | 80.0 | -0.0597 | -0.0578 | 0.379 | 0.60 |
| ProteinIK (V1) | 100.0 | 100.0 | 100.0 | 0.2 | 0.3 | 0.4 | 50 | 60.0 | 60.0 | -0.0484 | -0.0505 | 0.594 | 0.00 |
| ProteinIK Raw Biology (V6) | 100.0 | 100.0 | 100.0 | 21.1 | 21.5 | 21.6 | 125 | 60.0 | 60.0 | -0.0394 | -0.0419 | 0.016 | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 80.0 | 80.0 | 80.0 | 1.7 | 4.4 | 4.9 | 1 | 40.0 | 40.0 | -0.0183 | -0.0180 | 181.766 | 0.20 |
| PyBullet native IK |   –   | 100.0 | 100.0 | 2.4 | 5.2 | 5.7 |   –   | 80.0 | 80.0 | -0.0588 | -0.0609 | 0.009 |   –   |

---

### Provenance — every solver runs as NATIVE compiled code

This is `master_full.md` re-run **entirely in the native system**. Every solver is
either a genuine imported library or a native-C++ port — none is interpreted Python,
so the speed columns are apples-to-apples.

| Solver | Native implementation |
|:--|:--|
| **TRAC-IK** | REAL TRAC-IK — TRACLabs C++/KDL/NLopt via `tracikpy` |
| **Jacobian DLS** | REAL Robotics Toolbox (Corke) Levenberg–Marquardt, single-shot |
| **Multi-start** | REAL Robotics Toolbox `ik_LM` with native random restarts |
| **ProteinIK V1 / V4 / V4+o2 / V4-calib / V6** | **native C++/Eigen ports** (backend/cpp/, `pik_native`) of the project's own solvers — same logic/weights/tolerances, FK & energy parity to ≤1e-11, success/collision statistically identical to the Python (only the RNG stream differs) |
| Analytical (planar3dof) | the project's exact closed-form |
| PyBullet native IK | REAL PyBullet `calculateInverseKinematics` |
| CCD, FABRIK | in-repo algorithm — no genuine DH-native upstream exists |

Homotopy (CCH-IK) and Fixed-λ are excluded from this benchmark. Numbers differ from
`master_full.md` **by design**: native compiled solvers, not Python — e.g. ProteinIK V4
now runs sub-millisecond and competes with TRAC-IK on speed as well as quality.
