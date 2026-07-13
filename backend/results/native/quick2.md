# Master Sim Benchmark — every solver × arm × scenario, scored in PyBullet + MuJoCo

- **5** trials/seed × seeds **[1]** (n=5 per cell)  |  warm-up 2 untimed solves/cell
- Arms: planar3dof, ur5, franka_panda  |  Scenarios: open_space, near_singular, cluttered
- Engines: PyBullet + MuJoCo  |  generated 2026-07-13 18:47:08

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
| ur5 | open_space | ProteinIK Fast (V4 real-calib) | ProteinIK Fast (V4 real-calib) |
| ur5 | near_singular | ProteinIK Fast (V4 real-calib) | ProteinIK (V1) |
| ur5 | cluttered | Multi-start (real RTB ik_LM restarts) | Multi-start (real RTB ik_LM restarts) |
| franka_panda | open_space | Multi-start (real RTB ik_LM restarts) | Multi-start (real RTB ik_LM restarts) |
| franka_panda | near_singular | Multi-start (real RTB ik_LM restarts) | Multi-start (real RTB ik_LM restarts) |
| franka_panda | cluttered | ProteinIK Fast (V4) | ProteinIK Fast (V4) |

## planar3dof — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Analytical IK (Planar 3-DOF, exact) | 100.0 |   –   |   –   | 0.1 | 0.1 | 0.1 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| Multi-start (real RTB ik_LM restarts) | 100.0 |   –   |   –   | 0.2 | 0.2 | 0.2 | 19 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4) | 100.0 |   –   |   –   | 2.6 | 5.2 | 5.4 | 24 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 |   –   |   –   | 2.6 | 5.3 | 5.5 | 24 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 |   –   |   –   | 2.6 | 5.4 | 5.6 | 24 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK (V1) | 100.0 |   –   |   –   | 12.2 | 23.1 | 23.4 | 32 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 |   –   |   –   | 0.2 | 0.3 | 0.3 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| CCD (in-repo; no genuine upstream) | 80.0 |   –   |   –   | 27.2 | 82.3 | 96.2 | 82 |   –   |   –   |   –   |   –   |   –   | 0.40 |
| FABRIK (in-repo; no genuine upstream) | 80.0 |   –   |   –   | 18.9 | 55.3 | 64.5 | 42 |   –   |   –   |   –   |   –   |   –   | 0.40 |
| Jacobian DLS (real RTB LM, single-shot) | 80.0 |   –   |   –   | 0.2 | 0.2 | 0.2 | 10 |   –   |   –   |   –   |   –   |   –   | 0.00 |

## planar3dof — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Analytical IK (Planar 3-DOF, exact) | 100.0 |   –   |   –   | 0.1 | 0.1 | 0.1 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| Multi-start (real RTB ik_LM restarts) | 100.0 |   –   |   –   | 0.2 | 0.2 | 0.2 | 16 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4) | 100.0 |   –   |   –   | 9.5 | 20.7 | 22.7 | 83 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 |   –   |   –   | 9.6 | 21.1 | 23.0 | 83 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 |   –   |   –   | 9.5 | 21.0 | 23.0 | 83 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK (V1) | 100.0 |   –   |   –   | 33.9 | 67.5 | 69.1 | 74 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 |   –   |   –   | 0.1 | 0.2 | 0.2 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| Jacobian DLS (real RTB LM, single-shot) | 80.0 |   –   |   –   | 0.2 | 0.2 | 0.2 | 14 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| CCD (in-repo; no genuine upstream) | 20.0 |   –   |   –   | 79.0 | 96.6 | 96.8 | 248 |   –   |   –   |   –   |   –   |   –   | 0.40 |
| FABRIK (in-repo; no genuine upstream) | 0.0 |   –   |   –   | 69.4 | 70.6 | 70.8 | 150 |   –   |   –   |   –   |   –   |   –   | 1.20 |

## planar3dof — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Analytical IK (Planar 3-DOF, exact) | 100.0 |   –   |   –   | 0.1 | 0.1 | 0.1 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| Multi-start (real RTB ik_LM restarts) | 100.0 |   –   |   –   | 0.2 | 0.3 | 0.3 | 30 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4) | 100.0 |   –   |   –   | 138.3 | 378.6 | 434.3 | 382 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 |   –   |   –   | 137.7 | 376.5 | 431.7 | 382 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 |   –   |   –   | 62.3 | 130.2 | 136.1 | 253 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 |   –   |   –   | 0.2 | 0.2 | 0.2 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| Jacobian DLS (real RTB LM, single-shot) | 80.0 |   –   |   –   | 0.2 | 0.2 | 0.2 | 16 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK (V1) | 80.0 |   –   |   –   | 25.9 | 79.6 | 92.4 | 60 |   –   |   –   |   –   |   –   |   –   | 0.20 |
| FABRIK (in-repo; no genuine upstream) | 40.0 |   –   |   –   | 45.1 | 68.9 | 69.0 | 99 |   –   |   –   |   –   |   –   |   –   | 1.40 |
| CCD (in-repo; no genuine upstream) | 20.0 |   –   |   –   | 80.8 | 100.5 | 100.7 | 244 |   –   |   –   |   –   |   –   |   –   | 1.40 |

## ur5 — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM restarts) | 100.0 | 100.0 | 100.0 | 0.6 | 0.6 | 0.6 | 28 | 20.0 | 20.0 | 0.0021 | 0.0041 | 0.003 | 0.00 |
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 2.2 | 4.4 | 4.9 | 15 | 20.0 | 20.0 | 0.0062 | 0.0082 | 0.067 | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 55.7 | 116.9 | 127.0 | 193 | 0.0 | 0.0 | 0.0107 | 0.0126 | 0.295 | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 2.2 | 4.3 | 4.9 | 15 | 20.0 | 20.0 | 0.0062 | 0.0082 | 0.067 | 0.00 |
| ProteinIK (V1) | 100.0 | 100.0 | 100.0 | 10.7 | 13.0 | 13.4 | 20 | 20.0 | 20.0 | 0.0019 | 0.0039 | 0.300 | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 0.3 | 0.5 | 0.5 | 1 | 20.0 | 20.0 | 0.0070 | 0.0064 | 0.003 | 0.00 |
| FABRIK (in-repo; no genuine upstream) | 80.0 | 80.0 | 80.0 | 73.1 | 139.7 | 146.8 | 72 | 20.0 | 20.0 | 0.0076 | 0.0068 | 8.393 | 0.20 |
| CCD (in-repo; no genuine upstream) | 60.0 | 60.0 | 60.0 | 156.0 | 232.0 | 232.1 | 202 | 0.0 | 0.0 | 0.0112 | 0.0131 | 15.132 | 0.60 |
| Jacobian DLS (real RTB LM, single-shot) | 60.0 | 60.0 | 60.0 | 0.6 | 0.7 | 0.7 | 15 | 40.0 | 40.0 | -0.0098 | -0.0078 | 333.944 | 0.00 |
| PyBullet native IK |   –   | 100.0 | 100.0 | 1.4 | 2.5 | 2.6 |   –   | 0.0 | 0.0 | 0.0086 | 0.0105 | 0.009 |   –   |

## ur5 — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM restarts) | 100.0 | 100.0 | 100.0 | 0.6 | 0.6 | 0.7 | 25 | 60.0 | 60.0 | -0.0124 | -0.0105 | 0.000 | 0.00 |
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 16.1 | 59.7 | 71.2 | 42 | 60.0 | 60.0 | -0.0125 | -0.0105 | 0.399 | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 50.5 | 108.2 | 114.9 | 163 | 40.0 | 40.0 | -0.0006 | 0.0014 | 0.394 | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 7.2 | 23.9 | 28.2 | 35 | 60.0 | 60.0 | -0.0189 | -0.0169 | 0.351 | 0.00 |
| ProteinIK (V1) | 100.0 | 100.0 | 100.0 | 35.5 | 80.6 | 85.7 | 45 | 40.0 | 20.0 | 0.0020 | 0.0014 | 0.779 | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 0.5 | 1.1 | 1.2 | 1 | 40.0 | 40.0 | -0.0115 | -0.0113 | 0.002 | 0.00 |
| Jacobian DLS (real RTB LM, single-shot) | 80.0 | 80.0 | 80.0 | 0.6 | 0.8 | 0.9 | 52 | 40.0 | 40.0 | -0.0062 | -0.0097 | 46.698 | 0.00 |
| FABRIK (in-repo; no genuine upstream) | 40.0 | 40.0 | 40.0 | 95.0 | 144.5 | 144.6 | 99 | 40.0 | 40.0 | -0.0095 | -0.0104 | 38.432 | 0.40 |
| CCD (in-repo; no genuine upstream) | 0.0 | 0.0 | 0.0 | 221.1 | 222.6 | 222.7 | 300 | 40.0 | 40.0 | -0.0241 | -0.0250 | 69.668 | 1.00 |
| PyBullet native IK |   –   | 40.0 | 40.0 | 6.0 | 6.8 | 6.8 |   –   | 60.0 | 60.0 | -0.0251 | -0.0231 | 6.131 |   –   |

## ur5 — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM restarts) | 100.0 | 100.0 | 100.0 | 0.6 | 0.8 | 0.8 | 43 | 40.0 | 40.0 | -0.0267 | -0.0273 | 0.002 | 0.00 |
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 6.3 | 17.2 | 19.7 | 49 | 60.0 | 60.0 | -0.0145 | -0.0125 | 0.232 | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 44.0 | 108.8 | 124.5 | 173 | 80.0 | 80.0 | -0.0231 | -0.0211 | 0.378 | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 5.8 | 15.6 | 17.8 | 46 | 60.0 | 60.0 | -0.0161 | -0.0142 | 0.225 | 0.00 |
| ProteinIK (V1) | 100.0 | 100.0 | 100.0 | 36.7 | 95.7 | 110.3 | 44 | 60.0 | 60.0 | -0.0190 | -0.0170 | 0.717 | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 0.5 | 0.9 | 0.9 | 1 | 60.0 | 60.0 | -0.0286 | -0.0289 | 0.000 | 0.00 |
| FABRIK (in-repo; no genuine upstream) | 40.0 | 40.0 | 40.0 | 93.7 | 143.4 | 143.4 | 99 | 80.0 | 80.0 | -0.0339 | -0.0319 | 107.624 | 0.60 |
| Jacobian DLS (real RTB LM, single-shot) | 40.0 | 40.0 | 40.0 | 0.6 | 0.8 | 0.9 | 52 | 40.0 | 40.0 | -0.0080 | -0.0060 | 290.674 | 0.00 |
| CCD (in-repo; no genuine upstream) | 20.0 | 20.0 | 20.0 | 181.5 | 227.9 | 228.3 | 243 | 40.0 | 40.0 | -0.0172 | -0.0180 | 64.242 | 0.80 |
| PyBullet native IK |   –   | 60.0 | 60.0 | 4.0 | 7.1 | 7.1 |   –   | 60.0 | 60.0 | -0.0280 | -0.0260 | 5.724 |   –   |

## franka_panda — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM restarts) | 100.0 | 100.0 | 100.0 | 0.9 | 1.3 | 1.3 | 98 | 20.0 | 20.0 | -0.0007 | 0.0013 | 0.001 | 0.00 |
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 15.6 | 39.1 | 44.3 | 96 | 20.0 | 20.0 | -0.0019 | 0.0001 | 0.588 | 0.40 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 15.3 | 38.3 | 43.4 | 96 | 20.0 | 20.0 | -0.0019 | 0.0001 | 0.588 | 0.40 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 9.7 | 15.8 | 16.4 | 85 | 20.0 | 20.0 | -0.0015 | 0.0005 | 0.439 | 0.40 |
| ProteinIK (V1) | 100.0 | 100.0 | 100.0 | 31.8 | 59.7 | 60.0 | 36 | 40.0 | 40.0 | -0.0107 | -0.0087 | 0.532 | 0.20 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 0.7 | 1.0 | 1.0 | 1 | 20.0 | 20.0 | -0.0029 | -0.0053 | 0.003 | 0.20 |
| CCD (in-repo; no genuine upstream) | 20.0 | 20.0 | 20.0 | 237.4 | 289.0 | 289.5 | 249 | 40.0 | 40.0 | -0.0062 | -0.0042 | 8.648 | 1.40 |
| FABRIK (in-repo; no genuine upstream) | 0.0 | 0.0 | 0.0 | 199.0 | 201.0 | 201.2 | 150 | 20.0 | 20.0 | 0.0060 | 0.0038 | 16.387 | 1.40 |
| Jacobian DLS (real RTB LM, single-shot) | 0.0 | 0.0 | 0.0 | 0.7 | 0.8 | 0.8 | 12 | 0.0 | 0.0 | 0.0184 | 0.0204 | 911.829 | 0.00 |
| PyBullet native IK |   –   | 100.0 | 100.0 | 2.8 | 7.8 | 9.0 |   –   | 40.0 | 40.0 | -0.0158 | -0.0138 | 0.012 |   –   |

## franka_panda — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM restarts) | 100.0 | 100.0 | 100.0 | 0.8 | 0.9 | 0.9 | 61 | 0.0 | 0.0 | 0.0192 | 0.0169 | 0.000 | 0.00 |
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 12.2 | 20.4 | 21.8 | 108 | 0.0 | 0.0 | 0.0151 | 0.0126 | 0.531 | 0.40 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 12.2 | 20.2 | 21.5 | 108 | 0.0 | 0.0 | 0.0151 | 0.0126 | 0.531 | 0.40 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 12.2 | 20.3 | 21.7 | 108 | 0.0 | 0.0 | 0.0151 | 0.0126 | 0.531 | 0.40 |
| ProteinIK (V1) | 100.0 | 100.0 | 100.0 | 31.5 | 51.4 | 53.1 | 35 | 20.0 | 0.0 | 0.0152 | 0.0172 | 0.657 | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 1.1 | 2.3 | 2.3 | 1 | 0.0 | 0.0 | 0.0190 | 0.0169 | 0.002 | 0.00 |
| Jacobian DLS (real RTB LM, single-shot) | 20.0 | 20.0 | 20.0 | 0.8 | 1.0 | 1.1 | 52 | 0.0 | 0.0 | 0.0175 | 0.0194 | 544.610 | 0.00 |
| CCD (in-repo; no genuine upstream) | 0.0 | 0.0 | 0.0 | 283.4 | 284.0 | 284.0 | 300 | 40.0 | 40.0 | 0.0090 | 0.0068 | 18.380 | 2.20 |
| FABRIK (in-repo; no genuine upstream) | 0.0 | 0.0 | 0.0 | 194.3 | 198.1 | 198.2 | 150 | 0.0 | 0.0 | 0.0197 | 0.0177 | 193.520 | 2.00 |
| PyBullet native IK |   –   | 80.0 | 80.0 | 5.6 | 9.4 | 9.4 |   –   | 0.0 | 0.0 | 0.0181 | 0.0201 | 30.100 |   –   |

## franka_panda — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM restarts) | 100.0 | 100.0 | 100.0 | 1.2 | 2.4 | 2.7 | 171 | 80.0 | 80.0 | -0.0425 | -0.0406 | 0.001 | 0.00 |
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 40.6 | 101.2 | 106.7 | 138 | 60.0 | 60.0 | -0.0360 | -0.0340 | 0.269 | 0.20 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 178.9 | 426.5 | 460.6 | 313 | 60.0 | 60.0 | -0.0361 | -0.0341 | 0.179 | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 39.8 | 99.3 | 104.7 | 138 | 60.0 | 60.0 | -0.0360 | -0.0340 | 0.269 | 0.20 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 1.8 | 3.6 | 3.7 | 1 | 80.0 | 80.0 | -0.0619 | -0.0599 | 0.002 | 0.20 |
| ProteinIK (V1) | 80.0 | 80.0 | 80.0 | 75.1 | 193.0 | 221.5 | 72 | 80.0 | 80.0 | -0.0698 | -0.0719 | 11.801 | 0.20 |
| FABRIK (in-repo; no genuine upstream) | 40.0 | 40.0 | 40.0 | 147.8 | 198.8 | 199.1 | 112 | 100.0 | 100.0 | -0.0456 | -0.0436 | 63.927 | 1.00 |
| CCD (in-repo; no genuine upstream) | 0.0 | 0.0 | 0.0 | 278.2 | 279.4 | 279.5 | 300 | 80.0 | 80.0 | -0.0379 | -0.0359 | 9.822 | 1.40 |
| Jacobian DLS (real RTB LM, single-shot) | 0.0 | 0.0 | 0.0 | 0.7 | 0.8 | 0.9 | 16 | 20.0 | 20.0 | 0.0144 | 0.0120 | 876.895 | 0.00 |
| PyBullet native IK |   –   | 100.0 | 100.0 | 2.6 | 5.7 | 6.2 |   –   | 80.0 | 80.0 | -0.0588 | -0.0609 | 0.009 |   –   |
