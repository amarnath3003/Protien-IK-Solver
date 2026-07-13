# Master Sim Benchmark — every solver × arm × scenario, scored in PyBullet + MuJoCo

- **100** trials/seed × seeds **[1, 2, 3]** (n=300 per cell)  |  warm-up 8 untimed solves/cell
- Arms: planar3dof, ur5, franka_panda  |  Scenarios: open_space, near_singular, cluttered
- Engines: PyBullet + MuJoCo  |  generated 2026-07-13 19:11:45

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
| planar3dof | open_space | — | — |
| planar3dof | near_singular | — | — |
| planar3dof | cluttered | — | — |
| ur5 | open_space | ProteinIK Fast (V4 real-calib) | ProteinIK Fast (V4 real-calib) |
| ur5 | near_singular | ProteinIK Fast (V4 real-calib) | ProteinIK Fast (V4 real-calib) |
| ur5 | cluttered | ProteinIK Fast (V4 real-calib) | ProteinIK Fast (V4 real-calib) |
| franka_panda | open_space | Multi-start (real RTB ik_LM restarts) | Multi-start (real RTB ik_LM restarts) |
| franka_panda | near_singular | TRAC-IK (real C++ TRACLabs) | Multi-start (real RTB ik_LM restarts) |
| franka_panda | cluttered | TRAC-IK (real C++ TRACLabs) | TRAC-IK (real C++ TRACLabs) |

## planar3dof — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Analytical IK (Planar 3-DOF, exact) | 100.0 |   –   |   –   | 0.1 | 0.1 | 0.1 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| Multi-start (real RTB ik_LM restarts) | 100.0 |   –   |   –   | 0.2 | 0.2 | 0.3 | 18 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4) | 100.0 |   –   |   –   | 10.7 | 35.2 | 184.0 | 58 |   –   |   –   |   –   |   –   |   –   | 0.01 |
| ProteinIK Fast (V4 real-calib) | 100.0 |   –   |   –   | 10.8 | 36.4 | 188.2 | 58 |   –   |   –   |   –   |   –   |   –   | 0.01 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 |   –   |   –   | 11.1 | 23.0 | 199.2 | 56 |   –   |   –   |   –   |   –   |   –   | 0.01 |
| TRAC-IK (real C++ TRACLabs) | 100.0 |   –   |   –   | 0.2 | 0.3 | 0.3 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK (V1) | 98.3 |   –   |   –   | 14.7 | 45.8 | 90.3 | 38 |   –   |   –   |   –   |   –   |   –   | 0.02 |
| Jacobian DLS (real RTB LM, single-shot) | 66.3 |   –   |   –   | 0.2 | 0.3 | 0.3 | 11 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| CCD (in-repo; no genuine upstream) | 49.0 |   –   |   –   | 53.6 | 96.0 | 97.0 | 171 |   –   |   –   |   –   |   –   |   –   | 0.62 |
| FABRIK (in-repo; no genuine upstream) | 44.0 |   –   |   –   | 40.4 | 67.0 | 68.2 | 93 |   –   |   –   |   –   |   –   |   –   | 0.77 |

## planar3dof — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Analytical IK (Planar 3-DOF, exact) | 100.0 |   –   |   –   | 0.1 | 0.1 | 0.1 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| Multi-start (real RTB ik_LM restarts) | 100.0 |   –   |   –   | 0.2 | 0.3 | 0.3 | 25 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4) | 100.0 |   –   |   –   | 63.5 | 251.8 | 713.1 | 183 |   –   |   –   |   –   |   –   |   –   | 0.01 |
| ProteinIK Fast (V4 real-calib) | 100.0 |   –   |   –   | 63.3 | 244.8 | 721.6 | 183 |   –   |   –   |   –   |   –   |   –   | 0.01 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 |   –   |   –   | 51.5 | 222.3 | 719.7 | 151 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 |   –   |   –   | 0.2 | 0.3 | 0.4 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK (V1) | 86.3 |   –   |   –   | 39.0 | 91.4 | 93.4 | 89 |   –   |   –   |   –   |   –   |   –   | 0.17 |
| Jacobian DLS (real RTB LM, single-shot) | 64.3 |   –   |   –   | 0.2 | 0.2 | 0.2 | 14 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| CCD (in-repo; no genuine upstream) | 26.0 |   –   |   –   | 75.2 | 95.8 | 97.4 | 240 |   –   |   –   |   –   |   –   |   –   | 0.75 |
| FABRIK (in-repo; no genuine upstream) | 22.7 |   –   |   –   | 55.1 | 67.8 | 69.4 | 126 |   –   |   –   |   –   |   –   |   –   | 0.90 |

## planar3dof — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Analytical IK (Planar 3-DOF, exact) | 100.0 |   –   |   –   | 0.1 | 0.1 | 0.1 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| Multi-start (real RTB ik_LM restarts) | 100.0 |   –   |   –   | 0.2 | 0.2 | 0.3 | 20 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4) | 100.0 |   –   |   –   | 66.1 | 266.7 | 472.8 | 219 |   –   |   –   |   –   |   –   |   –   | 0.01 |
| ProteinIK Fast (V4 real-calib) | 100.0 |   –   |   –   | 66.0 | 268.1 | 476.8 | 219 |   –   |   –   |   –   |   –   |   –   | 0.01 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 |   –   |   –   | 43.0 | 184.8 | 328.7 | 173 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 |   –   |   –   | 0.2 | 0.3 | 0.4 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK (V1) | 91.7 |   –   |   –   | 25.6 | 90.7 | 92.1 | 61 |   –   |   –   |   –   |   –   |   –   | 0.08 |
| Jacobian DLS (real RTB LM, single-shot) | 67.7 |   –   |   –   | 0.2 | 0.2 | 0.2 | 12 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| FABRIK (in-repo; no genuine upstream) | 24.3 |   –   |   –   | 51.2 | 67.3 | 68.6 | 118 |   –   |   –   |   –   |   –   |   –   | 1.16 |
| CCD (in-repo; no genuine upstream) | 21.7 |   –   |   –   | 75.1 | 96.4 | 98.9 | 240 |   –   |   –   |   –   |   –   |   –   | 0.99 |

## ur5 — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM restarts) | 100.0 | 100.0 | 100.0 | 0.6 | 0.7 | 0.9 | 35 | 35.3 | 33.0 | -0.0041 | -0.0034 | 0.001 | 0.00 |
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 12.6 | 24.6 | 235.6 | 54 | 28.0 | 26.3 | 0.0012 | 0.0018 | 0.258 | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 101.8 | 390.2 | 586.8 | 254 | 25.7 | 23.3 | 0.0026 | 0.0033 | 0.295 | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 10.6 | 20.3 | 225.6 | 52 | 27.7 | 26.0 | 0.0012 | 0.0017 | 0.257 | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 99.3 | 99.3 | 99.3 | 0.6 | 1.5 | 4.3 | 1 | 37.0 | 35.3 | -0.0042 | -0.0037 | 6.463 | 0.00 |
| ProteinIK (V1) | 94.0 | 94.0 | 94.0 | 37.8 | 182.1 | 185.5 | 48 | 32.3 | 31.3 | -0.0022 | -0.0012 | 24.069 | 0.03 |
| Jacobian DLS (real RTB LM, single-shot) | 72.3 | 72.3 | 72.3 | 0.6 | 0.9 | 0.9 | 38 | 39.3 | 38.0 | -0.0049 | -0.0039 | 238.974 | 0.00 |
| FABRIK (in-repo; no genuine upstream) | 49.7 | 49.7 | 49.7 | 90.3 | 147.6 | 149.6 | 94 | 43.3 | 41.3 | -0.0072 | -0.0061 | 48.643 | 0.45 |
| CCD (in-repo; no genuine upstream) | 43.7 | 43.7 | 43.7 | 143.2 | 224.9 | 231.9 | 195 | 38.0 | 37.3 | -0.0069 | -0.0059 | 39.706 | 0.59 |
| PyBullet native IK |   –   | 69.3 | 69.3 | 4.1 | 7.0 | 7.1 |   –   | 43.7 | 42.3 | -0.0064 | -0.0053 | 11.968 |   –   |

## ur5 — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM restarts) | 100.0 | 100.0 | 100.0 | 0.6 | 0.8 | 1.0 | 40 | 45.3 | 44.7 | -0.0143 | -0.0134 | 0.001 | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 117.6 | 479.1 | 727.9 | 265 | 34.0 | 32.0 | -0.0041 | -0.0036 | 0.387 | 0.01 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 17.0 | 44.7 | 249.2 | 74 | 39.3 | 38.0 | -0.0064 | -0.0055 | 0.375 | 0.01 |
| ProteinIK Fast (V4) | 100.0 | 99.7 | 99.7 | 24.3 | 166.2 | 305.3 | 81 | 38.3 | 37.3 | -0.0054 | -0.0046 | 0.358 | 0.01 |
| TRAC-IK (real C++ TRACLabs) | 99.7 | 99.7 | 99.7 | 0.5 | 1.1 | 2.1 | 1 | 46.0 | 44.7 | -0.0151 | -0.0144 | 1.180 | 0.00 |
| ProteinIK (V1) | 90.7 | 90.7 | 90.7 | 56.8 | 182.9 | 188.5 | 67 | 39.7 | 37.3 | -0.0086 | -0.0080 | 29.699 | 0.08 |
| Jacobian DLS (real RTB LM, single-shot) | 69.7 | 69.7 | 69.7 | 0.6 | 0.9 | 0.9 | 39 | 47.7 | 46.7 | -0.0122 | -0.0113 | 233.102 | 0.00 |
| FABRIK (in-repo; no genuine upstream) | 34.7 | 34.7 | 34.7 | 107.4 | 143.2 | 145.1 | 114 | 53.0 | 51.0 | -0.0168 | -0.0156 | 51.907 | 0.51 |
| CCD (in-repo; no genuine upstream) | 32.0 | 32.0 | 32.0 | 165.8 | 224.8 | 229.2 | 227 | 50.0 | 48.3 | -0.0181 | -0.0172 | 37.322 | 0.61 |
| PyBullet native IK |   –   | 46.0 | 46.0 | 5.9 | 7.0 | 7.2 |   –   | 51.3 | 50.3 | -0.0160 | -0.0148 | 13.372 |   –   |

## ur5 — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM restarts) | 100.0 | 100.0 | 100.0 | 0.6 | 0.7 | 0.8 | 28 | 71.7 | 71.0 | -0.0331 | -0.0317 | 0.001 | 0.00 |
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 23.7 | 113.4 | 271.9 | 82 | 57.7 | 56.7 | -0.0184 | -0.0180 | 0.314 | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 88.6 | 287.1 | 534.1 | 217 | 55.0 | 54.0 | -0.0171 | -0.0169 | 0.364 | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 15.1 | 48.5 | 203.8 | 70 | 57.3 | 56.3 | -0.0191 | -0.0187 | 0.323 | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 0.4 | 0.9 | 1.5 | 1 | 74.0 | 71.3 | -0.0369 | -0.0357 | 0.002 | 0.00 |
| ProteinIK (V1) | 89.7 | 89.7 | 89.7 | 59.5 | 183.2 | 185.0 | 70 | 66.3 | 64.0 | -0.0261 | -0.0249 | 14.136 | 0.05 |
| Jacobian DLS (real RTB LM, single-shot) | 77.0 | 77.0 | 77.0 | 0.6 | 0.7 | 0.9 | 27 | 64.3 | 63.3 | -0.0283 | -0.0271 | 132.837 | 0.00 |
| CCD (in-repo; no genuine upstream) | 41.0 | 41.0 | 41.0 | 148.6 | 220.6 | 224.5 | 204 | 70.3 | 67.7 | -0.0375 | -0.0363 | 22.268 | 0.51 |
| FABRIK (in-repo; no genuine upstream) | 37.0 | 37.0 | 37.0 | 100.8 | 144.5 | 148.7 | 107 | 68.0 | 67.0 | -0.0328 | -0.0319 | 43.424 | 0.47 |
| PyBullet native IK |   –   | 55.0 | 55.0 | 5.5 | 7.1 | 7.7 |   –   | 72.0 | 71.7 | -0.0369 | -0.0352 | 6.753 |   –   |

## franka_panda — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 99.7 | 99.7 | 19.3 | 126.5 | 296.8 | 77 | 8.7 | 8.7 | 0.0150 | 0.0122 | 0.373 | 0.29 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 99.7 | 99.7 | 20.8 | 129.2 | 319.0 | 79 | 8.7 | 8.7 | 0.0150 | 0.0124 | 0.367 | 0.29 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 99.7 | 99.7 | 19.3 | 126.7 | 295.3 | 77 | 8.7 | 8.7 | 0.0150 | 0.0122 | 0.370 | 0.29 |
| Multi-start (real RTB ik_LM restarts) | 99.3 | 99.3 | 99.3 | 0.9 | 1.2 | 1.5 | 72 | 7.0 | 5.3 | 0.0157 | 0.0148 | 3.702 | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 99.3 | 99.3 | 99.3 | 0.8 | 2.4 | 4.5 | 1 | 10.0 | 8.7 | 0.0142 | 0.0135 | 3.510 | 0.10 |
| ProteinIK (V1) | 98.0 | 98.0 | 98.0 | 58.1 | 156.5 | 214.7 | 59 | 10.3 | 8.7 | 0.0146 | 0.0138 | 2.605 | 0.12 |
| Jacobian DLS (real RTB LM, single-shot) | 28.3 | 28.3 | 28.3 | 0.8 | 1.2 | 1.2 | 36 | 8.3 | 7.0 | 0.0148 | 0.0134 | 561.739 | 0.00 |
| CCD (in-repo; no genuine upstream) | 23.0 | 23.0 | 23.0 | 221.4 | 272.5 | 275.3 | 249 | 19.0 | 17.3 | 0.0106 | 0.0092 | 24.512 | 1.16 |
| FABRIK (in-repo; no genuine upstream) | 18.0 | 18.0 | 18.0 | 164.6 | 193.4 | 194.7 | 131 | 20.7 | 20.0 | 0.0106 | 0.0101 | 49.261 | 1.28 |
| PyBullet native IK |   –   | 78.0 | 78.0 | 4.5 | 9.2 | 9.6 |   –   | 32.0 | 31.0 | -0.0045 | -0.0053 | 13.261 |   –   |

## franka_panda — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 24.9 | 134.6 | 313.6 | 78 | 9.3 | 8.3 | 0.0144 | 0.0131 | 0.424 | 0.23 |
| ProteinIK Fast (V4) | 99.7 | 99.3 | 99.3 | 26.2 | 132.3 | 528.0 | 81 | 9.0 | 8.0 | 0.0144 | 0.0131 | 0.431 | 0.23 |
| ProteinIK Fast (V4+o2 IAM) | 99.7 | 99.3 | 99.3 | 26.5 | 134.9 | 543.4 | 81 | 9.0 | 8.0 | 0.0144 | 0.0131 | 0.431 | 0.23 |
| Multi-start (real RTB ik_LM restarts) | 98.0 | 98.0 | 98.0 | 1.0 | 1.8 | 4.4 | 121 | 8.0 | 6.7 | 0.0155 | 0.0143 | 16.456 | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 98.0 | 98.0 | 98.0 | 1.3 | 4.1 | 5.1 | 1 | 7.7 | 7.3 | 0.0148 | 0.0140 | 21.122 | 0.06 |
| ProteinIK (V1) | 93.0 | 93.0 | 93.0 | 69.5 | 221.9 | 265.9 | 65 | 9.7 | 7.3 | 0.0148 | 0.0140 | 15.085 | 0.16 |
| Jacobian DLS (real RTB LM, single-shot) | 31.0 | 31.0 | 31.0 | 0.8 | 1.2 | 1.2 | 34 | 12.0 | 10.3 | 0.0143 | 0.0133 | 560.523 | 0.00 |
| CCD (in-repo; no genuine upstream) | 11.7 | 11.7 | 11.7 | 266.4 | 299.0 | 348.7 | 280 | 15.0 | 14.3 | 0.0123 | 0.0122 | 26.662 | 1.08 |
| FABRIK (in-repo; no genuine upstream) | 11.3 | 11.3 | 11.3 | 184.8 | 198.5 | 201.6 | 141 | 20.0 | 19.7 | 0.0094 | 0.0095 | 55.857 | 1.19 |
| PyBullet native IK |   –   | 61.7 | 61.7 | 7.4 | 9.5 | 9.7 |   –   | 22.3 | 22.0 | 0.0038 | 0.0029 | 17.523 |   –   |

## franka_panda — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4+o2 IAM) | 99.0 | 98.7 | 98.7 | 99.3 | 565.5 | 712.9 | 202 | 80.7 | 80.0 | -0.0471 | -0.0458 | 0.678 | 0.32 |
| ProteinIK Fast (V4) | 99.0 | 98.3 | 98.3 | 110.5 | 563.0 | 728.6 | 217 | 80.7 | 80.0 | -0.0475 | -0.0463 | 0.685 | 0.33 |
| ProteinIK Fast (V4 real-calib) | 98.0 | 97.7 | 97.7 | 150.3 | 627.5 | 849.4 | 275 | 79.7 | 79.0 | -0.0466 | -0.0455 | 0.513 | 0.29 |
| Multi-start (real RTB ik_LM restarts) | 93.0 | 93.0 | 93.0 | 1.3 | 4.3 | 6.4 | 229 | 75.7 | 75.0 | -0.0483 | -0.0474 | 48.643 | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 91.3 | 91.3 | 91.3 | 1.4 | 5.1 | 5.1 | 1 | 74.3 | 73.7 | -0.0441 | -0.0428 | 72.348 | 0.07 |
| ProteinIK (V1) | 83.3 | 83.3 | 83.3 | 98.1 | 228.2 | 236.1 | 91 | 75.0 | 75.0 | -0.0433 | -0.0425 | 47.723 | 0.44 |
| FABRIK (in-repo; no genuine upstream) | 22.7 | 22.7 | 22.7 | 161.5 | 198.9 | 199.5 | 123 | 76.7 | 76.3 | -0.0426 | -0.0416 | 50.023 | 1.45 |
| Jacobian DLS (real RTB LM, single-shot) | 19.0 | 19.0 | 19.0 | 0.8 | 0.8 | 1.2 | 22 | 21.3 | 19.7 | 0.0036 | 0.0018 | 618.901 | 0.00 |
| CCD (in-repo; no genuine upstream) | 12.3 | 12.3 | 12.3 | 254.5 | 282.9 | 284.1 | 272 | 82.7 | 82.0 | -0.0513 | -0.0500 | 18.656 | 1.71 |
| PyBullet native IK |   –   | 92.3 | 92.3 | 3.1 | 9.1 | 9.2 |   –   | 86.0 | 85.7 | -0.0644 | -0.0631 | 4.313 |   –   |
