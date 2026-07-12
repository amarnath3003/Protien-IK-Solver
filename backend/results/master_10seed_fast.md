# Master Sim Benchmark — every solver × arm × scenario, scored in PyBullet + MuJoCo

- **100** trials/seed × seeds **[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]** (n=1000 per cell)  |  warm-up 8 untimed solves/cell
- Arms: ur5, franka_panda  |  Scenarios: open_space, near_singular, cluttered
- Engines: PyBullet + MuJoCo  |  generated 2026-07-10 18:29:27

Each solver runs **once** on our DH `RobotSpec` core; that `q_final` is then scored by two independent real-mesh oracles — **PB** = PyBullet, **MJ** = MuJoCo (identical URDF & non-adjacent link pairs). The capsule proxy the solvers optimize against is **not** used to evaluate them here — only real-mesh collision counts (planar3dof has no URDF, so it carries success/speed only). `PyBullet native IK` is the sim's own solver on the identical targets. Timing is wall-clock (OS noise on mean/p95/p99); success/collision/error columns are deterministic given the seed.

## Oracle validation — DH ≡ PyBullet ≡ MuJoCo

### A. Forward-kinematics agreement

| Arm | n | DH↔PB resid | DH↔MJ resid | PB↔MJ max pos | max orient |
|:--|--:|--:|--:|--:|--:|
| franka_panda | 2000 | 6.6e-07 (tool) | 8.0e-16 (tool) | 5.90e-08 m | 4.09e-07 rad |

### B. Self-collision agreement — PyBullet vs MuJoCo

| Arm | n | PB col% | MJ col% | PB↔MJ sign-agree% | PB↔MJ corr |
|:--|--:|--:|--:|--:|--:|
| franka_panda | 2000 | 9.2 | 8.3 | 99.1 | 0.876 |

## Verdict — lowest real-mesh-collision solver per cell (among ≥90% success)

| Arm | Scenario | **PyBullet** | **MuJoCo** |
|:--|:--|:--|:--|
| ur5 | open_space | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| ur5 | near_singular | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| ur5 | cluttered | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| franka_panda | open_space | Multi-start | Multi-start |
| franka_panda | near_singular | Multi-start | Multi-start |
| franka_panda | cluttered | ProteinIK Fast (V4) | ProteinIK Fast (V4) |

## ur5 — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 99.8 | 99.8 | 17.2 | 39.9 | 359.0 | 56 | 28.6 | 26.3 | 0.0010 | 0.0017 | 0.268 | 0.00 |
| TRAC-IK style | 99.0 | 99.0 | 99.0 | 13.7 | 48.5 | 93.7 | 42 | 31.1 | 29.4 | -0.0033 | -0.0026 | 0.729 | 0.01 |
| Multi-start | 97.8 | 97.8 | 97.8 | 99.2 | 150.3 | 165.1 | 294 | 30.7 | 29.5 | -0.0021 | -0.0012 | 1.268 | 0.01 |
| ProteinIK (V1) | 94.5 | 94.5 | 94.5 | 40.4 | 200.7 | 206.3 | 46 | 32.6 | 30.2 | -0.0016 | -0.0010 | 17.613 | 0.03 |
| Jacobian (DLS) | 52.2 | 52.2 | 52.2 | 34.5 | 69.4 | 70.9 | 105 | 36.8 | 34.0 | -0.0052 | -0.0046 | 70.420 | 0.55 |
| FABRIK | 47.4 | 47.3 | 47.4 | 101.5 | 162.4 | 167.0 | 95 | 40.7 | 38.9 | -0.0064 | -0.0055 | 49.635 | 0.48 |
| CCD | 45.1 | 45.1 | 45.1 | 161.1 | 261.5 | 262.9 | 191 | 36.9 | 35.0 | -0.0057 | -0.0048 | 36.639 | 0.55 |
| PyBullet native IK |   –   | 72.4 | 72.4 | 8.0 | 14.5 | 15.1 |   –   | 43.0 | 40.8 | -0.0068 | -0.0058 | 11.094 |   –   |

## ur5 — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 99.8 | 99.7 | 99.7 | 29.6 | 203.3 | 502.0 | 82 | 40.1 | 38.5 | -0.0079 | -0.0071 | 0.374 | 0.01 |
| TRAC-IK style | 97.2 | 97.2 | 97.2 | 19.2 | 69.9 | 97.4 | 59 | 47.4 | 46.1 | -0.0155 | -0.0145 | 0.875 | 0.03 |
| Multi-start | 96.4 | 96.4 | 96.4 | 107.9 | 149.5 | 155.7 | 335 | 44.0 | 42.9 | -0.0124 | -0.0116 | 1.120 | 0.02 |
| ProteinIK (V1) | 89.4 | 89.4 | 89.4 | 68.3 | 205.8 | 208.5 | 72 | 42.6 | 40.8 | -0.0108 | -0.0101 | 31.873 | 0.07 |
| Jacobian (DLS) | 53.2 | 53.2 | 53.2 | 37.6 | 69.8 | 70.1 | 109 | 48.6 | 47.3 | -0.0167 | -0.0157 | 68.235 | 0.52 |
| FABRIK | 36.2 | 36.2 | 36.2 | 123.0 | 170.8 | 171.3 | 111 | 54.4 | 53.0 | -0.0182 | -0.0172 | 48.175 | 0.44 |
| CCD | 35.7 | 35.7 | 35.7 | 190.2 | 263.6 | 267.1 | 220 | 51.2 | 49.3 | -0.0191 | -0.0182 | 31.329 | 0.56 |
| PyBullet native IK |   –   | 45.8 | 45.8 | 11.9 | 14.2 | 14.6 |   –   | 52.8 | 51.3 | -0.0178 | -0.0168 | 11.886 |   –   |

## ur5 — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 33.5 | 227.3 | 372.8 | 92 | 59.6 | 58.1 | -0.0203 | -0.0193 | 0.337 | 0.01 |
| Multi-start | 98.1 | 98.3 | 98.3 | 102.2 | 146.1 | 157.8 | 316 | 67.2 | 65.8 | -0.0279 | -0.0269 | 0.698 | 0.01 |
| TRAC-IK style | 97.9 | 97.9 | 97.9 | 16.4 | 61.7 | 94.7 | 52 | 71.0 | 70.0 | -0.0340 | -0.0329 | 0.766 | 0.02 |
| ProteinIK (V1) | 88.2 | 88.2 | 88.2 | 73.3 | 218.7 | 220.1 | 73 | 63.4 | 61.5 | -0.0258 | -0.0248 | 20.077 | 0.05 |
| Jacobian (DLS) | 60.7 | 60.7 | 60.7 | 31.1 | 65.8 | 67.7 | 97 | 70.9 | 70.0 | -0.0361 | -0.0350 | 31.985 | 0.43 |
| CCD | 41.3 | 41.3 | 41.3 | 176.1 | 264.0 | 289.7 | 206 | 73.6 | 71.4 | -0.0401 | -0.0389 | 18.703 | 0.50 |
| FABRIK | 39.1 | 39.1 | 39.1 | 114.8 | 170.0 | 171.5 | 106 | 71.4 | 70.0 | -0.0366 | -0.0354 | 33.294 | 0.43 |
| PyBullet native IK |   –   | 54.0 | 54.0 | 11.8 | 14.9 | 15.6 |   –   | 74.4 | 73.3 | -0.0377 | -0.0365 | 5.843 |   –   |

## franka_panda — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 99.8 | 99.7 | 99.7 | 22.3 | 140.4 | 379.9 | 75 | 7.5 | 6.7 | 0.0150 | 0.0138 | 0.390 | 0.27 |
| TRAC-IK style | 98.5 | 98.5 | 98.5 | 19.0 | 60.9 | 109.0 | 52 | 7.4 | 6.8 | 0.0152 | 0.0138 | 0.781 | 0.30 |
| Multi-start | 97.0 | 97.0 | 97.0 | 124.7 | 169.8 | 184.1 | 334 | 6.8 | 6.2 | 0.0154 | 0.0138 | 0.957 | 0.12 |
| ProteinIK (V1) | 96.9 | 96.9 | 96.9 | 61.8 | 187.9 | 249.4 | 56 | 8.3 | 7.2 | 0.0146 | 0.0132 | 7.724 | 0.14 |
| Jacobian (DLS) | 47.6 | 47.6 | 47.6 | 45.3 | 78.4 | 78.7 | 117 | 13.1 | 12.1 | 0.0120 | 0.0117 | 144.679 | 1.63 |
| CCD | 27.1 | 27.1 | 27.1 | 249.4 | 322.3 | 333.6 | 242 | 18.3 | 17.5 | 0.0102 | 0.0093 | 25.286 | 1.10 |
| FABRIK | 21.1 | 21.1 | 21.1 | 182.1 | 221.2 | 228.5 | 127 | 18.5 | 18.3 | 0.0105 | 0.0098 | 53.316 | 1.30 |
| PyBullet native IK |   –   | 80.8 | 80.8 | 6.0 | 12.9 | 14.0 |   –   | 29.5 | 28.9 | -0.0028 | -0.0037 | 12.327 |   –   |

## franka_panda — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 99.5 | 99.4 | 99.4 | 31.8 | 161.7 | 652.4 | 80 | 10.0 | 8.9 | 0.0143 | 0.0135 | 0.442 | 0.23 |
| TRAC-IK style | 97.0 | 97.0 | 97.0 | 24.3 | 93.8 | 115.1 | 64 | 10.2 | 9.6 | 0.0140 | 0.0131 | 0.975 | 0.29 |
| Multi-start | 95.1 | 95.1 | 95.1 | 126.2 | 170.8 | 174.4 | 351 | 8.0 | 7.1 | 0.0147 | 0.0137 | 1.307 | 0.12 |
| ProteinIK (V1) | 92.5 | 92.5 | 92.5 | 76.3 | 243.6 | 260.3 | 67 | 9.3 | 8.2 | 0.0145 | 0.0136 | 19.263 | 0.14 |
| Jacobian (DLS) | 48.9 | 48.9 | 48.9 | 42.4 | 73.8 | 76.1 | 117 | 13.1 | 12.5 | 0.0122 | 0.0117 | 118.297 | 1.39 |
| CCD | 15.5 | 15.5 | 15.5 | 282.4 | 324.0 | 325.8 | 273 | 17.2 | 16.7 | 0.0112 | 0.0109 | 24.856 | 1.07 |
| FABRIK | 12.7 | 12.7 | 12.7 | 207.7 | 228.6 | 230.0 | 139 | 19.3 | 18.9 | 0.0101 | 0.0097 | 51.406 | 1.20 |
| PyBullet native IK |   –   | 62.3 | 62.3 | 10.4 | 14.0 | 14.2 |   –   | 23.2 | 22.9 | 0.0015 | 0.0011 | 16.750 |   –   |

## franka_panda — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 98.7 | 98.3 | 98.3 | 123.1 | 656.0 | 832.5 | 222 | 81.7 | 81.2 | -0.0467 | -0.0454 | 0.674 | 0.30 |
| TRAC-IK style | 92.6 | 92.6 | 92.6 | 28.6 | 105.4 | 108.4 | 81 | 82.4 | 81.8 | -0.0502 | -0.0490 | 1.625 | 0.45 |
| Multi-start | 88.4 | 88.4 | 88.4 | 132.9 | 175.9 | 183.5 | 363 | 82.0 | 81.3 | -0.0492 | -0.0481 | 2.558 | 0.30 |
| ProteinIK (V1) | 85.5 | 85.5 | 85.5 | 97.8 | 250.5 | 256.3 | 84 | 78.1 | 77.4 | -0.0441 | -0.0430 | 38.987 | 0.39 |
| Jacobian (DLS) | 37.4 | 37.4 | 37.4 | 51.4 | 78.9 | 81.6 | 135 | 71.9 | 71.5 | -0.0427 | -0.0416 | 116.618 | 2.04 |
| FABRIK | 20.5 | 20.5 | 20.5 | 175.4 | 221.5 | 224.2 | 125 | 81.1 | 80.7 | -0.0454 | -0.0441 | 50.258 | 1.58 |
| CCD | 15.0 | 15.0 | 15.0 | 275.4 | 325.6 | 337.0 | 264 | 83.9 | 83.3 | -0.0511 | -0.0499 | 21.744 | 1.69 |
| PyBullet native IK |   –   | 94.6 | 94.6 | 4.1 | 12.5 | 12.8 |   –   | 87.6 | 87.3 | -0.0654 | -0.0639 | 3.588 |   –   |
