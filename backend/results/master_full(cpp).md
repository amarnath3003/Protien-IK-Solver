# Master Sim Benchmark — every solver × arm × scenario, scored in PyBullet + MuJoCo

- **100** trials/seed × seeds **[1, 2, 3]** (n=300 per cell)  |  warm-up 8 untimed solves/cell
- Arms: planar3dof, ur5, franka_panda  |  Scenarios: open_space, near_singular, cluttered
- Engines: PyBullet + MuJoCo  |  generated 2026-07-13 20:09:40

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
| ur5 | open_space | ProteinIK Raw Biology (V6) | ProteinIK Raw Biology (V6) |
| ur5 | near_singular | ProteinIK Raw Biology (V6) | ProteinIK Raw Biology (V6) |
| ur5 | cluttered | ProteinIK Raw Biology (V6) | ProteinIK Raw Biology (V6) |
| franka_panda | open_space | Multi-start (real RTB ik_LM restarts) | Multi-start (real RTB ik_LM restarts) |
| franka_panda | near_singular | ProteinIK Raw Biology (V6) | Multi-start (real RTB ik_LM restarts) |
| franka_panda | cluttered | Multi-start (real RTB ik_LM restarts) | Multi-start (real RTB ik_LM restarts) |

## planar3dof — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Analytical IK (Planar 3-DOF, exact) | 100.0 |   –   |   –   | 0.1 | 0.1 | 0.1 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| Multi-start (real RTB ik_LM restarts) | 100.0 |   –   |   –   | 0.2 | 0.2 | 0.3 | 18 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4) | 100.0 |   –   |   –   | 0.1 | 0.3 | 1.2 | 70 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 |   –   |   –   | 0.1 | 0.3 | 1.2 | 70 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 |   –   |   –   | 0.1 | 0.3 | 0.8 | 67 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 |   –   |   –   | 0.2 | 0.2 | 0.4 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Raw Biology (V6) | 99.7 |   –   |   –   | 3.6 | 3.9 | 4.1 | 100 |   –   |   –   |   –   |   –   |   –   | 0.01 |
| ProteinIK (V1) | 99.3 |   –   |   –   | 0.0 | 0.1 | 0.2 | 38 |   –   |   –   |   –   |   –   |   –   | 0.01 |
| Jacobian DLS (real RTB LM, single-shot) | 66.3 |   –   |   –   | 0.2 | 0.2 | 0.2 | 11 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| CCD (in-repo; no genuine upstream) | 49.0 |   –   |   –   | 52.7 | 94.3 | 98.7 | 171 |   –   |   –   |   –   |   –   |   –   | 0.62 |
| FABRIK (in-repo; no genuine upstream) | 44.0 |   –   |   –   | 39.8 | 65.5 | 66.4 | 93 |   –   |   –   |   –   |   –   |   –   | 0.77 |

## planar3dof — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Analytical IK (Planar 3-DOF, exact) | 100.0 |   –   |   –   | 0.1 | 0.1 | 0.1 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| Multi-start (real RTB ik_LM restarts) | 100.0 |   –   |   –   | 0.2 | 0.3 | 0.3 | 26 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 |   –   |   –   | 0.2 | 0.3 | 0.4 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4) | 99.7 |   –   |   –   | 0.3 | 1.5 | 2.4 | 178 |   –   |   –   |   –   |   –   |   –   | 0.01 |
| ProteinIK Fast (V4 real-calib) | 99.7 |   –   |   –   | 0.3 | 1.5 | 2.4 | 178 |   –   |   –   |   –   |   –   |   –   | 0.01 |
| ProteinIK Fast (V4+o2 IAM) | 99.7 |   –   |   –   | 0.2 | 0.9 | 2.5 | 142 |   –   |   –   |   –   |   –   |   –   | 0.01 |
| ProteinIK Raw Biology (V6) | 99.3 |   –   |   –   | 3.7 | 3.8 | 4.0 | 100 |   –   |   –   |   –   |   –   |   –   | 0.03 |
| ProteinIK (V1) | 83.3 |   –   |   –   | 0.1 | 0.3 | 0.3 | 95 |   –   |   –   |   –   |   –   |   –   | 0.22 |
| Jacobian DLS (real RTB LM, single-shot) | 64.3 |   –   |   –   | 0.2 | 0.2 | 0.2 | 14 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| CCD (in-repo; no genuine upstream) | 26.0 |   –   |   –   | 73.9 | 94.6 | 99.3 | 240 |   –   |   –   |   –   |   –   |   –   | 0.75 |
| FABRIK (in-repo; no genuine upstream) | 22.7 |   –   |   –   | 53.6 | 66.3 | 68.0 | 126 |   –   |   –   |   –   |   –   |   –   | 0.90 |

## planar3dof — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Analytical IK (Planar 3-DOF, exact) | 100.0 |   –   |   –   | 0.1 | 0.1 | 0.1 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| Multi-start (real RTB ik_LM restarts) | 100.0 |   –   |   –   | 0.2 | 0.2 | 0.2 | 19 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4) | 100.0 |   –   |   –   | 0.3 | 1.0 | 2.0 | 208 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 |   –   |   –   | 0.3 | 1.1 | 2.0 | 208 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 |   –   |   –   | 0.2 | 0.8 | 1.7 | 174 |   –   |   –   |   –   |   –   |   –   | 0.01 |
| TRAC-IK (real C++ TRACLabs) | 100.0 |   –   |   –   | 0.2 | 0.3 | 0.3 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Raw Biology (V6) | 97.7 |   –   |   –   | 3.7 | 4.1 | 4.3 | 100 |   –   |   –   |   –   |   –   |   –   | 0.02 |
| ProteinIK (V1) | 91.0 |   –   |   –   | 0.1 | 0.3 | 0.3 | 61 |   –   |   –   |   –   |   –   |   –   | 0.10 |
| Jacobian DLS (real RTB LM, single-shot) | 67.7 |   –   |   –   | 0.2 | 0.2 | 0.2 | 12 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| FABRIK (in-repo; no genuine upstream) | 24.3 |   –   |   –   | 50.3 | 66.4 | 67.4 | 118 |   –   |   –   |   –   |   –   |   –   | 1.16 |
| CCD (in-repo; no genuine upstream) | 21.7 |   –   |   –   | 73.7 | 94.2 | 96.1 | 240 |   –   |   –   |   –   |   –   |   –   | 0.99 |

## ur5 — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM restarts) | 100.0 | 100.0 | 100.0 | 0.6 | 0.7 | 1.0 | 35 | 37.3 | 35.7 | -0.0053 | -0.0045 | 0.002 | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 0.5 | 1.7 | 2.6 | 1 | 29.7 | 28.0 | -0.0017 | -0.0012 | 0.002 | 0.00 |
| ProteinIK Fast (V4) | 99.7 | 99.7 | 99.7 | 0.1 | 0.2 | 1.5 | 55 | 26.7 | 25.7 | 0.0015 | 0.0020 | 0.354 | 0.00 |
| ProteinIK Fast (V4 real-calib) | 99.7 | 99.7 | 99.7 | 0.6 | 1.9 | 3.5 | 242 | 23.0 | 21.7 | 0.0024 | 0.0030 | 0.400 | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 99.7 | 99.7 | 99.7 | 0.1 | 0.2 | 1.2 | 53 | 27.3 | 26.7 | 0.0009 | 0.0015 | 0.349 | 0.01 |
| ProteinIK Raw Biology (V6) | 99.3 | 99.3 | 99.3 | 13.6 | 14.1 | 14.7 | 100 | 13.7 | 12.0 | 0.0055 | 0.0054 | 0.224 | 0.02 |
| ProteinIK (V1) | 97.3 | 97.3 | 97.3 | 0.1 | 0.5 | 0.7 | 44 | 29.3 | 28.3 | -0.0008 | -0.0001 | 9.008 | 0.02 |
| Jacobian DLS (real RTB LM, single-shot) | 72.3 | 72.3 | 72.3 | 0.6 | 0.9 | 0.9 | 38 | 39.0 | 37.3 | -0.0048 | -0.0039 | 238.610 | 0.00 |
| FABRIK (in-repo; no genuine upstream) | 49.7 | 49.7 | 49.7 | 86.9 | 139.8 | 141.8 | 94 | 43.3 | 41.3 | -0.0072 | -0.0061 | 48.643 | 0.45 |
| CCD (in-repo; no genuine upstream) | 43.7 | 43.7 | 43.7 | 139.4 | 216.9 | 221.5 | 195 | 38.0 | 37.3 | -0.0069 | -0.0059 | 39.706 | 0.59 |
| PyBullet native IK |   –   | 69.3 | 69.3 | 4.0 | 6.7 | 6.8 |   –   | 43.7 | 42.3 | -0.0064 | -0.0053 | 11.968 |   –   |

## ur5 — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM restarts) | 100.0 | 100.0 | 100.0 | 0.6 | 0.7 | 1.0 | 39 | 43.0 | 42.0 | -0.0129 | -0.0124 | 0.002 | 0.00 |
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 0.2 | 0.7 | 2.6 | 83 | 38.0 | 36.7 | -0.0064 | -0.0056 | 0.335 | 0.01 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 0.6 | 2.2 | 3.2 | 259 | 35.0 | 33.3 | -0.0051 | -0.0043 | 0.348 | 0.01 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 0.1 | 0.4 | 1.5 | 75 | 37.3 | 36.0 | -0.0058 | -0.0051 | 0.349 | 0.01 |
| ProteinIK Raw Biology (V6) | 99.7 | 99.7 | 99.7 | 13.6 | 14.0 | 14.5 | 100 | 24.7 | 24.0 | -0.0006 | -0.0003 | 0.612 | 0.02 |
| TRAC-IK (real C++ TRACLabs) | 99.3 | 99.3 | 99.3 | 0.5 | 1.3 | 2.9 | 1 | 44.3 | 42.7 | -0.0140 | -0.0130 | 5.997 | 0.00 |
| ProteinIK (V1) | 88.3 | 88.3 | 88.3 | 0.2 | 0.7 | 0.7 | 71 | 40.3 | 38.7 | -0.0103 | -0.0095 | 33.850 | 0.06 |
| Jacobian DLS (real RTB LM, single-shot) | 69.7 | 69.7 | 69.7 | 0.6 | 0.9 | 0.9 | 39 | 49.0 | 47.0 | -0.0144 | -0.0135 | 224.267 | 0.00 |
| FABRIK (in-repo; no genuine upstream) | 34.7 | 34.7 | 34.7 | 104.7 | 139.8 | 141.6 | 114 | 53.0 | 51.0 | -0.0168 | -0.0156 | 51.907 | 0.51 |
| CCD (in-repo; no genuine upstream) | 32.0 | 32.0 | 32.0 | 160.9 | 214.4 | 218.4 | 227 | 50.0 | 48.3 | -0.0181 | -0.0172 | 37.322 | 0.61 |
| PyBullet native IK |   –   | 46.0 | 46.0 | 5.8 | 6.8 | 7.2 |   –   | 51.3 | 50.3 | -0.0160 | -0.0148 | 13.372 |   –   |

## ur5 — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM restarts) | 100.0 | 100.0 | 100.0 | 0.5 | 0.6 | 0.7 | 25 | 81.3 | 79.3 | -0.0386 | -0.0372 | 0.001 | 0.00 |
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 0.2 | 1.3 | 1.7 | 93 | 55.3 | 54.7 | -0.0176 | -0.0173 | 0.338 | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 0.5 | 1.5 | 2.1 | 208 | 54.7 | 53.0 | -0.0181 | -0.0174 | 0.346 | 0.01 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 0.1 | 0.4 | 1.9 | 80 | 57.7 | 57.0 | -0.0199 | -0.0195 | 0.325 | 0.00 |
| ProteinIK Raw Biology (V6) | 100.0 | 100.0 | 100.0 | 13.8 | 14.6 | 15.0 | 100 | 46.3 | 44.3 | -0.0128 | -0.0124 | 0.312 | 0.01 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 0.4 | 1.0 | 1.5 | 1 | 71.3 | 70.0 | -0.0342 | -0.0331 | 0.002 | 0.00 |
| ProteinIK (V1) | 89.3 | 89.3 | 89.3 | 0.2 | 0.7 | 0.7 | 67 | 64.7 | 63.0 | -0.0243 | -0.0234 | 22.437 | 0.06 |
| Jacobian DLS (real RTB LM, single-shot) | 77.0 | 77.0 | 77.0 | 0.5 | 0.7 | 0.9 | 27 | 62.3 | 61.0 | -0.0278 | -0.0266 | 138.144 | 0.00 |
| CCD (in-repo; no genuine upstream) | 41.0 | 41.0 | 41.0 | 145.6 | 215.5 | 220.2 | 204 | 70.3 | 67.7 | -0.0375 | -0.0363 | 22.268 | 0.51 |
| FABRIK (in-repo; no genuine upstream) | 37.0 | 37.0 | 37.0 | 98.3 | 140.0 | 143.6 | 107 | 68.0 | 67.0 | -0.0328 | -0.0319 | 43.424 | 0.47 |
| PyBullet native IK |   –   | 55.0 | 55.0 | 5.4 | 7.0 | 7.3 |   –   | 72.0 | 71.7 | -0.0369 | -0.0352 | 6.753 |   –   |

## franka_panda — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 0.1 | 0.4 | 1.4 | 71 | 8.3 | 7.7 | 0.0148 | 0.0129 | 0.375 | 0.29 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 0.1 | 0.4 | 1.4 | 71 | 8.3 | 7.7 | 0.0149 | 0.0131 | 0.374 | 0.28 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 0.1 | 0.4 | 1.4 | 71 | 8.3 | 7.7 | 0.0148 | 0.0129 | 0.375 | 0.29 |
| Multi-start (real RTB ik_LM restarts) | 99.3 | 99.3 | 99.3 | 0.8 | 1.1 | 1.9 | 79 | 5.0 | 3.7 | 0.0168 | 0.0151 | 4.365 | 0.00 |
| ProteinIK Raw Biology (V6) | 99.3 | 99.3 | 99.3 | 20.8 | 21.7 | 22.6 | 125 | 8.0 | 6.3 | 0.0157 | 0.0139 | 0.211 | 0.30 |
| TRAC-IK (real C++ TRACLabs) | 98.7 | 98.7 | 98.7 | 0.9 | 2.7 | 5.1 | 1 | 8.7 | 7.0 | 0.0148 | 0.0142 | 11.533 | 0.06 |
| ProteinIK (V1) | 96.3 | 96.3 | 96.3 | 0.2 | 0.6 | 0.7 | 58 | 7.7 | 6.7 | 0.0148 | 0.0141 | 8.023 | 0.12 |
| Jacobian DLS (real RTB LM, single-shot) | 28.3 | 28.3 | 28.3 | 0.7 | 1.1 | 1.1 | 36 | 12.0 | 10.0 | 0.0141 | 0.0134 | 543.450 | 0.00 |
| CCD (in-repo; no genuine upstream) | 23.0 | 23.0 | 23.0 | 217.1 | 264.0 | 266.2 | 249 | 19.0 | 17.3 | 0.0106 | 0.0092 | 24.512 | 1.16 |
| FABRIK (in-repo; no genuine upstream) | 18.0 | 18.0 | 18.0 | 162.1 | 196.5 | 201.9 | 131 | 20.7 | 20.0 | 0.0106 | 0.0101 | 49.261 | 1.28 |
| PyBullet native IK |   –   | 78.0 | 78.0 | 4.4 | 9.1 | 9.7 |   –   | 32.0 | 31.0 | -0.0045 | -0.0053 | 13.261 |   –   |

## franka_panda — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 99.7 | 99.7 | 0.2 | 0.7 | 2.0 | 83 | 9.3 | 8.7 | 0.0142 | 0.0136 | 0.412 | 0.23 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 99.7 | 99.7 | 0.2 | 0.7 | 2.6 | 84 | 9.3 | 8.7 | 0.0141 | 0.0134 | 0.405 | 0.23 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 99.7 | 99.7 | 0.2 | 0.7 | 1.9 | 83 | 9.3 | 8.7 | 0.0142 | 0.0136 | 0.412 | 0.23 |
| ProteinIK Raw Biology (V6) | 99.0 | 99.0 | 99.0 | 20.5 | 21.4 | 22.9 | 125 | 5.3 | 5.0 | 0.0158 | 0.0145 | 0.651 | 0.22 |
| TRAC-IK (real C++ TRACLabs) | 99.0 | 99.0 | 99.0 | 0.8 | 2.5 | 4.7 | 1 | 9.7 | 9.3 | 0.0147 | 0.0137 | 10.504 | 0.07 |
| Multi-start (real RTB ik_LM restarts) | 98.0 | 98.0 | 98.0 | 0.9 | 1.3 | 4.0 | 104 | 6.3 | 4.7 | 0.0158 | 0.0149 | 15.946 | 0.00 |
| ProteinIK (V1) | 92.3 | 92.3 | 92.3 | 0.2 | 0.7 | 0.7 | 65 | 9.0 | 7.7 | 0.0147 | 0.0136 | 15.209 | 0.18 |
| Jacobian DLS (real RTB LM, single-shot) | 31.0 | 31.0 | 31.0 | 0.7 | 1.1 | 1.1 | 34 | 10.0 | 9.0 | 0.0142 | 0.0137 | 573.806 | 0.00 |
| CCD (in-repo; no genuine upstream) | 11.7 | 11.7 | 11.7 | 252.5 | 282.1 | 284.1 | 280 | 15.0 | 14.3 | 0.0123 | 0.0122 | 26.662 | 1.08 |
| FABRIK (in-repo; no genuine upstream) | 11.3 | 11.3 | 11.3 | 171.4 | 184.3 | 186.1 | 141 | 20.0 | 19.7 | 0.0094 | 0.0095 | 55.857 | 1.19 |
| PyBullet native IK |   –   | 61.7 | 61.7 | 6.8 | 9.0 | 9.2 |   –   | 22.3 | 22.0 | 0.0038 | 0.0029 | 17.523 |   –   |

## franka_panda — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 98.3 | 98.3 | 98.3 | 0.7 | 3.4 | 4.7 | 228 | 80.0 | 80.0 | -0.0465 | -0.0457 | 0.583 | 0.36 |
| ProteinIK Fast (V4+o2 IAM) | 98.3 | 98.3 | 98.3 | 0.6 | 3.2 | 4.6 | 206 | 79.7 | 79.7 | -0.0469 | -0.0461 | 0.574 | 0.36 |
| ProteinIK Fast (V4 real-calib) | 98.0 | 98.0 | 98.0 | 0.8 | 3.5 | 4.7 | 275 | 78.7 | 78.3 | -0.0452 | -0.0444 | 0.488 | 0.31 |
| ProteinIK Raw Biology (V6) | 97.7 | 97.7 | 97.7 | 20.9 | 21.7 | 22.1 | 125 | 78.3 | 78.0 | -0.0448 | -0.0438 | 0.713 | 0.43 |
| TRAC-IK (real C++ TRACLabs) | 94.7 | 94.7 | 94.7 | 1.3 | 5.1 | 5.1 | 1 | 74.7 | 74.3 | -0.0449 | -0.0442 | 40.046 | 0.08 |
| Multi-start (real RTB ik_LM restarts) | 93.7 | 93.7 | 93.7 | 1.1 | 4.1 | 6.4 | 208 | 74.3 | 74.3 | -0.0449 | -0.0442 | 45.677 | 0.00 |
| ProteinIK (V1) | 80.7 | 80.7 | 80.7 | 0.3 | 0.8 | 0.8 | 92 | 74.0 | 72.7 | -0.0419 | -0.0409 | 45.687 | 0.39 |
| FABRIK (in-repo; no genuine upstream) | 22.7 | 22.7 | 22.7 | 149.8 | 184.8 | 190.7 | 123 | 76.7 | 76.3 | -0.0426 | -0.0416 | 50.023 | 1.45 |
| Jacobian DLS (real RTB LM, single-shot) | 19.0 | 19.0 | 19.0 | 0.7 | 0.7 | 1.1 | 22 | 22.7 | 20.7 | 0.0033 | 0.0024 | 606.523 | 0.00 |
| CCD (in-repo; no genuine upstream) | 12.3 | 12.3 | 12.3 | 236.6 | 263.5 | 264.1 | 272 | 82.7 | 82.0 | -0.0513 | -0.0500 | 18.656 | 1.71 |
| PyBullet native IK |   –   | 92.3 | 92.3 | 2.9 | 8.7 | 8.8 |   –   | 86.0 | 85.7 | -0.0644 | -0.0631 | 4.313 |   –   |

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
