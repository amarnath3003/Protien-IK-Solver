# Master Sim Benchmark — every solver × arm × scenario, scored in PyBullet + MuJoCo

- **50** trials/seed × seeds **[1, 2, 3]** (n=150 per cell)  |  warm-up 2 untimed solves/cell
- Arms: ur5, franka_panda  |  Scenarios: open_space, near_singular, cluttered
- Engines: PyBullet + MuJoCo  |  generated 2026-07-08 04:36:16

Each solver runs **once** on our DH `RobotSpec` core; that `q_final` is then scored by two independent real-mesh oracles — **PB** = PyBullet, **MJ** = MuJoCo (identical URDF & non-adjacent link pairs). The capsule proxy the solvers optimize against is **not** used to evaluate them here — only real-mesh collision counts (planar3dof has no URDF, so it carries success/speed only). `PyBullet native IK` is the sim's own solver on the identical targets. Timing is wall-clock (OS noise on mean/p95/p99); success/collision/error columns are deterministic given the seed.

## Verdict — lowest real-mesh-collision solver per cell (among ≥90% success)

| Arm | Scenario | **PyBullet** | **MuJoCo** |
|:--|:--|:--|:--|
| ur5 | open_space | TRAC-IK style | TRAC-IK style |
| ur5 | near_singular | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| ur5 | cluttered | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| franka_panda | open_space | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| franka_panda | near_singular | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| franka_panda | cluttered | TRAC-IK style | TRAC-IK style |

## ur5 — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 3.9 | 10.8 | 14.1 | 23 | 46.7 | 46.7 | 0.0047 | 0.0062 | 0.063 | 0.00 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 70.9 | 201.8 | 349.9 | 187 | 48.0 | 45.3 | 0.0040 | 0.0045 | 0.204 | 0.00 |
| TRAC-IK style | 100.0 | 100.0 | 100.0 | 4.7 | 15.5 | 21.7 | 15 | 13.3 | 13.3 | 0.0093 | 0.0109 | 0.729 | 0.00 |

## ur5 — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 12.3 | 34.1 | 48.1 | 54 | 22.0 | 22.0 | -0.0027 | -0.0026 | 0.222 | 0.03 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 68.7 | 245.9 | 551.3 | 189 | 29.3 | 29.3 | -0.0032 | -0.0032 | 0.277 | 0.01 |
| TRAC-IK style | 100.0 | 100.0 | 100.0 | 8.3 | 17.1 | 20.5 | 18 | 33.3 | 33.3 | -0.0076 | -0.0056 | 0.672 | 0.00 |

## ur5 — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 73.5 | 298.8 | 498.7 | 181 | 71.3 | 71.3 | -0.0364 | -0.0345 | 0.663 | 0.04 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 118.4 | 391.2 | 818.5 | 248 | 71.3 | 71.3 | -0.0370 | -0.0350 | 0.666 | 0.03 |
| TRAC-IK style | 88.7 | 88.7 | 88.7 | 41.8 | 109.7 | 131.4 | 111 | 66.7 | 66.7 | -0.0416 | -0.0408 | 0.700 | 0.05 |

## franka_panda — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 12.0 | 23.5 | 52.1 | 75 | 0.0 | 0.0 | 0.0197 | 0.0181 | 0.397 | 0.21 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 11.5 | 22.8 | 43.7 | 75 | 0.0 | 0.0 | 0.0197 | 0.0181 | 0.397 | 0.21 |
| TRAC-IK style | 100.0 | 100.0 | 100.0 | 26.8 | 65.1 | 94.1 | 73 | 0.0 | 0.0 | 0.0197 | 0.0190 | 0.662 | 0.25 |

## franka_panda — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 20.1 | 63.7 | 170.9 | 90 | 0.0 | 0.0 | 0.0198 | 0.0191 | 0.504 | 0.18 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 20.8 | 60.6 | 246.1 | 90 | 0.0 | 0.0 | 0.0198 | 0.0191 | 0.505 | 0.17 |
| TRAC-IK style | 100.0 | 100.0 | 100.0 | 19.8 | 62.0 | 80.2 | 50 | 0.0 | 0.0 | 0.0198 | 0.0137 | 0.881 | 0.46 |

## franka_panda — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 50.8 | 287.7 | 444.5 | 145 | 85.3 | 85.3 | -0.0549 | -0.0539 | 0.306 | 0.23 |
| ProteinIK Fast (V4 real-calib) | 100.0 | 100.0 | 100.0 | 99.0 | 339.2 | 526.1 | 225 | 86.0 | 86.0 | -0.0549 | -0.0538 | 0.318 | 0.25 |
| TRAC-IK style | 98.7 | 98.7 | 98.7 | 23.3 | 65.5 | 102.5 | 67 | 84.0 | 84.0 | -0.0544 | -0.0536 | 0.528 | 0.29 |
