# Master Sim Benchmark — every solver × arm × scenario, scored in PyBullet + MuJoCo

- **100** trials/seed × seeds **[1]** (n=100 per cell)  |  warm-up 2 untimed solves/cell
- Arms: ur5, franka_panda  |  Scenarios: open_space, near_singular, cluttered
- Engines: PyBullet + MuJoCo  |  generated 2026-07-08 03:08:25

Each solver runs **once** on our DH `RobotSpec` core; that `q_final` is scored three ways — **our** capsule proxy (what the solver optimizes), **PB** = PyBullet real mesh, **MJ** = MuJoCo real mesh (identical URDF & non-adjacent link pairs). `PyBullet native IK` is the sim's own solver on the identical targets. Timing is wall-clock (OS noise on mean/p95/p99); success/collision/error columns are deterministic given the seed.

## Verdict — lowest-collision solver per cell (among ≥90% success)

| Arm | Scenario | our proxy | **PyBullet** | **MuJoCo** |
|:--|:--|:--|:--|:--|
| ur5 | open_space | ProteinIK Fast (V4) | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| ur5 | near_singular | ProteinIK Fast (V4) | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| ur5 | cluttered | ProteinIK Fast (V4) | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| franka_panda | open_space | ProteinIK Fast (V4) | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| franka_panda | near_singular | ProteinIK Fast (V4) | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| franka_panda | cluttered | ProteinIK Fast (V4) | ProteinIK Fast (V4) | ProteinIK Fast (V4) |

## ur5 — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | our col% | PB col% | MJ col% | our clr | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 1.1 | 1.4 | 1.5 | 5 | 0.0 | 0.0 | 0.0 | 0.0196 | 0.0118 | 0.0138 | 0.007 | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 1.3 | 1.8 | 2.6 | 5 | 0.0 | 0.0 | 0.0 | 0.0196 | 0.0118 | 0.0138 | 0.007 | 0.00 |
| TRAC-IK style | 100.0 | 100.0 | 100.0 | 1.4 | 1.6 | 1.7 | 6 | 0.0 | 0.0 | 0.0 | 0.0196 | 0.0118 | 0.0138 | 0.955 | 0.00 |

## ur5 — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | our col% | PB col% | MJ col% | our clr | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 1.9 | 3.2 | 3.4 | 8 | 0.0 | 0.0 | 0.0 | 0.0119 | 0.0067 | 0.0087 | 0.093 | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 1.5 | 1.9 | 2.5 | 8 | 0.0 | 0.0 | 0.0 | 0.0119 | 0.0067 | 0.0087 | 0.093 | 0.00 |
| TRAC-IK style | 100.0 | 100.0 | 100.0 | 9.8 | 11.1 | 11.6 | 35 | 0.0 | 0.0 | 0.0 | 0.0138 | 0.0086 | 0.0106 | 0.868 | 0.00 |

## ur5 — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | our col% | PB col% | MJ col% | our clr | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 23.8 | 118.8 | 177.0 | 107 | 5.0 | 100.0 | 100.0 | 0.0172 | -0.0267 | -0.0247 | 0.544 | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 22.7 | 118.2 | 198.0 | 103 | 5.0 | 100.0 | 100.0 | 0.0168 | -0.0273 | -0.0253 | 0.533 | 0.00 |
| TRAC-IK style | 95.0 | 95.0 | 95.0 | 37.9 | 78.2 | 82.3 | 138 | 36.0 | 100.0 | 100.0 | -0.0113 | -0.0436 | -0.0416 | 0.861 | 0.06 |

## franka_panda — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | our col% | PB col% | MJ col% | our clr | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 9.0 | 18.9 | 19.8 | 86 | 0.0 | 0.0 | 0.0 | 0.0325 | 0.0193 | 0.0174 | 0.508 | 0.14 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 8.9 | 18.7 | 19.4 | 86 | 0.0 | 0.0 | 0.0 | 0.0325 | 0.0193 | 0.0174 | 0.508 | 0.14 |
| TRAC-IK style | 99.0 | 99.0 | 99.0 | 25.1 | 74.8 | 82.1 | 85 | 0.0 | 0.0 | 0.0 | 0.0325 | 0.0194 | 0.0184 | 0.833 | 0.18 |

## franka_panda — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | our col% | PB col% | MJ col% | our clr | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 16.9 | 60.1 | 161.8 | 85 | 0.0 | 0.0 | 0.0 | 0.0325 | 0.0199 | 0.0193 | 0.445 | 0.32 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 16.2 | 70.1 | 163.4 | 85 | 0.0 | 0.0 | 0.0 | 0.0325 | 0.0199 | 0.0193 | 0.445 | 0.32 |
| TRAC-IK style | 100.0 | 100.0 | 100.0 | 3.7 | 5.6 | 6.8 | 12 | 0.0 | 0.0 | 0.0 | 0.0325 | 0.0197 | 0.0000 | 0.898 | 1.00 |

## franka_panda — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | our col% | PB col% | MJ col% | our clr | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 84.6 | 303.7 | 425.1 | 210 | 20.0 | 100.0 | 100.0 | 0.0104 | -0.0822 | -0.0802 | 0.272 | 0.17 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 76.0 | 266.5 | 411.8 | 201 | 27.0 | 100.0 | 100.0 | 0.0062 | -0.0827 | -0.0807 | 0.278 | 0.15 |
| TRAC-IK style | 93.0 | 93.0 | 93.0 | 27.5 | 84.5 | 90.7 | 93 | 57.0 | 100.0 | 100.0 | -0.0062 | -0.0807 | -0.0787 | 1.773 | 0.27 |
