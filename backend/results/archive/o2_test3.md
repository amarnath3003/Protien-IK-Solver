# Master Sim Benchmark — every solver × arm × scenario, scored in PyBullet + MuJoCo

- **50** trials/seed × seeds **[1, 2, 3]** (n=150 per cell)  |  warm-up 2 untimed solves/cell
- Arms: ur5, franka_panda  |  Scenarios: open_space, near_singular, cluttered
- Engines: PyBullet + MuJoCo  |  generated 2026-07-08 03:14:32

Each solver runs **once** on our DH `RobotSpec` core; that `q_final` is scored three ways — **our** capsule proxy (what the solver optimizes), **PB** = PyBullet real mesh, **MJ** = MuJoCo real mesh (identical URDF & non-adjacent link pairs). `PyBullet native IK` is the sim's own solver on the identical targets. Timing is wall-clock (OS noise on mean/p95/p99); success/collision/error columns are deterministic given the seed.

## Verdict — lowest-collision solver per cell (among ≥90% success)

| Arm | Scenario | our proxy | **PyBullet** | **MuJoCo** |
|:--|:--|:--|:--|:--|
| ur5 | open_space | ProteinIK Fast (V4) | TRAC-IK style | TRAC-IK style |
| ur5 | near_singular | ProteinIK Fast (V4) | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| ur5 | cluttered | ProteinIK Fast (V4) | ProteinIK Fast (V4+o2 IAM) | ProteinIK Fast (V4+o2 IAM) |
| franka_panda | open_space | ProteinIK Fast (V4) | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| franka_panda | near_singular | ProteinIK Fast (V4) | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| franka_panda | cluttered | ProteinIK Fast (V4) | TRAC-IK style | TRAC-IK style |

## ur5 — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | our col% | PB col% | MJ col% | our clr | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 2.8 | 7.6 | 9.7 | 23 | 0.0 | 46.7 | 46.7 | 0.0196 | 0.0047 | 0.0062 | 0.063 | 0.00 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 3.0 | 8.3 | 10.8 | 23 | 0.0 | 46.7 | 46.7 | 0.0196 | 0.0047 | 0.0062 | 0.063 | 0.00 |
| TRAC-IK style | 100.0 | 100.0 | 100.0 | 3.8 | 11.6 | 15.9 | 15 | 0.0 | 13.3 | 13.3 | 0.0196 | 0.0093 | 0.0109 | 0.729 | 0.00 |

## ur5 — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | our col% | PB col% | MJ col% | our clr | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 8.6 | 23.7 | 38.7 | 54 | 2.7 | 22.0 | 22.0 | 0.0160 | -0.0027 | -0.0026 | 0.222 | 0.03 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 7.5 | 18.8 | 66.6 | 53 | 4.0 | 23.3 | 23.3 | 0.0155 | -0.0039 | -0.0038 | 0.210 | 0.03 |
| TRAC-IK style | 100.0 | 100.0 | 100.0 | 4.7 | 9.6 | 10.8 | 18 | 33.3 | 33.3 | 33.3 | 0.0049 | -0.0076 | -0.0056 | 0.672 | 0.00 |

## ur5 — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | our col% | PB col% | MJ col% | our clr | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 56.0 | 224.2 | 343.7 | 181 | 68.0 | 71.3 | 71.3 | -0.0188 | -0.0364 | -0.0345 | 0.663 | 0.04 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 48.9 | 210.5 | 258.9 | 165 | 68.0 | 68.7 | 68.7 | -0.0194 | -0.0360 | -0.0343 | 0.648 | 0.03 |
| TRAC-IK style | 88.7 | 88.7 | 88.7 | 29.9 | 79.0 | 93.5 | 111 | 78.0 | 66.7 | 66.7 | -0.0330 | -0.0416 | -0.0408 | 0.700 | 0.05 |

## franka_panda — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | our col% | PB col% | MJ col% | our clr | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 9.0 | 19.3 | 33.8 | 75 | 0.0 | 0.0 | 0.0 | 0.0325 | 0.0197 | 0.0181 | 0.397 | 0.21 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 9.5 | 19.5 | 39.9 | 75 | 0.0 | 0.0 | 0.0 | 0.0325 | 0.0197 | 0.0181 | 0.397 | 0.21 |
| TRAC-IK style | 100.0 | 100.0 | 100.0 | 22.1 | 48.2 | 75.9 | 73 | 0.0 | 0.0 | 0.0 | 0.0325 | 0.0197 | 0.0190 | 0.662 | 0.25 |

## franka_panda — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | our col% | PB col% | MJ col% | our clr | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 16.4 | 58.7 | 123.6 | 90 | 0.0 | 0.0 | 0.0 | 0.0325 | 0.0198 | 0.0191 | 0.504 | 0.18 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 16.3 | 53.5 | 128.1 | 90 | 0.0 | 0.0 | 0.0 | 0.0325 | 0.0198 | 0.0191 | 0.504 | 0.18 |
| TRAC-IK style | 100.0 | 100.0 | 100.0 | 14.8 | 42.6 | 62.7 | 50 | 0.0 | 0.0 | 0.0 | 0.0325 | 0.0198 | 0.0137 | 0.881 | 0.46 |

## franka_panda — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | our col% | PB col% | MJ col% | our clr | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 40.4 | 234.5 | 355.3 | 145 | 7.3 | 85.3 | 85.3 | 0.0217 | -0.0549 | -0.0539 | 0.306 | 0.23 |
| ProteinIK Fast (V4+o2 IAM) | 100.0 | 100.0 | 100.0 | 38.2 | 226.5 | 295.9 | 139 | 10.0 | 85.3 | 85.3 | 0.0195 | -0.0557 | -0.0547 | 0.280 | 0.19 |
| TRAC-IK style | 98.7 | 98.7 | 98.7 | 19.2 | 54.4 | 83.4 | 67 | 34.7 | 84.0 | 84.0 | 0.0084 | -0.0544 | -0.0536 | 0.528 | 0.29 |
