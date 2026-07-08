# Master Sim Benchmark — every solver × arm × scenario, scored in PyBullet + MuJoCo

- **100** trials/seed × seeds **[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]** (n=1000 per cell)  |  warm-up 8 untimed solves/cell
- Arms: ur5  |  Scenarios: open_space, near_singular, cluttered
- Engines: PyBullet + MuJoCo  |  generated 2026-07-09 02:22:44

Each solver runs **once** on our DH `RobotSpec` core; that `q_final` is then scored by two independent real-mesh oracles — **PB** = PyBullet, **MJ** = MuJoCo (identical URDF & non-adjacent link pairs). The capsule proxy the solvers optimize against is **not** used to evaluate them here — only real-mesh collision counts (planar3dof has no URDF, so it carries success/speed only). `PyBullet native IK` is the sim's own solver on the identical targets. Timing is wall-clock (OS noise on mean/p95/p99); success/collision/error columns are deterministic given the seed.

## Verdict — lowest real-mesh-collision solver per cell (among ≥90% success)

| Arm | Scenario | **PyBullet** | **MuJoCo** |
|:--|:--|:--|:--|
| ur5 | open_space | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| ur5 | near_singular | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| ur5 | cluttered | ProteinIK Fast (V4) | ProteinIK Fast (V4) |

## ur5 — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 99.8 | 99.8 | 15.5 | 37.0 | 312.9 | 57 | 28.6 | 26.3 | 0.0010 | 0.0019 | 0.267 | 0.00 |
| TRAC-IK style | 99.0 | 99.0 | 99.0 | 11.1 | 38.5 | 73.6 | 42 | 31.1 | 29.4 | -0.0033 | -0.0025 | 0.729 | 0.01 |
| Multi-start | 97.8 | 97.8 | 97.8 | 77.1 | 116.8 | 127.1 | 294 | 30.6 | 29.5 | -0.0021 | -0.0010 | 1.268 | 0.01 |
| ProteinIK (V1) | 94.5 | 94.5 | 94.5 | 32.7 | 157.4 | 167.3 | 47 | 32.6 | 30.2 | -0.0016 | -0.0009 | 17.613 | 0.03 |

## ur5 — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 99.9 | 99.8 | 99.8 | 25.7 | 163.3 | 419.2 | 82 | 40.1 | 38.5 | -0.0079 | -0.0070 | 0.373 | 0.01 |
| TRAC-IK style | 97.2 | 97.2 | 97.2 | 15.5 | 56.4 | 78.3 | 59 | 47.4 | 46.1 | -0.0155 | -0.0145 | 0.875 | 0.03 |
| Multi-start | 96.4 | 96.4 | 96.4 | 88.4 | 123.9 | 129.7 | 335 | 44.0 | 42.9 | -0.0124 | -0.0114 | 1.120 | 0.02 |
| ProteinIK (V1) | 89.4 | 89.4 | 89.4 | 54.3 | 161.7 | 170.9 | 72 | 42.6 | 40.8 | -0.0108 | -0.0102 | 31.873 | 0.07 |

## ur5 — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 26.7 | 186.0 | 297.3 | 92 | 59.6 | 58.1 | -0.0203 | -0.0194 | 0.337 | 0.01 |
| Multi-start | 98.1 | 98.3 | 98.3 | 82.4 | 117.5 | 127.3 | 316 | 67.1 | 65.7 | -0.0278 | -0.0268 | 0.698 | 0.01 |
| TRAC-IK style | 97.9 | 97.9 | 97.9 | 13.8 | 52.1 | 78.5 | 52 | 71.0 | 70.0 | -0.0340 | -0.0328 | 0.766 | 0.02 |
| ProteinIK (V1) | 88.2 | 88.2 | 88.2 | 54.4 | 160.4 | 165.4 | 73 | 63.4 | 61.5 | -0.0258 | -0.0248 | 20.077 | 0.05 |
