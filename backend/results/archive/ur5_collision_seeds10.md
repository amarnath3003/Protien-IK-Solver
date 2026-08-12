# Master Sim Benchmark — every solver × arm × scenario, scored in PyBullet + MuJoCo

- **100** trials/seed × seeds **[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]** (n=1000 per cell)  |  warm-up 8 untimed solves/cell
- Arms: ur5  |  Scenarios: open_space, near_singular, cluttered
- Engines: PyBullet + MuJoCo  |  generated 2026-07-09 02:05:42

Each solver runs **once** on our DH `RobotSpec` core; that `q_final` is then scored by two independent real-mesh oracles — **PB** = PyBullet, **MJ** = MuJoCo (identical URDF & non-adjacent link pairs). The capsule proxy the solvers optimize against is **not** used to evaluate them here — only real-mesh collision counts (planar3dof has no URDF, so it carries success/speed only). `PyBullet native IK` is the sim's own solver on the identical targets. Timing is wall-clock (OS noise on mean/p95/p99); success/collision/error columns are deterministic given the seed.

## Verdict — lowest real-mesh-collision solver per cell (among ≥90% success)

| Arm | Scenario | **PyBullet** | **MuJoCo** |
|:--|:--|:--|:--|
| ur5 | open_space | ProteinIK (V1) | ProteinIK (V1) |
| ur5 | near_singular | ProteinIK (V1) | ProteinIK (V1) |
| ur5 | cluttered | ProteinIK Fast (V4) | ProteinIK Fast (V4) |

## ur5 — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 9.3 | 17.9 | 130.6 | 57 | 36.1 | 33.8 | -0.0015 | -0.0002 | 0.234 | 0.00 |
| TRAC-IK style | 100.0 | 100.0 | 100.0 | 7.2 | 17.9 | 34.9 | 28 | 30.6 | 28.8 | 0.0004 | 0.0017 | 0.625 | 0.00 |
| Multi-start | 99.8 | 99.8 | 99.8 | 74.0 | 109.0 | 119.8 | 283 | 39.0 | 34.4 | -0.0033 | -0.0025 | 0.374 | 0.00 |
| ProteinIK (V1) | 98.8 | 98.8 | 98.8 | 21.4 | 70.3 | 157.4 | 34 | 27.8 | 27.1 | 0.0002 | 0.0009 | 7.327 | 0.01 |

## ur5 — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| TRAC-IK style | 99.9 | 99.9 | 99.9 | 12.6 | 29.6 | 53.3 | 45 | 46.8 | 46.2 | -0.0123 | -0.0109 | 0.979 | 0.01 |
| ProteinIK Fast (V4) | 100.0 | 99.7 | 99.7 | 21.0 | 114.5 | 237.6 | 81 | 40.0 | 38.8 | -0.0095 | -0.0088 | 0.334 | 0.01 |
| Multi-start | 99.4 | 99.4 | 99.4 | 89.2 | 116.7 | 126.9 | 339 | 44.6 | 43.2 | -0.0131 | -0.0122 | 1.345 | 0.01 |
| ProteinIK (V1) | 93.8 | 93.8 | 93.8 | 48.4 | 155.4 | 158.3 | 67 | 32.4 | 32.2 | -0.0079 | -0.0080 | 16.482 | 0.03 |

## ur5 — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 99.7 | 99.7 | 38.1 | 208.2 | 298.7 | 127 | 57.0 | 56.1 | -0.0206 | -0.0192 | 0.489 | 0.02 |
| Multi-start | 98.8 | 98.8 | 98.8 | 88.0 | 119.3 | 124.3 | 343 | 65.0 | 64.7 | -0.0318 | -0.0308 | 0.626 | 0.01 |
| TRAC-IK style | 96.5 | 96.5 | 96.5 | 16.3 | 61.8 | 77.2 | 63 | 71.1 | 71.1 | -0.0317 | -0.0306 | 0.764 | 0.03 |
| ProteinIK (V1) | 79.1 | 79.1 | 79.1 | 66.7 | 158.8 | 165.5 | 87 | 76.6 | 76.4 | -0.0342 | -0.0326 | 37.142 | 0.09 |
