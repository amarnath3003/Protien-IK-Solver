# Master Sim Benchmark — every solver × arm × scenario, scored in PyBullet + MuJoCo

- **5** trials/seed × seeds **[1]** (n=5 per cell)  |  warm-up 2 untimed solves/cell
- Arms: planar3dof, ur5, franka_panda  |  Scenarios: open_space, near_singular, cluttered
- Engines: PyBullet + MuJoCo  |  generated 2026-07-13 18:34:47

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
| ur5 | open_space | TRAC-IK (real C++ TRACLabs) | TRAC-IK (real C++ TRACLabs) |
| ur5 | near_singular | ProteinIK Fast (V4) | ProteinIK Fast (V4) |
| ur5 | cluttered | TRAC-IK (real C++ TRACLabs) | TRAC-IK (real C++ TRACLabs) |
| franka_panda | open_space | Multi-start (real RTB ik_LM) | Multi-start (real RTB ik_LM) |
| franka_panda | near_singular | Multi-start (real RTB ik_LM) | Multi-start (real RTB ik_LM) |
| franka_panda | cluttered | Multi-start (real RTB ik_LM) | Multi-start (real RTB ik_LM) |

## planar3dof — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Analytical IK (Planar 3-DOF, exact) | 100.0 |   –   |   –   | 0.1 | 0.1 | 0.1 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| Multi-start (real RTB ik_LM) | 100.0 |   –   |   –   | 0.2 | 0.3 | 0.3 | 19 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4) | 100.0 |   –   |   –   | 2.6 | 5.3 | 5.5 | 24 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 |   –   |   –   | 0.2 | 0.3 | 0.4 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| Jacobian DLS (real Orocos KDL LMA) | 60.0 |   –   |   –   | 0.1 | 0.1 | 0.1 | 1 |   –   |   –   |   –   |   –   |   –   | 0.40 |

## planar3dof — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Analytical IK (Planar 3-DOF, exact) | 100.0 |   –   |   –   | 0.1 | 0.1 | 0.1 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| Multi-start (real RTB ik_LM) | 100.0 |   –   |   –   | 0.2 | 0.2 | 0.2 | 19 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4) | 100.0 |   –   |   –   | 9.0 | 19.8 | 21.6 | 83 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 |   –   |   –   | 0.2 | 0.2 | 0.2 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| Jacobian DLS (real Orocos KDL LMA) | 20.0 |   –   |   –   | 0.1 | 0.1 | 0.1 | 1 |   –   |   –   |   –   |   –   |   –   | 0.80 |

## planar3dof — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Analytical IK (Planar 3-DOF, exact) | 100.0 |   –   |   –   | 0.1 | 0.1 | 0.1 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| Multi-start (real RTB ik_LM) | 100.0 |   –   |   –   | 0.2 | 0.3 | 0.3 | 42 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| ProteinIK Fast (V4) | 100.0 |   –   |   –   | 132.6 | 362.9 | 416.1 | 382 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 |   –   |   –   | 0.2 | 0.4 | 0.4 | 1 |   –   |   –   |   –   |   –   |   –   | 0.00 |
| Jacobian DLS (real Orocos KDL LMA) | 20.0 |   –   |   –   | 0.1 | 0.1 | 0.1 | 1 |   –   |   –   |   –   |   –   |   –   | 1.00 |

## ur5 — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM) | 100.0 | 100.0 | 100.0 | 0.6 | 0.7 | 0.7 | 15 | 40.0 | 40.0 | -0.0023 | -0.0003 | 0.002 | 0.00 |
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 2.2 | 4.5 | 5.1 | 15 | 20.0 | 20.0 | 0.0062 | 0.0082 | 0.067 | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 0.4 | 0.6 | 0.6 | 1 | 0.0 | 0.0 | 0.0118 | 0.0112 | 0.002 | 0.00 |
| Jacobian DLS (real Orocos KDL LMA) | 40.0 | 40.0 | 40.0 | 0.1 | 0.1 | 0.1 | 1 | 0.0 | 0.0 | 0.0074 | 0.0094 | 46.329 | 0.80 |
| PyBullet native IK |   –   | 100.0 | 100.0 | 1.5 | 2.6 | 2.7 |   –   | 0.0 | 0.0 | 0.0086 | 0.0105 | 0.009 |   –   |

## ur5 — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM) | 100.0 | 100.0 | 100.0 | 0.6 | 0.6 | 0.6 | 24 | 100.0 | 80.0 | -0.0307 | -0.0287 | 0.001 | 0.00 |
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 16.3 | 60.4 | 72.1 | 42 | 60.0 | 60.0 | -0.0125 | -0.0105 | 0.399 | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 0.3 | 0.4 | 0.4 | 1 | 80.0 | 80.0 | -0.0266 | -0.0246 | 0.003 | 0.00 |
| Jacobian DLS (real Orocos KDL LMA) | 80.0 | 80.0 | 80.0 | 0.2 | 0.4 | 0.4 | 1 | 60.0 | 60.0 | -0.0134 | -0.0142 | 70.254 | 0.40 |
| PyBullet native IK |   –   | 40.0 | 40.0 | 6.1 | 6.8 | 6.8 |   –   | 60.0 | 60.0 | -0.0251 | -0.0231 | 6.131 |   –   |

## ur5 — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM) | 100.0 | 100.0 | 100.0 | 0.6 | 0.6 | 0.6 | 30 | 60.0 | 60.0 | -0.0252 | -0.0258 | 0.000 | 0.00 |
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 6.0 | 16.7 | 19.1 | 49 | 60.0 | 60.0 | -0.0145 | -0.0125 | 0.232 | 0.00 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 0.5 | 1.2 | 1.4 | 1 | 40.0 | 40.0 | -0.0111 | -0.0091 | 0.000 | 0.00 |
| Jacobian DLS (real Orocos KDL LMA) | 60.0 | 60.0 | 60.0 | 0.2 | 0.3 | 0.3 | 1 | 60.0 | 60.0 | -0.0268 | -0.0248 | 189.163 | 0.80 |
| PyBullet native IK |   –   | 60.0 | 60.0 | 3.9 | 7.0 | 7.0 |   –   | 60.0 | 60.0 | -0.0280 | -0.0260 | 5.724 |   –   |

## franka_panda — open_space

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM) | 100.0 | 100.0 | 100.0 | 0.8 | 1.0 | 1.1 | 52 | 20.0 | 20.0 | -0.0045 | -0.0026 | 0.002 | 0.00 |
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 15.0 | 37.5 | 42.5 | 96 | 20.0 | 20.0 | -0.0019 | 0.0001 | 0.588 | 0.40 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 0.5 | 0.6 | 0.6 | 1 | 20.0 | 20.0 | -0.0014 | 0.0006 | 0.002 | 0.00 |
| Jacobian DLS (real Orocos KDL LMA) | 0.0 | 0.0 | 0.0 | 0.2 | 0.4 | 0.5 | 1 | 0.0 | 0.0 | 0.0162 | 0.0147 | 422.392 | 2.00 |
| PyBullet native IK |   –   | 100.0 | 100.0 | 2.6 | 7.5 | 8.6 |   –   | 40.0 | 40.0 | -0.0158 | -0.0138 | 0.012 |   –   |

## franka_panda — near_singular

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM) | 100.0 | 100.0 | 100.0 | 0.7 | 0.8 | 0.8 | 26 | 0.0 | 0.0 | 0.0160 | 0.0139 | 0.003 | 0.00 |
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 11.7 | 19.2 | 20.5 | 108 | 0.0 | 0.0 | 0.0151 | 0.0126 | 0.531 | 0.40 |
| TRAC-IK (real C++ TRACLabs) | 100.0 | 100.0 | 100.0 | 0.8 | 1.6 | 1.7 | 1 | 0.0 | 0.0 | 0.0193 | 0.0169 | 0.004 | 0.20 |
| Jacobian DLS (real Orocos KDL LMA) | 0.0 | 0.0 | 0.0 | 0.2 | 0.3 | 0.4 | 1 | 20.0 | 20.0 | 0.0050 | 0.0069 | 407.155 | 1.60 |
| PyBullet native IK |   –   | 80.0 | 80.0 | 5.4 | 9.0 | 9.0 |   –   | 0.0 | 0.0 | 0.0181 | 0.0201 | 30.100 |   –   |

## franka_panda — cluttered

| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Multi-start (real RTB ik_LM) | 100.0 | 100.0 | 100.0 | 0.9 | 1.1 | 1.1 | 87 | 60.0 | 60.0 | -0.0454 | -0.0478 | 0.001 | 0.00 |
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 38.3 | 95.6 | 100.8 | 138 | 60.0 | 60.0 | -0.0360 | -0.0340 | 0.269 | 0.20 |
| TRAC-IK (real C++ TRACLabs) | 80.0 | 80.0 | 80.0 | 1.4 | 4.2 | 4.9 | 1 | 40.0 | 40.0 | -0.0155 | -0.0135 | 181.765 | 0.00 |
| Jacobian DLS (real Orocos KDL LMA) | 0.0 | 0.0 | 0.0 | 0.2 | 0.2 | 0.2 | 1 | 40.0 | 40.0 | -0.0085 | -0.0065 | 367.887 | 3.00 |
| PyBullet native IK |   –   | 100.0 | 100.0 | 2.5 | 5.4 | 5.9 |   –   | 80.0 | 80.0 | -0.0588 | -0.0609 | 0.009 |   –   |
