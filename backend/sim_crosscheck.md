# Sim Cross-Check — PyBullet vs MuJoCo vs our DH (Phase 4)

Both simulators load the **identical** URDF (classic UR5 `ur5_robot.urdf`, franka_ros `panda.urdf`), so this isolates *engine* differences from *model* differences. MuJoCo reads link frames from `data.xmat` (no wxyz/xyzw quaternion hazard) and closest distances from `mj_geomDistance` over the same non-adjacent link pairs PyBullet queries with `getClosestPoints`.

## A. Forward-kinematics agreement (three-way)

| Robot | n | DH↔PyBullet resid | DH↔MuJoCo resid | PyBullet↔MuJoCo max pos | max orient |
|:--|--:|--:|--:|--:|--:|
| ur5 | 2000 | 9.5e-07 (base) | 4.2e-08 (base) | 4.11e-08 m | 5.93e-07 rad |
| franka_panda | 2000 | 6.6e-07 (tool) | 8.7e-16 (tool) | 5.90e-08 m | 4.09e-07 rad |

All three kinematic models agree to floating-point noise: our hand-typed DH, PyBullet, and MuJoCo are the same robot. This independently re-confirms the corrected modified-DH Panda on a second engine.

## B. Collision agreement (real mesh, both engines vs the capsule proxy)

| Robot | n | proxy col% | PyBullet col% | MuJoCo col% | PB↔MJ sign-agree% | PB↔MJ corr | proxy false-clear vs PB | vs MJ |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|
| ur5 | 3000 | 18.1 | 38.3 | 36.1 | 97.8 | 0.991 | 20.9% | 18.7% |
| franka_panda | 3000 | 0.6 | 9.2 | 8.2 | 99.0 | 0.880 | 8.5% | 7.6% |

- Both independent real-mesh engines report **far more** collision than the capsule proxy — the Phase-3 "proxy is optimistic" result is engine-independent.
- PyBullet and MuJoCo agree with each other on the collide/clear call at the sign-agree% above (residual disagreement is near-boundary convex-hull noise between two independent collision implementations).

## C. Solver collision edge — does it replicate on the second engine?

Every solver's `q_final` scored in **both** engines (real mesh self-collision).

### ur5 — open_space

| Solver | succ% | PyBullet col% | MuJoCo col% | col-call agree% | PB clear m | MJ clear m |
|:--|--:|--:|--:|--:|--:|--:|
| ProteinIK Raw Biology (V6) | 100.0 | 12.0 | 12.0 | 100.0 | +0.0055 | +0.0062 |
| ProteinIK Fast (V4) | 100.0 | 31.0 | 30.0 | 99.0 | +0.0004 | +0.0013 |
| TRAC-IK style | 98.0 | 34.0 | 33.0 | 99.0 | -0.0044 | -0.0041 |
| Multi-start | 97.0 | 35.0 | 34.0 | 99.0 | -0.0033 | -0.0018 |
| ProteinIK (V1) | 92.0 | 40.0 | 39.0 | 99.0 | -0.0028 | -0.0017 |

### ur5 — near_singular

| Solver | succ% | PyBullet col% | MuJoCo col% | col-call agree% | PB clear m | MJ clear m |
|:--|--:|--:|--:|--:|--:|--:|
| ProteinIK Raw Biology (V6) | 99.0 | 31.0 | 31.0 | 100.0 | -0.0040 | -0.0035 |
| ProteinIK Fast (V4) | 100.0 | 41.0 | 40.0 | 99.0 | -0.0066 | -0.0061 |
| ProteinIK (V1) | 88.0 | 43.0 | 41.0 | 98.0 | -0.0123 | -0.0118 |
| TRAC-IK style | 99.0 | 47.0 | 45.0 | 98.0 | -0.0154 | -0.0145 |
| Multi-start | 98.0 | 47.0 | 44.0 | 97.0 | -0.0125 | -0.0115 |

### ur5 — cluttered

| Solver | succ% | PyBullet col% | MuJoCo col% | col-call agree% | PB clear m | MJ clear m |
|:--|--:|--:|--:|--:|--:|--:|
| ProteinIK Raw Biology (V6) | 100.0 | 51.0 | 51.0 | 100.0 | -0.0158 | -0.0153 |
| ProteinIK Fast (V4) | 100.0 | 56.0 | 56.0 | 100.0 | -0.0186 | -0.0176 |
| ProteinIK (V1) | 90.0 | 64.0 | 61.0 | 97.0 | -0.0244 | -0.0232 |
| Multi-start | 98.0 | 65.0 | 63.0 | 98.0 | -0.0231 | -0.0223 |
| TRAC-IK style | 96.0 | 66.0 | 65.0 | 99.0 | -0.0332 | -0.0324 |

### franka_panda — open_space

| Solver | succ% | PyBullet col% | MuJoCo col% | col-call agree% | PB clear m | MJ clear m |
|:--|--:|--:|--:|--:|--:|--:|
| Multi-start | 99.0 | 7.0 | 6.0 | 99.0 | +0.0155 | +0.0138 |
| TRAC-IK style | 99.0 | 10.0 | 8.0 | 98.0 | +0.0142 | +0.0140 |
| ProteinIK Raw Biology (V6) | 100.0 | 10.0 | 9.0 | 99.0 | +0.0137 | +0.0128 |
| ProteinIK Fast (V4) | 100.0 | 11.0 | 11.0 | 100.0 | +0.0133 | +0.0122 |
| ProteinIK (V1) | 97.0 | 12.0 | 10.0 | 98.0 | +0.0134 | +0.0126 |

### franka_panda — near_singular

| Solver | succ% | PyBullet col% | MuJoCo col% | col-call agree% | PB clear m | MJ clear m |
|:--|--:|--:|--:|--:|--:|--:|
| ProteinIK Raw Biology (V6) | 98.0 | 9.0 | 8.0 | 99.0 | +0.0150 | +0.0140 |
| ProteinIK (V1) | 90.0 | 11.0 | 8.0 | 97.0 | +0.0142 | +0.0131 |
| TRAC-IK style | 99.0 | 12.0 | 11.0 | 99.0 | +0.0131 | +0.0116 |
| Multi-start | 97.0 | 12.0 | 11.0 | 99.0 | +0.0137 | +0.0125 |
| ProteinIK Fast (V4) | 100.0 | 13.0 | 11.0 | 98.0 | +0.0126 | +0.0121 |

### franka_panda — cluttered

| Solver | succ% | PyBullet col% | MuJoCo col% | col-call agree% | PB clear m | MJ clear m |
|:--|--:|--:|--:|--:|--:|--:|
| ProteinIK (V1) | 82.0 | 72.0 | 72.0 | 100.0 | -0.0426 | -0.0416 |
| ProteinIK Raw Biology (V6) | 94.0 | 76.0 | 75.0 | 99.0 | -0.0426 | -0.0417 |
| TRAC-IK style | 90.0 | 78.0 | 78.0 | 100.0 | -0.0482 | -0.0475 |
| ProteinIK Fast (V4) | 99.0 | 79.0 | 78.0 | 99.0 | -0.0475 | -0.0466 |
| Multi-start | 86.0 | 80.0 | 79.0 | 99.0 | -0.0476 | -0.0465 |
