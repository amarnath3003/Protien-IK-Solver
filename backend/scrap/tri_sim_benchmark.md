# Tri-Simulator Mini-Benchmark — V4 vs baselines under 3 collision models

- Trials/seed **50** × seeds **[1, 2]** (n=100/cell)  |  same `q_final` scored 3 ways
- **our** = capsule proxy (what the solver optimizes) · **PB** = PyBullet real mesh · **MJ** = MuJoCo real mesh (identical URDF & link pairs)
- Collision **winner** = lowest-collision solver among those ≥90% success. Paper uses **PB + MJ**; `our` shown to expose the proxy's optimism.

## Verdict — which solver collides least (high-success solvers only)

| Robot | Scenario | our sim | **PyBullet** | **MuJoCo** |
|:--|:--|:--|:--|:--|
| ur5 | open_space | **ProteinIK Fast (V4)** | **ProteinIK Fast (V4)** | **ProteinIK Fast (V4)** |
| ur5 | near_singular | **ProteinIK Fast (V4)** | **ProteinIK Fast (V4)** | **ProteinIK Fast (V4)** |
| ur5 | cluttered | **ProteinIK Fast (V4)** | **ProteinIK Fast (V4)** | **ProteinIK Fast (V4)** |
| franka_panda | open_space | TRAC-IK style | Multi-start | Multi-start |
| franka_panda | near_singular | **ProteinIK Fast (V4)** | ProteinIK (V1) | ProteinIK (V1) |
| franka_panda | cluttered | **ProteinIK Fast (V4)** | TRAC-IK style | **ProteinIK Fast (V4)** |

A **bold** cell = V4 wins that (arm, scenario, simulator).

## ur5 — open_space

| Solver | succ% | our col% | **PB col%** | **MJ col%** | our clear | PB clear | MJ clear |
|:--|--:|--:|--:|--:|--:|--:|--:|
| Jacobian (DLS) | 57 | 11 | 29 | 26 | +0.0135 | -0.0026 | -0.0028 |
| ProteinIK Fast (V4) ⟵ V4 | 100 | 2 | 31 | 30 | +0.0178 | +0.0004 | +0.0013 |
| TRAC-IK style | 98 | 18 | 34 | 33 | +0.0104 | -0.0044 | -0.0041 |
| FABRIK | 51 | 14 | 34 | 34 | +0.0103 | -0.0032 | -0.0019 |
| Multi-start | 97 | 12 | 35 | 34 | +0.0136 | -0.0033 | -0.0018 |
| CCD | 43 | 25 | 37 | 37 | +0.0048 | -0.0061 | -0.0051 |
| ProteinIK (V1) | 92 | 9 | 40 | 39 | +0.0144 | -0.0028 | -0.0017 |

## ur5 — near_singular

| Solver | succ% | our col% | **PB col%** | **MJ col%** | our clear | PB clear | MJ clear |
|:--|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) ⟵ V4 | 100 | 8 | 41 | 40 | +0.0140 | -0.0066 | -0.0061 |
| ProteinIK (V1) | 88 | 19 | 43 | 41 | +0.0078 | -0.0123 | -0.0118 |
| CCD | 30 | 37 | 45 | 43 | -0.0052 | -0.0160 | -0.0152 |
| Jacobian (DLS) | 46 | 28 | 46 | 45 | -0.0027 | -0.0184 | -0.0180 |
| TRAC-IK style | 99 | 28 | 47 | 45 | +0.0011 | -0.0154 | -0.0145 |
| Multi-start | 98 | 20 | 47 | 44 | +0.0065 | -0.0125 | -0.0115 |
| FABRIK | 30 | 26 | 53 | 50 | +0.0009 | -0.0177 | -0.0169 |

## ur5 — cluttered

| Solver | succ% | our col% | **PB col%** | **MJ col%** | our clear | PB clear | MJ clear |
|:--|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) ⟵ V4 | 100 | 19 | 56 | 56 | +0.0078 | -0.0186 | -0.0176 |
| ProteinIK (V1) | 90 | 32 | 64 | 61 | -0.0004 | -0.0244 | -0.0232 |
| Multi-start | 98 | 40 | 65 | 63 | -0.0050 | -0.0231 | -0.0223 |
| TRAC-IK style | 96 | 49 | 66 | 65 | -0.0155 | -0.0332 | -0.0324 |
| CCD | 38 | 58 | 68 | 64 | -0.0230 | -0.0364 | -0.0360 |
| Jacobian (DLS) | 53 | 51 | 69 | 67 | -0.0221 | -0.0349 | -0.0343 |
| FABRIK | 26 | 54 | 70 | 70 | -0.0201 | -0.0362 | -0.0349 |

## franka_panda — open_space

| Solver | succ% | our col% | **PB col%** | **MJ col%** | our clear | PB clear | MJ clear |
|:--|--:|--:|--:|--:|--:|--:|--:|
| Multi-start | 99 | 0 | 7 | 6 | +0.0324 | +0.0155 | +0.0138 |
| TRAC-IK style | 99 | 0 | 10 | 8 | +0.0324 | +0.0142 | +0.0140 |
| ProteinIK Fast (V4) ⟵ V4 | 100 | 1 | 11 | 11 | +0.0321 | +0.0133 | +0.0122 |
| ProteinIK (V1) | 97 | 0 | 12 | 10 | +0.0323 | +0.0134 | +0.0126 |
| Jacobian (DLS) | 56 | 1 | 14 | 12 | +0.0317 | +0.0125 | +0.0123 |
| CCD | 28 | 1 | 17 | 16 | +0.0320 | +0.0118 | +0.0112 |
| FABRIK | 21 | 0 | 23 | 22 | +0.0321 | +0.0100 | +0.0099 |

## franka_panda — near_singular

| Solver | succ% | our col% | **PB col%** | **MJ col%** | our clear | PB clear | MJ clear |
|:--|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK (V1) | 90 | 1 | 11 | 8 | +0.0320 | +0.0142 | +0.0131 |
| TRAC-IK style | 99 | 0 | 12 | 11 | +0.0325 | +0.0131 | +0.0116 |
| Multi-start | 97 | 0 | 12 | 11 | +0.0324 | +0.0137 | +0.0125 |
| ProteinIK Fast (V4) ⟵ V4 | 100 | 0 | 13 | 11 | +0.0324 | +0.0126 | +0.0121 |
| Jacobian (DLS) | 49 | 3 | 17 | 17 | +0.0299 | +0.0081 | +0.0069 |
| CCD | 10 | 0 | 19 | 18 | +0.0322 | +0.0097 | +0.0099 |
| FABRIK | 13 | 1 | 23 | 23 | +0.0317 | +0.0084 | +0.0078 |

## franka_panda — cluttered

| Solver | succ% | our col% | **PB col%** | **MJ col%** | our clear | PB clear | MJ clear |
|:--|--:|--:|--:|--:|--:|--:|--:|
| Jacobian (DLS) | 33 | 34 | 71 | 71 | +0.0084 | -0.0422 | -0.0410 |
| ProteinIK (V1) | 82 | 29 | 72 | 72 | +0.0102 | -0.0426 | -0.0416 |
| FABRIK | 24 | 29 | 77 | 76 | +0.0079 | -0.0425 | -0.0414 |
| TRAC-IK style | 90 | 37 | 78 | 78 | +0.0051 | -0.0482 | -0.0475 |
| ProteinIK Fast (V4) ⟵ V4 | 99 | 29 | 79 | 78 | +0.0099 | -0.0475 | -0.0466 |
| Multi-start | 86 | 33 | 80 | 79 | +0.0045 | -0.0476 | -0.0465 |
| CCD | 13 | 46 | 80 | 80 | -0.0006 | -0.0480 | -0.0464 |
