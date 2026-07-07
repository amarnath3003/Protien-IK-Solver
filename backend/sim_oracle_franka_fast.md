# Sim-Oracle Benchmark (PyBullet)

- Trials/seed: **50**  |  Seeds: **[1, 2, 3]**  (n=150 per cell)
- Robots: franka_panda  |  Scenarios: open_space, near_singular, cluttered

Each solver runs on our DH `RobotSpec` core; `q_final` is then re-scored in PyBullet (real FK + real mesh self-collision). `PyBullet native IK` is the sim's own solver on the identical targets.

**How to read it:** `our_succ` vs `sim_succ` tests whether a solve we call good survives an independent simulator's FK (Phase-1 parity, end to end). `our_col` vs `sim_col` tests whether our capsule collision proxy matches real mesh collision (the Phase-3 headline, previewed).

## franka_panda — open_space

| Solver | our_succ% | sim_succ% | agree% | our_col% | sim_col% | our_clear m | sim_clear m | our_pos mm | sim_pos mm | Mean ms | p95 ms |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 0.7 | 8.7 | 0.0322 | 0.0147 | 0.378 | 0.378 | 30.6 | 179.8 |
| TRAC-IK style | 99.3 | 99.3 | 100.0 | 0.0 | 7.3 | 0.0324 | 0.0151 | 0.570 | 0.570 | 21.6 | 65.3 |
| Multi-start | 99.3 | 99.3 | 100.0 | 0.0 | 6.0 | 0.0324 | 0.0158 | 0.451 | 0.451 | 150.5 | 202.7 |
| ProteinIK (V1) | 96.7 | 96.7 | 100.0 | 0.0 | 11.3 | 0.0323 | 0.0138 | 7.346 | 7.346 | 78.7 | 256.1 |
| Jacobian (DLS) | 49.3 | 49.3 | 100.0 | 0.7 | 12.7 | 0.0320 | 0.0134 | 121.116 | 121.116 | 55.7 | 118.8 |
| CCD | 24.0 | 24.0 | 100.0 | 0.7 | 18.0 | 0.0322 | 0.0109 | 19.591 | 19.591 | 299.9 | 461.9 |
| FABRIK | 22.7 | 22.7 | 100.0 | 0.0 | 22.0 | 0.0322 | 0.0106 | 45.177 | 45.177 | 204.5 | 258.6 |
| PyBullet native IK |   -   | 81.3 |   -   |   -   | 28.0 |   -   | -0.0018 |   -   | 11.219 | 11.0 | 26.1 |

## franka_panda — near_singular

| Solver | our_succ% | sim_succ% | agree% | our_col% | sim_col% | our_clear m | sim_clear m | our_pos mm | sim_pos mm | Mean ms | p95 ms |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 0.0 | 10.7 | 0.0324 | 0.0133 | 0.397 | 0.397 | 38.2 | 197.7 |
| TRAC-IK style | 99.3 | 99.3 | 100.0 | 0.0 | 11.3 | 0.0325 | 0.0134 | 0.820 | 0.820 | 25.6 | 89.3 |
| Multi-start | 97.3 | 97.3 | 100.0 | 0.0 | 10.7 | 0.0325 | 0.0140 | 0.804 | 0.804 | 160.8 | 222.4 |
| ProteinIK (V1) | 90.0 | 90.0 | 100.0 | 0.7 | 8.7 | 0.0322 | 0.0149 | 20.245 | 20.245 | 100.4 | 302.8 |
| Jacobian (DLS) | 50.0 | 50.0 | 100.0 | 2.0 | 12.7 | 0.0307 | 0.0111 | 84.372 | 84.372 | 51.8 | 97.9 |
| FABRIK | 13.3 | 13.3 | 100.0 | 0.7 | 22.0 | 0.0319 | 0.0092 | 50.764 | 50.764 | 223.3 | 259.2 |
| CCD | 12.7 | 12.7 | 100.0 | 0.0 | 18.7 | 0.0323 | 0.0105 | 23.084 | 23.084 | 318.0 | 384.0 |
| PyBullet native IK |   -   | 64.7 |   -   |   -   | 24.0 |   -   | 0.0023 |   -   | 16.006 | 17.7 | 28.9 |

## franka_panda — cluttered

| Solver | our_succ% | sim_succ% | agree% | our_col% | sim_col% | our_clear m | sim_clear m | our_pos mm | sim_pos mm | Mean ms | p95 ms |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 98.7 | 98.0 | 99.3 | 30.7 | 78.7 | 0.0096 | -0.0461 | 0.897 | 0.897 | 144.7 | 753.7 |
| TRAC-IK style | 91.3 | 91.3 | 100.0 | 40.0 | 78.0 | 0.0031 | -0.0481 | 1.881 | 1.881 | 37.1 | 126.9 |
| Multi-start | 85.3 | 85.3 | 100.0 | 36.0 | 78.0 | 0.0039 | -0.0466 | 3.984 | 3.984 | 167.7 | 228.6 |
| ProteinIK (V1) | 83.3 | 83.3 | 100.0 | 31.3 | 72.0 | 0.0090 | -0.0414 | 32.017 | 32.017 | 114.0 | 286.1 |
| Jacobian (DLS) | 32.7 | 32.7 | 100.0 | 37.3 | 68.7 | 0.0075 | -0.0414 | 151.112 | 151.112 | 67.8 | 111.9 |
| FABRIK | 20.7 | 20.7 | 100.0 | 31.3 | 74.7 | 0.0083 | -0.0409 | 59.219 | 59.219 | 204.0 | 262.4 |
| CCD | 11.3 | 11.3 | 100.0 | 46.7 | 81.3 | -0.0001 | -0.0497 | 16.976 | 16.976 | 318.2 | 370.0 |
| PyBullet native IK |   -   | 89.3 |   -   |   -   | 85.3 |   -   | -0.0620 |   -   | 4.917 | 8.1 | 23.2 |
