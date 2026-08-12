# Sim-Oracle Benchmark (PyBullet)

- Trials/seed: **50**  |  Seeds: **[1, 2, 3]**  (n=150 per cell)
- Robots: ur5  |  Scenarios: open_space, near_singular, cluttered

Each solver runs on our DH `RobotSpec` core; `q_final` is then re-scored in PyBullet (real FK + real mesh self-collision). `PyBullet native IK` is the sim's own solver on the identical targets.

**How to read it:** `our_succ` vs `sim_succ` tests whether a solve we call good survives an independent simulator's FK (Phase-1 parity, end to end). `our_col` vs `sim_col` tests whether our capsule collision proxy matches real mesh collision (the Phase-3 headline, previewed).

## ur5 — open_space

| Solver | our_succ% | sim_succ% | agree% | our_col% | sim_col% | our_clear m | sim_clear m | our_pos mm | sim_pos mm | Mean ms | p95 ms |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 2.7 | 29.3 | 0.0175 | 0.0003 | 0.254 | 0.254 | 19.9 | 25.3 |
| ProteinIK Raw Biology (V6) | 99.3 | 99.3 | 100.0 | 2.7 | 14.0 | 0.0173 | 0.0054 | 0.200 | 0.200 | 6109.5 | 13493.2 |
| TRAC-IK style | 98.7 | 98.7 | 100.0 | 17.3 | 32.7 | 0.0109 | -0.0042 | 0.775 | 0.775 | 18.7 | 72.4 |
| Multi-start | 97.3 | 97.3 | 100.0 | 13.3 | 34.7 | 0.0130 | -0.0038 | 0.962 | 0.962 | 141.2 | 226.6 |
| ProteinIK (V1) | 92.0 | 92.0 | 100.0 | 12.7 | 36.7 | 0.0135 | -0.0023 | 31.951 | 31.951 | 61.9 | 255.2 |
| Jacobian (DLS) | 56.7 | 56.7 | 100.0 | 11.3 | 28.0 | 0.0130 | -0.0029 | 59.807 | 59.807 | 93.5 | 338.3 |
| FABRIK | 53.3 | 53.3 | 100.0 | 20.7 | 42.7 | 0.0072 | -0.0077 | 47.157 | 47.157 | 172.8 | 387.7 |
| CCD | 44.7 | 44.7 | 100.0 | 24.7 | 38.0 | 0.0040 | -0.0077 | 34.811 | 34.811 | 770.1 | 1611.7 |
| PyBullet native IK |   -   | 74.7 |   -   |   -   | 42.0 |   -   | -0.0059 |   -   | 8.251 | 16.1 | 38.3 |

## ur5 — near_singular

| Solver | our_succ% | sim_succ% | agree% | our_col% | sim_col% | our_clear m | sim_clear m | our_pos mm | sim_pos mm | Mean ms | p95 ms |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| TRAC-IK style | 99.3 | 99.3 | 100.0 | 29.3 | 48.7 | 0.0004 | -0.0160 | 0.783 | 0.783 | 28.9 | 98.3 |
| ProteinIK Fast (V4) | 100.0 | 99.3 | 99.3 | 8.7 | 39.3 | 0.0144 | -0.0050 | 0.348 | 0.348 | 44.8 | 250.9 |
| ProteinIK Raw Biology (V6) | 98.7 | 98.7 | 100.0 | 8.0 | 26.0 | 0.0142 | -0.0015 | 0.527 | 0.527 | 3343.5 | 3955.5 |
| Multi-start | 98.0 | 98.0 | 100.0 | 22.0 | 44.7 | 0.0058 | -0.0114 | 0.654 | 0.654 | 156.3 | 237.2 |
| ProteinIK (V1) | 90.0 | 90.0 | 100.0 | 18.0 | 39.3 | 0.0093 | -0.0089 | 38.776 | 38.776 | 86.0 | 285.1 |
| Jacobian (DLS) | 47.3 | 47.3 | 100.0 | 29.3 | 48.7 | -0.0032 | -0.0188 | 71.744 | 71.744 | 71.5 | 160.2 |
| FABRIK | 33.3 | 33.3 | 100.0 | 28.0 | 53.3 | -0.0008 | -0.0177 | 56.287 | 56.287 | 167.0 | 255.8 |
| CCD | 32.0 | 32.0 | 100.0 | 36.7 | 46.7 | -0.0066 | -0.0175 | 43.430 | 43.430 | 375.7 | 689.7 |
| PyBullet native IK |   -   | 46.0 |   -   |   -   | 54.0 |   -   | -0.0162 |   -   | 14.410 | 11.2 | 14.7 |

## ur5 — cluttered

| Solver | our_succ% | sim_succ% | agree% | our_col% | sim_col% | our_clear m | sim_clear m | our_pos mm | sim_pos mm | Mean ms | p95 ms |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | 100.0 | 100.0 | 100.0 | 17.3 | 56.0 | 0.0089 | -0.0165 | 0.316 | 0.316 | 26.7 | 153.6 |
| ProteinIK Raw Biology (V6) | 100.0 | 100.0 | 100.0 | 16.0 | 48.7 | 0.0097 | -0.0135 | 0.288 | 0.288 | 3003.1 | 3399.3 |
| Multi-start | 98.7 | 98.7 | 100.0 | 38.7 | 65.3 | -0.0050 | -0.0240 | 0.552 | 0.552 | 120.4 | 197.5 |
| TRAC-IK style | 97.3 | 97.3 | 100.0 | 44.0 | 64.7 | -0.0117 | -0.0302 | 0.735 | 0.735 | 19.7 | 68.9 |
| ProteinIK (V1) | 90.7 | 90.7 | 100.0 | 33.3 | 65.3 | -0.0031 | -0.0238 | 13.776 | 13.776 | 93.6 | 325.3 |
| Jacobian (DLS) | 57.3 | 57.3 | 100.0 | 48.7 | 68.0 | -0.0196 | -0.0347 | 38.068 | 38.068 | 34.1 | 80.3 |
| CCD | 42.0 | 42.0 | 100.0 | 60.0 | 70.0 | -0.0241 | -0.0365 | 19.607 | 19.607 | 177.2 | 329.2 |
| FABRIK | 34.0 | 34.0 | 100.0 | 51.3 | 70.0 | -0.0179 | -0.0348 | 35.130 | 35.130 | 134.3 | 237.2 |
| PyBullet native IK |   -   | 54.0 |   -   |   -   | 71.3 |   -   | -0.0359 |   -   | 5.788 | 9.0 | 12.6 |
