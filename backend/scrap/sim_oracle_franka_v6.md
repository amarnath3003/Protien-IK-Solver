# Sim-Oracle Benchmark (PyBullet)

- Trials/seed: **30**  |  Seeds: **[1, 2]**  (n=60 per cell)
- Robots: franka_panda  |  Scenarios: open_space, near_singular, cluttered

Each solver runs on our DH `RobotSpec` core; `q_final` is then re-scored in PyBullet (real FK + real mesh self-collision). `PyBullet native IK` is the sim's own solver on the identical targets.

**How to read it:** `our_succ` vs `sim_succ` tests whether a solve we call good survives an independent simulator's FK (Phase-1 parity, end to end). `our_col` vs `sim_col` tests whether our capsule collision proxy matches real mesh collision (the Phase-3 headline, previewed).

## franka_panda — open_space

| Solver | our_succ% | sim_succ% | agree% | our_col% | sim_col% | our_clear m | sim_clear m | our_pos mm | sim_pos mm | Mean ms | p95 ms |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Raw Biology (V6) | 100.0 | 100.0 | 100.0 | 0.0 | 10.0 | 0.0323 | 0.0128 | 0.172 | 0.172 | 4612.4 | 5441.6 |

## franka_panda — near_singular

| Solver | our_succ% | sim_succ% | agree% | our_col% | sim_col% | our_clear m | sim_clear m | our_pos mm | sim_pos mm | Mean ms | p95 ms |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Raw Biology (V6) | 98.3 | 98.3 | 100.0 | 0.0 | 6.7 | 0.0325 | 0.0156 | 0.497 | 0.497 | 4274.6 | 5035.1 |

## franka_panda — cluttered

| Solver | our_succ% | sim_succ% | agree% | our_col% | sim_col% | our_clear m | sim_clear m | our_pos mm | sim_pos mm | Mean ms | p95 ms |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Raw Biology (V6) | 91.7 | 91.7 | 100.0 | 26.7 | 86.7 | 0.0092 | -0.0508 | 1.814 | 1.814 | 4042.8 | 4454.9 |
