# Benchmark Numbers (committed result files)

> **✅ FRESH FULL SWEEP DONE → use `backend/v1v4_full_benchmark.{csv,md}` for the paper** (all 3 robots ×
> 3 scenarios, baselines + V1 + V4 + analytical, N=300, V5/V6 excluded). Headline: **V4 beats TRAC-IK on
> success in all 9 cells (98–100% vs 91–99%); collision edge real on planar+UR5 (e.g. UR5 open collide
> 3.0% vs 17.3%; UR5/planar cluttered V4 clash-free +0.008/+0.020 vs TRAC colliding −0.013/−0.006),
> marginal on Franka; V4 p50 ~2× faster than TRAC-IK on UR5 but a latency TAIL inflates the mean, worst on
> Franka (7–10× slower).** V1 beats only the simple baselines. The old committed
> `master_benchmark_results.csv` below is **Franka-only (superseded)** — kept for provenance only.

## Column dictionary

**`master_benchmark_results.csv`** (from `master_benchmark.py` METRIC_FIELDS): `robot, scenario, solver,
display_name, n` (=trials×seeds=300), `success_pct, mean_ms, p50_ms, p95_ms, p99_ms, mean_iters, collision_pct`
(=100·mean(min_self<0)), `mean_clearance_m` (=mean min_self_distance, the collision metric), `mean_pos_err_mm,
mean_orient_err_mrad, mean_joint_limit_violations, mean_restarts`.

**`v5_verify*.csv`** (from `v5_benchmark.py`): adds `mode` (diagnostics/ablation), `label, is_v5, compA/B/C,
c_threshold, lambda_beta, ci95_pp` (Wilson half-width), and V5 diagnostics `mean_conflict_index,
mean_lambda_final, lambda_lt08_pct, mean_difficulty`.

## Committed master benchmark — Franka Panda only, N=300

### franka_panda — open_space
| Solver | Succ% | Mean ms | p50 | p95 | Clear m | Collide% |
| :-- | --: | --: | --: | --: | --: | --: |
| Jacobian (DLS) | 24.33 | 57.97 | 63.59 | 103.44 | -0.01667 | 65.67 |
| CCD | 16.33 | 233.07 | 256.65 | 301.32 | -0.01806 | 69.00 |
| FABRIK | 16.67 | 152.06 | 166.74 | 195.06 | -0.01882 | 70.33 |
| TRAC-IK style | 95.00 | 26.12 | 19.26 | 86.08 | -0.02069 | 75.33 |
| Multi-start | 86.33 | 127.46 | 130.01 | 159.25 | -0.02026 | 75.67 |
| **ProteinIK (V1)** | 81.00 | 85.25 | 57.63 | 205.23 | -0.01894 | 72.33 |
| **ProteinIK Fast (V4)** | **99.67** | 193.62 | 125.12 | 632.90 | -0.01742 | 72.00 |

### franka_panda — near_singular
| Solver | Succ% | Mean ms | Clear m | Collide% |
| :-- | --: | --: | --: | --: |
| Jacobian (DLS) | 27.67 | 48.79 | -0.01526 | 63.33 |
| CCD | 18.00 | 226.59 | -0.01813 | 67.67 |
| FABRIK | 18.00 | 159.11 | -0.01911 | 70.33 |
| TRAC-IK style | 91.33 | 30.23 | -0.01805 | 68.33 |
| Multi-start | 82.33 | 128.51 | -0.01720 | 67.67 |
| **ProteinIK (V1)** | 72.00 | 101.16 | -0.01693 | 68.00 |
| **ProteinIK Fast (V4)** | **98.00** | 219.69 | -0.01528 | 65.67 |

### franka_panda — cluttered
| Solver | Succ% | Mean ms | Clear m | Collide% |
| :-- | --: | --: | --: | --: |
| Jacobian (DLS) | 25.00 | 54.80 | -0.03152 | 83.33 |
| CCD | 19.67 | 238.54 | -0.03500 | 89.00 |
| FABRIK | 20.33 | 138.90 | -0.03200 | 85.33 |
| TRAC-IK style | 91.00 | 27.24 | -0.04221 | 99.00 |
| Multi-start | 83.33 | 124.22 | -0.04216 | 98.67 |
| **ProteinIK (V1)** | 74.67 | 106.00 | -0.03847 | 95.67 |
| **ProteinIK Fast (V4)** | **98.33** | 319.70 | -0.04001 | 98.67 |

### What the Franka data says (honest)
- **V4 = success leader** (98–99.7%), beats every baseline; beats TRAC-IK by +4.7/+6.7/+7.3 pp, Multi-start +13–16 pp.
- **V1 beats only simple baselines** (Jacobian/CCD/FABRIK 16–28%); **trails** TRAC-IK & Multi-start.
- **V4 latency cost:** 194–320 ms mean vs TRAC-IK 26–30 ms → **7–12× slower** on Franka.
- **No collision edge here:** every solver collides 65–99% with negative mean clearance; V4 ≈ or worse than simple
  baselines. **This is Franka under a proxy that is elbow-pinned (see 06_core §3)** — the collision story must be
  re-examined on UR5/Planar in the fresh sweep, and read via `mean_clearance_m`, not just collide%.

## V5 ablation (UR5) — for the report only

Full V5 (A1B1C1) does **not** beat fixed-λ / all-off (N=100): open 94 vs 95, near 93 vs 93, **cluttered 92 vs 98**.
Cost 1117–1448 ms mean vs V4 7.8–23.5 ms (**~50–190×**). Claims "94% near vs 90% fixed" and the difficulty
rank-ordering are **not in the data**. See `03_v5_cchik.md §8`.

## V6 / Raw
**Absent from all committed CSVs.** No `protein_raw` rows anywhere. V6 quality numbers exist only in
`raw_notes.md` (thought log) and the phase experiments — see `04_v6_raw.md`.
