# Clean-Solve Benchmark (PyBullet-certified collision selection)

- K candidates: **16**  |  trials/seed 60 × seeds [1, 2]
- `single`: honest one-shot (K=1). `clean`: best of K by real PyBullet clearance.

## ur5

| Solver | Scenario | succ% | single col% | **clean col%** | single clear | clean clear | mean cand | ms/solve (clean) |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|
| ProteinIK Fast (V4) | open_space | 100.0 | 32.5 | **5.8** | +0.0002 | +0.0093 | 16.0 | 238 |
| ProteinIK Fast (V4) | near_singular | 100.0 | 42.0 | **24.2** | -0.0073 | +0.0011 | 16.0 | 434 |
| ProteinIK Fast (V4) | cluttered | 100.0 | 60.0 | **40.8** | -0.0194 | -0.0096 | 16.0 | 486 |
