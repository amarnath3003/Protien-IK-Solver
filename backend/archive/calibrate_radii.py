"""
Phase 3 calibration: pick a capsule-radius inflation that makes the self-collision
proxy conservative w.r.t. real meshes, so a solver optimizing it (V4) actually
avoids real collisions. See sim_oracle_findings.md §7.

For a uniform per-link radius increase Δr, every non-adjacent pair's combined
radius grows by 2Δr, so the proxy min-distance shifts by exactly −2Δr. That lets us
evaluate any Δr offline against already-measured (proxy, real) pairs — no re-solve.

We report, per Δr:
  * false-clear%  — proxy'(=proxy−2Δr) ≥ 0 while real meshes interpenetrate (DANGER;
    the quadrant a proxy-optimizing solver cannot see). Drive this down.
  * false-alarm% — proxy' < 0 while really clear (over-conservative; may cost reach).
  * a blended cost that weights false-clear 4× (danger ≫ nuisance).

Usage:  PYTHONPATH=. .venv-sim/Scripts/python -m bench.calibrate_radii ur5 4000
"""

from __future__ import annotations

import sys

import numpy as np

from app.core.kinematics import get_robot_spec, self_collision_min_distance
from app.sim.pybullet_backend import PyBulletBackend

_REAL_THRESHOLD = 0.8
_FALSE_CLEAR_WEIGHT = 4.0  # a missed real collision is much worse than a false alarm


def sample(robot: str, n: int, seed: int = 0):
    spec = get_robot_spec(robot)
    rng = np.random.default_rng(seed)
    proxy = np.empty(n)
    real = np.empty(n)
    with PyBulletBackend(robot, verify_samples=200) as bk:
        for i in range(n):
            q = spec.random_config(rng)
            proxy[i] = self_collision_min_distance(spec, q)
            _, real[i] = bk.self_collision(q, threshold=_REAL_THRESHOLD)
    return spec, proxy, real


def sweep(proxy: np.ndarray, real: np.ndarray):
    real_col = real < 0.0
    rows = []
    best = None
    for dr in np.linspace(0.0, 0.030, 31):        # 0..30 mm per link
        proxy2 = proxy - 2.0 * dr
        proxy2_col = proxy2 < 0.0
        false_clear = float(np.mean(real_col & ~proxy2_col))
        false_alarm = float(np.mean(~real_col & proxy2_col))
        cost = _FALSE_CLEAR_WEIGHT * false_clear + false_alarm
        rows.append((dr, false_clear, false_alarm, cost))
        if best is None or cost < best[3]:
            best = (dr, false_clear, false_alarm, cost)
    return rows, best


def main(argv) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    robot = argv[0] if argv and argv[0] else "ur5"
    n = int(argv[1]) if len(argv) > 1 else 4000

    spec, proxy, real = sample(robot, n)
    rows, best = sweep(proxy, real)

    print(f"=== radius-inflation sweep: {robot} (n={n}) ===")
    print(f"  current link_radius: {np.array2string(spec.link_radius, precision=3)}")
    print(f"  {'Δr(mm)':>7} {'false_clear%':>12} {'false_alarm%':>12} {'cost':>8}")
    for dr, fc, fa, c in rows:
        mark = "  <== best" if abs(dr - best[0]) < 1e-9 else ""
        print(f"  {dr*1000:7.1f} {fc*100:12.1f} {fa*100:12.1f} {c*100:8.1f}{mark}")

    dr = best[0]
    new_radius = spec.link_radius + dr
    print(f"\n  RECOMMENDED Δr = {dr*1000:.1f} mm  "
          f"(false_clear {best[1]*100:.1f}%, false_alarm {best[2]*100:.1f}%)")
    print(f"  new link_radius = np.array({np.array2string(new_radius, precision=4, separator=', ')})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
