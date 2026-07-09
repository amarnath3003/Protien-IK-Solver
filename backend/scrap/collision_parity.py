"""
Phase 3 (sim_migration_plan.md §5): quantify capsule-proxy vs real-mesh collision.

Phase 2 compared collision at the *rate* level (what fraction of a solver's
solutions collide, proxy vs real). This is the *config* level: for the same random
configuration, how does our capsule ``self_collision_min_distance`` compare to
PyBullet's real-mesh ``getClosestPoints`` minimum? That is the plan's literal
Phase-3 task, and it produces two things:

  1. **The honest characterization** — is the proxy biased (optimistic/pessimistic),
     by how much, and how often does it get the *sign* wrong (say "clear" when the
     real meshes actually interpenetrate — the dangerous case a solver optimizing
     the proxy would happily accept).
  2. **The calibration recipe** — the smallest safety margin ``δ`` such that
     "proxy clearance > δ" implies "really clear" with high probability. Feeding
     that δ (or a recalibrated per-link radius) back into the collision energy is
     how V4's collision term can be made to track reality instead of an optimistic
     surrogate. See sim_oracle_findings.md §7.

Runs headless; needs PyBullet, so execute in ``.venv-sim``:

    PYTHONPATH=. .venv-sim/Scripts/python -m bench.collision_parity
    PYTHONPATH=. .venv-sim/Scripts/python -m bench.collision_parity ur5 5000
"""

from __future__ import annotations

import sys

import numpy as np

from app.core.kinematics import get_robot_spec, self_collision_min_distance
from app.sim.pybullet_backend import PyBulletBackend

# Query real closest points out to this range so clearances aren't clamped near the
# collision boundary (the regime that matters); far-apart pairs beyond it are
# irrelevant to a collision metric.
_REAL_THRESHOLD = 0.8


def _summ(x):
    x = np.asarray(x)
    return (f"mean={x.mean():+.4f} median={np.median(x):+.4f} "
            f"p10={np.percentile(x, 10):+.4f} p90={np.percentile(x, 90):+.4f}")


def characterize(robot: str, n: int, seed: int = 0) -> dict:
    """Sample N random configs; compare proxy vs real min self-distance.

    Also attributes each real collision to the link *pair* whose meshes are
    closest (``self_collision_detail``), so the report can say *where* the capsule
    proxy is wrong -- which pair drives real collision overall, and which pairs the
    proxy's dangerous "false-clear" verdicts land on.
    """
    spec = get_robot_spec(robot)
    rng = np.random.default_rng(seed)
    proxy = np.empty(n)
    real = np.empty(n)
    drivers: list[tuple[str, str] | None] = []
    with PyBulletBackend(robot, verify_samples=200) as bk:
        for i in range(n):
            q = spec.random_config(rng)
            proxy[i] = self_collision_min_distance(spec, q)
            _, real[i], pair = bk.self_collision_detail(q, threshold=_REAL_THRESHOLD)
            drivers.append(pair)

    # Per-pair attribution: among real-colliding configs, and among the dangerous
    # false-clear configs (proxy says clear, meshes interpenetrate), tally the
    # driving pair. Concentrated tallies == a specific geometry the proxy misses.
    def _tally(mask):
        counts: dict[tuple[str, str], int] = {}
        for i in range(n):
            if mask[i] and drivers[i] is not None:
                counts[drivers[i]] = counts.get(drivers[i], 0) + 1
        total = sum(counts.values())
        ranked = sorted(counts.items(), key=lambda kv: -kv[1])
        return [(f"{a}|{b}", c, 100 * c / total) for (a, b), c in ranked], total

    gap = proxy - real                          # >0 == proxy optimistic (claims more clearance)
    real_collide = real < 0.0
    proxy_collide = proxy < 0.0
    driver_real, n_driver_real = _tally(real_collide)
    driver_fc, n_driver_fc = _tally(real_collide & ~proxy_collide)
    false_clear = np.mean(real_collide & ~proxy_collide)   # proxy "safe", really colliding (DANGER)
    false_alarm = np.mean(~real_collide & proxy_collide)   # proxy "collide", really clear
    agree_sign = np.mean(real_collide == proxy_collide)

    # correlation near the boundary (both within 0.1 m) — where a collision metric
    # actually has to be right; far-clear configs are trivially correlated.
    near = (np.abs(proxy) < 0.1) & (np.abs(real) < 0.1)
    corr = float(np.corrcoef(proxy, real)[0, 1])
    corr_near = float(np.corrcoef(proxy[near], real[near])[0, 1]) if near.sum() > 20 else float("nan")

    # Calibrated margin: smallest δ so that P(real<0 | proxy>=δ) <= 1%.
    delta_grid = np.linspace(0.0, 0.15, 151)
    delta_star = None
    for d in delta_grid:
        mask = proxy >= d
        if mask.sum() == 0:
            continue
        residual_risk = np.mean(real[mask] < 0.0)
        if residual_risk <= 0.01:
            delta_star = float(d)
            break

    return {
        "robot": robot, "n": n,
        "proxy": proxy, "real": real, "gap": gap,
        "real_collide_pct": 100 * float(real_collide.mean()),
        "proxy_collide_pct": 100 * float(proxy_collide.mean()),
        "false_clear_pct": 100 * float(false_clear),
        "false_alarm_pct": 100 * float(false_alarm),
        "agree_sign_pct": 100 * float(agree_sign),
        "corr": corr, "corr_near": corr_near,
        "delta_star": delta_star,
        "near_count": int(near.sum()),
        "driver_real": driver_real, "n_driver_real": n_driver_real,
        "driver_false_clear": driver_fc, "n_driver_false_clear": n_driver_fc,
    }


def report(r: dict) -> str:
    lines = [
        f"=== {r['robot']}  (n={r['n']}) ===",
        f"  real collide : {r['real_collide_pct']:5.1f}%   "
        f"proxy collide: {r['proxy_collide_pct']:5.1f}%   "
        f"sign-agree: {r['agree_sign_pct']:5.1f}%",
        f"  FALSE-CLEAR  : {r['false_clear_pct']:5.1f}%  "
        f"(proxy says SAFE but meshes interpenetrate — the dangerous quadrant)",
        f"  false-alarm  : {r['false_alarm_pct']:5.1f}%  (proxy says collide, really clear)",
        f"  optimism gap : {_summ(r['gap'])}  (proxy − real; >0 = proxy over-claims clearance)",
        f"  correlation  : all={r['corr']:.3f}  near-boundary={r['corr_near']:.3f} "
        f"(n_near={r['near_count']})",
    ]
    if r["delta_star"] is not None:
        lines.append(f"  calibrated δ : {r['delta_star']:.3f} m  "
                     f"⇒ requiring proxy-clearance > {r['delta_star']*1000:.0f} mm "
                     f"makes real-collision risk ≤ 1%")
    else:
        lines.append("  calibrated δ : none in [0,0.15] m brings residual risk ≤1% "
                     "(proxy too weakly correlated — recalibrate radii/geometry, not just margin)")

    def _top(rows, k=3):
        return ", ".join(f"{name} {pct:.0f}%" for name, _, pct in rows[:k]) or "(none)"
    lines.append(f"  drives real  : {_top(r['driver_real'])}  "
                 f"(link pair closest in real-colliding configs)")
    lines.append(f"  drives FALSE-CLEAR : {_top(r['driver_false_clear'])}  "
                 f"(pairs the capsule proxy misses)")
    return "\n".join(lines)


def write_markdown(results: list[dict], path: str) -> None:
    lines = ["# Collision-Proxy vs Real-Mesh Parity (Phase 3)", "",
             "Per-config comparison of our capsule `self_collision_min_distance` "
             "against PyBullet real-mesh `getClosestPoints`, over random configs.", "",
             "| Robot | n | real col% | proxy col% | sign-agree% | **false-clear%** | "
             "false-alarm% | gap mean (m) | corr (near) | calibrated δ (m) |",
             "|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|"]
    for r in results:
        d = f"{r['delta_star']:.3f}" if r["delta_star"] is not None else "none≤0.15"
        lines.append(
            f"| {r['robot']} | {r['n']} | {r['real_collide_pct']:.1f} | "
            f"{r['proxy_collide_pct']:.1f} | {r['agree_sign_pct']:.1f} | "
            f"**{r['false_clear_pct']:.1f}** | {r['false_alarm_pct']:.1f} | "
            f"{float(np.mean(r['gap'])):+.4f} | {r['corr_near']:.3f} | {d} |")
    lines += ["",
              "- **false-clear%** = proxy reports clearance ≥ 0 while the real meshes "
              "interpenetrate. This is the quadrant a solver optimizing the proxy "
              "cannot see — it accepts these as collision-free. Driving it down is the "
              "point of recalibration.",
              "- **gap** = proxy − real; positive means the proxy over-claims clearance "
              "(optimistic).",
              "- **calibrated δ** = smallest proxy-clearance threshold that makes real "
              "collision ≤1% likely; a usable safety margin for the collision energy "
              "if the correlation is strong enough.", ""]

    # Per-pair mechanism: which link pairs drive real collision, and which the proxy
    # misses (false-clears). Localizes *why* the proxy is optimistic.
    lines += ["## Where the proxy fails — per-link-pair attribution", "",
              "For each config the driving pair is the one whose real meshes are "
              "closest. Below: the pairs that drive real collision overall, and the "
              "pairs behind the proxy's dangerous false-clears.", ""]
    for r in results:
        lines += [f"### {r['robot']}", "",
                  "| driver of… | top link pairs (share of that quadrant) |",
                  "|:--|:--|"]
        def _fmt_pairs(rows, k=5):
            return ", ".join(f"`{name}` {pct:.0f}%" for name, _, pct in rows[:k]) or "(none)"
        lines += [f"| real collision (n={r['n_driver_real']}) | {_fmt_pairs(r['driver_real'])} |",
                  f"| **false-clear** (n={r['n_driver_false_clear']}) | "
                  f"{_fmt_pairs(r['driver_false_clear'])} |", ""]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def main(argv) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    robots = ["ur5", "franka_panda"]
    n = 3000
    if len(argv) >= 1 and argv[0]:
        robots = [argv[0]]
    if len(argv) >= 2:
        n = int(argv[1])

    results = []
    for robot in robots:
        r = characterize(robot, n)
        results.append(r)
        print(report(r), flush=True)
        print(flush=True)

    write_markdown(results, "collision_parity.md")
    print("Wrote collision_parity.md", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
