"""Probe: can PyBullet and MuJoCo score the generated planar N-DOF arms, and do
they agree with the capsule proxy?

Answers three questions before we touch the DOF-scaling benchmark:
  1. Does each engine's URDF importer accept <capsule>? (falls back to <cylinder>)
  2. Does the generated URDF's FK match our DH FK? (must be ~1e-15)
  3. Does each engine's self-collision distance match the proxy's, per config?

Run (WSL Ubuntu-2204, from backend/):
    PYTHONPATH=. python3 scrap/planar_sim_probe.py
"""
from __future__ import annotations

import traceback

import numpy as np

from app.core.kinematics import (
    end_effector_pose, self_collision_min_distance,
)
from app.sim import planar_model

DOFS = [4, 6, 8, 12, 16]
N_SAMPLES = 200


def try_load(robot: str, engine: str):
    """Return (backend_or_None, error_string)."""
    try:
        if engine == "pybullet":
            from app.sim.pybullet_backend import PyBulletBackend
            return PyBulletBackend(robot), ""
        from app.sim.mujoco_backend import MuJoCoBackend
        return MuJoCoBackend(robot), ""
    except Exception as exc:  # noqa: BLE001 - probe reports, never raises
        return None, f"{type(exc).__name__}: {exc}"


def probe(geom_type: str) -> None:
    print(f"\n{'='*78}\n### geom_type = {geom_type}\n{'='*78}", flush=True)
    for n in DOFS:
        robot = f"planar{n}dof"
        # re-register cleanly so a second pass with another geom_type rebuilds
        planar_model.GENERATED_URDFS.pop(robot, None)
        from app.core.kinematics import ROBOT_REGISTRY
        ROBOT_REGISTRY.pop(robot, None)
        try:
            planar_model.register_planar_arm(n, geom_type=geom_type)
        except Exception as exc:  # noqa: BLE001
            print(f"[{robot}] register FAILED: {exc}")
            continue

        spec = planar_model.planar_ndof_spec(n)
        rng = np.random.default_rng(0)
        qs = rng.uniform(-np.pi, np.pi, size=(N_SAMPLES, n))

        for engine in ("pybullet", "mujoco"):
            be, err = try_load(robot, engine)
            if be is None:
                print(f"[{robot:>11}] {engine:<9} LOAD FAILED  {err.splitlines()[0][:90]}")
                continue

            fk_res = 0.0
            d_proxy = np.empty(N_SAMPLES)
            d_engine = np.empty(N_SAMPLES)
            for k, q in enumerate(qs):
                T_dh = end_effector_pose(spec, q)
                T_sim = be.fk(q)
                fk_res = max(fk_res, float(np.abs(be.dh_to_sim(T_dh) - T_sim).max()))
                d_proxy[k] = self_collision_min_distance(spec, q)
                _, d_engine[k] = be.self_collision(q)

            # proxy saturates at +inf-free values; engine saturates at threshold.
            # Compare only where neither is saturated, and compare collision VERDICTS
            # everywhere (that is what clean_pct actually uses).
            v_proxy = d_proxy < 0.0
            v_engine = d_engine < 0.0
            agree = float((v_proxy == v_engine).mean() * 100.0)
            both = np.isfinite(d_proxy) & np.isfinite(d_engine) & (d_engine < 0.49)
            gap = float(np.abs(d_proxy[both] - d_engine[both]).max()) if both.any() else float("nan")

            print(f"[{robot:>11}] {engine:<9} fk_res={fk_res:.2e}  "
                  f"verdict_agree={agree:6.2f}%  max|d_proxy-d_engine|={gap:.2e}  "
                  f"collide%: proxy={v_proxy.mean()*100:5.1f} engine={v_engine.mean()*100:5.1f}")
            be.close()


if __name__ == "__main__":
    for gt in ("capsule", "cylinder"):
        try:
            probe(gt)
        except Exception:  # noqa: BLE001
            traceback.print_exc()
