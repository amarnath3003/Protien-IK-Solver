"""
Raw (V6) Phase 5 tests — the assembled Langevin folding solver.

Run from backend/:  pytest tests/test_raw_solver.py -v

Raw is the slow quality solver, so these are kept lean (a handful of solves).
Covers: valid SolveResult + Raw diagnostics, a step trace with folding phases,
reaching easy targets, registry/API wiring.
"""

from __future__ import annotations

import numpy as np
import pytest
from fastapi.testclient import TestClient

from app.main import app
from app.core.kinematics import ur5_spec, end_effector_pose
from app.solvers.protein_raw import solve_protein_raw

client = TestClient(app)


def test_raw_returns_valid_result_with_diagnostics():
    spec = ur5_spec()
    rng = np.random.default_rng(0)
    T = end_effector_pose(spec, spec.random_config(rng))
    r = solve_protein_raw(spec, spec.random_config(rng), T, rng,
                          max_iters=30, collect_steps=True)
    assert r.solver_name == "protein_raw"
    assert len(r.q_final) == spec.n_joints
    assert r.t_glass > 0.0 and r.sigma_ratio > 0.0
    assert np.isfinite(r.free_energy)
    # the step trace records the folding phases
    phases = {s.phase for s in r.steps}
    assert any(p.startswith("raw_") for p in phases)


def test_raw_reaches_easy_targets():
    spec = ur5_spec()
    rng = np.random.default_rng(2)
    ok = 0
    for _ in range(5):
        T = end_effector_pose(spec, spec.random_config(rng))
        r = solve_protein_raw(spec, spec.random_config(rng), T, rng, max_iters=30)
        ok += int(r.success)
    assert ok >= 3, f"only solved {ok}/5 easy targets"


def test_raw_registered_and_served():
    ids = [s["id"] for s in client.get("/api/solvers?robot=ur5").json()]
    assert "protein_raw" in ids


def test_raw_via_api():
    target = client.post("/api/random-target", json={"seed": 7}).json()
    res = client.post("/api/solve", json={
        "solver": "protein_raw", "seed": 7, "target": target, "collect_steps": False,
    })
    assert res.status_code == 200, res.text
    data = res.json()
    assert "sigma_ratio" in data and "t_glass" in data and "free_energy" in data
