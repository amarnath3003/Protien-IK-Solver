"""
ProteinIK backend API.

Endpoints:
  GET  /api/robot                  -- robot (UR5) DH spec, for frontend rendering
  GET  /api/solvers                -- list of available solvers + display names
  POST /api/solve                  -- run one solver once, return full result (+ optional step trace)
  POST /api/random-target          -- generate a random reachable target pose
  POST /api/benchmark               -- batch run: N trials x M solvers, return aggregated metrics
  WS   /ws/solve                   -- live step-by-step streaming of a single solve

Run with: uvicorn app.main:app --reload --port 8000
"""

from __future__ import annotations

import asyncio
import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware

from app.core.kinematics import ur5_spec, end_effector_pose
from app.api.schemas import (
    SolveRequest, RandomTargetRequest, BatchBenchmarkRequest, RobotSpecResponse, TargetPose,
)
from app.api.quaternion import pose_to_transform, transform_to_pose
from app.solvers.registry import run_solver, SOLVER_REGISTRY, SOLVER_DISPLAY_NAMES
from app.api.scenarios import generate_target

app = FastAPI(title="ProteinIK API")

# app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # dev-friendly; tighten before any real deployment
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

SPEC = ur5_spec()


@app.get("/api/robot", response_model=RobotSpecResponse)
def get_robot():
    return RobotSpecResponse(
        name=SPEC.name,
        n_joints=SPEC.n_joints,
        a=SPEC.a.tolist(),
        d=SPEC.d.tolist(),
        alpha=SPEC.alpha.tolist(),
        joint_limits=SPEC.joint_limits.tolist(),
        link_radius=SPEC.link_radius.tolist(),
    )


@app.get("/api/solvers")
def get_solvers():
    return [{"id": k, "name": SOLVER_DISPLAY_NAMES.get(k, k)} for k in SOLVER_REGISTRY]


@app.post("/api/random-target")
def random_target(req: RandomTargetRequest):
    rng = np.random.default_rng(req.seed)
    q_true = rng.uniform(-np.pi, np.pi, SPEC.n_joints)
    T = end_effector_pose(SPEC, q_true)
    pos, quat = transform_to_pose(T)
    return {"position": pos, "quaternion": quat, "q_reference": q_true.tolist()}


@app.post("/api/solve")
def solve(req: SolveRequest):
    if req.solver not in SOLVER_REGISTRY:
        raise HTTPException(400, f"Unknown solver '{req.solver}'. Available: {list(SOLVER_REGISTRY)}")

    rng = np.random.default_rng(req.seed)
    q0 = np.array(req.q0) if req.q0 is not None else SPEC.random_config(rng)
    T_target = pose_to_transform(req.target.position, req.target.quaternion)

    result = run_solver(req.solver, SPEC, q0, T_target, rng, collect_steps=req.collect_steps)
    return result.to_dict(include_steps=req.collect_steps)


@app.post("/api/benchmark")
def benchmark(req: BatchBenchmarkRequest):
    for s in req.solvers:
        if s not in SOLVER_REGISTRY:
            raise HTTPException(400, f"Unknown solver '{s}'. Available: {list(SOLVER_REGISTRY)}")

    target_rng = np.random.default_rng(req.seed)
    results_by_solver = {s: {"successes": 0, "times_ms": [], "iters": [],
                              "pos_errors": [], "restarts": [], "min_self_distances": []}
                          for s in req.solvers}

    for trial in range(req.n_trials):
        q0, T_target = generate_target(SPEC, target_rng, req.scenario)
        for s in req.solvers:
            solver_rng = np.random.default_rng(hash((req.seed, trial, s)) % (2**31))
            r = run_solver(s, SPEC, q0.copy(), T_target, solver_rng, collect_steps=False)
            bucket = results_by_solver[s]
            bucket["successes"] += int(r.success)
            bucket["times_ms"].append(r.wall_time_ms)
            bucket["iters"].append(r.iterations)
            bucket["pos_errors"].append(r.pos_error)
            bucket["restarts"].append(r.restarts)
            bucket["min_self_distances"].append(r.min_self_distance)

    summary = {}
    for s, bucket in results_by_solver.items():
        n = req.n_trials
        summary[s] = {
            "display_name": SOLVER_DISPLAY_NAMES.get(s, s),
            "success_rate": bucket["successes"] / n,
            "mean_time_ms": float(np.mean(bucket["times_ms"])),
            "p50_time_ms": float(np.percentile(bucket["times_ms"], 50)),
            "p95_time_ms": float(np.percentile(bucket["times_ms"], 95)),
            "mean_iters": float(np.mean(bucket["iters"])),
            "mean_pos_error": float(np.mean(bucket["pos_errors"])),
            "mean_restarts": float(np.mean(bucket["restarts"])),
            "mean_min_self_distance": float(np.mean(bucket["min_self_distances"])),
            "collision_rate": float(np.mean([d < 0 for d in bucket["min_self_distances"]])),
            "n_trials": n,
        }
    return {"scenario": req.scenario, "n_trials": req.n_trials, "results": summary}


@app.websocket("/ws/solve")
async def ws_solve(websocket: WebSocket):
    """Streams a single solve step-by-step as it runs. Since our solvers
    are synchronous numpy code (not natively async/generator-based), we
    run the full solve first (fast, <1s) and then stream the recorded
    step trace back at a fixed cadence -- giving the frontend a live
    'replay' feel without needing to restructure every solver into a
    generator/coroutine.
    """
    await websocket.accept()
    try:
        while True:
            msg = await websocket.receive_json()
            solver_name = msg["solver"]
            if solver_name not in SOLVER_REGISTRY:
                await websocket.send_json({"type": "error", "message": f"Unknown solver '{solver_name}'"})
                continue

            seed = msg.get("seed")
            rng = np.random.default_rng(seed)
            q0 = np.array(msg["q0"]) if msg.get("q0") is not None else SPEC.random_config(rng)
            T_target = pose_to_transform(msg["target"]["position"], msg["target"]["quaternion"])

            result = run_solver(solver_name, SPEC, q0, T_target, rng, collect_steps=True)

            await websocket.send_json({"type": "start", "solver": solver_name, "total_steps": len(result.steps)})

            step_delay = float(msg.get("step_delay_ms", 40)) / 1000.0
            for step in result.steps:
                await websocket.send_json({"type": "step", "data": step.to_dict()})
                if step_delay > 0:
                    await asyncio.sleep(step_delay)

            await websocket.send_json({"type": "done", "data": result.to_dict(include_steps=False)})
    except WebSocketDisconnect:
        pass
