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
import concurrent.futures
import os

import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware

from app.core.kinematics import ur5_spec, end_effector_pose, get_robot_spec, ROBOT_REGISTRY
from app.api.schemas import (
    SolveRequest, RandomTargetRequest, BatchBenchmarkRequest, RobotSpecResponse, TargetPose,
)
from app.api.quaternion import pose_to_transform, transform_to_pose
from app.solvers.registry import run_solver, SOLVER_REGISTRY, SOLVER_DISPLAY_NAMES, get_solvers_for_robot
from app.api.scenarios import generate_target

app = FastAPI(title="ProteinIK API")

# CORS: restrict origins in production by setting the ALLOWED_ORIGINS environment
# variable (comma-separated). Falls back to "*" for local development convenience.
_raw_origins = os.environ.get("ALLOWED_ORIGINS", "")
_allowed_origins = [o.strip() for o in _raw_origins.split(",") if o.strip()] or ["*"]

app.add_middleware(
    CORSMiddleware,
    allow_origins=_allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pre-build all specs so the first request isn't slower (spec creation is cheap
# but avoids any edge case of lazy-init on a hot path).
_SPEC_CACHE: dict[str, object] = {name: fn() for name, fn in ROBOT_REGISTRY.items()}


def _get_spec(robot: str):
    """Return the cached RobotSpec for the requested robot name."""
    if robot not in _SPEC_CACHE:
        raise HTTPException(400, f"Unknown robot '{robot}'. Available: {list(_SPEC_CACHE)}")
    return _SPEC_CACHE[robot]

# Thread pool for running blocking numpy solver code off the asyncio event loop.
# Without this, a long solve / full benchmark blocks all WebSocket streams and
# any concurrent requests for its entire duration.
_executor = concurrent.futures.ThreadPoolExecutor(
    max_workers=max(4, (os.cpu_count() or 4)),
    thread_name_prefix="ik-solver",
)


@app.get("/api/robots")
def get_robots():
    """List all available robot arms."""
    return [{"id": name, "n_joints": spec.n_joints}
            for name, spec in _SPEC_CACHE.items()]


@app.get("/api/robot", response_model=RobotSpecResponse)
def get_robot(robot: str = "ur5"):
    spec = _get_spec(robot)
    return RobotSpecResponse(
        name=spec.name,
        n_joints=spec.n_joints,
        a=spec.a.tolist(),
        d=spec.d.tolist(),
        alpha=spec.alpha.tolist(),
        joint_limits=spec.joint_limits.tolist(),
        link_radius=spec.link_radius.tolist(),
    )


@app.get("/api/solvers")
def get_solvers(robot: str = "ur5"):
    valid = get_solvers_for_robot(robot)
    return [{"id": k, "name": SOLVER_DISPLAY_NAMES.get(k, k)}
            for k in SOLVER_REGISTRY if k in valid]


@app.post("/api/random-target")
def random_target(req: RandomTargetRequest):
    spec = _get_spec(req.robot)
    rng = np.random.default_rng(req.seed)
    q_true = rng.uniform(-np.pi, np.pi, spec.n_joints)
    T = end_effector_pose(spec, q_true)
    pos, quat = transform_to_pose(T)
    return {"position": pos, "quaternion": quat, "q_reference": q_true.tolist()}


def _run_solve_sync(req: SolveRequest) -> dict:
    """Pure synchronous solve — safe to call in a thread pool executor."""
    spec = _get_spec(req.robot)
    rng = np.random.default_rng(req.seed)
    q0 = np.array(req.q0) if req.q0 is not None else spec.random_config(rng)
    T_target = pose_to_transform(req.target.position, req.target.quaternion)
    try:
        result = run_solver(req.solver, spec, q0, T_target, rng, collect_steps=req.collect_steps)
    except ValueError as exc:
        raise HTTPException(400, str(exc)) from exc
    return result.to_dict(include_steps=req.collect_steps)


@app.post("/api/solve")
async def solve(req: SolveRequest):
    if req.solver not in SOLVER_REGISTRY:
        raise HTTPException(400, f"Unknown solver '{req.solver}'. Available: {list(SOLVER_REGISTRY)}")
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(_executor, _run_solve_sync, req)


def _run_benchmark_sync(req: BatchBenchmarkRequest) -> dict:
    """Pure synchronous benchmark — safe to call in a thread pool executor."""
    spec = _get_spec(req.robot)
    target_rng = np.random.default_rng(req.seed)
    results_by_solver = {s: {"successes": 0, "times_ms": [], "iters": [],
                              "pos_errors": [], "orient_errors": [], "restarts": [],
                              "min_self_distances": []}
                          for s in req.solvers}

    for trial in range(req.n_trials):
        q0, T_target = generate_target(spec, target_rng, req.scenario)
        for s in req.solvers:
            solver_rng = np.random.default_rng(hash((req.seed, trial, s)) % (2**31))
            r = run_solver(s, spec, q0.copy(), T_target, solver_rng, collect_steps=False)
            bucket = results_by_solver[s]
            bucket["successes"] += int(r.success)
            bucket["times_ms"].append(r.wall_time_ms)
            bucket["iters"].append(r.iterations)
            bucket["pos_errors"].append(r.pos_error)
            bucket["orient_errors"].append(r.orient_error)
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
            "mean_orient_error": float(np.mean(bucket["orient_errors"])),
            "mean_restarts": float(np.mean(bucket["restarts"])),
            "mean_min_self_distance": float(np.mean(bucket["min_self_distances"])),
            "collision_rate": float(np.mean([d < 0 for d in bucket["min_self_distances"]])),
            "n_trials": n,
        }
    return {"scenario": req.scenario, "n_trials": req.n_trials, "results": summary}


@app.post("/api/benchmark")
async def benchmark(req: BatchBenchmarkRequest):
    for s in req.solvers:
        if s not in SOLVER_REGISTRY:
            raise HTTPException(400, f"Unknown solver '{s}'. Available: {list(SOLVER_REGISTRY)}")
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(_executor, _run_benchmark_sync, req)


@app.websocket("/ws/solve")
async def ws_solve(websocket: WebSocket):
    """Streams a single solve step-by-step as it runs. Since our solvers
    are synchronous numpy code (not natively async/generator-based), we
    run the full solve in a thread-pool executor (keeping the event loop
    responsive for all other active WebSocket connections), then stream
    the recorded step trace back at a fixed cadence -- giving the frontend
    a live 'replay' feel.
    """
    await websocket.accept()
    loop = asyncio.get_running_loop()
    try:
        while True:
            msg = await websocket.receive_json()
            solver_name = msg["solver"]
            if solver_name not in SOLVER_REGISTRY:
                await websocket.send_json({"type": "error", "message": f"Unknown solver '{solver_name}'"})
                continue

            seed = msg.get("seed")
            robot_name = msg.get("robot", "ur5")
            if robot_name not in _SPEC_CACHE:
                await websocket.send_json({"type": "error", "message": f"Unknown robot '{robot_name}'"})
                continue
            ws_spec = _SPEC_CACHE[robot_name]
            rng = np.random.default_rng(seed)
            q0 = np.array(msg["q0"]) if msg.get("q0") is not None else ws_spec.random_config(rng)
            T_target = pose_to_transform(msg["target"]["position"], msg["target"]["quaternion"])

            # Run the blocking solve in the thread pool so other WS connections
            # and API requests are not blocked during the solve.
            result = await loop.run_in_executor(
                _executor,
                lambda: run_solver(solver_name, ws_spec, q0, T_target, rng, collect_steps=True),
            )

            await websocket.send_json({"type": "start", "solver": solver_name, "total_steps": len(result.steps)})

            step_delay = float(msg.get("step_delay_ms", 40)) / 1000.0
            for step in result.steps:
                await websocket.send_json({"type": "step", "data": step.to_dict()})
                if step_delay > 0:
                    await asyncio.sleep(step_delay)

            await websocket.send_json({"type": "done", "data": result.to_dict(include_steps=False)})
    except WebSocketDisconnect:
        pass
