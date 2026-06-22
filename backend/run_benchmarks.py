from app.main import app
from fastapi.testclient import TestClient
import json

client = TestClient(app)

solvers = [
    "protein_ik", "multi_start", "jacobian_dls"
]

scenarios = ["open_space", "near_singular", "cluttered"]

print("# Protein IK Upgrade Benchmark Results\n")

for scenario in scenarios:
    print(f"## Scenario: {scenario}")
    res = client.post("/api/benchmark", json={
        "solvers": solvers,
        "n_trials": 100,
        "scenario": scenario,
        "seed": 42
    })
    
    if res.status_code == 200:
        data = res.json()
        print("| Solver | Success Rate | Mean Time (ms) | Mean Restarts |")
        print("|---|---|---|---|")
        for solver_res in data:
            name = solver_res['solver']
            sr = solver_res['success_rate'] * 100
            mt = solver_res['mean_time_ms']
            restarts = solver_res['mean_restarts']
            print(f"| {name} | {sr:.1f}% | {mt:.1f} | {restarts:.1f} |")
    else:
        print("Error:", res.text)
    print("\n")
