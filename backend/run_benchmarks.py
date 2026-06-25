from app.main import app
from fastapi.testclient import TestClient

client = TestClient(app)

solvers_res = client.get("/api/solvers")
solvers = [s["id"] for s in solvers_res.json()]

scenarios = ["open_space", "near_singular", "cluttered"]

print("# Protein IK Upgrade Benchmark Results (All Solvers)\n")

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
        print("| Solver | Success Rate | Mean Time (ms) | Mean Restarts | Collision Rate |")
        print("|---|---|---|---|---|")
        for solver_name, solver_res in data['results'].items():
            sr = solver_res['success_rate'] * 100
            mt = solver_res['mean_time_ms']
            restarts = solver_res['mean_restarts']
            cr = solver_res['collision_rate'] * 100
            print(f"| {solver_res['display_name']} | {sr:.1f}% | {mt:.1f} | {restarts:.1f} | {cr:.1f}% |")
    else:
        print("Error:", res.text)
    print("\n")
