import asyncio
from app.main import app
from fastapi.testclient import TestClient

client = TestClient(app)

def test_api():
    res = client.get("/api/robot")
    print("robot:", res.status_code)
    
    res = client.get("/api/solvers")
    print("solvers:", res.json())
    solvers = [s["id"] for s in res.json()]
    
    target = client.post("/api/random-target", json={"seed": 42}).json()
    
    for s in solvers:
        print(f"testing solver: {s}")
        solve_res = client.post("/api/solve", json={
            "solver": s,
            "seed": 42,
            "target": target,
            "q0": [0, 0, 0, 0, 0, 0]
        })
        print(s, solve_res.status_code)
        if solve_res.status_code != 200:
            print("ERROR", solve_res.text)
        
    print("testing benchmark")
    bench_res = client.post("/api/benchmark", json={
        "solvers": solvers,
        "n_trials": 2,
        "scenario": "open_space",
        "seed": 42
    })
    print("benchmark:", bench_res.status_code)
    if bench_res.status_code != 200:
        print("ERROR", bench_res.text)

if __name__ == "__main__":
    test_api()
