from app.main import app
from fastapi.testclient import TestClient
import json

client = TestClient(app)

res = client.post("/api/benchmark", json={
    "solvers": ["protein_ik"],
    "n_trials": 1,
    "scenario": "open_space",
    "seed": 42
})
print(res.json())
