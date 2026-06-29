"""
Smoke benchmark for the Planar 3-DOF arm.
Run: python smoke_bench_3dof.py
"""
import sys, requests, json
sys.stdout.reconfigure(encoding="utf-8")


BASE = "http://localhost:8000"

# 1. Check robots endpoint
robots = requests.get(f"{BASE}/api/robots").json()
print("Robots:", json.dumps(robots, indent=2))

# 2. Check planar3dof spec
spec = requests.get(f"{BASE}/api/robot?robot=planar3dof").json()
print("\nPlanar3DOF spec:", json.dumps(spec, indent=2))

# 3. Smoke solve — one shot
target_resp = requests.post(f"{BASE}/api/random-target", json={"seed": 1, "robot": "planar3dof"}).json()
print("\nRandom target:", json.dumps(target_resp, omit="q_reference" and {} or target_resp, indent=2)
      if False else json.dumps({k: v for k, v in target_resp.items() if k != "q_reference"}, indent=2))

SOLVERS = [
    "jacobian_dls", "ccd", "fabrik", "trac_ik_style", "multi_start",
    "protein_ik", "protein_fast", "fixed_lambda_ik", "protein_homotopy",
    "analytical_planar3dof",
]

# 4. Smoke benchmark: 20 trials on planar3dof, open_space
print("\nRunning 3-DOF smoke benchmark (20 trials, open_space) ...")
bench = requests.post(f"{BASE}/api/benchmark", json={
    "solvers": SOLVERS,
    "robot": "planar3dof",
    "n_trials": 20,
    "seed": 42,
    "scenario": "open_space",
}, timeout=180).json()

print("\n=== 3-DOF Smoke Benchmark — open_space, 20 trials ===")
hdr = f"{'Solver':<28} {'Success%':>9} {'Mean ms':>9} {'Mean iters':>11}"
print(hdr)
print("-" * len(hdr))
for sid, res in bench["results"].items():
    print(
        f"{res['display_name']:<28} "
        f"{res['success_rate'] * 100:>8.1f}% "
        f"{res['mean_time_ms']:>9.1f} "
        f"{res['mean_iters']:>11.1f}"
    )

# 5. UR5 sanity check (same 20 trials)
print("\nRunning UR5 sanity benchmark (20 trials, open_space) ...")
bench_ur5 = requests.post(f"{BASE}/api/benchmark", json={
    "solvers": [s for s in SOLVERS if s != "analytical_planar3dof"],
    "robot": "ur5",
    "n_trials": 20,
    "seed": 42,
    "scenario": "open_space",
}, timeout=180).json()

print("\n=== UR5 Sanity Benchmark — open_space, 20 trials ===")
print(hdr)
print("-" * len(hdr))
for sid, res in bench_ur5["results"].items():
    print(
        f"{res['display_name']:<28} "
        f"{res['success_rate'] * 100:>8.1f}% "
        f"{res['mean_time_ms']:>9.1f} "
        f"{res['mean_iters']:>11.1f}"
    )

print("\nSmoke benchmark complete!")
