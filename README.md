# ProteinIK

A protein-folding-inspired inverse kinematics solver for a 6-DOF (UR5)
robotic arm, benchmarked honestly against five other IK methods
(Jacobian/DLS, CCD, FABRIK, a TRAC-IK-style restart solver, and a
multi-start solver). Includes a FastAPI backend with live WebSocket
solve streaming + batch benchmarking, and a React + Three.js frontend
that renders all six solvers live, side by side, on the same target.

## Headline finding (read before you dig in)

ProteinIK consistently beats the simple classical baselines (Jacobian
DLS, CCD, FABRIK) on success rate. It does **not** beat the two
production-style baselines (TRAC-IK-style restart, Multi-start) on
success rate or speed, in any tested scenario (open space, near-singular
targets, or cluttered/self-collision-prone targets). It does show a
modest, consistent edge in self-collision avoidance in easier scenarios.
This is reported as measured — the frontend's footer states this plainly
rather than spinning it.

## Project layout

```
protein-ik/
  backend/    FastAPI + numpy solver suite
  frontend/   React + Three.js dashboard (Vite)
```

## Running the backend

Requires Python 3.10+.

```bash
cd backend
pip install -r requirements.txt
uvicorn app.main:app --reload --port 8000
```

Verify it's up: `curl http://localhost:8000/api/robot` should return the
UR5's DH parameters as JSON.

Key endpoints (all under `http://localhost:8000`):
- `GET  /api/robot` — robot DH spec
- `GET  /api/solvers` — list of available solver ids/names
- `POST /api/random-target` — generate a random reachable target pose
- `POST /api/solve` — run one solver once, with optional step trace
- `POST /api/benchmark` — batch-run N trials across solvers, returns aggregated metrics
- `WS   /ws/solve` — live step-by-step streaming of a single solve

Interactive API docs are auto-served at `http://localhost:8000/docs`.

## Running the frontend

Requires Node 18+.

```bash
cd frontend
npm install
npm run dev
```

Open the printed local URL (default `http://localhost:5173`). The
frontend expects the backend at `http://localhost:8000` by default; to
point it elsewhere, set `VITE_API_BASE` (e.g. in a `.env.local` file in
`frontend/`):

```
VITE_API_BASE=http://localhost:8000
```

If the frontend can't reach the backend, it shows a banner with the
exact command to start it — there's no silent failure mode.

## What's in the backend

- `app/core/kinematics.py` — DH-based forward kinematics, Jacobian,
  pose error, and self-collision distance, shared by every solver.
  Verified against finite-difference checks.
- `app/solvers/` — six solvers behind a uniform interface
  (`jacobian_dls.py`, `ccd.py`, `fabrik.py`, `trac_ik_style.py`,
  `multi_start.py`, `protein_ik.py`), plus the energy functions
  (`protein_energy.py`) and an experimental, disabled-by-default
  rotamer-library bias module (`rotamer_library.py`) — see its
  docstring and the comments in `protein_ik.py` for why it's off by
  default (tested, traded success rate for collision-avoidance with no
  net win).
- `app/api/` — Pydantic schemas, quaternion<->matrix conversion, and
  scenario generators (`open_space`, `near_singular`, `cluttered`) used
  by the benchmark endpoint.
- `app/main.py` — the FastAPI app wiring it all together.

## What's in the frontend

- `src/lib/kinematics.js` — client-side forward kinematics, a faithful
  JS port of the backend's DH math (verified to match numerically),
  used only for rendering joint positions in three.js. All actual
  solving happens server-side.
- `src/components/RobotArm.jsx` — renders one arm from a joint-angle
  array, with a collision-proximity color glow.
- `src/components/EnergyFunnel.jsx` — the signature visual: a funnel
  readout showing live error narrowing toward convergence.
- `src/hooks/useLiveSolve.js` — manages one solver's WebSocket stream.
- `src/App.jsx` — assembles the dashboard: a focused single-arm view, a
  6-up grid solving the same target simultaneously, and the batch
  benchmark panel.

## Notes on honesty in this codebase

Several comments in `protein_ik.py` and `fabrik.py` document mechanisms
that were tried, benchmarked, and either kept or reverted based on
measured results — including negative results (e.g. the rotamer bias,
a vectorial/domain-decomposition folding variant). These are left in
place deliberately so the reasoning is auditable, not just the
conclusion.
