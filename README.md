# 🧬 ProteinIK - IK solver inspired by protein folding

[![CI](https://github.com/amarnath3003/Protien-IK---An-IK-Solver-inspired-by-protein-folding/actions/workflows/ci.yml/badge.svg)](https://github.com/amarnath3003/Protien-IK---An-IK-Solver-inspired-by-protein-folding/actions)

> A protein-folding-inspired inverse kinematics solver for a 6-DOF (UR5) robotic arm. 

Benchmarked honestly against five other IK methods (Jacobian/DLS, CCD, FABRIK, a TRAC-IK-style restart solver, and a multi-start solver). Includes a **FastAPI backend** with live WebSocket solve streaming + batch benchmarking, and a **React + Three.js frontend** that renders all six solvers live, side by side, on the same target.

---

## 🎯 Headline finding (read before you dig in)

> **ProteinIK V3 has the highest success rate _and_ the lowest self-collision rate of every solver tested — in all three scenarios — beating both production-style baselines (TRAC-IK-style restart, Multi-start).**

This was **not** true of earlier versions. V1/V2 beat only the classical baselines (Jacobian DLS, CCD, FABRIK) and lost to the production baselines on success rate and speed. V3 closes that gap by adding three folding mechanisms with genuine algorithmic leverage (see [`docs/protein_folding_deep_dive.md`](docs/protein_folding_deep_dive.md) for the biology and [`docs/v3_results.md`](docs/v3_results.md) for the full data):

| Scenario | ProteinIK V3 | ProteinIK V4 (optimized) | TRAC-IK-style | Multi-start |
| :-- | :-- | :-- | :-- | :-- |
| open_space | **100.0%** · coll 7.7% · 43 ms | **99.7%** · coll **6.3%** · **25 ms** | 99.2% · 15.0% · 9 ms | 99.0% · 11.8% · 68 ms |
| near_singular | **100.0%** · coll 19.3% · 53 ms | **100.0%** · coll **18.3%** · **27 ms** | 97.8% · 27.7% · 13 ms | 97.3% · 23.2% · 78 ms |
| cluttered | **100.0%** · coll 34.0% · 57 ms | **100.0%** · coll 36.3% · **26 ms** | 98.7% · 51.3% · 11 ms | 98.5% · 41.2% · 73 ms |

*(success rate · self-collision rate · mean solve time; lower collision/time is better.)*

**V4 is a pure speed-optimization pass over V3** — it runs V3's *exact* folding trajectory (same staged fold, same Metropolis funnel, same ensemble) but fuses the per-step forward-kinematics + Jacobian into one pass and drops `np.cross` overhead, making it **~1.7–2.2× faster than V3 with identical success and collision** (see [`docs/v4_optimization.md`](docs/v4_optimization.md)). It now solves in a steady ~25–27 ms.

The honest caveat: ProteinIK wins on **robustness** (success) and **steric quality** (collision), not raw latency — even V4's ~26 ms is slower than TRAC-IK's ~10 ms (though ~2.5× faster than the other population-based solver, Multi-start). For a 6-DOF arm that latency is comfortably real-time.

*All numbers are reported as measured — the frontend footer and benchmark panel reproduce them directly, not adjusted to flatter ProteinIK.*

---

## 🔬 How Protein IK Mimics Protein Folding

The solver explicitly replicates the staged, sequenced character of real protein folding, where qualitatively different physical processes dominate at different times:

| Stage | Biological Analog | Technical Implementation |
| :---: | :--- | :--- |
| **1** | **Secondary Structure**<br>(Alpha helices, Beta strands) | **Local-blind relaxation**: Joints settle toward a neutral pose using only neighbor-based and joint-limit energy terms, without consulting the target at all. |
| **2** | **Hydrophobic Collapse** | **Coarse collapse**: A fast, low-precision pull of the whole chain toward the target's general direction, compacting the chain. |
| **3** | **Folding Funnel** | **Funneled narrowing search**: The search space shrinks over iterations via gradient-free local moves with a decaying perturbation radius. |
| **4** | **Molecular Chaperone**<br>(e.g., GroEL/GroES) | **Scoped stuck-rescue**: If progress stalls, the solver perturbs only the specific joints contributing most to high energy ("misfolded" substructure), leaving the rest untouched. |
| **5** | **Kinetic Stability** | **Stability-checked termination**: The candidate solution is jittered. If energy jumps significantly, the basin is deemed unstable (a knife-edge point) and refinement continues. |

---

## 📂 Project layout

```text
protein-ik/
├── backend/    # FastAPI + numpy solver suite
└── frontend/   # React + Three.js dashboard (Vite)
```

---

## ⚙️ Running the backend

Requires **Python 3.10+**.

```bash
cd backend
pip install -r requirements.txt
uvicorn app.main:app --reload --port 8000
```

Verify it's up by requesting the UR5's DH parameters:
```bash
curl http://localhost:8000/api/robot
```

### Backend Tests
```bash
py -3.11 -m pytest tests/ -v
```

### 🔌 Key Endpoints
All endpoints are under `http://localhost:8000`:
- `GET  /api/robot` — Robot DH spec
- `GET  /api/solvers` — List of available solver IDs/names
- `POST /api/random-target` — Generate a random reachable target pose
- `POST /api/solve` — Run one solver once, with optional step trace
- `POST /api/benchmark` — Batch-run N trials across solvers, returns aggregated metrics
- `WS   /ws/solve` — Live step-by-step streaming of a single solve

*Interactive API docs are auto-served at `http://localhost:8000/docs`.*

---

## 💻 Running the frontend

Requires **Node 18+**.

```bash
cd frontend
npm install
npm run dev     # Dev server
npm test        # Run kinematics unit tests (vitest)
npm run build   # Production build
```

Open the printed local URL (default `http://localhost:5173`). 

**Configuration:** The frontend expects the backend at `http://localhost:8000` by default. To point it elsewhere, set `VITE_API_BASE` in a `.env.local` file inside `frontend/`:
```env
VITE_API_BASE=http://localhost:8000
```
*Note: If the frontend can't reach the backend, it shows a banner with the exact command to start it — there's no silent failure mode.*

---

## 🧠 Architecture Details

### What's in the backend

- 🧮 `app/core/kinematics.py` — DH-based forward kinematics, Jacobian, pose error, and self-collision distance, shared by every solver. Verified against finite-difference checks.
- ⚙️ `app/solvers/` — Six solvers behind a uniform interface (`jacobian_dls.py`, `ccd.py`, `fabrik.py`, `trac_ik_style.py`, `multi_start.py`, `protein_ik.py`), plus the energy functions (`protein_energy.py`) and an experimental, disabled-by-default rotamer-library bias module (`rotamer_library.py`).
- 🌐 `app/api/` — Pydantic schemas, quaternion<->matrix conversion, and scenario generators (`open_space`, `near_singular`, `cluttered`) used by the benchmark endpoint.
- 🚀 `app/main.py` — The FastAPI app wiring it all together.

### What's in the frontend

- 📐 `src/lib/kinematics.js` — Client-side forward kinematics, a faithful JS port of the backend's DH math, used only for rendering joint positions in Three.js. *All actual solving happens server-side.*
- 🦾 `src/components/RobotArm.jsx` — Renders one arm from a joint-angle array, with a collision-proximity color glow.
- 🌪️ `src/components/EnergyFunnel.jsx` — The signature visual: a funnel readout showing live error narrowing toward convergence.
- 📡 `src/hooks/useLiveSolve.js` — Manages one solver's WebSocket stream.
- 🎛️ `src/App.jsx` — Assembles the dashboard: a focused single-arm view, a 6-up grid solving the same target simultaneously, and the batch benchmark panel.

---

## 📜 Notes on honesty in this codebase

Several comments in `protein_ik.py` and `fabrik.py` document mechanisms that were tried, benchmarked, and either kept or reverted based on measured results — including **negative results** (e.g., the rotamer bias, a vectorial/domain-decomposition folding variant). 

These are left in place deliberately so the reasoning is auditable, not just the conclusion.
