# ProteinIK — IK Solver Inspired by Protein Folding

[![CI](https://github.com/amarnath3003/Protien-IK---An-IK-Solver-inspired-by-protein-folding/actions/workflows/ci.yml/badge.svg)](https://github.com/amarnath3003/Protien-IK---An-IK-Solver-inspired-by-protein-folding/actions)

> A protein-folding-inspired inverse kinematics solver that has grown from a single staged-fold algorithm into a multi-version research platform supporting three robot configurations, ten solvers, and a live benchmarking dashboard.

---

## Versions at a Glance

| Version | Solver ID | Name | Status | Core Idea |
| :---: | :--- | :--- | :---: | :--- |
| **V1** | `protein_ik` | ProteinIK | Live | 5-stage biology-mimicking fold (blind relax → collapse → funnel → chaperone → stability gate) |
| **V4** | `protein_fast` | ProteinIK Fast | Live | V1's exact algorithm, fused FK+Jacobian kernel — same results, fewer allocations, faster per step |
| **V5** | `protein_homotopy` | CCH-IK | Live | Conflict-Controlled Homotopy: λ advances based on cosine conflict between task and constraint gradients |
| **V6** | `protein_raw` | ProteinIK Raw Biology | Pending | Physics-truer simulation of real folding kinetics; implementation not yet started |

All three live versions run simultaneously on the same target in the frontend dashboard so you can compare them head-to-head in real time.

---

## What This Project Is

An honest, research-oriented comparison platform. The core claim is that biology-inspired sequencing — specifically the staged structure of protein folding — can be a useful design principle for constrained iterative IK, not just a metaphor. Each version of ProteinIK tests a different interpretation of that principle. The baselines (Jacobian DLS, CCD, FABRIK, TRAC-IK-style, Multi-start) are included with the same interface so no result is cherry-picked against a weak field.

### Headline finding

**ProteinIK V1 consistently beats simple classical baselines** (Jacobian DLS, CCD, FABRIK) on success rate. It does **not** beat the two production-style baselines (TRAC-IK-style, Multi-start) on success rate or raw speed in any tested scenario. It does show a modest, consistent edge in self-collision avoidance. CCH-IK (V5) extends this with a theoretically grounded homotopy path and a novel conflict-index diagnostic.

*The frontend's footer states this plainly rather than spinning it.*

---

## Robots Supported

| Robot | DOF | Notes |
| :--- | :---: | :--- |
| **Planar 3-DOF** | 3 | 3-link planar RRR arm. Includes a gold-standard closed-form analytical solver for ground-truth comparison. |
| **UR5** | 6 | Universal Robots UR5. The primary benchmark arm; all five stages of the biology analogy were developed and tuned here. |
| **Franka Panda** | 7 | Redundant 7-DOF arm. Null-space degrees of freedom allow ProteinIK's energy terms to act beyond target-reaching, enabling active self-collision minimization and joint-limit avoidance in the redundant direction. |

All three robots are selectable in the frontend via the robot picker. Solvers that are robot-specific (e.g., the analytical solver is only valid on Planar 3-DOF) are filtered automatically.

---

## All Solvers

### Classical Baselines
| Solver | ID | Role |
| :--- | :--- | :--- |
| Jacobian DLS | `jacobian_dls` | Damped-least-squares Jacobian pseudoinverse |
| CCD | `ccd` | Cyclic Coordinate Descent |
| FABRIK | `fabrik` | Forward-And-Backward Reaching IK |
| TRAC-IK style | `trac_ik_style` | Global random restart with DLS |
| Multi-start | `multi_start` | Multiple independent random seeds |
| Analytical (Planar 3-DOF) | `analytical_planar3dof` | Exact closed-form — ground truth for planar arm |

### ProteinIK Family
| Solver | ID | Role |
| :--- | :--- | :--- |
| ProteinIK V1 | `protein_ik` | Original staged-fold algorithm |
| ProteinIK Fast (V4) | `protein_fast` | Speed-optimized V1 — bit-identical results |
| CCH-IK (V5) | `protein_homotopy` | Conflict-Controlled Homotopy |
| Fixed-λ Baseline | `fixed_lambda_ik` | V5 ablation baseline — same energy as CCH-IK but no conflict detection |

---

## Version Details

### V1 — ProteinIK

The original contribution. Rather than minimizing one fixed energy function from iteration 1 (as every classical energy-based IK method does), V1 replicates the staged, sequenced character of real protein folding:

| Stage | Biological Analog | What It Does |
| :---: | :--- | :--- |
| 1 | Secondary structure (helices, strands) | Local-blind relaxation: joints settle using only neighbor and joint-limit energy, no target consulted at all |
| 2 | Hydrophobic collapse | Coarse, low-precision pull of the chain toward the target's general direction |
| 3 | Folding funnel | Main refinement: target attraction + collision + smoothness, perturbation radius decays over iterations |
| 4 | Molecular chaperone (GroEL/GroES) | Scoped stuck-rescue: perturbs only the joints contributing most to high energy, leaves the rest untouched |
| 5 | Kinetic native-state stability | Stability-gated termination: jitters the candidate and rejects it if energy jumps (knife-edge point check) |

The scoped rescue in Stage 4 is the key design difference from TRAC-IK: rather than a global random restart, only the "misfolded" substructure is perturbed.

---

### V4 — ProteinIK Fast

A pure speed pass over V1 (which itself inherited from V3). **No algorithmic changes.** The same staged fold runs, same success rate, same collision behavior, same iteration count — only the per-step arithmetic is faster.

Profiling showed time was dominated by two things:
1. `geometric_jacobian()` calling `np.cross`, which carries per-call normalization overhead on every joint.
2. Every gradient/LM step computing forward kinematics twice: once for the pose, once inside the Jacobian.

V4 replaces both with a single fused `_fast_pose_jac()` primitive that traverses the FK chain once and derives both the end-effector pose and the full Jacobian with explicit cross products. The output is bit-identical to the reference implementation — verified to zero difference. The result is a genuine speed improvement with no behavioral trade-off.

---

### V5 — CCH-IK (Conflict-Controlled Homotopy IK)

A theoretically distinct approach. Instead of the staged folding metaphor, V5 is grounded in homotopy path-following for constrained optimization.

**Core idea:** Define a combined energy `E(q, λ) = (1-λ)·E_task + λ·E_constraints` and follow the solution path as λ sweeps from 0 (task only) to 1 (full constraints). The key innovation is that λ does not advance on a fixed schedule — it advances only when the gradient conflict between task and constraint components is low (cosine similarity below a threshold). When gradients conflict, λ is held until the solver finds a local resolution.

**Theoretical basis:** The Implicit Function Theorem (Allgower & Georg 1990) guarantees a locally smooth solution path q(λ) exists when the Hessian is non-singular. The path breaks at kinematic singularities; no global convergence claim is made.

**Biological motivation (honest):** The minimal frustration principle — proteins fold fast because their energy landscapes minimize gradient conflicts between native interactions. This is the design intuition only. All algorithmic decisions are justified by the optimization theory.

**Novel diagnostic outputs:**

| Output | Range | Meaning |
| :--- | :---: | :--- |
| `conflict_index` (C) | [-1, 1] | Full-vector cosine between task and constraint gradients at solution. C < 0: cooperative; C ≈ 0: orthogonal; C > 0: conflicted |
| `lambda_final` (λ) | [0, 1] | How far constraints were introduced. λ < 0.8 means constraints were not fully active at convergence |

The `fixed_lambda_ik` solver uses the same energy function but advances λ linearly (no conflict detection), isolating V5's contribution to the ablation.

---

### V6 — Raw Biology (Pending)

Not yet started. The intent is a physics-truer simulation of actual folding kinetics: Langevin dynamics, realistic energy landscape topology, and less engineering compromise. No timeline.

---

## Project Layout

```
protein-ik/
├── backend/               # FastAPI + NumPy solver suite
│   ├── app/
│   │   ├── core/          # DH kinematics, Jacobian, collision distance (shared)
│   │   ├── solvers/       # All solvers behind a uniform interface
│   │   │   ├── protein_ik.py               # V1
│   │   │   ├── protein_fast/               # V4 (package)
│   │   │   ├── protein_homotopy/           # V5 — CCH-IK (package)
│   │   │   ├── fixed_lambda_ik.py          # V5 ablation baseline
│   │   │   ├── jacobian_dls.py
│   │   │   ├── ccd.py
│   │   │   ├── fabrik.py
│   │   │   ├── trac_ik_style.py
│   │   │   ├── multi_start.py
│   │   │   ├── analytical_planar3dof.py
│   │   │   └── registry.py                # Uniform solver dispatch
│   │   └── api/           # Schemas, quaternion utils, scenario generators
│   └── tests/
└── frontend/              # React + Three.js dashboard (Vite)
    └── src/
        ├── components/    # RobotArm, EnergyFunnel, BenchmarkPanel, SolverCard
        ├── hooks/         # useLiveSolve (WebSocket management)
        └── lib/           # Client-side DH kinematics, solver metadata, API client
```

---

## Running the Backend

Requires **Python 3.10+**.

```bash
cd backend
pip install -r requirements.txt
uvicorn app.main:app --reload --port 8000
```

Verify it's up:
```bash
curl http://localhost:8000/api/robots
```

### Backend Tests
```bash
py -3.11 -m pytest tests/ -v
```

### API Endpoints

All under `http://localhost:8000`:

| Method | Path | Description |
| :--- | :--- | :--- |
| `GET` | `/api/robots` | List all available robot arms |
| `GET` | `/api/robot?robot=ur5` | DH spec for a specific robot |
| `GET` | `/api/solvers?robot=ur5` | Solvers valid for that robot |
| `POST` | `/api/random-target` | Generate a random reachable target pose |
| `POST` | `/api/solve` | Run one solver once (optional step trace) |
| `POST` | `/api/benchmark` | Batch-run N trials across solvers; returns aggregated metrics |
| `WS` | `/ws/solve` | Live step-by-step streaming of a single solve |

Interactive docs auto-served at `http://localhost:8000/docs`.

### Benchmark Scenarios

| Scenario | Description |
| :--- | :--- |
| `open_space` | Uniform random targets, no obstacle bias |
| `near_singular` | Targets biased toward low-manipulability configurations |
| `cluttered` | Targets biased toward tight, near-self-collision configurations |

---

## Running the Frontend

Requires **Node 18+**.

```bash
cd frontend
npm install
npm run dev     # Dev server at http://localhost:5173
npm test        # Kinematics unit tests (Vitest)
npm run build   # Production build
```

The frontend expects the backend at `http://localhost:8000`. To point it elsewhere:

```env
# frontend/.env.local
VITE_API_BASE=http://localhost:8000
```

If the frontend can't reach the backend it shows a banner with the exact command to start it — no silent failure.

---

## Frontend Features

- **Robot selector** — switch between Planar 3-DOF, UR5, and Franka Panda live; solver list updates automatically
- **Live solve grid** — all active solvers run simultaneously on the same target, rendered side by side in Three.js
- **Focused solver view** — click any solver card to expand it with a full energy funnel readout and phase trace
- **Energy funnel visual** — the signature diagnostic: live error narrowing toward convergence with per-phase annotations (including CCH-IK's λ-advancement and conflict-hold phases)
- **Batch benchmark panel** — run N trials (default 60) per scenario and compare success rate, mean/p50/p95 solve time, collision rate, and convergence across all solvers in a table
- **Collision proximity glow** — robot links color-shift based on self-collision clearance

---

## Architecture Notes

**Backend concurrency:** The solver code is synchronous NumPy. All solve and benchmark calls run in a thread-pool executor (`ThreadPoolExecutor`, sized to CPU count), keeping the asyncio event loop free for concurrent WebSocket connections. Benchmarks with many trials never block live solve streams.

**Shared kinematics core:** `app/core/kinematics.py` provides DH-based forward kinematics, the geometric Jacobian, pose error, and self-collision distance. Every solver uses the same primitives. The Jacobian is verified against finite-difference checks; the V4 fused version is verified to be bit-identical to the reference.

**Uniform solver interface:** Every solver is registered as `(spec, q0, T_target, rng, collect_steps) -> SolveResult`. The API and benchmark runner have no solver-specific branching.

**Client-side kinematics:** `src/lib/kinematics.js` is a faithful JS port of the DH math, used only for rendering joint positions in Three.js. All actual solving is server-side.

---

## Honesty in This Codebase

Several comments in `protein_ik.py`, `protein_homotopy/solver.py`, and `fabrik.py` document mechanisms that were tried, benchmarked, and either kept or reverted based on measured results — including negative results (rotamer bias, vectorial/domain-decomposition folding variants, fixed-λ schedule). These are left in place deliberately so the reasoning is auditable, not just the conclusion.

V5's solver docstring explicitly distinguishes what is claimed (conflict-controlled λ advancement as an algorithmic contribution), what is theoretical grounding (IFT), and what is design intuition only (biological motivation).
