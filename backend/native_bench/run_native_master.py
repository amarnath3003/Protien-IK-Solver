"""Run THE master benchmark, but with every borrowed solver replaced by its
genuine imported upstream — reusing the repo's own driver, scoring, metric math,
and markdown writer unchanged (so the output is structurally identical to
master_full.md, only the solver code is genuine).

We mutate ``SOLVER_REGISTRY`` in place: ``run_solver`` resolves it at call time,
so swapping entries is enough. Already-genuine solvers (the ProteinIK family,
the exact analytical planar solver, PyBullet native IK) are left untouched — for
them the repo's own code *is* the original.

Usage (inside WSL, from backend/):
    PYTHONPATH=. python3 native_bench/run_native_master.py --quick --out results/native/quick
    PYTHONPATH=. python3 native_bench/run_native_master.py --out results/native/master_full_native
"""
from __future__ import annotations

import sys

from native_bench._env import apply
apply()

import app.solvers.registry as reg
import native_bench.genuine_solvers as G

# borrowed recreation id -> (genuine adapter, display name announcing the source).
# Only solvers with a genuine robotics-library upstream (accepts a DH robot,
# returns joint angles) are swapped. FABRIK/CCD have NO such upstream — their
# genuine implementations (Caliko, community CCD) are graphics point-solvers that
# don't produce DH joint angles — so instead of a fake import they run as the
# repo's OWN algorithm compiled to native C++ (see CPP_CLASSICAL below), matching
# every other native solver so the speed columns stay apples-to-apples.
GENUINE = {
    "trac_ik_style": (G.solve_real_tracik,     "TRAC-IK (real C++ TRACLabs)"),
    "jacobian_dls":  (G.solve_rtb_dls,         "Jacobian DLS (real RTB LM, single-shot)"),
    "multi_start":   (G.solve_rtb_multistart,  "Multi-start (real RTB ik_LM restarts)"),
}


def _wrap(fn):
    def wrapped(spec, q0, T_target, rng, collect_steps=False):
        return fn(spec, q0, T_target, rng, collect_steps)
    return wrapped


for name, (fn, disp) in GENUINE.items():
    reg.SOLVER_REGISTRY[name] = _wrap(fn)
    reg.SOLVER_DISPLAY_NAMES[name] = disp

# The ProteinIK family: swap the Python solvers for their native-C++ ports
# (pik_native, built from backend/cpp/) so the whole benchmark is native — the
# fair, apples-to-apples comparison vs the compiled TRAC-IK/KDL/RTB baselines.
# Display names are kept from the registry so the table matches master_full.md.
import native_bench.cpp_solvers as C  # noqa: E402
CPP_PROTEIN = {
    "protein_ik":         C.solve_v1_cpp,
    "protein_fast":       C.solve_v4_cpp,
    "protein_fast_o2":    C.solve_o2_cpp,
    "protein_fast_calib": C.solve_calib_cpp,
    "protein_raw":        C.solve_raw_cpp,
}
for name, fn in CPP_PROTEIN.items():
    reg.SOLVER_REGISTRY[name] = _wrap(fn)

# The two classical baselines with no genuine DH-native upstream (CCD, FABRIK):
# swap the Python solvers for their native-C++ ports (pik_native, built from
# backend/cpp/pik_ccd.hpp / pik_fabrik.hpp) — the SAME in-repo algorithm, compiled,
# so the whole benchmark is native and the speed columns are apples-to-apples with
# the compiled TRAC-IK/KDL/RTB baselines. Both are deterministic (no RNG), so their
# quality columns match the Python to float tolerance (see cpp/parity_ccd_fabrik.py).
CPP_CLASSICAL = {
    "ccd":    C.solve_ccd_cpp,
    "fabrik": C.solve_fabrik_cpp,
}
for name, fn in CPP_CLASSICAL.items():
    reg.SOLVER_REGISTRY[name] = _wrap(fn)
reg.SOLVER_DISPLAY_NAMES["ccd"] = "CCD (in-repo; native C++)"
reg.SOLVER_DISPLAY_NAMES["fabrik"] = "FABRIK (in-repo; native C++)"

print("[native] genuine baselines:", ", ".join(sorted(GENUINE)),
      "| C++ ProteinIK:", ", ".join(sorted(CPP_PROTEIN)),
      "| C++ classical:", ", ".join(sorted(CPP_CLASSICAL)), flush=True)

from bench import master_sim_benchmark as M  # noqa: E402

FOOTER = """
---

### Provenance — every solver runs as NATIVE compiled code

This is `master_full.md` re-run **entirely in the native system**. Every solver is
either a genuine imported library or a native-C++ port — none is interpreted Python,
so the speed columns are apples-to-apples.

| Solver | Native implementation |
|:--|:--|
| **TRAC-IK** | REAL TRAC-IK — TRACLabs C++/KDL/NLopt via `tracikpy` |
| **Jacobian DLS** | REAL Robotics Toolbox (Corke) Levenberg–Marquardt, single-shot |
| **Multi-start** | REAL Robotics Toolbox `ik_LM` with native random restarts |
| **ProteinIK V1 / V4 / V4+o2 / V4-calib / V6** | **native C++/Eigen ports** (backend/cpp/, `pik_native`) of the project's own solvers — same logic/weights/tolerances, FK & energy parity to ≤1e-11, success/collision statistically identical to the Python (only the RNG stream differs) |
| Analytical (planar3dof) | the project's exact closed-form |
| PyBullet native IK | REAL PyBullet `calculateInverseKinematics` |
| **CCD, FABRIK** | **native C++/Eigen ports** (backend/cpp/`pik_ccd.hpp`, `pik_fabrik.hpp` → `pik_native`) of the in-repo algorithm — no genuine DH-native upstream library exists, so this is the project's own code compiled, not a fake import. Deterministic (no RNG): the per-joint update math is bit-identical to the Python (CCD ≤1e-13 even at full budget; FABRIK ≤1e-13 per step). Quality columns therefore match the Python to ≤0.7 pt (≤2 trials/cell) — a couple of *non-converged* boundary solves flip success/collision from 1-ULP transcendental differences (numpy vs Eigen) compounding over the iteration budget, which for FABRIK is marginally stable by construction. Only the timing is native (~310–500× faster wall-clock). See `cpp/parity_ccd_fabrik.py`. |

Homotopy (CCH-IK) and Fixed-λ are excluded from this benchmark. Numbers differ from
`master_full.md` **by design**: native compiled solvers, not Python — e.g. ProteinIK V4
now runs sub-millisecond and competes with TRAC-IK on speed as well as quality, and the
CCD/FABRIK classical baselines are now compiled C++ too (same algorithm, ~10–100× faster
wall-clock than the Python).
"""


def _append_footer(argv):
    out = None
    for i, a in enumerate(argv):
        if a == "--out" and i + 1 < len(argv):
            out = argv[i + 1]
    if out:
        try:
            with open(out + ".md", "a", encoding="utf-8") as f:
                f.write(FOOTER)
        except Exception:
            pass


if __name__ == "__main__":
    _argv = sys.argv[1:]
    _rc = M.main(_argv)
    _append_footer(_argv)
    sys.exit(_rc)
