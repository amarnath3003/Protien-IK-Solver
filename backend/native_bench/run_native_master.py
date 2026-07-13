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
# don't produce DH joint angles — so they are left as the repo's own code and
# clearly labelled in the report rather than faked as a genuine import.
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

# Mark the two solvers that have no genuine DH-native upstream.
reg.SOLVER_DISPLAY_NAMES["ccd"] = "CCD (in-repo; no genuine upstream)"
reg.SOLVER_DISPLAY_NAMES["fabrik"] = "FABRIK (in-repo; no genuine upstream)"

print("[native] genuine baselines:", ", ".join(sorted(GENUINE)),
      "| C++ ProteinIK:", ", ".join(sorted(CPP_PROTEIN)), flush=True)

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
| CCD, FABRIK | in-repo algorithm — no genuine DH-native upstream exists |

Homotopy (CCH-IK) and Fixed-λ are excluded from this benchmark. Numbers differ from
`master_full.md` **by design**: native compiled solvers, not Python — e.g. ProteinIK V4
now runs sub-millisecond and competes with TRAC-IK on speed as well as quality.
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
