"""Run the use-case experiments (``scrap/usecase_experiments.py``) — in
particular the DOF-scaling sweep, EXP E — with every borrowed solver replaced
by its genuine / native-compiled implementation, exactly like
``run_native_master.py`` does for the master benchmark.

Motivation: the DOF-scaling table (paper Table 5 / ``tab_dof_scaling.tex``)
compares KineticFold against TRAC-IK-style. Run through the plain registry it
used the *interpreted-Python* ``trac_ik_style`` recreation and the *interpreted*
``protein_fast``. This runner makes the comparison native and genuine:

  * ``trac_ik_style`` -> REAL TRAC-IK   (tracikpy: TRACLabs C++/KDL/NLopt)
  * ``protein_fast``  -> native C++/Eigen KineticFold port (pik_native, cpp/pik_v4.hpp)

Both adapters accept an arbitrary planar N-DOF ``RobotSpec`` (real TRAC-IK builds
its chain from a DH-generated URDF, FK-parity 2e-16; the C++ V4 builds a runtime
DH robot), so the whole 4..16-DOF sweep runs natively.

``usecase_experiments`` resolves ``SOLVER_REGISTRY`` at call time via
``run_solver``, so swapping entries in place is enough — the experiment code is
reused verbatim.

Usage (inside WSL Ubuntu-2204, from backend/):
    export ROBOT_DESCRIPTIONS_CACHE=/mnt/c/Users/Amarnath/.cache/robot_descriptions
    PYTHONPATH=. python3 native_bench/run_native_usecase.py --only E \
        --out results/native/dof_scaling_native.json
"""
from __future__ import annotations

import sys

from native_bench._env import apply
apply()

import app.solvers.registry as reg
import native_bench.genuine_solvers as G
import native_bench.cpp_solvers as C


def _wrap(fn):
    def wrapped(spec, q0, T_target, rng, collect_steps=False):
        return fn(spec, q0, T_target, rng, collect_steps)
    return wrapped


# Genuine imported robotics libraries (accept a DH robot, return joint angles).
GENUINE = {
    "trac_ik_style": (G.solve_real_tracik,    "TRAC-IK (real C++ TRACLabs)"),
    "jacobian_dls":  (G.solve_rtb_dls,        "Jacobian DLS (real RTB LM, single-shot)"),
    "multi_start":   (G.solve_rtb_multistart, "Multi-start (real RTB ik_LM restarts)"),
}
for name, (fn, disp) in GENUINE.items():
    reg.SOLVER_REGISTRY[name] = _wrap(fn)
    reg.SOLVER_DISPLAY_NAMES[name] = disp

# The ProteinIK family: native-C++ ports (pik_native, built from backend/cpp/).
CPP_PROTEIN = {
    "protein_ik":         C.solve_v1_cpp,
    "protein_fast":       C.solve_v4_cpp,
    "protein_fast_o2":    C.solve_o2_cpp,
    "protein_fast_calib": C.solve_calib_cpp,
    "protein_raw":        C.solve_raw_cpp,
}
for name, fn in CPP_PROTEIN.items():
    reg.SOLVER_REGISTRY[name] = _wrap(fn)

# The two classical baselines with no genuine DH-native upstream: native-C++ ports.
CPP_CLASSICAL = {
    "ccd":    C.solve_ccd_cpp,
    "fabrik": C.solve_fabrik_cpp,
}
for name, fn in CPP_CLASSICAL.items():
    reg.SOLVER_REGISTRY[name] = _wrap(fn)
reg.SOLVER_DISPLAY_NAMES["ccd"] = "CCD (in-repo; native C++)"
reg.SOLVER_DISPLAY_NAMES["fabrik"] = "FABRIK (in-repo; native C++)"

print("[native-usecase] genuine baselines:", ", ".join(sorted(GENUINE)),
      "| C++ ProteinIK:", ", ".join(sorted(CPP_PROTEIN)),
      "| C++ classical:", ", ".join(sorted(CPP_CLASSICAL)), flush=True)

# usecase_experiments lives in backend/scrap/; make it importable.
sys.path.insert(0, "/mnt/c/Coding Projects/Protien IK/backend/scrap")
import usecase_experiments as U  # noqa: E402


if __name__ == "__main__":
    sys.exit(U.main(sys.argv[1:]))
