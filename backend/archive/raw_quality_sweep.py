"""
Raw (V6) — fair-shot QUALITY sweep (does biophysical energy beat V4 on quality?)

research_direction.md's open question: given a solve reaches the target, is Raw's
SOLUTION qualitatively better (lower self-collision, better conditioned) than V4?

We compare on UR5 cluttered — the collision-stressing regime — reporting, on
each solver's successful solves: mean self-collision clearance (min_self), mean
manipulability, and the per-target WIN RATE vs V4 (fraction of commonly-solved
targets where the solver's solution is more clash-free than V4's). We also sweep
a few Raw weight calibrations to give it a fair chance.

Run:  python raw_quality_sweep.py
"""

from __future__ import annotations

import sys
import numpy as np

sys.stdout.reconfigure(encoding="utf-8")

from app.core.kinematics import ur5_spec, self_collision_min_distance
from app.api.scenarios import generate_target
from app.api.scenarios import _manipulability
from app.solvers.protein_fast import solve_protein_fast
from app.solvers.protein_homotopy import solve_protein_homotopy
from app.solvers.protein_raw import solve_protein_raw

N = 24
spec = ur5_spec()
tgen = np.random.default_rng(42)
TARGETS = [generate_target(spec, tgen, "cluttered") for _ in range(N)]


def run(label, fn):
    """Return per-target dict of (success, min_self, manip)."""
    out = []
    for i, (q0, T) in enumerate(TARGETS):
        rng = np.random.default_rng(2000 + i)
        r = fn(q0.copy(), T, rng)
        q = np.array(r.q_final)
        out.append((r.success, self_collision_min_distance(spec, q), _manipulability(spec, q)))
    return label, out


SOLVERS = [
    ("V4 Fast",          lambda q0, T, rng: solve_protein_fast(spec, q0, T, rng)),
    ("V5 CCH-IK",        lambda q0, T, rng: solve_protein_homotopy(spec, q0, T, rng)),
    ("Raw default",      lambda q0, T, rng: solve_protein_raw(spec, q0, T, rng)),
    ("Raw hi-entropy",   lambda q0, T, rng: solve_protein_raw(spec, q0, T, rng, w_s=1.5, m_entropy=24)),
    ("Raw hi-bio",       lambda q0, T, rng: solve_protein_raw(spec, q0, T, rng, w_lj=1.0, w_hb=0.8, w_s=1.0, m_entropy=24)),
]

results = dict(run(lbl, fn) for lbl, fn in SOLVERS)
v4 = results["V4 Fast"]

print(f"UR5 / cluttered / N={N}  — quality on SUCCESSFUL solves (min_self higher = more clash-free)\n")
print(f"{'solver':<16}{'succ':>6}{'mean min_self':>15}{'mean manip':>12}{'win-vs-V4':>11}")
print("-" * 60)
for lbl, out in results.items():
    succ = [o for o in out if o[0]]
    sr = len(succ) / N
    msd = float(np.mean([o[1] for o in succ])) if succ else float("nan")
    man = float(np.mean([o[2] for o in succ])) if succ else float("nan")
    # win vs V4 on targets both solved
    both = [(o[1], v[1]) for o, v in zip(out, v4) if o[0] and v[0]]
    win = float(np.mean([a > b for a, b in both])) if both else float("nan")
    print(f"{lbl:<16}{sr*100:>5.0f}%{msd:>+15.4f}{man:>12.4f}{win*100:>10.0f}%")
