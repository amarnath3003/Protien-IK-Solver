"""ProteinIK V4.1 — incremental-FK speed pass over V4 (protein_fast).

Same staged fold, same trajectory, bit-identical results — fewer and cheaper
floating-point operations in the hottest loop. See solver.py for details.
"""

from app.solvers.protein_fast_v41.solver import solve_protein_fast_v41

__all__ = ["solve_protein_fast_v41"]
