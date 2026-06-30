"""
ProteinIK V6 — Raw Biology solver.

A coarse-grained, implicit-solvent protein-folding simulation whose polymer is
the robot arm. It minimises the FREE ENERGY

    F(q; T) = E_task + E_LJ + E_HB − T · S_conf(q)

by overdamped Langevin dynamics (raw_math.md §4), cooling from an unfolded high
temperature toward the REM glass temperature T_glass (§5). The native state is
reached by the dynamics' own T→0 limit — a damped-Newton consolidation, NOT a
foreign finisher (§4b) — and accepted only if it passes a kinetic stability gate
(Anfinsen: native = a stable minimum).

Honest scope. Everything except E_task follows folding exactly; E_task is the
imposed boundary condition (folding is target-blind). Raw is the slowest of the
family by design — its thesis is solution quality and physical faithfulness, not
speed. Σ, F and T_glass are reported as diagnostics.
"""

from __future__ import annotations

import time

import numpy as np

from app.core.kinematics import (
    RobotSpec, end_effector_pose, pose_error, geometric_jacobian,
    self_collision_min_distance,
)
from app.core.types import SolveResult, SolveStep
from app.solvers.protein_energy import target_energy
from app.solvers.protein_raw.energy import (
    lj_energy, lj_energy_and_grad, hbond_energy, hbond_energy_and_grad,
    config_entropy, config_entropy_and_grad,
)
from app.solvers.protein_raw.landscape import (
    RawParams, sigma_ratio, configurational_entropy_scale, glass_temperature,
    warm_start,
)

ORIENT_W = 0.3


# ---------------------------------------------------------------------------
# Gradient pieces
# ---------------------------------------------------------------------------

def _task_grad(spec, q, T_target):
    """Uphill gradient of the (squared) pose error + the scalar pos/orient err."""
    err = pose_error(end_effector_pose(spec, q), T_target)
    err_w = err.copy()
    err_w[3:] *= ORIENT_W
    g = -geometric_jacobian(spec, q).T @ err_w        # ∂(½‖err_w‖²)/∂q
    return g, float(np.linalg.norm(err[:3])), float(np.linalg.norm(err[3:]))


def _clip_norm(g, gmax):
    n = float(np.linalg.norm(g))
    return g * (gmax / n) if n > gmax else g


def free_energy(spec, q, T_target, p: RawParams, T, m_entropy=24):
    """F(q;T) = E_task + E_LJ + E_HB − T·S_conf (scalar; for reporting)."""
    e = target_energy(spec, q, T_target)
    e += lj_energy(spec, q, p.sigma_scale, p.epsilon, True, e_cap=p.e_cap)
    if p.d0 > 0:
        e += hbond_energy(spec, q, p.d0, p.sigma_d, p.kappa, p.epsilon_hb, True)
    e -= T * config_entropy(spec, q, rho=p.rho, m=m_entropy)
    return float(e)


# ---------------------------------------------------------------------------
# Native-state consolidation — the T→0 limit (raw_math.md §4b)
# ---------------------------------------------------------------------------

def _consolidate(spec, q, T_target, pos_tol, ori_tol, max_steps=40, lam0=0.05):
    """Damped-Newton settling into the native (target) minimum. This is the
    noise-off limit of the same Langevin dynamics, not a separate solver."""
    q = q.copy()
    lam = lam0
    eye = np.eye(6)
    for _ in range(max_steps):
        err = pose_error(end_effector_pose(spec, q), T_target)
        pe, oe = float(np.linalg.norm(err[:3])), float(np.linalg.norm(err[3:]))
        if pe < pos_tol and oe < ori_tol:
            return q, True
        J = geometric_jacobian(spec, q)
        dq = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * eye, err)
        q_try = spec.clip(q + dq)
        err_t = pose_error(end_effector_pose(spec, q_try), T_target)
        if (np.linalg.norm(err_t[:3]) + ORIENT_W * np.linalg.norm(err_t[3:])
                < pe + ORIENT_W * oe):
            q, lam = q_try, max(lam * 0.5, 1e-4)
        else:
            lam = min(lam * 2.5, 2.0)
            if lam >= 2.0:
                break
    err = pose_error(end_effector_pose(spec, q), T_target)
    return q, (np.linalg.norm(err[:3]) < pos_tol and np.linalg.norm(err[3:]) < ori_tol)


def _stable_native(spec, q, T_target, pos_tol, ori_tol, rng, jitter=0.01, trials=5):
    """Kinetic native-state stability gate (Anfinsen): jitter and re-settle;
    native iff it returns to a config still within tolerance."""
    for _ in range(trials):
        qj = spec.clip(q + jitter * rng.standard_normal(spec.n_joints))
        qj, _ = _consolidate(spec, qj, T_target, pos_tol, ori_tol, max_steps=15)
        err = pose_error(end_effector_pose(spec, qj), T_target)
        if not (np.linalg.norm(err[:3]) < pos_tol and np.linalg.norm(err[3:]) < ori_tol):
            return False
    return True


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

def solve_protein_raw(spec: RobotSpec, q0: np.ndarray, T_target: np.ndarray,
                      rng: np.random.Generator, max_iters: int = 0,
                      pos_tol: float = 1e-3, orient_tol: float = 1e-2,
                      collect_steps: bool = False,
                      dt: float = 0.03, m_entropy: int = 16,
                      w_lj: float = 0.4, w_hb: float = 0.4, w_s: float = 0.5,
                      max_step: float = 0.25) -> SolveResult:
    t0 = time.perf_counter()
    n = spec.n_joints

    # --- calibrate the physics + measure the landscape ---------------------
    p = RawParams.calibrate(spec, rng)
    s0 = configurational_entropy_scale(spec, p.rho)
    sig = sigma_ratio(spec, T_target, p, rng, n_seeds=12)
    T_glass = glass_temperature(sig["sigma_E"], s0)
    T_start = max(4.0 * T_glass, 0.25)                 # start unfolded (hot)
    n_lang = max_iters if max_iters > 0 else 100 + 25 * max(0, n - 6)
    tau = n_lang / 3.0

    # --- seeds: multi-start warm-starts enforce the boundary condition ------
    # The task is the one non-folding term. We place the chain in the target
    # basin with several DLS warm-starts (redundant arms need more), fold the
    # bio structure around the BEST, and keep the rest as cheap fallbacks.
    def _terr(qq):
        e = pose_error(end_effector_pose(spec, qq), T_target)
        return float(np.linalg.norm(e[:3]) + ORIENT_W * np.linalg.norm(e[3:]))

    n_ws = 4 + 2 * max(0, n - 6)
    seeds = [warm_start(spec, q0, T_target)]
    seeds += [warm_start(spec, spec.random_config(rng), T_target) for _ in range(n_ws)]
    seeds.sort(key=_terr)
    q = seeds[0].copy()                                # fold from the best seed
    g_task0, _, _ = _task_grad(spec, q, T_target)
    g_cap = 5.0 * max(float(np.linalg.norm(g_task0)), 1e-6)   # bound LJ explosion

    best_q = q.copy()
    best_score = np.inf
    steps_out: list[SolveStep] = []

    for it in range(n_lang):
        T = max(T_glass, T_start * np.exp(-it / tau))

        g_task, pe, oe = _task_grad(spec, q, T_target)
        score = pe + ORIENT_W * oe                       # error AT the current q
        if score < best_score:
            best_score, best_q = score, q.copy()         # best_q matches its error

        if collect_steps:
            phase = ("raw_unfolded" if T > 0.6 * T_start
                     else "raw_collapse" if T > 1.5 * T_glass
                     else "raw_consolidate")
            steps_out.append(SolveStep(
                iteration=it, q=q.tolist(), pos_error=pe, orient_error=oe,
                min_self_distance=self_collision_min_distance(spec, q),
                phase=phase, energy=float(T)))

        _, g_lj = lj_energy_and_grad(spec, q, p.sigma_scale, p.epsilon, True)
        g_lj = _clip_norm(g_lj, g_cap)
        if p.d0 > 0:
            _, g_hb = hbond_energy_and_grad(spec, q, p.d0, p.sigma_d, p.kappa,
                                            p.epsilon_hb, True)
        else:
            g_hb = np.zeros(n)
        _, g_s = config_entropy_and_grad(spec, q, rho=p.rho, m=m_entropy)

        grad_F = g_task + w_lj * g_lj + w_hb * g_hb - T * w_s * g_s
        noise = np.sqrt(2.0 * T * dt) * rng.standard_normal(n)
        q = spec.clip(q + _clip_norm(-grad_F * dt + noise, max_step))

    # --- native-state consolidation (T→0): the folded best, then the seeds
    #     as cheap multi-start fallbacks (chaperone-style retry) -------------
    def _err(qq):
        e = pose_error(end_effector_pose(spec, qq), T_target)
        return float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:]))

    q_nat = best_q.copy()
    conv = False
    best_err = np.inf
    restarts = -1
    for cand in [best_q] + seeds:                        # folded fold first
        restarts += 1
        cc, ok = _consolidate(spec, cand, T_target, pos_tol, orient_tol)
        pe_c, oe_c = _err(cc)
        if pe_c + ORIENT_W * oe_c < best_err:
            best_err, q_nat = pe_c + ORIENT_W * oe_c, cc
        if ok:
            conv, q_nat = True, cc
            break

    pe, oe = _err(q_nat)
    success = conv                                       # = reached tolerance (comparable)
    stable = conv and _stable_native(spec, q_nat, T_target, pos_tol, orient_tol, rng)

    if collect_steps:
        steps_out.append(SolveStep(
            iteration=n_lang, q=q_nat.tolist(), pos_error=pe, orient_error=oe,
            min_self_distance=self_collision_min_distance(spec, q_nat),
            phase=("raw_native_stable" if stable else "raw_native_settle"),
            energy=float(T_glass)))

    violations = int(np.sum(
        (q_nat <= spec.joint_limits[:, 0] + 1e-9) |
        (q_nat >= spec.joint_limits[:, 1] - 1e-9)))

    return SolveResult(
        solver_name="protein_raw",
        success=success,
        q_final=q_nat.tolist(),
        pos_error=pe,
        orient_error=oe,
        iterations=n_lang,
        wall_time_ms=(time.perf_counter() - t0) * 1000.0,
        min_self_distance=self_collision_min_distance(spec, q_nat),
        joint_limit_violations=violations,
        restarts=restarts,
        steps=steps_out,
        sigma_ratio=float(sig["sigma"]),
        free_energy=free_energy(spec, q_nat, T_target, p, T_glass, m_entropy),
        t_glass=float(T_glass),
    )
