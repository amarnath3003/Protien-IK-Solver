"""
THE master benchmark — every solver × every arm × every scenario × every metric,
scored end-to-end under BOTH real-mesh simulators (PyBullet + MuJoCo) in one sweep.

This is the single, reproducible, paper-grade artifact that supersedes the three
narrower runners it consolidates (``master_benchmark.py`` = core/proxy only,
``bench/sim_benchmark.py`` = PyBullet oracle, ``bench/sim_crosscheck.py`` /
``bench/tri_sim_benchmark.py`` = the MuJoCo cross-check). Its defining move is
**solve once, score three ways**: every solver runs a single time on the fast numpy
``RobotSpec`` core, and that identical ``q_final`` is then judged by

  1. **our capsule proxy** (what the solver actually optimizes against),
  2. **PyBullet** real-mesh FK + ``getClosestPoints`` self-collision, and
  3. **MuJoCo**  real-mesh FK + ``mj_geomDistance`` self-collision

on the *identical* URDF and the *identical* non-adjacent link pairs. So every
(arm, scenario, solver) cell carries our number and both independent oracles' numbers
side by side, with no solver ever compared on an easier target draw (targets are
generated once per (arm, scenario, seed) and shared across all solvers).

Coverage of a default full run
------------------------------
  * **Arms**   — ``planar3dof`` (core/proxy only; no canonical URDF, validated
                 analytically), ``ur5``, ``franka_panda`` (all three evaluators).
  * **Scenarios** — ``open_space``, ``near_singular``, ``cluttered``.
  * **Solvers** — the *entire* live registry valid for each arm (the two ~1 s
                 homotopy solvers included by default; drop with ``--skip-slow``),
                 plus a **PyBullet native-IK** baseline column on the sim arms,
                 itself scored in both engines.
  * **Metrics** — success / wall-clock percentiles (mean/p50/p95/p99) / iterations /
                 self-collision rate / clearance / pos & orient error / joint-limit
                 violations / restarts, from our FK **and** from each oracle; plus
                 the CCH-IK / V6 diagnostics (conflict, difficulty, Σ, free energy).
  * **Oracle self-validation** — before trusting the numbers, each sim arm gets a
                 three-way **FK-agreement** and **collision-agreement** block
                 (DH ≡ PyBullet ≡ MuJoCo), so the run proves its own oracles.

Built for an unattended overnight run on a fast machine
-------------------------------------------------------
  * **Crash-safe** — the CSV is rewritten after *every* completed cell, so an
    interruption at hour 3 loses at most the in-progress cell (the failure mode that
    once wiped a whole Franka sweep — results written only at the end — cannot recur).
  * **Resumable** — ``--resume`` reads the existing CSV and skips cells already done;
    restart after a crash/reboot and it picks up where it left off.
  * **Isolated cells** — a solver that throws is logged and recorded as a failed cell;
    the sweep continues rather than aborting.
  * **Self-degrading** — if MuJoCo can't be constructed (e.g. not installed on the
    target box) the run continues PyBullet-only with a loud warning, rather than dying.
  * **Observable** — per-cell progress with elapsed time and a running ETA; a JSON
    run-manifest (config + library versions + timing) written alongside the results;
    the Markdown report regenerated after each arm so partial output is always readable.
  * **UTF-8 stdout** — solver display names carry non-latin-1 glyphs (the ``λ`` in
    "Fixed-λ Homotopy"); Windows cp1252 stdout would otherwise crash a progress print.

Environment
-----------
Needs ``pybullet`` + ``mujoco`` (+ ``numpy``), which on this project live only in the
separate ``backend/.venv-sim`` (Python 3.12). Run it from ``backend/`` as a module::

    PYTHONPATH=. .venv-sim/Scripts/python -m bench.master_sim_benchmark            # full run
    PYTHONPATH=. .venv-sim/Scripts/python -m bench.master_sim_benchmark --quick    # smoke
    PYTHONPATH=. .venv-sim/Scripts/python -m bench.master_sim_benchmark --skip-slow --resume

or via the convenience launcher ``bench/run_master_benchmark.ps1`` (Windows), which
finds the venv, tees a timestamped log, and passes any extra args through.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import platform
import sys
import time
import traceback

import numpy as np

from app.core.kinematics import (
    get_robot_spec, self_collision_min_distance,
)
from app.api.scenarios import generate_target
from app.solvers.registry import (
    SOLVER_DISPLAY_NAMES, get_solvers_for_robot, run_solver,
)

# Both backend modules import their heavy sim lib lazily (only inside __init__), so
# importing them here is always safe even on an env without pybullet/mujoco.
from app.sim.pybullet_backend import PyBulletBackend
from app.sim.mujoco_backend import MuJoCoBackend
from app.sim.parity import _rel_angle


# ---- what "everything" means -------------------------------------------------

ALL_ROBOTS = ["planar3dof", "ur5", "franka_panda"]
ALL_SCENARIOS = ["open_space", "near_singular", "cluttered"]
# Arms with a canonical URDF and therefore a real-mesh sim oracle. planar3dof has no
# standard URDF (validated analytically instead), so it gets core/proxy columns only.
SIM_ROBOTS = {"ur5", "franka_panda"}
# The two homotopy solvers cost ~1 s/solve; --skip-slow drops them for a fast sweep.
SLOW_SOLVERS = {"protein_homotopy", "fixed_lambda_ik"}
# Synthetic "solver" id for PyBullet's own IK baseline (sim arms only).
NATIVE = "pybullet_native_ik"
# A solver clears this success bar to be counted in the collision "winner" verdict.
HIGH_SUCCESS = 90.0

# Full wide-CSV schema — every metric from every evaluator, one row per cell.
CSV_COLUMNS = [
    "robot", "scenario", "solver", "display_name", "n", "status",
    # core / proxy (our FK + capsule self-collision proxy)
    "success_pct", "mean_ms", "p50_ms", "p95_ms", "p99_ms", "mean_iters",
    "our_collision_pct", "our_mean_clearance_m",
    "our_mean_pos_mm", "our_mean_orient_mrad",
    "mean_joint_limit_violations", "mean_restarts",
    "mean_conflict_index", "mean_difficulty_score",
    "mean_sigma_ratio", "mean_free_energy",
    # PyBullet oracle (real mesh)
    "pb_success_pct", "pb_success_agree_pct", "pb_collision_pct",
    "pb_mean_clearance_m", "pb_mean_pos_mm", "pb_mean_orient_mrad",
    # MuJoCo oracle (real mesh)
    "mj_success_pct", "mj_success_agree_pct", "mj_collision_pct",
    "mj_mean_clearance_m", "mj_mean_pos_mm", "mj_mean_orient_mrad",
    # cross-engine
    "pb_mj_collision_agree_pct",
]


def _pct(x: float) -> float:
    return 100.0 * float(x)


def _nanmean(xs: list[float]) -> float:
    return float(np.mean(xs)) if xs else float("nan")


# ---------------------------------------------------------------------------
# one (robot, scenario, solver) cell — solve once, score in every oracle
# ---------------------------------------------------------------------------

def bench_cell(robot: str, scenario: str, solver: str, spec,
               pb: PyBulletBackend | None, mj: MuJoCoBackend | None,
               targets_by_seed: dict[int, list], seeds: list[int],
               warmup: int) -> dict:
    """Run one cell across all seeds and return its wide metric row.

    Targets are passed in pre-generated per seed (identical across solvers) so the
    comparison is apples-to-apples. Each solve's ``q_final`` is scored by whichever
    oracles are available; the ``pb``/``mj`` columns are NaN when that backend is
    absent (planar3dof, or a degraded run). ``solver == NATIVE`` runs PyBullet's own
    IK instead of one of ours on the identical targets, leaving the ``our_*`` columns
    NaN (there is no "our-FK" notion for the sim's native solver).
    """
    is_native = (solver == NATIVE)
    n = 0
    tms: list[float] = []
    iters: list[float] = []
    succ = 0
    # our-FK / proxy accumulators (skipped for native)
    our_col = 0
    our_clear, our_pos_mm, our_orient_mr = [], [], []
    jlv, restarts = [], []
    conflict, difficulty, sigma, free_energy = [], [], [], []
    have_our = 0
    # oracle accumulators
    pb_succ = pb_col = pb_agree = 0
    mj_succ = mj_col = mj_agree = 0
    pb_mj_col_agree = 0
    pb_clear, pb_pos_mm, pb_orient_mr = [], [], []
    mj_clear, mj_pos_mm, mj_orient_mr = [], [], []

    def solve_once(q0, T_dh, rng):
        """Return (q_final, wall_ms, result_or_None). result is None for native."""
        if is_native:
            t0 = time.perf_counter()
            q = pb.native_ik(pb.dh_to_sim(T_dh), q0)
            return np.asarray(q, dtype=float), (time.perf_counter() - t0) * 1000.0, None
        r = run_solver(solver, spec, q0, T_dh, rng)
        return np.asarray(r.q_final, dtype=float), r.wall_time_ms, r

    for seed in seeds:
        targets = targets_by_seed[seed]
        n_trials = len(targets)

        # Untimed warm-up: remove first-call / allocation transients from timing.
        for w in range(warmup):
            q0, T_dh = targets[w % n_trials]
            solve_once(q0, T_dh, np.random.default_rng(10_000 + w))

        for i, (q0, T_dh) in enumerate(targets):
            n += 1
            rng = np.random.default_rng(seed * 1_000_003 + i)
            q_final, wall_ms, r = solve_once(q0, T_dh, rng)
            tms.append(wall_ms)

            our_ok = None
            if r is not None:
                have_our += 1
                our_ok = bool(r.success)
                succ += int(our_ok)
                iters.append(r.iterations)
                our_col += int(r.min_self_distance < 0)
                our_clear.append(float(r.min_self_distance))
                our_pos_mm.append(r.pos_error * 1000.0)
                our_orient_mr.append(r.orient_error * 1000.0)
                jlv.append(r.joint_limit_violations)
                restarts.append(r.restarts)
                conflict.append(r.conflict_index)
                difficulty.append(r.difficulty_score)
                sigma.append(r.sigma_ratio)
                free_energy.append(r.free_energy)

            if pb is not None:
                sp = pb.score(q_final, T_dh)
                pb_succ += int(sp.sim_success)
                pb_col += int(sp.sim_in_collision)
                pb_clear.append(sp.sim_min_self_distance)
                pb_pos_mm.append(sp.sim_pos_error * 1000.0)
                pb_orient_mr.append(sp.sim_orient_error * 1000.0)
                if our_ok is not None:
                    pb_agree += int(our_ok == sp.sim_success)
            if mj is not None:
                sm = mj.score(q_final, T_dh)
                mj_succ += int(sm.sim_success)
                mj_col += int(sm.sim_in_collision)
                mj_clear.append(sm.sim_min_self_distance)
                mj_pos_mm.append(sm.sim_pos_error * 1000.0)
                mj_orient_mr.append(sm.sim_orient_error * 1000.0)
                if our_ok is not None:
                    mj_agree += int(our_ok == sm.sim_success)
            if pb is not None and mj is not None:
                pb_mj_col_agree += int(sp.sim_in_collision == sm.sim_in_collision)

    tms = np.array(tms)
    has_pb, has_mj = pb is not None, mj is not None
    has_both = has_pb and has_mj
    return {
        "robot": robot, "scenario": scenario, "solver": solver,
        "display_name": SOLVER_DISPLAY_NAMES.get(solver, "PyBullet native IK"),
        "n": n, "status": "ok",
        # core / proxy
        "success_pct": _pct(succ / have_our) if have_our else float("nan"),
        "mean_ms": float(tms.mean()),
        "p50_ms": float(np.percentile(tms, 50)),
        "p95_ms": float(np.percentile(tms, 95)),
        "p99_ms": float(np.percentile(tms, 99)),
        "mean_iters": _nanmean(iters),
        "our_collision_pct": _pct(our_col / have_our) if have_our else float("nan"),
        "our_mean_clearance_m": _nanmean(our_clear),
        "our_mean_pos_mm": _nanmean(our_pos_mm),
        "our_mean_orient_mrad": _nanmean(our_orient_mr),
        "mean_joint_limit_violations": _nanmean(jlv),
        "mean_restarts": _nanmean(restarts),
        "mean_conflict_index": _nanmean(conflict),
        "mean_difficulty_score": _nanmean(difficulty),
        "mean_sigma_ratio": _nanmean(sigma),
        "mean_free_energy": _nanmean(free_energy),
        # PyBullet oracle
        "pb_success_pct": _pct(pb_succ / n) if has_pb else float("nan"),
        "pb_success_agree_pct": (_pct(pb_agree / n) if (has_pb and have_our) else float("nan")),
        "pb_collision_pct": _pct(pb_col / n) if has_pb else float("nan"),
        "pb_mean_clearance_m": _nanmean(pb_clear),
        "pb_mean_pos_mm": _nanmean(pb_pos_mm),
        "pb_mean_orient_mrad": _nanmean(pb_orient_mr),
        # MuJoCo oracle
        "mj_success_pct": _pct(mj_succ / n) if has_mj else float("nan"),
        "mj_success_agree_pct": (_pct(mj_agree / n) if (has_mj and have_our) else float("nan")),
        "mj_collision_pct": _pct(mj_col / n) if has_mj else float("nan"),
        "mj_mean_clearance_m": _nanmean(mj_clear),
        "mj_mean_pos_mm": _nanmean(mj_pos_mm),
        "mj_mean_orient_mrad": _nanmean(mj_orient_mr),
        # cross-engine
        "pb_mj_collision_agree_pct": _pct(pb_mj_col_agree / n) if has_both else float("nan"),
    }


# ---------------------------------------------------------------------------
# oracle self-validation (three-way FK + collision agreement) per sim arm
# ---------------------------------------------------------------------------

def fk_agreement(pb: PyBulletBackend, mj: MuJoCoBackend | None, spec,
                 n: int, seed: int) -> dict:
    """DH ≡ PyBullet ≡ MuJoCo forward-kinematics agreement over ``n`` random configs.

    Both engines are already validated against our DH at construction (constant frame
    offset, residual < 1e-4). This measures PyBullet-vs-MuJoCo directly; three-way
    agreement to float noise means the *robot model* is not in question before we read
    any solver number off it.
    """
    rng = np.random.default_rng(seed)
    dpos = np.empty(n)
    dang = np.empty(n)
    for i in range(n):
        q = spec.random_config(rng)
        Tp = pb.fk(q)
        Tm = mj.fk(q) if mj is not None else Tp
        dpos[i] = np.linalg.norm(Tp[:3, 3] - Tm[:3, 3])
        dang[i] = _rel_angle(Tp[:3, :3], Tm[:3, :3])
    return {
        "n": n,
        "max_pos": float(dpos.max()), "mean_pos": float(dpos.mean()),
        "max_orient": float(dang.max()), "mean_orient": float(dang.mean()),
        "dh_pb_residual": float(pb.offset_residual), "pb_offset": pb.offset_side,
        "dh_mj_residual": float(mj.offset_residual) if mj is not None else float("nan"),
        "mj_offset": mj.offset_side if mj is not None else "—",
    }


def collision_agreement(pb: PyBulletBackend, mj: MuJoCoBackend | None, spec,
                        n: int, seed: int, threshold: float = 0.8) -> dict:
    """Capsule-proxy vs PyBullet vs MuJoCo real-mesh self-collision over ``n`` configs.

    Confirms (engine-independently) that the proxy is optimistic vs real mesh, and
    quantifies how far the two independent real-mesh engines agree with each other on
    the collide/clear call.
    """
    rng = np.random.default_rng(seed)
    proxy = np.empty(n)
    pbd = np.empty(n)
    mjd = np.empty(n)
    for i in range(n):
        q = spec.random_config(rng)
        proxy[i] = self_collision_min_distance(spec, q)
        _, pbd[i] = pb.self_collision(q, threshold=threshold)
        mjd[i] = mj.self_collision(q, threshold=threshold)[1] if mj is not None else np.nan
    pcol, pbcol = proxy < 0, pbd < 0
    out = {
        "n": n,
        "proxy_col_pct": _pct(pcol.mean()),
        "pb_col_pct": _pct(pbcol.mean()),
        "proxy_false_clear_vs_pb": _pct(((pbd < 0) & (proxy >= 0)).mean()),
    }
    if mj is not None:
        mjcol = mjd < 0
        out.update({
            "mj_col_pct": _pct(mjcol.mean()),
            "pb_mj_sign_agree_pct": _pct((pbcol == mjcol).mean()),
            "pb_mj_corr": float(np.corrcoef(pbd, mjd)[0, 1]),
            "proxy_false_clear_vs_mj": _pct(((mjd < 0) & (proxy >= 0)).mean()),
        })
    else:
        out.update({"mj_col_pct": float("nan"), "pb_mj_sign_agree_pct": float("nan"),
                    "pb_mj_corr": float("nan"), "proxy_false_clear_vs_mj": float("nan")})
    return out


# ---------------------------------------------------------------------------
# output
# ---------------------------------------------------------------------------

def write_csv(rows: list[dict], path: str) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLUMNS, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)


def _f(x, nd=1):
    """Format a possibly-NaN float for a markdown cell."""
    return "  –  " if (x is None or (isinstance(x, float) and np.isnan(x))) else f"{x:.{nd}f}"


def _winner(cells: list[dict], key: str) -> str:
    """Display name of the lowest-collision solver under ``key`` among the
    high-success ones (excludes native, which has no our-success number)."""
    pool = [c for c in cells if c["solver"] != NATIVE
            and not np.isnan(c["success_pct"]) and c["success_pct"] >= HIGH_SUCCESS
            and not np.isnan(c.get(key, float("nan")))]
    if not pool:
        return "—"
    return min(pool, key=lambda c: c[key])["display_name"]


def write_markdown(rows: list[dict], path: str, meta: dict,
                   fk: dict, col: dict) -> None:
    by_cell: dict[tuple, list[dict]] = {}
    for r in rows:
        by_cell.setdefault((r["robot"], r["scenario"]), []).append(r)

    L = [
        "# Master Sim Benchmark — every solver × arm × scenario, scored in PyBullet + MuJoCo",
        "",
        f"- **{meta['trials']}** trials/seed × seeds **{meta['seeds']}** "
        f"(n={meta['trials'] * len(meta['seeds'])} per cell)  |  warm-up "
        f"{meta['warmup']} untimed solves/cell",
        f"- Arms: {', '.join(meta['robots'])}  |  Scenarios: {', '.join(meta['scenarios'])}",
        f"- Engines: {meta['engines']}  |  generated {meta.get('finished_at', '(in progress)')}",
        "",
        "Each solver runs **once** on our DH `RobotSpec` core; that `q_final` is then scored "
        "by two independent real-mesh oracles — **PB** = PyBullet, **MJ** = MuJoCo (identical "
        "URDF & non-adjacent link pairs). The capsule proxy the solvers optimize against is "
        "**not** used to evaluate them here — only real-mesh collision counts (planar3dof has "
        "no URDF, so it carries success/speed only). "
        "`PyBullet native IK` is the sim's own solver on the identical targets. "
        "Timing is wall-clock (OS noise on mean/p95/p99); success/collision/error columns "
        "are deterministic given the seed.",
        "",
    ]

    # -- oracle self-validation (only if we ran it) -----------------------
    if fk:
        L += ["## Oracle validation — DH ≡ PyBullet ≡ MuJoCo", "",
              "### A. Forward-kinematics agreement", "",
              "| Arm | n | DH↔PB resid | DH↔MJ resid | PB↔MJ max pos | max orient |",
              "|:--|--:|--:|--:|--:|--:|"]
        for robot in meta["robots"]:
            f = fk.get(robot)
            if not f:
                continue
            L.append(f"| {robot} | {f['n']} | {f['dh_pb_residual']:.1e} ({f['pb_offset']}) | "
                     f"{f['dh_mj_residual']:.1e} ({f['mj_offset']}) | "
                     f"{f['max_pos']:.2e} m | {f['max_orient']:.2e} rad |")
        L += ["", "### B. Self-collision agreement — PyBullet vs MuJoCo", "",
              "| Arm | n | PB col% | MJ col% | PB↔MJ sign-agree% | PB↔MJ corr |",
              "|:--|--:|--:|--:|--:|--:|"]
        for robot in meta["robots"]:
            c = col.get(robot)
            if not c:
                continue
            L.append(f"| {robot} | {c['n']} | {_f(c['pb_col_pct'])} | "
                     f"{_f(c['mj_col_pct'])} | {_f(c['pb_mj_sign_agree_pct'])} | "
                     f"{_f(c['pb_mj_corr'], 3)} |")
        L.append("")

    # -- headline verdict: who collides least, under each collision model --
    L += ["## Verdict — lowest real-mesh-collision solver per cell "
          f"(among ≥{HIGH_SUCCESS:.0f}% success)", "",
          "| Arm | Scenario | **PyBullet** | **MuJoCo** |",
          "|:--|:--|:--|:--|"]
    for robot in meta["robots"]:
        for scenario in meta["scenarios"]:
            c = by_cell.get((robot, scenario))
            if not c:
                continue
            L.append(f"| {robot} | {scenario} | "
                     f"{_winner(c, 'pb_collision_pct')} | {_winner(c, 'mj_collision_pct')} |")
    L.append("")

    # -- per-cell full tables ---------------------------------------------
    for robot in meta["robots"]:
        for scenario in meta["scenarios"]:
            cell = by_cell.get((robot, scenario))
            if not cell:
                continue
            # solvers by sim (or our) success desc; native baseline last
            body = [r for r in cell if r["solver"] != NATIVE]
            key = "pb_success_pct" if robot in SIM_ROBOTS else "success_pct"
            body.sort(key=lambda x: (-(x[key] if not np.isnan(x[key]) else -1)))
            body += [r for r in cell if r["solver"] == NATIVE]
            L += [f"## {robot} — {scenario}", "",
                  "| Solver | Succ% | PB succ% | MJ succ% | Mean ms | p95 | p99 | Iters | "
                  "PB col% | MJ col% | PB clr | MJ clr | PB pos mm | JLV |",
                  "|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|"]
            for r in body:
                L.append(
                    f"| {r['display_name']} | {_f(r['success_pct'])} | {_f(r['pb_success_pct'])} | "
                    f"{_f(r['mj_success_pct'])} | {_f(r['mean_ms'])} | {_f(r['p95_ms'])} | "
                    f"{_f(r['p99_ms'])} | {_f(r['mean_iters'], 0)} | "
                    f"{_f(r['pb_collision_pct'])} | {_f(r['mj_collision_pct'])} | "
                    f"{_f(r['pb_mean_clearance_m'], 4)} | "
                    f"{_f(r['mj_mean_clearance_m'], 4)} | {_f(r['pb_mean_pos_mm'], 3)} | "
                    f"{_f(r['mean_joint_limit_violations'], 2)} |")
            L.append("")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))


# ---------------------------------------------------------------------------
# driver
# ---------------------------------------------------------------------------

def _load_done(csv_path: str) -> tuple[list[dict], set]:
    """Read an existing results CSV for --resume: return (rows, done-keys)."""
    if not os.path.exists(csv_path):
        return [], set()
    rows: list[dict] = []
    done: set = set()
    with open(csv_path, newline="", encoding="utf-8") as f:
        for raw in csv.DictReader(f):
            # re-type numeric columns so re-emitted rows format identically
            row = dict(raw)
            for k, v in row.items():
                if k in ("robot", "scenario", "solver", "display_name", "status"):
                    continue
                try:
                    row[k] = float(v)
                except (ValueError, TypeError):
                    pass
            rows.append(row)
            if row.get("status", "ok") == "ok":
                done.add((row["robot"], row["scenario"], row["solver"]))
    return rows, done


def main(argv=None) -> int:
    # Force UTF-8 stdout — solver display names carry glyphs (λ) that crash cp1252
    # console/redirect encoding on Windows and would abort a progress print mid-sweep.
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

    ap = argparse.ArgumentParser(
        description="Master end-to-end benchmark: all solvers/arms/scenarios/metrics, "
                    "scored in PyBullet + MuJoCo.")
    ap.add_argument("--trials", type=int, default=100, help="trials per seed per cell (default 100)")
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3], help="RNG seeds (noise-averaged)")
    ap.add_argument("--warmup", type=int, default=8, help="untimed warm-up solves per cell")
    ap.add_argument("--robots", nargs="+", default=ALL_ROBOTS, choices=ALL_ROBOTS)
    ap.add_argument("--scenarios", nargs="+", default=ALL_SCENARIOS, choices=ALL_SCENARIOS)
    ap.add_argument("--solvers", nargs="+", default=None, help="subset of solver ids (default: all valid)")
    ap.add_argument("--skip-slow", action="store_true", help=f"drop slow solvers {sorted(SLOW_SOLVERS)}")
    ap.add_argument("--no-native", action="store_true", help="skip the PyBullet native-IK baseline")
    ap.add_argument("--no-mujoco", action="store_true", help="PyBullet only (skip the MuJoCo second oracle)")
    ap.add_argument("--no-pybullet", action="store_true", help="skip all sim oracles (core/proxy columns only)")
    ap.add_argument("--skip-validation", action="store_true", help="skip the FK/collision agreement blocks")
    ap.add_argument("--fk-samples", type=int, default=2000, help="configs for the FK-agreement check")
    ap.add_argument("--collision-samples", type=int, default=2000, help="configs for the collision-agreement check")
    ap.add_argument("--quick", action="store_true",
                    help="fast smoke preset: 5 trials, 1 seed, skip-slow, tiny validation")
    ap.add_argument("--resume", action="store_true", help="skip cells already present in the output CSV")
    ap.add_argument("--out", default="master_sim_benchmark", help="output stem (.csv/.md/.manifest.json)")
    args = ap.parse_args(argv)

    if args.quick:
        args.trials, args.seeds, args.warmup = 5, [1], 2
        args.skip_slow = True
        args.fk_samples = min(args.fk_samples, 200)
        args.collision_samples = min(args.collision_samples, 200)

    use_pb = not args.no_pybullet
    use_mj = use_pb and not args.no_mujoco
    engines = "PyBullet + MuJoCo" if use_mj else ("PyBullet only" if use_pb else "none (core/proxy only)")

    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    csv_path, md_path = args.out + ".csv", args.out + ".md"
    manifest_path = args.out + ".manifest.json"

    rows, done = _load_done(csv_path) if args.resume else ([], set())

    # -- enumerate the full work list up front (for ETA + logging) --------
    work: list[tuple[str, str, str]] = []
    for robot in args.robots:
        valid = get_solvers_for_robot(robot)
        solvers = [s for s in valid if (args.solvers is None or s in args.solvers)]
        if args.skip_slow:
            solvers = [s for s in solvers if s not in SLOW_SOLVERS]
        if use_pb and not args.no_native and robot in SIM_ROBOTS:
            solvers = solvers + [NATIVE]
        for scenario in args.scenarios:
            for solver in solvers:
                if (robot, scenario, solver) not in done:
                    work.append((robot, scenario, solver))

    total_cells = len(work) + len(done)
    print(f"=== Master sim benchmark ===", flush=True)
    print(f"  engines={engines}  n/cell={args.trials * len(args.seeds)}  "
          f"cells={total_cells} ({len(done)} already done, {len(work)} to run)", flush=True)
    print(f"  out={csv_path} / {md_path}", flush=True)

    fk_out: dict[str, dict] = {}
    col_out: dict[str, dict] = {}
    t_start = time.perf_counter()
    n_done = 0

    def checkpoint(finished=False):
        """Rewrite CSV + MD from the full row set so a crash never loses > 1 cell."""
        rows.sort(key=lambda r: (args.robots.index(r["robot"]) if r["robot"] in args.robots else 9,
                                 args.scenarios.index(r["scenario"]) if r["scenario"] in args.scenarios else 9,
                                 r["solver"]))
        write_csv(rows, csv_path)
        meta = dict(trials=args.trials, seeds=args.seeds, warmup=args.warmup,
                    robots=args.robots, scenarios=args.scenarios, engines=engines)
        if finished:
            meta["finished_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        write_markdown(rows, md_path, meta, fk_out, col_out)

    # -- run, arm by arm (backends opened once per sim arm) ---------------
    for robot in args.robots:
        arm_work = [w for w in work if w[0] == robot]
        if not arm_work:
            continue
        spec = get_robot_spec(robot)
        want_sim = use_pb and robot in SIM_ROBOTS

        pb = mj = None
        try:
            if want_sim:
                pb = PyBulletBackend(robot)
                print(f"[{robot}] PyBullet oracle ready: ee={pb.ee_link} "
                      f"offset={pb.offset_side} resid={pb.offset_residual:.2e}", flush=True)
                if use_mj:
                    try:
                        mj = MuJoCoBackend(robot, collision_link_names=pb.collision_link_names)
                        print(f"[{robot}] MuJoCo oracle ready: offset={mj.offset_side} "
                              f"resid={mj.offset_residual:.2e}", flush=True)
                    except Exception as e:
                        print(f"[{robot}] !! MuJoCo backend unavailable ({e!r}); "
                              f"continuing PyBullet-only for this arm.", flush=True)
                        mj = None

                if not args.skip_validation:
                    fk_out[robot] = fk_agreement(pb, mj, spec, args.fk_samples, seed=7)
                    f = fk_out[robot]
                    print(f"[{robot}]  A/FK  PB↔MJ max_pos={f['max_pos']:.2e}m "
                          f"max_orient={f['max_orient']:.2e}rad", flush=True)
                    col_out[robot] = collision_agreement(pb, mj, spec, args.collision_samples, seed=123)
                    c = col_out[robot]
                    print(f"[{robot}]  B/col proxy {c['proxy_col_pct']:.1f}%  "
                          f"PB {c['pb_col_pct']:.1f}%  MJ {_f(c['mj_col_pct'])}%  "
                          f"sign-agree {_f(c['pb_mj_sign_agree_pct'])}%", flush=True)
                    checkpoint()

            # cells for this arm
            for (rb, scenario, solver) in arm_work:
                # targets generated once per (arm, scenario, seed), shared across solvers.
                # NB: create ONE rng per seed and draw `trials` targets from it, so the
                # `trials` targets are DISTINCT. (Re-seeding default_rng(seed) inside the
                # inner loop would reset the stream every iteration and yield `trials`
                # identical targets — i.e. only one unique target per seed.)
                def _targets_for_seed(seed):
                    gen = np.random.default_rng(seed)
                    return [generate_target(spec, gen, scenario) for _ in range(args.trials)]
                targets_by_seed = {seed: _targets_for_seed(seed) for seed in args.seeds}
                disp = SOLVER_DISPLAY_NAMES.get(solver, "PyBullet native IK")
                t0 = time.perf_counter()
                try:
                    row = bench_cell(rb, scenario, solver, spec, pb, mj,
                                     targets_by_seed, args.seeds, args.warmup)
                except Exception as e:
                    traceback.print_exc()
                    row = {c: float("nan") for c in CSV_COLUMNS}
                    row.update({"robot": rb, "scenario": scenario, "solver": solver,
                                "display_name": disp, "n": 0, "status": f"ERROR: {e!r}"[:200]})
                rows.append(row)
                n_done += 1
                dt = time.perf_counter() - t0
                elapsed = time.perf_counter() - t_start
                eta = (elapsed / n_done) * (len(work) - n_done)
                print(f"[{n_done:>3}/{len(work)}] {rb:<12} {scenario:<13} {disp:<30} "
                      f"succ {_f(row['success_pct']):>5}  PBcol {_f(row['pb_collision_pct']):>5}  "
                      f"MJcol {_f(row['mj_collision_pct']):>5}  mean {_f(row['mean_ms']):>7}ms  "
                      f"[{dt:5.1f}s cell, ETA {eta/60:5.1f}m]", flush=True)
                checkpoint()  # crash-safe: persist after every cell
        finally:
            if mj is not None:
                mj.close()
            if pb is not None:
                pb.close()

    checkpoint(finished=True)

    # -- run manifest (config + environment + timing) --------------------
    total_s = time.perf_counter() - t_start
    manifest = {
        "finished_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "wall_seconds": round(total_s, 1),
        "config": {
            "trials": args.trials, "seeds": args.seeds, "warmup": args.warmup,
            "robots": args.robots, "scenarios": args.scenarios,
            "solvers": args.solvers, "skip_slow": args.skip_slow,
            "engines": engines, "native_baseline": (use_pb and not args.no_native),
            "fk_samples": args.fk_samples, "collision_samples": args.collision_samples,
        },
        "cells_total": total_cells, "cells_run_this_invocation": n_done,
        "n_per_cell": args.trials * len(args.seeds),
        "environment": {
            "python": platform.python_version(),
            "platform": platform.platform(),
            "numpy": np.__version__,
        },
    }
    try:
        import pybullet  # noqa
        manifest["environment"]["pybullet"] = getattr(pybullet, "getAPIVersion", lambda: "?")()
    except Exception:
        pass
    try:
        import mujoco  # noqa
        manifest["environment"]["mujoco"] = mujoco.__version__
    except Exception:
        pass
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)

    print(f"\nDone in {total_s/60:.1f} min. Wrote {csv_path}, {md_path}, {manifest_path} "
          f"({len(rows)} cells).", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
