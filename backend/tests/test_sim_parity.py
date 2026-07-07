"""
Phase-1 parity guard (sim_migration_plan.md §5-Phase-1): permanently pin the
DH <-> URDF forward-kinematics agreement so a future edit to a DH table or a
model swap can't silently break the "the model we solve is the model we
benchmark" guarantee.

These tests need a real simulator (PyBullet) and the robot models
(robot_descriptions). Both are optional deps that don't ship in the core
backend env (PyBullet has no Windows/Python-3.13 wheel — see
app/sim/README.md), so the whole module skips cleanly when they're absent. To
actually run it, use the sim venv:

    backend/.venv-sim/Scripts/python.exe -m pytest tests/test_sim_parity.py -v

What is asserted:
  * Phase-0: our hand-typed joint limits match the URDF — exactly for Panda
    (incl. the unusual always-negative joint-4 range), and no *genuine*
    disagreement for UR5 (we intentionally encode wider limits there).
  * Phase-1: for both arms, our DH EE pose agrees with the URDF EE link frame
    up to at most a CONSTANT tool transform — i.e. the per-config structural
    residual is < 1e-6. That is the real deliverable: the models are the same
    robot, differing only by a fixed frame convention the adapter can absorb.
"""

from __future__ import annotations

import numpy as np
import pytest

# robot_descriptions is pure-Python and resolves the URDFs (needed even for the
# joint-limit checks). PyBullet is only needed for the FK parity checks.
robot_descriptions = pytest.importorskip("robot_descriptions")

from app.core.kinematics import get_robot_spec
from app.sim.models import (
    validate_joint_limits, get_sim_model, resolve_urdf_path, urdf_movable_joints,
)


# ─── Phase 0: model / joint-limit validation (no simulator needed) ────────────

def test_panda_joint4_limit_matches_urdf():
    """The prime parity suspect (plan §0, risk #3): the always-negative elbow."""
    urdf = resolve_urdf_path("franka_panda")
    movable = urdf_movable_joints(urdf)
    revolute = [m for m in movable if m["type"] in ("revolute", "continuous")]
    assert len(revolute) == 7, "Panda URDF should expose 7 revolute arm joints"
    j4 = revolute[3]
    assert j4["name"] == "panda_joint4"
    assert j4["lower"] == pytest.approx(-3.0718, abs=1e-4)
    assert j4["upper"] == pytest.approx(-0.0698, abs=1e-4)

    # ...and it matches what we encoded in franka_panda_spec.
    spec = get_robot_spec("franka_panda")
    assert spec.joint_limits[3, 0] == pytest.approx(-3.0718, abs=1e-4)
    assert spec.joint_limits[3, 1] == pytest.approx(-0.0698, abs=1e-4)


def test_panda_all_joint_limits_match_urdf():
    """Every Panda joint limit must match the URDF (targets are only 'reachable'
    in-sim if the limits agree)."""
    check = validate_joint_limits("franka_panda", tol=1e-4)
    assert check.count_matches, check.summary()
    assert check.all_match, "Panda joint limits diverge from URDF:\n" + check.summary()


def test_ur5_joint_limits_no_genuine_disagreement():
    """UR5: we deliberately encode wider (+/-2pi) limits than the URDF. Assert
    there is no *genuine* disagreement — only the documented 'DH wider' case."""
    check = validate_joint_limits("ur5", tol=1e-4)
    assert check.count_matches, check.summary()
    genuine = [pj for pj in check.mismatches
               if pj["note"] != "DH wider than URDF (intentional; benign for FK)"]
    assert not genuine, "Unexpected UR5 limit disagreement:\n" + check.summary()


# ─── Phase 1: DH <-> PyBullet FK parity (needs PyBullet) ──────────────────────

pybullet = pytest.importorskip("pybullet")
from app.sim.parity import PyBulletFK, compute_parity, scan_ee_links, TOL_POS, TOL_ORIENT

# Enough configs to be convincing but keep the test fast; the CLI uses 10k.
_N = 1500


@pytest.fixture(scope="module", params=["ur5", "franka_panda"])
def parity_case(request):
    robot = request.param
    spec = get_robot_spec(robot)
    model = get_sim_model(robot)
    urdf = resolve_urdf_path(robot)
    oracle = PyBulletFK(urdf, base_link=model.base_link)
    yield robot, spec, model, oracle
    oracle.close()


def test_revolute_joint_count(parity_case):
    robot, spec, model, oracle = parity_case
    assert oracle.n_revolute() >= spec.n_joints, (
        f"{robot}: URDF has {oracle.n_revolute()} revolute joints, "
        f"DH spec needs {spec.n_joints}"
    )


def test_ur5_fk_parity_validated():
    """THE Phase-1 pass for UR5: our DH EE == URDF tool0 up to a CONSTANT base
    offset (the UR base/base_link 180 deg-Z flip), with negligible structural drift."""
    spec = get_robot_spec("ur5")
    model = get_sim_model("ur5")
    with PyBulletFK(resolve_urdf_path("ur5"), base_link=model.base_link) as oracle:
        ranked = scan_ee_links(spec, oracle, n_samples=200)
        assert ranked[0].ee_link == "tool0", "expected DH EE to land on tool0"
        res = compute_parity(spec, oracle, "tool0", _N)
    assert res.verdict == "constant_offset", res.summary()
    assert res.offset_side == "base", res.summary()
    assert res.const_offset_orient == pytest.approx(np.pi, abs=1e-4), res.summary()
    assert res.residual_max_pos < TOL_POS, res.summary()
    assert res.residual_max_orient < TOL_ORIENT, res.summary()


def test_panda_fk_parity_validated():
    """Phase-1 pass for Panda AFTER the modified-DH fix (kinematics.py now
    evaluates franka_panda_spec with dh_convention='modified'): our DH EE matches
    the URDF panda_link8 flange frame to <1e-6 with essentially IDENTITY offset."""
    spec = get_robot_spec("franka_panda")
    assert spec.dh_convention == "modified", "Panda must use modified DH"
    model = get_sim_model("franka_panda")
    with PyBulletFK(resolve_urdf_path("franka_panda"), base_link=model.base_link) as oracle:
        res = compute_parity(spec, oracle, "panda_link8", _N)
    assert res.verdict in ("exact", "constant_offset"), res.summary()
    assert res.const_offset_pos < 1e-4, res.summary()      # identity offset to link8
    assert res.const_offset_orient < 1e-4, res.summary()
    assert res.residual_max_pos < TOL_POS, res.summary()
    assert res.residual_max_orient < TOL_ORIENT, res.summary()


def test_panda_modified_dh_reconciles():
    """Positive proof of the Phase-1 root cause + fix: the SAME franka_panda_spec
    params, evaluated with a *modified*-DH (Craig) FK, match the URDF panda_link8
    frame exactly (identity base+tool offset). This is what fixing kinematics.py
    should reproduce."""
    spec = get_robot_spec("franka_panda")
    model = get_sim_model("franka_panda")

    def mdh(a_prev, alpha_prev, d, theta):
        ca, sa, ct, st = (np.cos(alpha_prev), np.sin(alpha_prev),
                          np.cos(theta), np.sin(theta))
        return np.array([[ct, -st, 0, a_prev],
                         [st * ca, ct * ca, -sa, -sa * d],
                         [st * sa, ct * sa, ca, ca * d],
                         [0, 0, 0, 1]])

    def panda_mdh_fk(q):
        T = np.eye(4)
        for i in range(spec.n_joints):
            T = T @ mdh(spec.a[i], spec.alpha[i], spec.d[i], q[i] + spec.theta_offset[i])
        return T

    rng = np.random.default_rng(0)
    with PyBulletFK(resolve_urdf_path("franka_panda"), base_link=model.base_link) as oracle:
        max_pos = max_orient = 0.0
        for _ in range(_N):
            q = spec.random_config(rng)
            T_sim = oracle.fk(q, "panda_link8")
            T_mdh = panda_mdh_fk(q)
            max_pos = max(max_pos, float(np.linalg.norm(T_mdh[:3, 3] - T_sim[:3, 3])))
            R = T_mdh[:3, :3].T @ T_sim[:3, :3]
            ang = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
            max_orient = max(max_orient, float(ang))

    assert max_pos < 1e-5, f"modified-DH pos mismatch {max_pos:.2e}"
    assert max_orient < 1e-5, f"modified-DH orient mismatch {max_orient:.2e}"


# ─── Phase 2: the PyBullet evaluation oracle (needs PyBullet) ─────────────────
#
# These guard the oracle that Phase-2 scoring rests on: the constant frame offset
# is really constant, DH->sim frame conversion round-trips, the oracle's FK agrees
# with our validated DH FK end to end, and PyBullet's own IK actually reaches the
# targets we hand it (so the baseline column is meaningful, not a broken strawman).

from app.core.kinematics import end_effector_pose
from app.sim.pybullet_backend import PyBulletBackend, OFFSET_RESIDUAL_TOL


@pytest.fixture(scope="module", params=["ur5", "franka_panda"])
def backend_case(request):
    with PyBulletBackend(request.param, verify_samples=300) as bk:
        yield request.param, bk


def test_backend_offset_is_constant(backend_case):
    """Construction measures the DH<->URDF offset and refuses to build if it isn't
    constant; assert the measured residual is comfortably within tolerance."""
    robot, bk = backend_case
    assert bk.offset_residual < OFFSET_RESIDUAL_TOL, (
        f"{robot}: offset residual {bk.offset_residual:.2e} not constant"
    )
    # Both arms' offset sides match what Phase-1 found.
    expected_side = {"ur5": "base", "franka_panda": "tool"}[robot]
    assert bk.offset_side == expected_side


def test_backend_fk_matches_dh_frame(backend_case):
    """The oracle's sim FK equals our DH FK pushed through the constant offset:
    fk(q) == dh_to_sim(end_effector_pose(q)) for random configs. This is the whole
    basis of scoring solutions in the sim frame."""
    robot, bk = backend_case
    rng = np.random.default_rng(7)
    max_pos = max_orient = 0.0
    for _ in range(400):
        q = bk.spec.random_config(rng)
        T_sim = bk.fk(q)
        T_conv = bk.dh_to_sim(end_effector_pose(bk.spec, q))
        max_pos = max(max_pos, float(np.linalg.norm(T_sim[:3, 3] - T_conv[:3, 3])))
        R = T_conv[:3, :3].T @ T_sim[:3, :3]
        max_orient = max(max_orient, float(np.arccos(
            np.clip((np.trace(R) - 1) / 2, -1, 1))))
    assert max_pos < 1e-5, f"{robot}: sim/DH frame drift {max_pos:.2e} m"
    assert max_orient < 1e-5, f"{robot}: sim/DH orient drift {max_orient:.2e} rad"


def test_backend_scores_reachable_config_success(backend_case):
    """A config scored against its OWN DH-frame EE pose must be a sim success with
    ~0 error — proves the score() path (frame convert + sim FK + tolerances) is
    self-consistent, not accidentally passing/failing everything."""
    robot, bk = backend_case
    rng = np.random.default_rng(11)
    for _ in range(50):
        q_seed, T_dh = bk.reachable_target(rng)
        sc = bk.score(q_seed, T_dh)
        assert sc.sim_success, f"{robot}: self-consistent config scored as failure"
        assert sc.sim_pos_error < 1e-4
        # in_collision flag must agree with the sign of the reported clearance.
        assert sc.sim_in_collision == (sc.sim_min_self_distance < 0.0)


def test_backend_native_ik_reaches_targets(backend_case):
    """PyBullet's own IK, given reachable sim-frame targets, must succeed on the
    clear majority — otherwise the baseline column would be an unfair strawman."""
    robot, bk = backend_case
    rng = np.random.default_rng(5)
    n_ok = 0
    N = 30
    for _ in range(N):
        q_seed, T_dh = bk.reachable_target(rng)
        q_ik = bk.native_ik(bk.dh_to_sim(T_dh), np.zeros(bk.spec.n_joints))
        n_ok += int(bk.score(q_ik, T_dh).sim_success)
    assert n_ok >= int(0.6 * N), f"{robot}: native IK only {n_ok}/{N} — baseline broken"


# ─── Phase 3: clean-solve (real-collision-certified selection) ────────────────

from app.sim.clean_solve import solve_clean


def test_clean_solve_never_worse_than_single_shot():
    """`solve_clean` selects the max-real-clearance candidate, so its returned
    clearance must be >= the single-shot (candidate-0) clearance on every target —
    selection can only help. Also checks bookkeeping (n_candidates ≤ K, clean count
    consistent)."""
    robot = "ur5"
    spec = get_robot_spec(robot)
    K = 8
    with PyBulletBackend(robot) as bk:
        for scen_seed in range(15):
            rng = np.random.default_rng(scen_seed)
            q0 = spec.random_config(rng)
            T = end_effector_pose(spec, spec.random_config(rng))
            cs = solve_clean(bk, "protein_fast", spec, q0, T, K=K, seed=scen_seed)
            if cs.result is None:
                continue
            assert cs.n_candidates <= K
            assert 0 <= cs.n_collision_free <= cs.n_candidates
            if cs.single_success:
                # selected clearance is never below the single-shot's
                assert cs.sim_min_self_distance >= cs.single_sim_min_self_distance - 1e-9


def test_clean_solve_reduces_collision_rate():
    """The whole point: over a batch, clean-solve's real-collision rate must be
    <= the single-shot rate (usually strictly lower). Uses open_space where a
    collision-free branch almost always exists."""
    from app.api.scenarios import generate_target
    robot = "ur5"
    spec = get_robot_spec(robot)
    single_col = clean_col = n = 0
    with PyBulletBackend(robot) as bk:
        gen = np.random.default_rng(3)
        targets = [generate_target(spec, gen, "open_space") for _ in range(25)]
        for ti, (q0, T) in enumerate(targets):
            cs = solve_clean(bk, "protein_fast", spec, q0, T, K=12, seed=ti)
            if cs.result is None or not cs.single_success:
                continue
            n += 1
            single_col += int(cs.single_in_collision)
            clean_col += int(cs.sim_in_collision)
    assert n >= 10
    assert clean_col <= single_col, (
        f"clean-solve did not reduce collision: single={single_col}/{n} "
        f"clean={clean_col}/{n}")


# ─── Phase 4: MuJoCo second oracle (needs mujoco) ─────────────────────────────
#
# The point of a second simulator is that it can DISAGREE. These guard that it
# does not: on the identical URDF, MuJoCo's FK matches both our DH and PyBullet to
# float noise (three-way agreement), and its real-mesh self-collision agrees with
# PyBullet's collide/clear verdict on the same matched link pairs. If either ever
# broke, a Phase-2/Phase-3 finding would no longer be engine-independent.

mujoco = pytest.importorskip("mujoco")
from app.sim.mujoco_backend import MuJoCoBackend, OFFSET_RESIDUAL_TOL as MJ_OFFSET_TOL


@pytest.fixture(scope="module", params=["ur5", "franka_panda"])
def mj_backend_case(request):
    with MuJoCoBackend(request.param, verify_samples=500) as bk:
        yield request.param, bk


def test_mujoco_offset_matches_pybullet_parity(mj_backend_case):
    """MuJoCo re-derives the SAME constant DH<->URDF offset PyBullet did, to a tiny
    residual: UR5 a base Rz(180 deg); Panda an identity tool offset. Independent
    confirmation of the (corrected modified-DH) model on a second engine."""
    robot, bk = mj_backend_case
    assert bk.offset_residual < MJ_OFFSET_TOL, (
        f"{robot}: MuJoCo offset residual {bk.offset_residual:.2e} not constant")
    expected_side = {"ur5": "base", "franka_panda": "tool"}[robot]
    assert bk.offset_side == expected_side
    if robot == "ur5":
        assert bk.parity.const_offset_orient == pytest.approx(np.pi, abs=1e-4)
    else:
        assert bk.parity.const_offset_pos < 1e-4
        assert bk.parity.const_offset_orient < 1e-4


def test_mujoco_fk_matches_dh_frame(mj_backend_case):
    """MuJoCo's link-frame FK equals our DH FK through the constant offset:
    fk(q) == dh_to_sim(end_effector_pose(q))."""
    robot, bk = mj_backend_case
    rng = np.random.default_rng(7)
    max_pos = max_orient = 0.0
    for _ in range(400):
        q = bk.spec.random_config(rng)
        T_sim = bk.fk(q)
        T_conv = bk.dh_to_sim(end_effector_pose(bk.spec, q))
        max_pos = max(max_pos, float(np.linalg.norm(T_sim[:3, 3] - T_conv[:3, 3])))
        R = T_conv[:3, :3].T @ T_sim[:3, :3]
        max_orient = max(max_orient, float(np.arccos(
            np.clip((np.trace(R) - 1) / 2, -1, 1))))
    assert max_pos < 1e-5, f"{robot}: MuJoCo/DH frame drift {max_pos:.2e} m"
    assert max_orient < 1e-5, f"{robot}: MuJoCo/DH orient drift {max_orient:.2e} rad"


def test_mujoco_fk_matches_pybullet_fk():
    """Direct engine-vs-engine FK: on the identical URDF, PyBullet and MuJoCo put
    the EE link frame in the same world pose to float noise. This is the explicit
    third leg of the three-way (DH/PyBullet/MuJoCo) agreement."""
    for robot in ("ur5", "franka_panda"):
        spec = get_robot_spec(robot)
        with PyBulletBackend(robot) as pb, MuJoCoBackend(robot) as mj:
            rng = np.random.default_rng(13)
            max_pos = max_orient = 0.0
            for _ in range(500):
                q = spec.random_config(rng)
                Tp, Tm = pb.fk(q), mj.fk(q)
                max_pos = max(max_pos, float(np.linalg.norm(Tp[:3, 3] - Tm[:3, 3])))
                R = Tp[:3, :3].T @ Tm[:3, :3]
                max_orient = max(max_orient, float(np.arccos(
                    np.clip((np.trace(R) - 1) / 2, -1, 1))))
        assert max_pos < 1e-5, f"{robot}: PyBullet/MuJoCo FK pos gap {max_pos:.2e} m"
        assert max_orient < 1e-4, f"{robot}: PyBullet/MuJoCo FK orient gap {max_orient:.2e} rad"


def test_mujoco_collision_agrees_with_pybullet():
    """On the SAME matched link pairs, MuJoCo and PyBullet agree on the collide/clear
    call for the large majority of random configs (two independent real-mesh
    implementations; residual disagreement is near-boundary convex-hull noise). This
    is what makes the Phase-3 'proxy is optimistic' result engine-independent."""
    for robot in ("ur5", "franka_panda"):
        spec = get_robot_spec(robot)
        with PyBulletBackend(robot) as pb, \
                MuJoCoBackend(robot, collision_link_names=pb.collision_link_names) as mj:
            # the meaningful pairs MuJoCo checks must be a subset of PyBullet's
            mj_pairs = {tuple(sorted((p.name_a, p.name_b))) for p in mj._collision_pairs}
            pb_pairs = {tuple(sorted((pb._idx_to_name[a], pb._idx_to_name[b])))
                        for (a, b) in pb._collision_pairs}
            assert mj_pairs.issubset(pb_pairs), f"{robot}: pair sets diverge"

            rng = np.random.default_rng(123)
            agree = n = 0
            for _ in range(400):
                q = spec.random_config(rng)
                pc = pb.self_collision(q, threshold=0.8)[0]
                mc = mj.self_collision(q, threshold=0.8)[0]
                agree += int(pc == mc)
                n += 1
        assert agree / n >= 0.9, f"{robot}: engines agree on only {agree}/{n} collide/clear calls"
