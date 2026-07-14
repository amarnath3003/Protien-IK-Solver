// FABRIK — generic-DOF native-C++ port of app/solvers/fabrik.py (function
// solve_fabrik), on the runtime `Robot` (dh_robot.hpp) so it runs on
// planar3dof / ur5 / franka_panda identically to the Python.
//
// FIDELITY: a 1:1 port of the Python's revolute-adapted FABRIK — alternating
// backward (tip->base) and forward (base->tip) reaching sweeps, each joint's
// reach projected onto its single DH rotation axis (the same axis projection
// CCD uses), with the wrist joints reserved for a dedicated per-iteration
// orientation-correction step (0.6 gain) and excluded from the position passes.
// Weights / tolerances / iteration budget / control flow are identical to the
// Python. FABRIK is fully deterministic (no RNG), so given identical
// (q0, T_target) this reproduces the Python's q_final / success / errors to
// floating-point tolerance — only the wall-clock timing differs.
#ifndef PIK_FABRIK_HPP
#define PIK_FABRIK_HPP

#include "dh_robot.hpp"
#include <set>
#include <cmath>
#include <algorithm>

namespace pik {

// Rotate joint i (about its own axis only) so the chain's tip points as closely
// as possible toward `desired`, as seen from joint i. Joints in `exclude` are
// left untouched. Mirrors fabrik._reach_joint_toward.
inline void fabrikReachJoint(const Robot& s, VecN& q, int i,
                             const Eigen::Vector3d& desired,
                             const std::set<int>& exclude) {
  if (exclude.count(i)) return;
  const int n = s.n;
  const int off = s.modified ? 1 : 0;
  std::vector<Eigen::Matrix4d> f; fkChain(s, q, f);
  Eigen::Vector3d z_i = f[off + i].block<3, 1>(0, 2);
  Eigen::Vector3d p_i = f[off + i].block<3, 1>(0, 3);
  Eigen::Vector3d p_end = f[n].block<3, 1>(0, 3);

  Eigen::Vector3d v_end = p_end - p_i;
  Eigen::Vector3d v_des = desired - p_i;
  Eigen::Vector3d v_end_proj = v_end - v_end.dot(z_i) * z_i;
  Eigen::Vector3d v_des_proj = v_des - v_des.dot(z_i) * z_i;
  double n_end = v_end_proj.norm();
  double n_des = v_des_proj.norm();
  if (n_end > 1e-9 && n_des > 1e-9) {
    Eigen::Vector3d v_end_u = v_end_proj / n_end;
    Eigen::Vector3d v_des_u = v_des_proj / n_des;
    double cos_a = std::min(std::max(v_end_u.dot(v_des_u), -1.0), 1.0);
    double sin_a = v_end_u.cross(v_des_u).dot(z_i);
    double angle = std::atan2(sin_a, cos_a);
    q[i] = std::min(std::max(q[i] + angle, s.lo[i]), s.hi[i]);
  }
}

inline PikResult solveFABRIK(const Robot& s, const VecN& q0,
                             const Eigen::Matrix4d& Ttgt,
                             int max_iters = 150, double pos_tol = 1e-3,
                             double orient_tol = 1e-2) {
  const int n = s.n;
  const int off = s.modified ? 1 : 0;
  VecN q = q0;
  Eigen::Vector3d target_pos = Ttgt.block<3, 1>(0, 3);
  Eigen::Matrix3d target_rot = Ttgt.block<3, 3>(0, 0);

  int n_wrist = (n >= 2) ? std::min(3, std::max(1, n / 2)) : 0;
  std::set<int> wrist;                       // std::set iterates ascending == sorted()
  for (int i = n - n_wrist; i < n; ++i) wrist.insert(i);

  bool success = false;
  int it = 0;
  std::vector<Eigen::Matrix4d> f;

  for (it = 1; it <= max_iters; ++it) {
    // wrist orientation nudge FIRST (small 0.6 step), recomputing the rotation
    // error fresh before each wrist joint (correcting one changes the EE
    // orientation the next is based on).
    for (int i : wrist) {
      fkChain(s, q, f);
      Eigen::Matrix3d R_end = f[n].block<3, 3>(0, 0);
      Eigen::Matrix3d R_err = target_rot * R_end.transpose();
      double cos_t = std::min(std::max((R_err.trace() - 1.0) / 2.0, -1.0), 1.0);
      double theta = std::acos(cos_t);
      if (theta > 1e-8) {
        Eigen::Vector3d axis(R_err(2, 1) - R_err(1, 2),
                             R_err(0, 2) - R_err(2, 0),
                             R_err(1, 0) - R_err(0, 1));
        axis /= (2.0 * std::sin(theta));
        Eigen::Vector3d z_i = f[off + i].block<3, 1>(0, 2);
        double orient_angle = axis.dot(z_i) * theta;
        q[i] = std::min(std::max(q[i] + 0.6 * orient_angle, s.lo[i]), s.hi[i]);
      }
    }

    // backward pass: tip -> base, each non-wrist joint reaches EE toward target.
    for (int i = n - 1; i >= 0; --i) fabrikReachJoint(s, q, i, target_pos, wrist);
    // forward pass: base -> tip, same reach primitive.
    for (int i = 0; i < n; ++i) fabrikReachJoint(s, q, i, target_pos, wrist);

    Vec6 err = poseError(eePose(s, q), Ttgt);
    double pos_e = err.head<3>().norm();
    double orient_e = err.tail<3>().norm();
    if (pos_e < pos_tol && orient_e < orient_tol) { success = true; break; }
  }
  if (it > max_iters) it = max_iters;

  PikResult R;
  Vec6 ef = poseError(eePose(s, q), Ttgt);
  R.success = success;
  R.q = q;
  R.pos_error = ef.head<3>().norm();
  R.orient_error = ef.tail<3>().norm();
  R.iterations = it;
  R.restarts = 0;
  R.min_self_distance = selfCollisionMinDist(s, q);
  return R;
}

}  // namespace pik
#endif
