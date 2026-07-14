// CCD (Cyclic Coordinate Descent) — generic-DOF native-C++ port of
// app/solvers/ccd.py (function solve_ccd), on the runtime `Robot` (dh_robot.hpp)
// so it runs on planar3dof / ur5 / franka_panda identically to the Python.
//
// FIDELITY: a 1:1 port of the DEFAULT Python path — base->tip sweeps, per-joint
// axis-projected position reach, plus the blended wrist orientation term
// (0.5 * orient_angle) on the last min(3, n//2) joints. Weights / tolerances /
// iteration budget / control flow are identical to the Python. CCD is fully
// deterministic (no RNG), so given identical (q0, T_target) this reproduces the
// Python's q_final / success / errors to floating-point tolerance — only the
// wall-clock timing differs (compiled vs interpreted).
#ifndef PIK_CCD_HPP
#define PIK_CCD_HPP

#include "dh_robot.hpp"
#include <set>
#include <cmath>
#include <algorithm>

namespace pik {

inline PikResult solveCCD(const Robot& s, const VecN& q0,
                          const Eigen::Matrix4d& Ttgt,
                          int max_iters = 300, double pos_tol = 1e-3,
                          double orient_tol = 1e-2) {
  const int n = s.n;
  const int off = s.modified ? 1 : 0;
  VecN q = q0;
  Eigen::Vector3d target_pos = Ttgt.block<3, 1>(0, 3);
  Eigen::Matrix3d target_rot = Ttgt.block<3, 3>(0, 0);

  // wrist count derived from DOF: 1 joint at n=3 up to 3 joints at n>=6.
  int n_wrist = (n >= 2) ? std::min(3, std::max(1, n / 2)) : 0;
  std::set<int> wrist;
  for (int i = n - n_wrist; i < n; ++i) wrist.insert(i);

  bool success = false;
  int it = 0;
  std::vector<Eigen::Matrix4d> f;

  for (it = 1; it <= max_iters; ++it) {
    // one full sweep = base -> tip, one joint rotation update each.
    for (int i = 0; i < n; ++i) {
      fkChain(s, q, f);  // re-evaluate after each joint update
      Eigen::Vector3d z_i = f[off + i].block<3, 1>(0, 2);
      Eigen::Vector3d p_i = f[off + i].block<3, 1>(0, 3);
      Eigen::Vector3d p_end = f[n].block<3, 1>(0, 3);
      Eigen::Matrix3d R_end = f[n].block<3, 3>(0, 0);

      // position term: vectors joint i -> EE and joint i -> target, projected
      // onto the plane perpendicular to z_i.
      Eigen::Vector3d v_end = p_end - p_i;
      Eigen::Vector3d v_target = target_pos - p_i;
      Eigen::Vector3d v_end_proj = v_end - v_end.dot(z_i) * z_i;
      Eigen::Vector3d v_target_proj = v_target - v_target.dot(z_i) * z_i;
      double n_end = v_end_proj.norm();
      double n_target = v_target_proj.norm();
      double pos_angle = 0.0;
      if (n_end > 1e-9 && n_target > 1e-9) {
        Eigen::Vector3d v_end_u = v_end_proj / n_end;
        Eigen::Vector3d v_target_u = v_target_proj / n_target;
        double cos_a = std::min(std::max(v_end_u.dot(v_target_u), -1.0), 1.0);
        double sin_a = v_end_u.cross(v_target_u).dot(z_i);
        pos_angle = std::atan2(sin_a, cos_a);
      }
      double angle = pos_angle;

      // orientation term, wrist joints only: reduce the relative-rotation
      // error's component along z_i, blended in (position dominant).
      if (wrist.count(i)) {
        Eigen::Matrix3d R_err = target_rot * R_end.transpose();
        double cos_t = std::min(std::max((R_err.trace() - 1.0) / 2.0, -1.0), 1.0);
        double theta = std::acos(cos_t);
        if (theta > 1e-8) {
          Eigen::Vector3d axis(R_err(2, 1) - R_err(1, 2),
                               R_err(0, 2) - R_err(2, 0),
                               R_err(1, 0) - R_err(0, 1));
          axis /= (2.0 * std::sin(theta));
          double orient_angle = axis.dot(z_i) * theta;
          angle = pos_angle + 0.5 * orient_angle;
        }
      }

      q[i] = std::min(std::max(q[i] + angle, s.lo[i]), s.hi[i]);
    }

    Vec6 err = poseError(eePose(s, q), Ttgt);
    double pos_e = err.head<3>().norm();
    double orient_e = err.tail<3>().norm();
    if (pos_e < pos_tol && orient_e < orient_tol) { success = true; break; }
  }
  if (it > max_iters) it = max_iters;  // loop-exit value matches Python's last `it`

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
