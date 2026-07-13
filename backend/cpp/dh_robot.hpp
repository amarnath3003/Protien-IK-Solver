// Generic runtime-DH kinematics + the ProteinIK energy stack, in native
// C++/Eigen. Generalizes the UR5-only ur5_dh.hpp to ANY serial revolute arm
// (variable DOF, standard OR modified/Craig DH) so every ProteinIK C++ solver
// runs on planar3dof / ur5 / franka_panda identically to the Python.
//
// Ported 1:1 from app/core/kinematics.py + app/solvers/protein_energy.py:
//   - standard vs modified DH link transforms (_dh_transform / _mdh_transform)
//   - joint-axis frame offset for the Jacobian (0 standard, 1 modified)
//   - capsule self-collision with the degenerate-segment skip (Franka)
//   - joint-limit / collision / smoothness / neutral / target energies
//   - frustration_index (chaperone-rescue targeting)
#ifndef DH_ROBOT_HPP
#define DH_ROBOT_HPP

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <random>

namespace pik {

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using VecN = Eigen::VectorXd;
using Jac  = Eigen::Matrix<double, 6, Eigen::Dynamic>;

// Runtime robot spec — mirrors app.core.kinematics.RobotSpec.
struct Robot {
  int n = 0;
  std::vector<double> a, d, alpha, toff, lo, hi, radius;
  bool modified = false;  // dh_convention == "modified" (Craig)

  VecN clip(const VecN& q) const {
    VecN o = q;
    for (int i = 0; i < n; ++i) o[i] = std::min(std::max(q[i], lo[i]), hi[i]);
    return o;
  }
};

// Uniform result struct returned by every ProteinIK C++ solver (extra
// diagnostic fields default to 0 for solvers that don't produce them).
struct PikResult {
  bool success = false;
  VecN q;
  double pos_error = 0, orient_error = 0, min_self_distance = 0;
  int iterations = 0, restarts = 0;
  double conflict_index = 0.0, lambda_final = 0.0;
};

// RNG mirroring the numpy Generator surface the Python solvers use. (Stream
// differs from numpy PCG64 — trajectories differ, distributions match.)
struct Rng {
  std::mt19937_64 gen;
  explicit Rng(uint64_t seed) : gen(seed) {}
  double uniform01() { return std::uniform_real_distribution<double>(0.0, 1.0)(gen); }
  double uniform(double a, double b) { return std::uniform_real_distribution<double>(a, b)(gen); }
  double normal(double mu, double sd) { return std::normal_distribution<double>(mu, sd)(gen); }
  VecN randomConfig(const Robot& s) {
    VecN q(s.n);
    for (int i = 0; i < s.n; ++i) q[i] = uniform(s.lo[i], s.hi[i]);
    return q;
  }
};

// ---- DH link transform (standard or modified) ------------------------------
inline Eigen::Matrix4d dhLocal(double theta, double d, double a, double alpha, bool modified) {
  double ct = std::cos(theta), st = std::sin(theta);
  double ca = std::cos(alpha), sa = std::sin(alpha);
  Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
  T(3, 3) = 1.0;
  if (modified) {  // Rot_x(alpha) Trans_x(a) Rot_z(theta) Trans_z(d)
    T(0, 0) = ct;      T(0, 1) = -st;     T(0, 2) = 0.0;  T(0, 3) = a;
    T(1, 0) = st * ca; T(1, 1) = ct * ca; T(1, 2) = -sa;  T(1, 3) = -sa * d;
    T(2, 0) = st * sa; T(2, 1) = ct * sa; T(2, 2) = ca;   T(2, 3) = ca * d;
  } else {           // Rot_z(theta) Trans_z(d) Trans_x(a) Rot_x(alpha)
    T(0, 0) = ct; T(0, 1) = -st * ca; T(0, 2) = st * sa;  T(0, 3) = a * ct;
    T(1, 0) = st; T(1, 1) = ct * ca;  T(1, 2) = -ct * sa; T(1, 3) = a * st;
    T(2, 0) = 0;  T(2, 1) = sa;       T(2, 2) = ca;       T(2, 3) = d;
  }
  return T;
}

// full chain: frames[0..n] (base=I .. EE); frames[i] = joint i origin frame.
inline void fkChain(const Robot& s, const VecN& q, std::vector<Eigen::Matrix4d>& f) {
  f.resize(s.n + 1);
  f[0] = Eigen::Matrix4d::Identity();
  for (int i = 0; i < s.n; ++i)
    f[i + 1] = f[i] * dhLocal(q[i] + s.toff[i], s.d[i], s.a[i], s.alpha[i], s.modified);
}

inline Eigen::Matrix4d eePose(const Robot& s, const VecN& q) {
  std::vector<Eigen::Matrix4d> f; fkChain(s, q, f); return f[s.n];
}

// geometric Jacobian; joint i axis lives in frame chain[off+i] (off=1 modified).
inline Jac geoJac(const Robot& s, const std::vector<Eigen::Matrix4d>& f) {
  int off = s.modified ? 1 : 0;
  Jac J(6, s.n);
  Eigen::Vector3d p_end = f[s.n].block<3, 1>(0, 3);
  for (int i = 0; i < s.n; ++i) {
    Eigen::Vector3d z = f[off + i].block<3, 1>(0, 2);
    Eigen::Vector3d p = f[off + i].block<3, 1>(0, 3);
    J.block<3, 1>(0, i) = z.cross(p_end - p);
    J.block<3, 1>(3, i) = z;
  }
  return J;
}

inline Vec6 poseError(const Eigen::Matrix4d& Tcur, const Eigen::Matrix4d& Ttgt) {
  Vec6 e;
  e.head<3>() = Ttgt.block<3, 1>(0, 3) - Tcur.block<3, 1>(0, 3);
  Eigen::Matrix3d Rerr = Ttgt.block<3, 3>(0, 0) * Tcur.block<3, 3>(0, 0).transpose();
  double cos_t = std::min(std::max((Rerr.trace() - 1.0) / 2.0, -1.0), 1.0);
  double theta = std::acos(cos_t);
  if (theta < 1e-8) { e.tail<3>().setZero(); }
  else {
    Eigen::Vector3d ax(Rerr(2, 1) - Rerr(1, 2), Rerr(0, 2) - Rerr(2, 0), Rerr(1, 0) - Rerr(0, 1));
    ax /= (2.0 * std::sin(theta));
    e.tail<3>() = ax * theta;
  }
  return e;
}

// ---- capsule self-collision (self_collision_min_distance_from_chain) -------
inline double segSegDist(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                         const Eigen::Vector3d& p3, const Eigen::Vector3d& p4) {
  Eigen::Vector3d d1 = p2 - p1, d2 = p4 - p3, r = p1 - p3;
  double a = d1.dot(d1), e = d2.dot(d2), f = d2.dot(r);
  const double eps = 1e-12;
  double s, t;
  if (a <= eps && e <= eps) return (p1 - p3).norm();
  if (a <= eps) { s = 0.0; t = std::min(std::max(f / e, 0.0), 1.0); }
  else {
    double c = d1.dot(r);
    if (e <= eps) { t = 0.0; s = std::min(std::max(-c / a, 0.0), 1.0); }
    else {
      double b = d1.dot(d2), denom = a * e - b * b;
      s = denom > eps ? std::min(std::max((b * f - c * e) / denom, 0.0), 1.0) : 0.0;
      t = (b * s + f) / e;
      if (t < 0.0)      { t = 0.0; s = std::min(std::max(-c / a, 0.0), 1.0); }
      else if (t > 1.0) { t = 1.0; s = std::min(std::max((b - c) / a, 0.0), 1.0); }
    }
  }
  Eigen::Vector3d c1 = p1 + d1 * s, c2 = p3 + d2 * t;
  return (c1 - c2).norm();
}

inline double selfCollisionMinDist(const Robot& s, const std::vector<Eigen::Matrix4d>& f) {
  const int nl = s.n;
  if (nl < 3) return 1.0;  // no non-adjacent pairs possible
  std::vector<Eigen::Vector3d> pts(nl + 1);
  for (int i = 0; i <= nl; ++i) pts[i] = f[i].block<3, 1>(0, 3);
  const double EPS2 = 1e-12;
  double min_d = std::numeric_limits<double>::infinity();
  for (int i = 0; i < nl - 1; ++i) {
    for (int j = i + 2; j < nl; ++j) {
      if ((pts[i + 1] - pts[j]).squaredNorm() < EPS2) continue;  // degenerate skip
      double dd = segSegDist(pts[i], pts[i + 1], pts[j], pts[j + 1]) - (s.radius[i] + s.radius[j]);
      if (dd < min_d) min_d = dd;
    }
  }
  return min_d;
}

inline double selfCollisionMinDist(const Robot& s, const VecN& q) {
  std::vector<Eigen::Matrix4d> f; fkChain(s, q, f);
  return selfCollisionMinDist(s, f);
}

// ---- energy terms (protein_energy.py) --------------------------------------
inline double jointLimitEnergy(const Robot& s, const VecN& q) {
  double sum = 0.0; const double margin = 0.05;
  for (int i = 0; i < s.n; ++i) {
    double frac = (q[i] - s.lo[i]) / (s.hi[i] - s.lo[i]);
    if (frac < margin)          sum += (margin - frac) * (margin - frac);
    else if (frac > 1 - margin) sum += (frac - (1 - margin)) * (frac - (1 - margin));
  }
  return sum * 50.0;
}

inline double collisionEnergyFromDist(double d_min) {
  if (d_min <= 0) return 100.0 + std::abs(d_min) * 100.0;
  const double safe = 0.05;
  if (d_min >= safe) return 0.0;
  double x = (safe - d_min) / safe;
  return x * x * 10.0;
}

inline double smoothnessEnergy(const VecN& q) {
  double sum = 0.0;
  for (int i = 0; i + 1 < q.size(); ++i) { double dd = q[i + 1] - q[i]; sum += dd * dd; }
  return sum * 0.5;
}

inline double neutralPoseEnergy(const VecN& q, const VecN& q_neutral) {
  return 0.5 * (q - q_neutral).squaredNorm();
}

// target_energy: ||pos|| + 0.3||orient||
inline double targetEnergy(const Robot& s, const VecN& q, const Eigen::Matrix4d& Ttgt) {
  Vec6 e = poseError(eePose(s, q), Ttgt);
  return e.head<3>().norm() + 0.3 * e.tail<3>().norm();
}

// total_energy_fast with weights (w_target,w_limit,w_collision,w_smooth)
inline double totalEnergy(const Robot& s, const VecN& q, const Eigen::Matrix4d& Ttgt,
                          double wt, double wl, double wc, double ws) {
  std::vector<Eigen::Matrix4d> f; fkChain(s, q, f);
  double e = 0.0;
  if (wt > 0) {
    Vec6 err = poseError(f[s.n], Ttgt);
    e += wt * (err.head<3>().norm() + 0.3 * err.tail<3>().norm());
  }
  if (wl > 0) e += wl * jointLimitEnergy(s, q);
  if (wc > 0) e += wc * collisionEnergyFromDist(selfCollisionMinDist(s, f));
  if (ws > 0) e += ws * smoothnessEnergy(q);
  return e;
}

// frustration_index() -> per-joint |local_pref - global_need|
inline VecN frustrationIndex(const Robot& s, const VecN& q, const Eigen::Matrix4d& Ttgt) {
  const int n = s.n;
  std::vector<Eigen::Matrix4d> f; fkChain(s, q, f);
  Vec6 err = poseError(f[n], Ttgt);
  Jac J = geoJac(s, f);
  VecN global_dq = J.transpose() * err;
  VecN q_global = q + global_dq;
  VecN q_local(n);
  for (int i = 0; i < n; ++i) {
    if (i == 0) q_local[i] = q[1];
    else if (i == n - 1) q_local[i] = q[n - 2];
    else q_local[i] = (q[i - 1] + q[i + 1]) / 2.0;
  }
  return (q_local - q_global).cwiseAbs();
}

// pose + Jacobian in one FK pass (protein_fast._fast_pose_jac)
inline void poseJac(const Robot& s, const VecN& q, Eigen::Matrix4d& pose, Jac& J) {
  std::vector<Eigen::Matrix4d> f; fkChain(s, q, f);
  pose = f[s.n]; J = geoJac(s, f);
}

// DLS step: dq = J^T (J J^T + lam^2 I)^-1 err   (6x6 solve, any DOF)
inline VecN dlsStep(const Jac& J, const Vec6& err, double lam) {
  Mat6 A = J * J.transpose() + (lam * lam) * Mat6::Identity();
  return J.transpose() * A.ldlt().solve(err);
}

}  // namespace pik
#endif
