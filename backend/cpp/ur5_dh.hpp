// UR5 kinematics + ProteinIK energy terms, ported 1:1 from the Python
// (app/core/kinematics.py + app/solvers/protein_energy.py) using Eigen.
// Standard-DH FK, geometric Jacobian, axis-angle pose error, capsule
// self-collision, and the V4 energy stack. Values match ur5_spec() exactly.
#ifndef UR5_DH_HPP
#define UR5_DH_HPP

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <vector>
#include <string>
#include <sstream>

namespace pik {

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using VecN = Eigen::VectorXd;

// ---- UR5 standard-DH spec (identical to ur5_spec()) -------------------------
struct Ur5 {
  static constexpr int N = 6;
  // a, d, alpha, theta_offset
  std::array<double, 6> a     {{0.0, -0.42500, -0.39225, 0.0, 0.0, 0.0}};
  std::array<double, 6> d     {{0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823}};
  std::array<double, 6> alpha {{M_PI/2, 0.0, 0.0, M_PI/2, -M_PI/2, 0.0}};
  std::array<double, 6> toff  {{0,0,0,0,0,0}};
  std::array<double, 6> lo    {{-2*M_PI,-2*M_PI,-2*M_PI,-2*M_PI,-2*M_PI,-2*M_PI}};
  std::array<double, 6> hi    {{ 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI}};
  std::array<double, 6> radius{{0.06, 0.05, 0.045, 0.04, 0.04, 0.035}};

  VecN clip(const VecN& q) const {
    VecN o = q;
    for (int i = 0; i < N; ++i) o[i] = std::min(std::max(q[i], lo[i]), hi[i]);
    return o;
  }
};

// standard-DH local transform: Rz(theta) Tz(d) Tx(a) Rx(alpha)
inline Eigen::Matrix4d dhLocal(double theta, double d, double a, double alpha) {
  double ct = std::cos(theta), st = std::sin(theta);
  double ca = std::cos(alpha), sa = std::sin(alpha);
  Eigen::Matrix4d T;
  T << ct, -st*ca,  st*sa, a*ct,
       st,  ct*ca, -ct*sa, a*st,
      0.0,     sa,     ca,    d,
      0.0,    0.0,    0.0,  1.0;
  return T;
}

// full chain: returns frames[0..N] (base=I .. EE). frames[i] is joint i origin.
inline void fkChain(const Ur5& s, const VecN& q,
                    std::array<Eigen::Matrix4d, Ur5::N + 1>& frames) {
  frames[0] = Eigen::Matrix4d::Identity();
  for (int i = 0; i < Ur5::N; ++i)
    frames[i+1] = frames[i] * dhLocal(q[i] + s.toff[i], s.d[i], s.a[i], s.alpha[i]);
}

inline Eigen::Matrix4d eePose(const Ur5& s, const VecN& q) {
  std::array<Eigen::Matrix4d, Ur5::N + 1> f;
  fkChain(s, q, f);
  return f[Ur5::N];
}

// geometric Jacobian (standard DH, axis-offset 0): J_v = z x (p_end - p_i), J_w = z
inline Mat6 geoJac(const Ur5& s, const std::array<Eigen::Matrix4d, Ur5::N+1>& f) {
  Mat6 J;
  Eigen::Vector3d p_end = f[Ur5::N].block<3,1>(0,3);
  for (int i = 0; i < Ur5::N; ++i) {
    Eigen::Vector3d z = f[i].block<3,1>(0,2);
    Eigen::Vector3d p = f[i].block<3,1>(0,3);
    Eigen::Vector3d jv = z.cross(p_end - p);
    J.block<3,1>(0,i) = jv;
    J.block<3,1>(3,i) = z;
  }
  return J;
}

// axis-angle pose error [p_err(3), o_err(3)]  (matches pose_error())
inline Vec6 poseError(const Eigen::Matrix4d& Tcur, const Eigen::Matrix4d& Ttgt) {
  Vec6 e;
  e.head<3>() = Ttgt.block<3,1>(0,3) - Tcur.block<3,1>(0,3);
  Eigen::Matrix3d Rerr = Ttgt.block<3,3>(0,0) * Tcur.block<3,3>(0,0).transpose();
  double cos_t = std::min(std::max((Rerr.trace() - 1.0) / 2.0, -1.0), 1.0);
  double theta = std::acos(cos_t);
  if (theta < 1e-8) { e.tail<3>().setZero(); }
  else {
    Eigen::Vector3d ax(Rerr(2,1)-Rerr(1,2), Rerr(0,2)-Rerr(2,0), Rerr(1,0)-Rerr(0,1));
    ax /= (2.0 * std::sin(theta));
    e.tail<3>() = ax * theta;
  }
  return e;
}

// ---- capsule self-collision (ports self_collision_min_distance_from_chain) --
inline double segSegDist(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                         const Eigen::Vector3d& p3, const Eigen::Vector3d& p4) {
  Eigen::Vector3d d1 = p2 - p1, d2 = p4 - p3, r = p1 - p3;
  double a = d1.dot(d1), e = d2.dot(d2), f = d2.dot(r);
  const double eps = 1e-12;
  double s, t;
  if (a <= eps && e <= eps) return (p1 - p3).norm();
  if (a <= eps) { s = 0.0; t = std::min(std::max(f/e, 0.0), 1.0); }
  else {
    double c = d1.dot(r);
    if (e <= eps) { t = 0.0; s = std::min(std::max(-c/a, 0.0), 1.0); }
    else {
      double b = d1.dot(d2), denom = a*e - b*b;
      s = denom > eps ? std::min(std::max((b*f - c*e)/denom, 0.0), 1.0) : 0.0;
      t = (b*s + f)/e;
      if (t < 0.0)      { t = 0.0; s = std::min(std::max(-c/a, 0.0), 1.0); }
      else if (t > 1.0) { t = 1.0; s = std::min(std::max((b - c)/a, 0.0), 1.0); }
    }
  }
  Eigen::Vector3d c1 = p1 + d1*s, c2 = p3 + d2*t;
  return (c1 - c2).norm();
}

inline double selfCollisionMinDist(const Ur5& s,
                                   const std::array<Eigen::Matrix4d, Ur5::N+1>& f) {
  std::array<Eigen::Vector3d, Ur5::N+1> pts;
  for (int i = 0; i <= Ur5::N; ++i) pts[i] = f[i].block<3,1>(0,3);
  const int nl = Ur5::N;
  const double EPS2 = 1e-12;
  double min_d = std::numeric_limits<double>::infinity();
  for (int i = 0; i < nl - 1; ++i) {
    for (int j = i + 2; j < nl; ++j) {
      if ((pts[i+1] - pts[j]).squaredNorm() < EPS2) continue;  // degenerate skip
      double dd = segSegDist(pts[i], pts[i+1], pts[j], pts[j+1]) - (s.radius[i] + s.radius[j]);
      if (dd < min_d) min_d = dd;
    }
  }
  return min_d;
}

inline double selfCollisionMinDist(const Ur5& s, const VecN& q) {
  std::array<Eigen::Matrix4d, Ur5::N+1> f; fkChain(s, q, f);
  return selfCollisionMinDist(s, f);
}

// ---- energy terms (protein_energy.py) --------------------------------------
inline double jointLimitEnergy(const Ur5& s, const VecN& q) {
  double sum = 0.0; const double margin = 0.05;
  for (int i = 0; i < Ur5::N; ++i) {
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
  for (int i = 0; i + 1 < Ur5::N; ++i) { double dd = q[i+1] - q[i]; sum += dd*dd; }
  return sum * 0.5;
}

// total_energy_fast with the V4 weights _W=(3,1,2,0.3)
inline double totalEnergy(const Ur5& s, const VecN& q, const Eigen::Matrix4d& Ttgt,
                          double wt, double wl, double wc, double ws) {
  std::array<Eigen::Matrix4d, Ur5::N+1> f; fkChain(s, q, f);
  double e = 0.0;
  if (wt > 0) {
    Vec6 err = poseError(f[Ur5::N], Ttgt);
    e += wt * (err.head<3>().norm() + 0.3 * err.tail<3>().norm());
  }
  if (wl > 0) e += wl * jointLimitEnergy(s, q);
  if (wc > 0) e += wc * collisionEnergyFromDist(selfCollisionMinDist(s, f));
  if (ws > 0) e += ws * smoothnessEnergy(q);
  return e;
}

// frustration_index() -> per-joint |local_pref - global_need|
inline VecN frustrationIndex(const Ur5& s, const VecN& q, const Eigen::Matrix4d& Ttgt) {
  const int n = Ur5::N;
  std::array<Eigen::Matrix4d, Ur5::N+1> f; fkChain(s, q, f);
  Vec6 err = poseError(f[n], Ttgt);
  Mat6 J = geoJac(s, f);
  VecN global_dq = J.transpose() * err;
  VecN q_global = q + global_dq;
  VecN q_local(n);
  for (int i = 0; i < n; ++i) {
    if (i == 0) q_local[i] = q[1];
    else if (i == n-1) q_local[i] = q[n-2];
    else q_local[i] = (q[i-1] + q[i+1]) / 2.0;
  }
  return (q_local - q_global).cwiseAbs();
}

}  // namespace pik
#endif
