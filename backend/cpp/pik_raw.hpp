// ProteinIK V6 — "Raw Biology" — generic-DOF port of app/solvers/protein_raw/*.
// A coarse-grained implicit-solvent protein-folding simulation whose polymer is
// the robot arm. It minimises the FREE ENERGY
//
//     F(q; T) = E_task + E_LJ + E_HB − T · S_conf(q)
//
// by overdamped Langevin dynamics, cooling from an unfolded high temperature
// toward the REM glass temperature T_glass, then consolidates to the native
// (target) minimum by a damped-Newton T→0 limit and quality-selects the
// lowest-enthalpy CLASH-FREE basin (Anfinsen).
//
// This solver carries its OWN energy model (protein_raw/energy.py) and landscape
// analysis (protein_raw/landscape.py); ONLY raw kinematics (FK chain, jacobian,
// pose error, capsule self-distance, DLS step, target_energy) come from
// dh_robot.hpp. Weights / cutoffs / schedules / budgets are identical to the
// Python; only the RNG stream differs (mt19937_64 vs numpy PCG64) — so the
// stochastic dynamics match STATISTICALLY (same order+count of draws), not
// bit-for-bit. The fixed entropy stencil is likewise reconstructed faithfully
// (common-random-number cloud of m×n normals) but with different actual values.
#ifndef PIK_RAW_HPP
#define PIK_RAW_HPP

#include "dh_robot.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

namespace pik {

static const double RAW_WELL     = std::pow(2.0, 1.0 / 6.0);  // 2^(1/6): LJ well location
static const double RAW_ORIENT_W = 0.3;                       // orientation weight

// ---------------------------------------------------------------------------
// small numeric helpers (numpy parity: population std, median-of-sorted)
// ---------------------------------------------------------------------------
inline double rawMean(const std::vector<double>& v) {
  double s = 0.0; for (double x : v) s += x; return v.empty() ? 0.0 : s / v.size();
}
inline double rawStd(const std::vector<double>& v) {  // np.std, ddof=0
  if (v.empty()) return 0.0;
  double mu = rawMean(v), s = 0.0;
  for (double x : v) { double dd = x - mu; s += dd * dd; }
  return std::sqrt(s / v.size());
}
inline double rawMedian(std::vector<double> v) {  // np.median
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  size_t m = v.size();
  return (m % 2 == 1) ? v[m / 2] : 0.5 * (v[m / 2 - 1] + v[m / 2]);
}
inline double sigmoidClip(double x) {  // _sigmoid: 1/(1+exp(-clip(x,-60,60)))
  double xc = std::min(std::max(x, -60.0), 60.0);
  return 1.0 / (1.0 + std::exp(-xc));
}

// ---------------------------------------------------------------------------
// geometry helpers (energy.py: bead_positions / _bead_radii / pair index sets)
// ---------------------------------------------------------------------------
inline std::vector<Eigen::Vector3d> beadPositions(const Robot& s, const VecN& q) {
  std::vector<Eigen::Matrix4d> f; fkChain(s, q, f);
  std::vector<Eigen::Vector3d> pts(s.n + 1);
  for (int i = 0; i <= s.n; ++i) pts[i] = f[i].block<3, 1>(0, 3);
  return pts;
}
inline std::vector<double> beadRadii(const Robot& s) {  // r ++ r[-1]
  std::vector<double> r(s.n + 1);
  for (int i = 0; i < s.n; ++i) r[i] = s.radius[i];
  r[s.n] = s.radius[s.n - 1];
  return r;
}
inline std::vector<std::pair<int, int>> nonadjacentPairs(int n_beads) {  // |i-j|>=2
  std::vector<std::pair<int, int>> v;
  for (int i = 0; i < n_beads; ++i)
    for (int j = i + 2; j < n_beads; ++j) v.push_back({i, j});
  return v;
}
inline std::vector<std::pair<int, int>> interiorPairs(int n_beads) {  // interior beads 1..nb-2
  std::vector<std::pair<int, int>> v;
  for (int i = 1; i < n_beads - 1; ++i)
    for (int j = i + 2; j < n_beads - 1; ++j) v.push_back({i, j});
  return v;
}
// joint axis (z) + point (p) per joint, DH-convention-aware (joint_axis_frames).
inline void jointAxisFrames(const Robot& s, const std::vector<Eigen::Matrix4d>& f,
                            std::vector<Eigen::Vector3d>& z, std::vector<Eigen::Vector3d>& p) {
  int off = s.modified ? 1 : 0;
  z.resize(s.n); p.resize(s.n);
  for (int k = 0; k < s.n; ++k) {
    z[k] = f[off + k].block<3, 1>(0, 2);
    p[k] = f[off + k].block<3, 1>(0, 3);
  }
}

// ---------------------------------------------------------------------------
// Phase 1 — Lennard-Jones 6-12 field  E_LJ = Σ_{|i-j|>=2} 4ε[(σ/d)^12 − S2(σ/d)^6]
// ---------------------------------------------------------------------------
inline double rawLjEnergy(const Robot& s, const VecN& q, double sigma_scale, double epsilon,
                          bool attractive, bool use_cap, double e_cap) {
  std::vector<Eigen::Vector3d> pts = beadPositions(s, q);
  auto pairs = nonadjacentPairs(s.n + 1);
  if (pairs.empty()) return 0.0;
  std::vector<double> rad = beadRadii(s);
  double s2 = attractive ? 1.0 : 0.0, E = 0.0;
  for (auto& pr : pairs) {
    double d = std::max((pts[pr.first] - pts[pr.second]).norm(), 1e-9);
    double sigma = sigma_scale * (rad[pr.first] + rad[pr.second]);
    double sr6 = std::pow(sigma / d, 6.0);
    double per = 4.0 * epsilon * (sr6 * sr6 - s2 * sr6);
    if (use_cap) per = std::min(per, e_cap);
    E += per;
  }
  return E;
}

// E_LJ + analytic dE/dq (the force the Langevin step consumes). No e_cap on grad.
inline void rawLjEnergyAndGrad(const Robot& s, const VecN& q, double sigma_scale, double epsilon,
                               bool attractive, double& E, VecN& grad) {
  std::vector<Eigen::Matrix4d> f; fkChain(s, q, f);
  int n = s.n, nb = n + 1;
  std::vector<Eigen::Vector3d> pts(nb);
  for (int i = 0; i < nb; ++i) pts[i] = f[i].block<3, 1>(0, 3);
  grad = VecN::Zero(n);
  auto pairs = nonadjacentPairs(nb);
  E = 0.0;
  if (pairs.empty()) return;
  std::vector<double> rad = beadRadii(s);
  double s2 = attractive ? 1.0 : 0.0;
  int P = (int)pairs.size();
  std::vector<Eigen::Vector3d> u(P);
  std::vector<double> dE_dd(P);
  for (int idx = 0; idx < P; ++idx) {
    int i = pairs[idx].first, j = pairs[idx].second;
    Eigen::Vector3d dv = pts[i] - pts[j];
    double d = std::max(dv.norm(), 1e-9);
    double sigma = sigma_scale * (rad[i] + rad[j]);
    double sr6 = std::pow(sigma / d, 6.0), sr12 = sr6 * sr6;
    E += 4.0 * epsilon * (sr12 - s2 * sr6);
    dE_dd[idx] = (24.0 * epsilon / d) * (s2 * sr6 - 2.0 * sr12);
    u[idx] = dv / d;
  }
  std::vector<Eigen::Vector3d> z, pj; jointAxisFrames(s, f, z, pj);
  for (int k = 0; k < n; ++k) {  // joint k moves only beads m > k: dP[m]=z_k×(p_m−p_k)
    for (int idx = 0; idx < P; ++idx) {
      int i = pairs[idx].first, j = pairs[idx].second;
      Eigen::Vector3d dPi = (i > k) ? z[k].cross(pts[i] - pj[k]) : Eigen::Vector3d::Zero();
      Eigen::Vector3d dPj = (j > k) ? z[k].cross(pts[j] - pj[k]) : Eigen::Vector3d::Zero();
      grad[k] += dE_dd[idx] * u[idx].dot(dPi - dPj);
    }
  }
}

// ---------------------------------------------------------------------------
// Phase 2 — directional hydrogen bond (interior beads carry backbone normals)
// ---------------------------------------------------------------------------
inline std::vector<Eigen::Vector3d> beadNormals(const std::vector<Eigen::Vector3d>& pts) {
  int nb = (int)pts.size();
  std::vector<Eigen::Vector3d> t(nb, Eigen::Vector3d::Zero());
  if (nb >= 3) {
    for (int i = 1; i < nb - 1; ++i) {
      Eigen::Vector3d c = (pts[i] - pts[i - 1]).cross(pts[i + 1] - pts[i]);
      double nc = c.norm();
      if (nc > 1e-9) t[i] = c / nc;  // else stays zero (initialized above)
    }
  }
  return t;
}
inline double hbDistanceFactor(double d, double d0, double sigma_d) {
  double x = d - d0;
  return std::exp(-(x * x) / (2.0 * sigma_d * sigma_d));
}
inline double hbAngleFactor(double x, double kappa) {
  return std::exp(-kappa * (1.0 - std::abs(x)));
}
inline double rawHbondEnergy(const Robot& s, const VecN& q, double d0, double sigma_d,
                             double kappa, double epsilon_hb, bool directional) {
  std::vector<Eigen::Vector3d> pts = beadPositions(s, q);
  auto pairs = interiorPairs(s.n + 1);
  if (pairs.empty()) return 0.0;
  std::vector<Eigen::Vector3d> t;
  if (directional) t = beadNormals(pts);
  double sum = 0.0;
  for (auto& pr : pairs) {
    int i = pr.first, j = pr.second;
    Eigen::Vector3d diff = pts[j] - pts[i];
    double d = std::max(diff.norm(), 1e-9);
    double F = hbDistanceFactor(d, d0, sigma_d), ang = 1.0;
    if (directional) {
      Eigen::Vector3d rhat = diff / d;
      ang = hbAngleFactor(t[i].dot(rhat), kappa) * hbAngleFactor(t[j].dot(rhat), kappa);
    }
    sum += F * ang;
  }
  return -epsilon_hb * sum;
}
// E_HB + FD force (2n FK passes, central differences, fd_eps=1e-6). No clip on q±.
inline void rawHbondEnergyAndGrad(const Robot& s, const VecN& q, double d0, double sigma_d,
                                  double kappa, double epsilon_hb, bool directional,
                                  double& E, VecN& g, double fd_eps = 1e-6) {
  E = rawHbondEnergy(s, q, d0, sigma_d, kappa, epsilon_hb, directional);
  int n = s.n; g = VecN::Zero(n);
  for (int i = 0; i < n; ++i) {
    VecN qp = q, qm = q; qp[i] += fd_eps; qm[i] -= fd_eps;
    double ep = rawHbondEnergy(s, qp, d0, sigma_d, kappa, epsilon_hb, directional);
    double em = rawHbondEnergy(s, qm, d0, sigma_d, kappa, epsilon_hb, directional);
    g[i] = (ep - em) / (2.0 * fd_eps);
  }
}
// preferred H-bond distance d0 = median interior-pair bead distance over randoms.
inline double calibrateHbondD0(const Robot& s, Rng& rng, int n_samples = 200) {
  auto pairs = interiorPairs(s.n + 1);
  if (pairs.empty()) return 0.0;
  std::vector<double> ds;
  for (int t = 0; t < n_samples; ++t) {
    std::vector<Eigen::Vector3d> pts = beadPositions(s, rng.randomConfig(s));
    for (auto& pr : pairs) ds.push_back((pts[pr.second] - pts[pr.first]).norm());
  }
  return rawMedian(ds);
}

// ---------------------------------------------------------------------------
// Phase 3 — configurational entropy  S = log Ω  (target-blind, clash-aware)
// ---------------------------------------------------------------------------
// Fixed local perturbation cloud (m,n), Gaussian std rho — common random numbers
// keep the FD entropy gradient smooth. Reconstructed faithfully from a separate
// deterministic RNG seeded with `seed` (does NOT touch the main solver stream).
inline std::vector<VecN> entropyStencil(int n, int m, double rho, uint64_t seed) {
  Rng sten(seed);
  std::vector<VecN> st(m, VecN(n));
  for (int k = 0; k < m; ++k)
    for (int i = 0; i < n; ++i) st[k][i] = rho * sten.normal(0.0, 1.0);
  return st;
}
inline double configEntropy(const Robot& s, const VecN& q, double rho, int m, double margin,
                            double alpha_clash, double alpha_lim, double floor,
                            const std::vector<VecN>* stencil, uint64_t seed) {
  std::vector<VecN> local;
  const std::vector<VecN>* st = stencil;
  if (st == nullptr) { local = entropyStencil(s.n, m, rho, seed); st = &local; }
  int M = (int)st->size();
  double sum_omega = 0.0;
  for (int k = 0; k < M; ++k) {
    VecN qk = q + (*st)[k];
    double lim = 1.0;  // soft joint-limit feasibility (Ramachandran-like)
    for (int j = 0; j < s.n; ++j)
      lim *= sigmoidClip(alpha_lim * (qk[j] - s.lo[j])) * sigmoidClip(alpha_lim * (s.hi[j] - qk[j]));
    double d = selfCollisionMinDist(s, qk);  // soft excluded-volume feasibility
    sum_omega += lim * sigmoidClip(alpha_clash * (d - margin));
  }
  double omega = sum_omega / M;
  return std::log(std::max(omega, floor));
}
inline void configEntropyAndGrad(const Robot& s, const VecN& q, double rho, int m,
                                 double& S, VecN& g, double margin = 0.0, double alpha_clash = 50.0,
                                 double alpha_lim = 30.0, double floor = 1e-6, uint64_t seed = 12345,
                                 double fd_eps = 1e-3) {
  std::vector<VecN> stencil = entropyStencil(s.n, m, rho, seed);
  S = configEntropy(s, q, rho, m, margin, alpha_clash, alpha_lim, floor, &stencil, seed);
  int n = s.n; g = VecN::Zero(n);
  for (int i = 0; i < n; ++i) {
    VecN qp = q, qm = q; qp[i] += fd_eps; qm[i] -= fd_eps;
    double sp = configEntropy(s, qp, rho, m, margin, alpha_clash, alpha_lim, floor, &stencil, seed);
    double sm = configEntropy(s, qm, rho, m, margin, alpha_clash, alpha_lim, floor, &stencil, seed);
    g[i] = (sp - sm) / (2.0 * fd_eps);
  }
}

// ---------------------------------------------------------------------------
// Phase 4 — landscape (landscape.py): calibrated params, Σ ratio, T_glass
// ---------------------------------------------------------------------------
struct RawParams {
  double sigma_scale = 1.0;  // LJ length scale (well at median bead spacing)
  double epsilon = 1.0;      // LJ depth
  double e_cap = 50.0;       // per-pair LJ repulsion cap (landscape sampling)
  double d0 = 0.0;           // H-bond preferred distance (0 ⇒ chain too short)
  double sigma_d = 0.05;     // H-bond distance width
  double kappa = 2.0;        // H-bond angular sharpness
  double epsilon_hb = 1.0;   // H-bond depth
  double rho = 0.15;         // entropy local-cloud scale (also sets S₀)
};

// RawParams.calibrate — fit geometry-dependent scales once per robot.
inline RawParams calibrateRawParams(const Robot& s, Rng& rng, int n_samples = 200) {
  RawParams p;
  std::vector<double> rad = beadRadii(s);
  auto pairs = nonadjacentPairs(s.n + 1);
  if (!pairs.empty()) {
    std::vector<double> ds;  // ||beadpos(rand_A)[I] − beadpos(rand_B)[J]|| — TWO draws/sample
    for (int t = 0; t < n_samples; ++t) {
      std::vector<Eigen::Vector3d> ptsA = beadPositions(s, rng.randomConfig(s));
      std::vector<Eigen::Vector3d> ptsB = beadPositions(s, rng.randomConfig(s));
      for (auto& pr : pairs) ds.push_back((ptsA[pr.first] - ptsB[pr.second]).norm());
    }
    double d_med = rawMedian(ds), meanr = 0.0;
    for (auto& pr : pairs) meanr += (rad[pr.first] + rad[pr.second]);
    meanr /= pairs.size();
    p.sigma_scale = d_med / (RAW_WELL * meanr);
  } else {
    p.sigma_scale = 1.0;
  }
  p.d0 = calibrateHbondD0(s, rng);
  p.sigma_d = (p.d0 > 0.0) ? 0.25 * p.d0 : 0.05;
  return p;
}

// E_LJ (capped) + E_HB — the target-blind biophysical potential.
inline double bioEnergy(const Robot& s, const VecN& q, const RawParams& p) {
  double e = rawLjEnergy(s, q, p.sigma_scale, p.epsilon, true, true, p.e_cap);
  if (p.d0 > 0.0) e += rawHbondEnergy(s, q, p.d0, p.sigma_d, p.kappa, p.epsilon_hb, true);
  return e;
}

// warm_start — cheap native-energy proxy: `steps` damped-least-squares steps.
inline VecN warmStart(const Robot& s, const VecN& q0, const Eigen::Matrix4d& Ttgt,
                      int steps = 40, double damping = 0.1) {
  VecN q = q0;
  for (int t = 0; t < steps; ++t) {
    Eigen::Matrix4d pose; Jac J; poseJac(s, q, pose, J);
    Vec6 err = poseError(pose, Ttgt);
    q = s.clip(q + dlsStep(J, err, damping));
  }
  return q;
}

// Σ = σ_E/ΔE (native = ensemble minimum). Returns sigma AND sigma_E (T_glass input).
inline void sigmaRatio(const Robot& s, const Eigen::Matrix4d& Ttgt, const RawParams& p, Rng& rng,
                       int n_seeds, int ws_steps, double& sigma, double& sigma_E) {
  std::vector<VecN> sols;
  for (int t = 0; t < n_seeds; ++t)
    sols.push_back(warmStart(s, rng.randomConfig(s), Ttgt, ws_steps, 0.1));
  std::vector<double> task, bio;
  for (auto& q : sols) { task.push_back(targetEnergy(s, q, Ttgt)); bio.push_back(bioEnergy(s, q, p)); }
  double w = rawStd(task) / std::max(rawStd(bio), 1e-9);  // balance to equal variance
  std::vector<double> comb(sols.size());
  for (size_t i = 0; i < sols.size(); ++i) comb[i] = task[i] + w * bio[i];
  double e_native = *std::min_element(comb.begin(), comb.end());
  sigma_E = rawStd(comb);
  double delta_E = rawMean(comb) - e_native;
  sigma = sigma_E / std::max(delta_E, 1e-8);
}

// S₀ ≈ Σ_j log(joint-range / cloud-cell): configurational entropy scale (REM).
inline double configEntropyScale(const Robot& s, double rho) {
  double sum = 0.0;
  for (int j = 0; j < s.n; ++j)
    sum += std::log(std::max((s.hi[j] - s.lo[j]) / (2.0 * rho), 1.0001));
  return sum;
}
// T_glass = σ_E / sqrt(2·S₀)  (REM, Bryngelson 1987).
inline double glassTemperature(double sigma_E, double s0) {
  return sigma_E / std::sqrt(2.0 * std::max(s0, 1e-6));
}

// ---------------------------------------------------------------------------
// Phase 5 — solver pieces (solver.py)
// ---------------------------------------------------------------------------
// F(q;T) = E_task + E_LJ + E_HB − T·S_conf (scalar; reporting + native selection).
inline double freeEnergy(const Robot& s, const VecN& q, const Eigen::Matrix4d& Ttgt,
                         const RawParams& p, double T, int m_entropy) {
  double e = targetEnergy(s, q, Ttgt);
  e += rawLjEnergy(s, q, p.sigma_scale, p.epsilon, true, true, p.e_cap);
  if (p.d0 > 0.0) e += rawHbondEnergy(s, q, p.d0, p.sigma_d, p.kappa, p.epsilon_hb, true);
  e -= T * configEntropy(s, q, p.rho, m_entropy, 0.0, 50.0, 30.0, 1e-6, nullptr, 12345);
  return e;
}

// _task_grad: uphill grad of ½‖err_w‖² + scalar pos/orient errors.
inline void taskGrad(const Robot& s, const VecN& q, const Eigen::Matrix4d& Ttgt,
                     VecN& g, double& pe, double& oe) {
  Eigen::Matrix4d pose; Jac J; poseJac(s, q, pose, J);
  Vec6 err = poseError(pose, Ttgt);
  pe = err.head<3>().norm(); oe = err.tail<3>().norm();
  Vec6 err_w = err; err_w.tail<3>() *= RAW_ORIENT_W;
  g = -(J.transpose() * err_w);
}
inline VecN clipNorm(const VecN& g, double gmax) {
  double nn = g.norm();
  return (nn > gmax) ? (g * (gmax / nn)) : g;
}

// _consolidate: damped-Newton settling into the native (target) minimum (T→0).
inline void consolidate(const Robot& s, const VecN& q0, const Eigen::Matrix4d& Ttgt,
                        double pos_tol, double ori_tol, VecN& q_out, bool& ok,
                        int max_steps = 40, double lam0 = 0.05) {
  VecN q = q0; double lam = lam0;
  for (int t = 0; t < max_steps; ++t) {
    Eigen::Matrix4d pose; Jac J; poseJac(s, q, pose, J);
    Vec6 err = poseError(pose, Ttgt);
    double pe = err.head<3>().norm(), oe = err.tail<3>().norm();
    if (pe < pos_tol && oe < ori_tol) { q_out = q; ok = true; return; }
    VecN q_try = s.clip(q + dlsStep(J, err, lam));
    Vec6 err_t = poseError(eePose(s, q_try), Ttgt);
    double pet = err_t.head<3>().norm(), oet = err_t.tail<3>().norm();
    if (pet + RAW_ORIENT_W * oet < pe + RAW_ORIENT_W * oe) {
      q = q_try; lam = std::max(lam * 0.5, 1e-4);
    } else {
      lam = std::min(lam * 2.5, 2.0);
      if (lam >= 2.0) break;
    }
  }
  Vec6 err = poseError(eePose(s, q), Ttgt);
  q_out = q; ok = (err.head<3>().norm() < pos_tol && err.tail<3>().norm() < ori_tol);
}

// _stable_native: Anfinsen kinetic gate — jitter and re-settle `trials` times.
inline bool stableNative(const Robot& s, const VecN& q, const Eigen::Matrix4d& Ttgt,
                         double pos_tol, double ori_tol, Rng& rng,
                         double jitter = 0.01, int trials = 5) {
  for (int t = 0; t < trials; ++t) {
    VecN qj = q;
    for (int i = 0; i < s.n; ++i) qj[i] += jitter * rng.normal(0.0, 1.0);
    qj = s.clip(qj);
    VecN qc; bool ok; consolidate(s, qj, Ttgt, pos_tol, ori_tol, qc, ok, 15);
    Vec6 err = poseError(eePose(s, qc), Ttgt);
    if (!(err.head<3>().norm() < pos_tol && err.tail<3>().norm() < ori_tol)) return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// solve_protein_raw — the full V6 entry point (generic DOF).
// Defaults mirror the Python signature (max_iters=0 ⇒ n_lang = 100 + 25·max(0,n−6)).
// ---------------------------------------------------------------------------
inline PikResult solveProteinRaw(const Robot& s, const VecN& q0, const Eigen::Matrix4d& Ttgt,
                                 Rng& rng, int max_iters = 0, double pos_tol = 1e-3,
                                 double orient_tol = 1e-2, double dt = 0.03, int m_entropy = 16,
                                 double w_lj = 0.4, double w_hb = 0.4, double w_s = 0.5,
                                 double max_step = 0.25) {
  const int n = s.n;

  // --- calibrate the physics + measure the landscape ----------------------
  RawParams p = calibrateRawParams(s, rng);
  double s0 = configEntropyScale(s, p.rho);
  double sigma_val, sigma_E; sigmaRatio(s, Ttgt, p, rng, 12, 60, sigma_val, sigma_E);
  (void)sigma_val;  // Σ ratio is a reporting-only diagnostic (not scored)
  double T_glass = glassTemperature(sigma_E, s0);
  double T_start = std::max(4.0 * T_glass, 0.25);            // start unfolded (hot)
  int n_lang = (max_iters > 0) ? max_iters : 100 + 25 * std::max(0, n - 6);
  double tau = n_lang / 3.0;

  // --- seeds: multi-start warm-starts enforce the boundary condition -------
  auto terr = [&](const VecN& qq) {
    Vec6 e = poseError(eePose(s, qq), Ttgt);
    return e.head<3>().norm() + RAW_ORIENT_W * e.tail<3>().norm();
  };
  int n_ws = 10 + 2 * std::max(0, n - 6);
  std::vector<VecN> seeds;
  seeds.push_back(warmStart(s, q0, Ttgt));                    // steps=40, damping=0.1
  for (int t = 0; t < n_ws; ++t) seeds.push_back(warmStart(s, rng.randomConfig(s), Ttgt));
  {  // stable sort by task error (Python list.sort is stable; keys computed once)
    std::vector<double> keys(seeds.size());
    for (size_t i = 0; i < seeds.size(); ++i) keys[i] = terr(seeds[i]);
    std::vector<int> order(seeds.size());
    for (size_t i = 0; i < seeds.size(); ++i) order[i] = (int)i;
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return keys[a] < keys[b]; });
    std::vector<VecN> sorted; sorted.reserve(seeds.size());
    for (int idx : order) sorted.push_back(seeds[idx]);
    seeds.swap(sorted);
  }
  VecN q = seeds[0];                                          // fold from the best seed
  VecN g_task0; double pe0, oe0; taskGrad(s, q, Ttgt, g_task0, pe0, oe0);
  double g_cap = 5.0 * std::max(g_task0.norm(), 1e-6);        // bound LJ explosion

  VecN best_q = q;
  double best_score = std::numeric_limits<double>::infinity();
  bool have_clean = false; VecN best_clean_q;
  double best_clean_clear = -std::numeric_limits<double>::infinity();

  // --- overdamped Langevin cooling ----------------------------------------
  for (int it = 0; it < n_lang; ++it) {
    double T = std::max(T_glass, T_start * std::exp(-(double)it / tau));

    VecN g_task; double pe, oe; taskGrad(s, q, Ttgt, g_task, pe, oe);
    double score = pe + RAW_ORIENT_W * oe;                    // error AT the current q
    if (score < best_score) { best_score = score; best_q = q; }
    if (pe < 0.08 && oe < 0.3) {                              // remember cleanest near-target
      double d_self = selfCollisionMinDist(s, q);
      if (d_self > best_clean_clear) { best_clean_clear = d_self; best_clean_q = q; have_clean = true; }
    }

    double E_lj; VecN g_lj; rawLjEnergyAndGrad(s, q, p.sigma_scale, p.epsilon, true, E_lj, g_lj);
    g_lj = clipNorm(g_lj, g_cap);
    VecN g_hb;
    if (p.d0 > 0.0) { double E_hb; rawHbondEnergyAndGrad(s, q, p.d0, p.sigma_d, p.kappa, p.epsilon_hb, true, E_hb, g_hb); }
    else g_hb = VecN::Zero(n);
    double S; VecN g_s; configEntropyAndGrad(s, q, p.rho, m_entropy, S, g_s);

    VecN grad_F = g_task + w_lj * g_lj + w_hb * g_hb - (T * w_s) * g_s;
    VecN noise(n);
    double amp = std::sqrt(2.0 * T * dt);
    for (int i = 0; i < n; ++i) noise[i] = amp * rng.normal(0.0, 1.0);  // standard_normal(n)
    q = s.clip(q + clipNorm(-grad_F * dt + noise, max_step));
  }

  // --- native-state selection (T→0): consolidate every candidate, then pick
  //     the LOWEST-ENTHALPY CLASH-FREE basin (Anfinsen) ---------------------
  std::vector<VecN> cands;
  if (have_clean) cands.push_back(best_clean_q);
  cands.push_back(best_q);
  for (auto& sd : seeds) cands.push_back(sd);

  struct Conv { double enthalpy, clearance; VecN q; };
  std::vector<Conv> converged;
  VecN fallback_q = best_q;
  double fallback_err = std::numeric_limits<double>::infinity();
  for (auto& cand : cands) {
    VecN cc; bool ok; consolidate(s, cand, Ttgt, pos_tol, orient_tol, cc, ok);
    if (ok) {
      converged.push_back({freeEnergy(s, cc, Ttgt, p, 0.0, m_entropy),
                           selfCollisionMinDist(s, cc), cc});
    }
    Vec6 e = poseError(eePose(s, cc), Ttgt);
    double we = e.head<3>().norm() + RAW_ORIENT_W * e.tail<3>().norm();
    if (we < fallback_err) { fallback_err = we; fallback_q = cc; }
  }

  VecN q_nat; bool conv;
  if (!converged.empty()) {
    const Conv* best = nullptr;
    for (auto& c : converged) if (c.clearance > 0.0) {          // min-enthalpy clash-free
      if (!best || c.enthalpy < best->enthalpy) best = &c;
    }
    if (best == nullptr) {                                      // else least-bad clearance
      best = &converged[0];
      for (auto& c : converged) if (c.clearance > best->clearance) best = &c;
    }
    q_nat = best->q; conv = true;
  } else {
    q_nat = fallback_q; conv = false;
  }
  int restarts = (int)cands.size() - 1;

  bool success = conv;
  // Anfinsen kinetic gate — computed for fidelity (consumes the last rng draws);
  // Python reports success = conv (NOT stable), so `stable` does not gate output.
  bool stable = conv && stableNative(s, q_nat, Ttgt, pos_tol, orient_tol, rng);
  (void)stable; (void)T_glass;

  PikResult R;
  Vec6 ef = poseError(eePose(s, q_nat), Ttgt);
  R.success = success;
  R.q = q_nat;
  R.pos_error = ef.head<3>().norm();
  R.orient_error = ef.tail<3>().norm();
  R.iterations = n_lang;
  R.restarts = restarts;
  R.min_self_distance = selfCollisionMinDist(s, q_nat);
  R.conflict_index = 0.0;
  R.lambda_final = 0.0;
  return R;
}

}  // namespace pik
#endif
