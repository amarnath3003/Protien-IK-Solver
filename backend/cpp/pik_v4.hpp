// ProteinIK V4 (protein_fast) — generic-DOF port of app/solvers/protein_fast/solver.py.
// Same logic as the UR5-only proteinik_v4.hpp, but on the runtime `Robot` (dh_robot.hpp)
// so it runs on planar3dof / ur5 / franka_panda. Weights/tolerances/budgets/acceptance
// identical to the Python; only the RNG stream differs.
#ifndef PIK_V4_HPP
#define PIK_V4_HPP

#include "dh_robot.hpp"
#include <vector>
#include <set>

namespace pik {

struct V4Result {
  bool success = false;
  VecN q;
  double pos_error = 0, orient_error = 0;
  int iterations = 0, restarts = 0;
  double min_self_distance = 0;
};

static const double W_T = 3.0, W_L = 1.0, W_C = 2.0, W_S = 0.3;

inline void combinedErr(const Robot& s, const VecN& q, const Eigen::Matrix4d& Ttgt,
                        double& pos_e, double& orient_e, double& combined) {
  Vec6 e = poseError(eePose(s, q), Ttgt);
  pos_e = e.head<3>().norm(); orient_e = e.tail<3>().norm();
  combined = pos_e + 0.3 * orient_e;
}

// _lm_polish_fast — adaptive Levenberg-Marquardt barrierless endgame
inline void lmPolish(const Robot& s, const VecN& q0, const Eigen::Matrix4d& Ttgt,
                     double pos_tol, double orient_tol, int max_steps,
                     VecN& q_out, double& e_out, bool& conv, int& steps_used,
                     double lam0 = 0.08) {
  VecN q = q0;
  Eigen::Matrix4d pose; Jac J; poseJac(s, q, pose, J);
  Vec6 err = poseError(pose, Ttgt);
  double pos_e = err.head<3>().norm(), orient_e = err.tail<3>().norm();
  double e_cur = pos_e + 0.3 * orient_e;
  double lam = lam0; steps_used = 0;
  for (int k = 0; k < max_steps; ++k) {
    if (pos_e < pos_tol && orient_e < orient_tol) {
      q_out = q; e_out = e_cur; conv = true; return;
    }
    steps_used++;
    VecN q_try = s.clip(q + dlsStep(J, err, lam));
    Eigen::Matrix4d pose_t; Jac J_t; poseJac(s, q_try, pose_t, J_t);
    Vec6 err_t = poseError(pose_t, Ttgt);
    double p_t = err_t.head<3>().norm(), o_t = err_t.tail<3>().norm();
    double e_try = p_t + 0.3 * o_t;
    if (e_try < e_cur) {
      q = q_try; J = J_t; err = err_t; pos_e = p_t; orient_e = o_t; e_cur = e_try;
      lam = std::max(lam * 0.5, 1e-4);
    } else {
      lam = std::min(lam * 2.5, 2.0);
      if (lam >= 2.0) break;
    }
  }
  q_out = q; e_out = e_cur; conv = (pos_e < pos_tol && orient_e < orient_tol);
}

// _fold_once — coarse collapse -> Metropolis funnel + LM endgame + chaperone rescue
inline void foldOnce(const Robot& s, const VecN& q0, const Eigen::Matrix4d& Ttgt, Rng& rng,
                     int max_iters, double pos_tol, double orient_tol,
                     int stage2_iters, int stuck_window, double stuck_eps,
                     VecN& best_q, double& best_combined, bool& conv, int& it_out, int& rescues_out) {
  const int n = s.n;
  VecN q = q0;
  int it = 0, rescues = 0;
  double pe, oe, cbest; combinedErr(s, q, Ttgt, pe, oe, cbest);
  best_q = q; best_combined = cbest;
  auto consider = [&](const VecN& cand) {
    double a, b, c; combinedErr(s, cand, Ttgt, a, b, c);
    if (c < best_combined) { best_combined = c; best_q = cand; }
  };

  // STAGE 2: coarse collapse
  for (int k = 0; k < stage2_iters; ++k) {
    it++;
    Eigen::Matrix4d pose; Jac J; poseJac(s, q, pose, J);
    Vec6 err = poseError(pose, Ttgt);
    q = s.clip(q + 0.4 * dlsStep(J, err, 0.15));
  }
  consider(q);

  // STAGE 3+4: Metropolis funnel narrowing + LM + rescue
  std::vector<double> recent;
  double search_radius = 0.5, radius_decay = 0.985;
  double cur_energy = totalEnergy(s, q, Ttgt, W_T, W_L, W_C, W_S);

  while (it < max_iters) {
    it++;
    double T0 = 0.3, Tf = 0.01;
    double frac = std::min((double)it / max_iters, 1.0);
    double temp = T0 * std::pow(Tf / T0, frac);

    if (it % 2 == 0) {
      for (int i = 0; i < n; ++i) {
        double cand = std::min(std::max(q[i] + rng.uniform(-search_radius, search_radius), s.lo[i]), s.hi[i]);
        VecN q_try = q; q_try[i] = cand;
        double e_try = totalEnergy(s, q_try, Ttgt, W_T, W_L, W_C, W_S);
        if (e_try < cur_energy ||
            rng.uniform01() < std::exp(-(e_try - cur_energy) / std::max(temp, 1e-6))) {
          q = q_try; cur_energy = e_try;
        }
      }
    }

    Eigen::Matrix4d pose; Jac J; poseJac(s, q, pose, J);
    Vec6 err = poseError(pose, Ttgt);
    double pos_e = err.head<3>().norm(), orient_e = err.tail<3>().norm();

    if (pos_e < 0.05 && orient_e < 0.2) {
      VecN qn; double en; bool cn; int sn;
      lmPolish(s, q, Ttgt, pos_tol, orient_tol, 12, qn, en, cn, sn);
      q = qn; cur_energy = totalEnergy(s, q, Ttgt, W_T, W_L, W_C, W_S); consider(q);
      if (cn) { conv = true; it_out = it; rescues_out = rescues; return; }
    } else {
      q = s.clip(q + dlsStep(J, err, 0.05));
      cur_energy = totalEnergy(s, q, Ttgt, W_T, W_L, W_C, W_S); consider(q);
    }
    search_radius *= radius_decay;

    combinedErr(s, q, Ttgt, pos_e, orient_e, cbest);
    if (pos_e < pos_tol && orient_e < orient_tol) {
      conv = true; it_out = it; rescues_out = rescues; return;
    }

    // STAGE 4: chaperone rescue
    recent.push_back(cur_energy);
    if ((int)recent.size() >= stuck_window && (recent.front() - recent.back()) < stuck_eps) {
      rescues++;
      std::set<int> ss{std::max(1, n / 6), std::max(1, n / 2), std::max(1, 5 * n / 6), n};
      std::vector<int> scope_sizes(ss.begin(), ss.end());
      int scope = scope_sizes[std::min((int)rescues - 1, (int)scope_sizes.size() - 1)];
      if (scope >= n) {
        q = rng.randomConfig(s);
      } else {
        VecN contrib = frustrationIndex(s, q, Ttgt);
        int worst = 0; for (int i = 1; i < n; ++i) if (contrib[i] > contrib[worst]) worst = i;
        int half = scope / 2;
        int lo_s = std::max(0, worst - half);
        int hi_s = std::min(n, lo_s + scope);
        lo_s = std::max(0, hi_s - scope);
        VecN fresh = rng.randomConfig(s);
        for (int j = lo_s; j < hi_s; ++j) q[j] = fresh[j];
      }
      search_radius = 0.5; recent.clear();
      cur_energy = totalEnergy(s, q, Ttgt, W_T, W_L, W_C, W_S); consider(q);
    }
  }
  conv = false; it_out = it; rescues_out = rescues;
}

// solve_protein_fast — the full V4 entry point (generic DOF).
// warm_seed_o2: if provided (non-null), Phase B replica 0 uses the o2 IAM
// partial-unfold warm start instead of q0 (see solver_o2.py). null -> base V4.
inline V4Result solveProteinFast(const Robot& s, const VecN& q0, const Eigen::Matrix4d& Ttgt,
                                 Rng& rng, int max_iters = 150, double pos_tol = 1e-3,
                                 double orient_tol = 1e-2, int stage2_iters = 10,
                                 int stuck_window = 10, double stuck_eps = 2e-4,
                                 int max_replicas = 6, bool o2_warmstart = false,
                                 double unfold_kick = 0.7) {
  const int n = s.n;

  // contact-order difficulty scaling of the per-replica budget
  double reach_needed = Ttgt.block<3, 1>(0, 3).norm();
  double max_reach = 0; for (int i = 0; i < n; ++i) max_reach += std::abs(s.a[i]) + std::abs(s.d[i]);
  double reach_ratio = max_reach > 0 ? std::min(reach_needed / max_reach, 1.0) : 0.0;
  { Eigen::Matrix4d p; Jac J0; poseJac(s, q0, p, J0);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J0);
    auto sv = svd.singularValues();
    double cond = sv(0) / (sv(sv.size() - 1) + 1e-6);
    double difficulty = 1.0 + std::min(reach_ratio, 1.0) + std::min(cond / 100.0, 1.0);
    stage2_iters = (int)(stage2_iters * difficulty);
  }

  VecN global_best_q; bool have_best = false;
  double global_best_combined = std::numeric_limits<double>::infinity();
  int total_iters = 0, total_rescues = 0;
  bool success = false;
  std::vector<std::pair<double, VecN>> converged;
  auto have_clean = [&]() { for (auto& c : converged) if (c.first >= 0.0) return true; return false; };

  // PHASE A: barrierless LM restarts
  for (int r = 0; r < max_replicas; ++r) {
    VecN seed = (r == 0) ? q0 : rng.randomConfig(s);
    VecN q_lm; double e_lm; bool conv; int st;
    lmPolish(s, seed, Ttgt, pos_tol, orient_tol, 30, q_lm, e_lm, conv, st);
    total_iters += st;
    if (e_lm < global_best_combined) { global_best_combined = e_lm; global_best_q = q_lm; have_best = true; }
    if (conv) {
      success = true;
      double dd = selfCollisionMinDist(s, q_lm);
      converged.push_back({dd, q_lm});
      if (dd >= 0.0) break;
    }
  }

  // PHASE B: stochastic funnel folding (only if no clean barrierless solution)
  if (!have_clean()) {
    // o2 warm start: best (least-clashing) converged Phase-A candidate, partially unfolded.
    VecN warm_seed; bool have_warm = false;
    int s2_warm = std::max(2, stage2_iters / 4);
    if (o2_warmstart && !converged.empty()) {
      auto best = std::max_element(converged.begin(), converged.end(),
                   [](const auto& a, const auto& b){ return a.first < b.first; });
      warm_seed = best->second; have_warm = true;
    }
    int phase_b_conv = 0;
    for (int replica = 0; replica < max_replicas; ++replica) {
      if (have_clean() || phase_b_conv >= 2) break;
      VecN seed; int s2_use = stage2_iters;
      if (o2_warmstart && replica == 0 && have_warm) {
        // GroEL/IAM partial unfold: kick the top-half most-frustrated joints.
        VecN contrib = frustrationIndex(s, warm_seed, Ttgt);
        int k = std::max(1, n / 2);
        std::vector<int> idx(n); for (int i = 0; i < n; ++i) idx[i] = i;
        std::sort(idx.begin(), idx.end(), [&](int a, int b){ return contrib[a] < contrib[b]; });
        seed = warm_seed;
        for (int t = n - k; t < n; ++t) {
          int j = idx[t];
          seed[j] = seed[j] + rng.uniform(-unfold_kick, unfold_kick);
        }
        seed = s.clip(seed);
        s2_use = s2_warm;
      } else {
        seed = (replica == 0) ? q0 : rng.randomConfig(s);
      }
      VecN bq; double bc; bool conv; int iters, rescues;
      foldOnce(s, seed, Ttgt, rng, max_iters, pos_tol, orient_tol,
               s2_use, stuck_window, stuck_eps, bq, bc, conv, iters, rescues);
      total_iters += iters; total_rescues += rescues;
      if (bc < global_best_combined) { global_best_combined = bc; global_best_q = bq; have_best = true; }
      if (conv) {
        success = true; phase_b_conv++;
        double dd = selfCollisionMinDist(s, bq);
        converged.push_back({dd, bq});
        if (dd >= 0.0) break;
      }
    }
  }

  if (!converged.empty()) {
    auto best = std::max_element(converged.begin(), converged.end(),
                 [](const auto& a, const auto& b){ return a.first < b.first; });
    global_best_q = best->second; have_best = true;
  }
  VecN q = have_best ? global_best_q : q0;

  // native-stability jitter gate
  if (success) {
    double pe, oe, base_combined; combinedErr(s, q, Ttgt, pe, oe, base_combined);
    int jitter_failures = 0, n_jit = 5;
    double sm = 0; for (int i = 0; i < n; ++i) sm += std::abs(s.a[i]) + std::abs(s.d[i]);
    double arm_reach = std::max(0.1, sm);
    double jitter_std = std::min(std::max(1e-3 / std::max(1.0, arm_reach), 1e-4), 5e-3);
    for (int k = 0; k < n_jit; ++k) {
      VecN qj = q; for (int i = 0; i < n; ++i) qj[i] += rng.normal(0, jitter_std); qj = s.clip(qj);
      double a, b, cj; combinedErr(s, qj, Ttgt, a, b, cj);
      if (cj > base_combined + 10 * (pos_tol + 0.3 * orient_tol)) jitter_failures++;
    }
    if (jitter_failures >= n_jit - 1) success = false;
  }

  V4Result R;
  Vec6 ef = poseError(eePose(s, q), Ttgt);
  R.success = success; R.q = q;
  R.pos_error = ef.head<3>().norm(); R.orient_error = ef.tail<3>().norm();
  R.iterations = total_iters; R.restarts = total_rescues;
  R.min_self_distance = selfCollisionMinDist(s, q);
  return R;
}

}  // namespace pik
#endif
