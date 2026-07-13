// ProteinIK V1 (protein_ik) — generic-DOF port of app/solvers/protein_ik.py
// (function solve_protein_ik, the staged folding solver). Runs on the runtime
// `Robot` (dh_robot.hpp) so it works on planar3dof / ur5 / franka_panda.
//
// FIDELITY: only the DEFAULT code path is ported — use_vectorial_folding=False
// AND use_rotamer_bias=False. The vectorial-folding domain-anchored branch and
// the rotamer-library-biased proposal branch are intentionally NOT ported (both
// are disabled in the benchmark). Weights / tolerances / stage budgets / control
// flow are identical to the Python; only the RNG stream differs.
#ifndef PIK_V1_HPP
#define PIK_V1_HPP

#include "dh_robot.hpp"
#include <vector>
#include <set>
#include <algorithm>
#include <cmath>

namespace pik {

// _per_joint_energy_contribution — one-sided finite-difference sensitivity of
// the Stage-3 total energy to each joint (eps=0.05, weights 3,1,2,0.3). NOT the
// analytic frustration_index. Base energy computed once and reused across joints.
inline VecN perJointEnergyContribution(const Robot& s, const VecN& q,
                                       const Eigen::Matrix4d& Ttgt) {
  const int n = s.n;
  const double W_TARGET = 3.0, W_LIMIT = 1.0, W_COLLISION = 2.0, W_SMOOTH = 0.3;
  double base = totalEnergy(s, q, Ttgt, W_TARGET, W_LIMIT, W_COLLISION, W_SMOOTH);
  VecN contributions = VecN::Zero(n);
  const double eps = 0.05;
  for (int i = 0; i < n; ++i) {
    VecN q_pert = q;
    q_pert[i] = std::min(std::max(q[i] + eps, s.lo[i]), s.hi[i]);
    double e_pert = totalEnergy(s, q_pert, Ttgt, W_TARGET, W_LIMIT, W_COLLISION, W_SMOOTH);
    contributions[i] = std::abs(e_pert - base);
  }
  return contributions;
}

// solve_protein_ik — staged folding IK (default path).
//   STAGE 1: local-blind relaxation (target-blind coordinate descent on
//            neutral + smoothness + joint-limit energy)
//   STAGE 2: coarse collapse (low-gain DLS, damping 0.15, scale 0.4)
//   STAGE 3+4: funnel narrowing (perturbation search every other iter) + one
//              gradient DLS step (damping 0.05) + escalating scoped IAM rescue
//   STAGE 5: stability jitter gate
inline PikResult solveProteinIK(const Robot& s, const VecN& q0,
                                const Eigen::Matrix4d& Ttgt, Rng& rng,
                                int max_iters = 200, double pos_tol = 1e-3,
                                double orient_tol = 1e-2, int stage1_iters = 6,
                                int stage2_iters = 10, int stuck_window = 10,
                                double stuck_eps = 2e-4, int max_rescues = 6) {
  const int n = s.n;
  VecN q = q0;
  VecN q_neutral = VecN::Zero(n);  // mid-range neutral pose used by stage 1
  bool success = false;
  int it = 0;
  int restarts = 0;      // counts scoped rescues (reported in PikResult.restarts)
  int rescues_used = 0;  // escalation ladder index
  (void)max_rescues;     // present for signature parity; unused in default path
                         // (matches Python, where max_rescues is never checked)

  auto converged = [&]() -> bool {
    Vec6 err = poseError(eePose(s, q), Ttgt);
    return err.head<3>().norm() < pos_tol && err.tail<3>().norm() < orient_tol;
  };

  // ---------- STAGE 1: local-blind relaxation ----------
  // Target-blind coordinate descent: only neutral-pose + neighbor-smoothness +
  // joint-limit energy. One joint at a time, trying q[i]-0.3 and q[i]+0.3.
  for (int s1 = 0; s1 < stage1_iters; ++s1) {
    it++;
    for (int i = 0; i < n; ++i) {
      double best_q_i = q[i];
      double best_e = neutralPoseEnergy(q, q_neutral) + smoothnessEnergy(q) +
                      jointLimitEnergy(s, q);
      const double raw[2] = { q[i] - 0.3, q[i] + 0.3 };
      for (int c = 0; c < 2; ++c) {
        double cand = std::min(std::max(raw[c], s.lo[i]), s.hi[i]);
        VecN q_try = q; q_try[i] = cand;
        double e_try = neutralPoseEnergy(q_try, q_neutral) + smoothnessEnergy(q_try) +
                       jointLimitEnergy(s, q_try);
        if (e_try < best_e) { best_e = e_try; best_q_i = cand; }
      }
      q[i] = best_q_i;
    }
  }

  // ---------- STAGE 2: coarse collapse ----------
  // Low-gain DLS pull toward target, imprecise on purpose (damping 0.15,
  // step scale 0.4). Sets success mid-loop on convergence but does NOT break.
  for (int s2 = 0; s2 < stage2_iters; ++s2) {
    it++;
    Eigen::Matrix4d pose; Jac J; poseJac(s, q, pose, J);
    Vec6 err = poseError(pose, Ttgt);
    q = s.clip(q + 0.4 * dlsStep(J, err, 0.15));
    if (converged()) success = true;
  }

  // ---------- STAGE 3 + 4: funneled narrowing search with scoped rescue ------
  std::vector<double> recent_energies;
  double search_radius = 0.5;
  const double radius_decay = 0.985;
  double cur_energy = totalEnergy(s, q, Ttgt, 3.0, 1.0, 2.0, 0.3);

  while (it < max_iters && !success) {
    it++;

    // narrowing local search: one small perturbation per joint every other
    // iteration, keep if it improves total energy (shrinking-radius funnel).
    if (it % 2 == 0) {
      for (int i = 0; i < n; ++i) {
        double cand = std::min(
            std::max(q[i] + rng.uniform(-search_radius, search_radius), s.lo[i]),
            s.hi[i]);
        VecN q_try = q; q_try[i] = cand;
        double e_try = totalEnergy(s, q_try, Ttgt, 3.0, 1.0, 2.0, 0.3);
        if (e_try < cur_energy) { q = q_try; cur_energy = e_try; }
      }
    }

    // one gradient DLS step toward target each iteration (damping 0.05)
    Eigen::Matrix4d pose; Jac J; poseJac(s, q, pose, J);
    Vec6 err = poseError(pose, Ttgt);
    q = s.clip(q + dlsStep(J, err, 0.05));
    cur_energy = totalEnergy(s, q, Ttgt, 3.0, 1.0, 2.0, 0.3);  // refresh after gradient step

    search_radius *= radius_decay;

    if (converged()) { success = true; break; }

    // ---------- STAGE 4: IAM-inspired escalating scoped rescue ----------
    recent_energies.push_back(cur_energy);
    if ((int)recent_energies.size() >= stuck_window) {
      // sliding window of the last stuck_window energies (recent_energies[-stuck_window:])
      double window_first = recent_energies[recent_energies.size() - stuck_window];
      double window_last = recent_energies.back();
      double progress = window_first - window_last;
      if (progress < stuck_eps) {
        restarts++;
        rescues_used++;
        // escalation ladder: fractions 1/6, 1/2, 5/6, full (deduped + sorted).
        std::set<int> ss{ std::max(1, n / 6), std::max(1, n / 2),
                          std::max(1, 5 * n / 6), n };
        std::vector<int> scope_sizes(ss.begin(), ss.end());
        int scope_idx = std::min(rescues_used - 1, (int)scope_sizes.size() - 1);
        int scope = scope_sizes[scope_idx];

        if (scope >= n) {
          // full unfold: fresh random seed for the whole chain (n uniforms)
          q = rng.randomConfig(s);
        } else {
          // partial unfold: contiguous window of `scope` joints centered on the
          // most "misfolded" (highest finite-diff sensitivity) joint.
          VecN contributions = perJointEnergyContribution(s, q, Ttgt);
          int worst = 0;  // np.argmax -> first index of the max
          for (int i = 1; i < n; ++i)
            if (contributions[i] > contributions[worst]) worst = i;
          int half = scope / 2;
          int lo = std::max(0, worst - half);
          int hi = std::min(n, lo + scope);
          lo = std::max(0, hi - scope);  // re-clamp if hi got capped
          // Python: q[j] = spec.random_config(rng)[j] INSIDE the loop — draws a
          // FULL random config (n uniforms) per j and keeps element j. Replicate
          // exactly to keep RNG stream consumption faithful.
          for (int j = lo; j < hi; ++j) {
            VecN fresh = rng.randomConfig(s);
            q[j] = fresh[j];
          }
        }

        search_radius = 0.5;       // reset funnel radius after a rescue
        recent_energies.clear();
        cur_energy = totalEnergy(s, q, Ttgt, 3.0, 1.0, 2.0, 0.3);
      }
    }
  }

  // ---------- STAGE 5: stability-checked termination ----------
  if (success) {
    Vec6 base_err = poseError(eePose(s, q), Ttgt);
    double base_combined = base_err.head<3>().norm() + 0.3 * base_err.tail<3>().norm();
    int jitter_failures = 0;
    const int n_jitter_trials = 5;
    double sm = 0.0;
    for (int i = 0; i < n; ++i) sm += std::abs(s.a[i]) + std::abs(s.d[i]);
    double arm_reach = std::max(0.1, sm);
    double jitter_std = std::min(std::max(1e-3 / std::max(1.0, arm_reach), 1e-4), 5e-3);
    for (int k = 0; k < n_jitter_trials; ++k) {
      VecN q_jitter = q;  // rng.normal(0, jitter_std, size=n): n draws in order
      for (int i = 0; i < n; ++i) q_jitter[i] += rng.normal(0.0, jitter_std);
      q_jitter = s.clip(q_jitter);
      Vec6 err_j = poseError(eePose(s, q_jitter), Ttgt);
      double combined_j = err_j.head<3>().norm() + 0.3 * err_j.tail<3>().norm();
      if (combined_j > base_combined + 10.0 * (pos_tol + 0.3 * orient_tol))
        jitter_failures++;
    }
    if (jitter_failures >= n_jitter_trials - 1) success = false;
  }

  PikResult R;
  Vec6 ef = poseError(eePose(s, q), Ttgt);
  R.success = success;
  R.q = q;
  R.pos_error = ef.head<3>().norm();
  R.orient_error = ef.tail<3>().norm();
  R.iterations = it;
  R.restarts = restarts;
  R.min_self_distance = selfCollisionMinDist(s, q);
  // conflict_index / lambda_final stay 0 (not produced by this solver).
  return R;
}

}  // namespace pik
#endif
