// pybind11 module exposing the native-C++ ProteinIK solvers + kinematics
// primitives (for FK/energy parity vs Python). Built into `pik_native.*.so`
// and imported by the benchmark adapters so the Python harness calls the C++
// solvers per-solve, exactly like it calls the Python ones.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "dh_robot.hpp"
#include "pik_v4.hpp"
#if __has_include("pik_v1.hpp")
#  include "pik_v1.hpp"
#  define HAVE_V1 1
#endif
#if __has_include("pik_homotopy.hpp")
#  include "pik_homotopy.hpp"
#  define HAVE_HOMOTOPY 1
#endif
#if __has_include("pik_raw.hpp")
#  include "pik_raw.hpp"
#  define HAVE_RAW 1
#endif
#if __has_include("pik_ccd.hpp")
#  include "pik_ccd.hpp"
#  define HAVE_CCD 1
#endif
#if __has_include("pik_fabrik.hpp")
#  include "pik_fabrik.hpp"
#  define HAVE_FABRIK 1
#endif

namespace py = pybind11;
using namespace pik;

static Robot make_robot(const std::vector<double>& a, const std::vector<double>& d,
                        const std::vector<double>& alpha, const std::vector<double>& toff,
                        const std::vector<double>& lo, const std::vector<double>& hi,
                        const std::vector<double>& radius, bool modified) {
  Robot s;
  s.n = (int)a.size();
  s.a = a; s.d = d; s.alpha = alpha; s.toff = toff;
  s.lo = lo; s.hi = hi; s.radius = radius; s.modified = modified;
  return s;
}

template <class R>
static py::dict to_dict(const R& r) {
  py::dict o;
  std::vector<double> q(r.q.data(), r.q.data() + r.q.size());
  o["q"] = q;
  o["success"] = r.success;
  o["iterations"] = r.iterations;
  o["restarts"] = r.restarts;
  o["pos_error"] = r.pos_error;
  o["orient_error"] = r.orient_error;
  o["min_self_distance"] = r.min_self_distance;
  o["conflict_index"] = (double)0.0;
  o["lambda_final"] = (double)0.0;
  return o;
}
// PikResult carries the two extra diagnostics.
static py::dict to_dict_pik(const PikResult& r) {
  py::dict o = to_dict(r);
  o["conflict_index"] = r.conflict_index;
  o["lambda_final"] = r.lambda_final;
  return o;
}

PYBIND11_MODULE(pik_native, m) {
  m.doc() = "Native C++ ProteinIK solvers (generic DOF, standard/modified DH)";

  py::class_<Robot>(m, "Robot");
  m.def("make_robot", &make_robot);

  // ---- kinematics/energy primitives (FK/energy parity vs Python) ----
  m.def("fk", [](const Robot& s, const VecN& q) { return eePose(s, q); });
  m.def("jacobian", [](const Robot& s, const VecN& q) {
    std::vector<Eigen::Matrix4d> f; fkChain(s, q, f); return Eigen::MatrixXd(geoJac(s, f));
  });
  m.def("pose_error", [](const Eigen::Matrix4d& Tc, const Eigen::Matrix4d& Tt) {
    return Vec6(poseError(Tc, Tt));
  });
  m.def("self_collision", [](const Robot& s, const VecN& q) { return selfCollisionMinDist(s, q); });
  m.def("total_energy", [](const Robot& s, const VecN& q, const Eigen::Matrix4d& T,
                           double wt, double wl, double wc, double ws) {
    return totalEnergy(s, q, T, wt, wl, wc, ws);
  });
  m.def("frustration", [](const Robot& s, const VecN& q, const Eigen::Matrix4d& T) {
    return VecN(frustrationIndex(s, q, T));
  });

  // ---- solvers ----
  // V4 (protein_fast); o2=true adds the IAM partial-unfold Phase-B warm start.
  m.def("solve_v4", [](const Robot& s, const VecN& q0, const Eigen::Matrix4d& T,
                       uint64_t seed, bool o2) {
    Rng rng(seed);
    return to_dict(solveProteinFast(s, q0, T, rng, 150, 1e-3, 1e-2, 10, 10, 2e-4, 6, o2));
  }, py::arg("robot"), py::arg("q0"), py::arg("T"), py::arg("seed"), py::arg("o2") = false);

#ifdef HAVE_V1
  m.def("solve_v1", [](const Robot& s, const VecN& q0, const Eigen::Matrix4d& T, uint64_t seed) {
    Rng rng(seed);
    return to_dict_pik(solveProteinIK(s, q0, T, rng));
  });
#endif
#ifdef HAVE_HOMOTOPY
  m.def("solve_homotopy", [](const Robot& s, const VecN& q0, const Eigen::Matrix4d& T, uint64_t seed) {
    Rng rng(seed);
    return to_dict_pik(solveHomotopy(s, q0, T, rng));
  });
  m.def("solve_fixed_lambda", [](const Robot& s, const VecN& q0, const Eigen::Matrix4d& T, uint64_t seed) {
    Rng rng(seed);
    return to_dict_pik(solveFixedLambda(s, q0, T, rng));
  });
#endif
#ifdef HAVE_RAW
  m.def("solve_raw", [](const Robot& s, const VecN& q0, const Eigen::Matrix4d& T, uint64_t seed) {
    Rng rng(seed);
    return to_dict_pik(solveProteinRaw(s, q0, T, rng));
  });
#endif
#ifdef HAVE_CCD
  // CCD is deterministic (no RNG); seed accepted for a uniform adapter signature.
  m.def("solve_ccd", [](const Robot& s, const VecN& q0, const Eigen::Matrix4d& T,
                        uint64_t, int max_iters) {
    return to_dict_pik(solveCCD(s, q0, T, max_iters));
  }, py::arg("robot"), py::arg("q0"), py::arg("T"), py::arg("seed"),
     py::arg("max_iters") = 300);
#endif
#ifdef HAVE_FABRIK
  m.def("solve_fabrik", [](const Robot& s, const VecN& q0, const Eigen::Matrix4d& T,
                           uint64_t, int max_iters) {
    return to_dict_pik(solveFABRIK(s, q0, T, max_iters));
  }, py::arg("robot"), py::arg("q0"), py::arg("T"), py::arg("seed"),
     py::arg("max_iters") = 150);
#endif
}
