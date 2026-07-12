// Native head-to-head: REAL TRAC-IK (C++/KDL) vs C++ ProteinIK V4, in ONE
// process, on IDENTICAL UR5 kinematics and IDENTICAL reachable targets.
// Both are compiled C++ — this removes the Python/WSL runtime confound and
// measures V4's TRUE algorithmic latency against TRAC-IK.
#include "proteinik_v4.hpp"
#include "trac_ik.hpp"
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <numeric>
#include <vector>
#include <iomanip>
#include <algorithm>

using namespace pik;
static const double POS_TOL = 1e-3, ORIENT_TOL = 1e-2;

// URDF generated from the UR5 DH table (identical to urdf_from_ur5_dh()).
static std::string genUrdf(const Ur5& s) {
  std::ostringstream o;
  o << std::setprecision(17);   // full double round-trip precision for the DH values
  o << "<?xml version=\"1.0\"?>\n<robot name=\"ur5_from_dh\">\n";
  o << "  <link name=\"base_link\"/>\n";
  auto inertialLink = [&](const std::string& nm){
    o << "  <link name=\"" << nm << "\">\n"
      << "    <inertial><mass value=\"1.0\"/><inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\""
         " iyy=\"0.01\" iyz=\"0\" izz=\"0.01\"/><origin xyz=\"0 0 0\" rpy=\"0 0 0\"/></inertial>\n"
      << "  </link>\n";
  };
  for (int i = 1; i <= Ur5::N; ++i) inertialLink("link_" + std::to_string(i));
  inertialLink("tool0");
  for (int i = 1; i <= Ur5::N; ++i) {
    std::string parent = (i == 1) ? "base_link" : ("link_" + std::to_string(i-1));
    double ox = 0, oy = 0, oz = 0, oroll = 0;
    if (i > 1) { ox = s.a[i-2]; oz = s.d[i-2]; oroll = s.alpha[i-2]; }
    o << "  <joint name=\"joint_" << i << "\" type=\"revolute\">\n"
      << "    <parent link=\"" << parent << "\"/><child link=\"link_" << i << "\"/>\n"
      << "    <origin xyz=\"" << ox << " " << oy << " " << oz << "\" rpy=\"" << oroll << " 0 0\"/>\n"
      << "    <axis xyz=\"0 0 1\"/>\n"
      << "    <limit lower=\"" << s.lo[i-1] << "\" upper=\"" << s.hi[i-1]
      << "\" effort=\"100\" velocity=\"3.14\"/>\n  </joint>\n";
  }
  o << "  <joint name=\"joint_tool\" type=\"fixed\">\n"
    << "    <parent link=\"link_" << Ur5::N << "\"/><child link=\"tool0\"/>\n"
    << "    <origin xyz=\"" << s.a[Ur5::N-1] << " 0 " << s.d[Ur5::N-1]
    << "\" rpy=\"" << s.alpha[Ur5::N-1] << " 0 0\"/>\n  </joint>\n";
  o << "</robot>\n";
  return o.str();
}

static KDL::Frame toKDL(const Eigen::Matrix4d& T) {
  return KDL::Frame(
    KDL::Rotation(T(0,0),T(0,1),T(0,2), T(1,0),T(1,1),T(1,2), T(2,0),T(2,1),T(2,2)),
    KDL::Vector(T(0,3),T(1,3),T(2,3)));
}

struct Acc { std::vector<int> ok; std::vector<double> pos, orient, ms; };
static double mean(const std::vector<double>& v){ double s=0; for(double x:v) s+=x; return s/v.size(); }
static double meanOkPos(const std::vector<double>& v){ double s=0; int n=0; for(double x:v) if(std::isfinite(x)){s+=x;++n;} return n?s/n:0; }
static double pctl(std::vector<double> v, double p){ std::sort(v.begin(),v.end()); return v[std::min((size_t)(p/100.0*v.size()),v.size()-1)]; }
static double rate(const std::vector<int>& v){ double s=0; for(int x:v) s+=x; return 100.0*s/v.size(); }

int main(int argc, char** argv) {
  int trials = 100; uint64_t seed = 0; double timeout = 0.05;
  for (int i = 1; i < argc; ++i) {
    if (!strcmp(argv[i], "--trials") && i+1<argc) trials = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--seed") && i+1<argc) seed = strtoull(argv[++i], nullptr, 10);
    else if (!strcmp(argv[i], "--timeout") && i+1<argc) timeout = atof(argv[++i]);
  }

  Ur5 s;
  std::string urdf = genUrdf(s);
  TRAC_IK::TRAC_IK ik("base_link", "tool0", urdf, timeout, 1e-5, TRAC_IK::Speed);
  KDL::Chain chain; ik.getKDLChain(chain);
  printf("Real TRAC-IK: %u KDL joints, budget %.1f ms/solve\n",
         chain.getNrOfJoints(), timeout * 1000);

  // FK parity: our DH FK vs TRAC-IK's KDL chain (sanity, like the Python run)
  {
    Rng pr(seed);
    double maxd = 0;
    KDL::ChainFkSolverPos_recursive fk(chain);
    for (int k = 0; k < 200; ++k) {
      VecN q = pr.randomConfig(s);
      KDL::JntArray ja(Ur5::N); for (int i=0;i<Ur5::N;++i) ja(i)=q[i];
      KDL::Frame f; fk.JntToCart(ja, f);
      Eigen::Matrix4d ours = eePose(s, q);
      double dp = std::max({std::abs(f.p.x()-ours(0,3)), std::abs(f.p.y()-ours(1,3)), std::abs(f.p.z()-ours(2,3))});
      maxd = std::max(maxd, dp);
    }
    printf("FK parity (our DH vs TRAC-IK KDL): max |pos diff| = %.2e -> %s\n\n",
           maxd, maxd < 1e-9 ? "IDENTICAL kinematics" : "DIFFER");
  }

  Rng tgt(seed + 1);            // target generation
  Acc real, v4; std::vector<int> v4_collfree; int v4_rescues = 0;

  for (int t = 0; t < trials; ++t) {
    VecN q_ref = tgt.randomConfig(s);
    Eigen::Matrix4d T = eePose(s, q_ref);   // guaranteed-reachable target
    VecN qinit = tgt.randomConfig(s);

    // --- real TRAC-IK ---
    {
      KDL::JntArray qi(Ur5::N), qout(Ur5::N);
      for (int i=0;i<Ur5::N;++i) qi(i)=qinit[i];
      auto t0 = std::chrono::steady_clock::now();
      int rc = ik.CartToJnt(qi, toKDL(T), qout);
      double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-t0).count();
      real.ms.push_back(ms);
      if (rc > 0) {
        VecN qo(Ur5::N); for (int i=0;i<Ur5::N;++i) qo[i]=qout(i);
        Vec6 e = poseError(eePose(s, qo), T);
        double pe=e.head<3>().norm(), oe=e.tail<3>().norm();
        real.ok.push_back(pe<POS_TOL && oe<ORIENT_TOL); real.pos.push_back(pe); real.orient.push_back(oe);
      } else { real.ok.push_back(0); real.pos.push_back(1.0/0.0); real.orient.push_back(1.0/0.0); }
    }

    // --- C++ ProteinIK V4 ---
    {
      Rng rng(3000 + t);
      auto t0 = std::chrono::steady_clock::now();
      V4Result r = solveProteinFast(s, qinit, T, rng);
      double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-t0).count();
      v4.ms.push_back(ms);
      bool okv = r.success && r.pos_error<POS_TOL && r.orient_error<ORIENT_TOL;
      v4.ok.push_back(okv); v4.pos.push_back(r.pos_error); v4.orient.push_back(r.orient_error);
      v4_rescues += r.restarts;
      if (okv) v4_collfree.push_back(r.min_self_distance >= 0.0);
    }
  }

  printf("=== REAL TRAC-IK vs C++ ProteinIK V4 — UR5 (identical DH), %d reachable targets, seed=%llu ===\n\n",
         trials, (unsigned long long)seed);
  printf("%-26s %9s %13s %18s %9s %8s %8s\n", "Solver","Success%","Mean pos(mm)","Mean orient(mrad)","Mean ms","p50 ms","p95 ms");
  printf("--------------------------------------------------------------------------------------------------\n");
  auto row = [&](const char* nm, Acc& a){
    printf("%-26s %8.1f%% %13.3f %18.3f %9.3f %8.3f %8.3f\n", nm, rate(a.ok),
           1000*meanOkPos(a.pos), 1000*meanOkPos(a.orient), mean(a.ms), pctl(a.ms,50), pctl(a.ms,95));
  };
  row("REAL TRAC-IK (C++)", real);
  row("ProteinIK V4 (C++)", v4);
  double cf = v4_collfree.empty()?0:100.0*std::accumulate(v4_collfree.begin(),v4_collfree.end(),0)/v4_collfree.size();
  printf("\n(V4: %d chaperone rescues total, %.2f/trial, %.0f%% of V4 solves self-collision-free)\n",
         v4_rescues, (double)v4_rescues/trials, cf);
  printf("V4 vs real TRAC-IK: mean latency %.1fx, p50 %.1fx.\n",
         mean(v4.ms)/std::max(mean(real.ms),1e-9), pctl(v4.ms,50)/std::max(pctl(real.ms,50),1e-9));
  return 0;
}
