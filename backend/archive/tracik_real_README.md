# Real TRAC-IK comparison (via WSL)

`trac_ik_style` is our re-implementation of TRAC-IK's KDL-DLS+random-restart half.
This directory can benchmark it against the **genuine** TRAC-IK C++/KDL/NLopt
library (`tracikpy`) on **identical kinematics** (the URDF is generated from our
own UR5 DH table; FK parity is asserted to ~1e-16 at runtime).

## Why WSL
Real TRAC-IK needs the KDL/NLopt/urdfdom C++ stack — not installable natively on
Windows. This machine also has **hardware virtualization disabled in BIOS**, so
WSL2 and Docker don't work. We therefore use **WSL1** + Ubuntu 22.04 installed on
the **F: drive** (`F:\WSL\Ubuntu2204`, distro name `Ubuntu-2204`).

## What was installed (one-time)
- WSL1 Ubuntu-22.04 on F: (`wsl --install Ubuntu-22.04 --location F:\WSL\Ubuntu2204 --version 1`)
- apt: `build-essential cmake swig python3-dev python3-numpy libeigen3-dev
  liborocos-kdl-dev libnlopt-cxx-dev liburdfdom-dev libtinyxml2-dev`
- `tracikpy` (github.com/mjd3/tracikpy), built **ROS-free** with two local patches:
  1. add `#include <iostream>` to its src (newer GCC needs it),
  2. replace ROS `urdf::Model::initString` (pluginlib/dlopen — **segfaults on
     WSL1**) with `urdf::parseURDF` from liburdfdom, plus a vendored header-only
     `kdl_parser::treeFromUrdfModel`. No ROS / AMENT / pluginlib at runtime.

## Reproduce
```powershell
wsl -d Ubuntu-2204 -u root -- bash -lc 'cd /tmp && python3 -u "/mnt/c/Coding Projects/Protien IK/backend/scrap/tracik_real_compare.py" --trials 100 --seed 0 --timeout 0.05'
```

## Result (UR5, 100 reachable targets, identical DH; FK parity ~5e-16)

Three solvers, same targets. Real TRAC-IK at its 50 ms budget; medians over seeds 0–2.

| Solver | Success | Mean pos | Mean ms | p50 ms | p95 ms |
|---|---|---|---|---|---|
| **REAL TRAC-IK** | 100% | 0.002 mm | ~0.6 | ~0.4 | ~1.5 |
| `trac_ik_style` (ours) | 98–100% | ~0.6–1.3 mm | ~12–15 | ~7–8 | ~40–48 |
| **ProteinIK V4** (`protein_fast`) | 99–100% | ~0.25–0.31 mm | ~10–21 | ~3.3–4.3 | ~38–48 |

Per-seed V4 vs real TRAC-IK (mean-latency ratio): seed0 34.6×, seed1 17.8×, seed2 18.1×.
V4 uses far fewer restarts than `trac_ik_style` (0.14–0.22 vs 1.4–2.2 /trial);
95–97% of V4's solves are self-collision-free.

**Reading it (the headline the paper needed):**
- **Success:** V4 **ties** real TRAC-IK (99–100% vs 100%) on reachable UR5 targets.
- **Latency:** V4 is **NOT speed-competitive** with real TRAC-IK — ~8–10× slower
  at the **median** (p50 ~3.3–4.3 ms vs ~0.4 ms) and ~18–35× at the **mean**. V4's
  barrierless fast-path *median* actually beats `trac_ik_style` (~3.3 vs ~7 ms), but
  its Phase-B stochastic-fold escalation on frustrated seeds gives it a **heavy tail**
  (p95 ~40 ms) that real TRAC-IK (p95 ~1.5 ms) never has.
- This directly settles the `00_INDEX.md` flagged-unverified claim
  *"V4 speed-competitive with TRAC-IK on UR5"*: **on the real library, it is not.**

**Caveat:** V4 is Python on WSL1; TRAC-IK is compiled C++. Part of the latency gap
is language/runtime, not algorithm — so this is an *upper bound* on the true
algorithmic gap. Both run in the same WSL env, so the comparison is internally
consistent. `trac_ik_style` remains a faithful success-rate proxy for TRAC-IK.

Timing note: at TRAC-IK's default 5 ms budget it returns in ~0.6 ms mean (it
stops as soon as it converges); the budget is a ceiling, not a target.
