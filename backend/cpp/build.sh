#!/bin/bash
# Build the native V4-vs-TRAC-IK benchmark inside WSL.
# Links against the TRAC-IK C++ objects already compiled by the tracikpy build
# (ROS-free: vendored kdl_parser + urdfdom parseURDF) plus KDL / NLopt / urdfdom.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
TK=/root/tracikpy
OBJ=$TK/build/temp.linux-x86_64-3.10/tracikpy/src

if [ ! -f "$OBJ/trac_ik.o" ]; then
  echo "ERROR: TRAC-IK objects not found at $OBJ — rebuild tracikpy first." >&2
  exit 1
fi

g++ -O2 -std=c++14 -fPIC \
  -I"$HERE" -I"$TK/tracikpy/include" -I/usr/include/eigen3 \
  "$HERE/bench_v4_vs_tracik.cpp" \
  "$OBJ/trac_ik.o" "$OBJ/nlopt_ik.o" "$OBJ/kdl_tl.o" \
  -lorocos-kdl -lnlopt -lurdfdom_model -lurdfdom_world \
  -o /tmp/bench_v4_vs_tracik

echo "built -> /tmp/bench_v4_vs_tracik"
