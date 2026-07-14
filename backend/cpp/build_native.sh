#!/bin/bash
# Build the pik_native pybind11 extension (all ProteinIK C++ solvers + CCD/FABRIK)
# inside WSL Ubuntu-2204. Run:
#   wsl -d Ubuntu-2204 -u root -- bash "/mnt/c/Coding Projects/Protien IK/backend/cpp/build_native.sh"
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"
EXT="$(python3 -c 'import sysconfig; print(sysconfig.get_config_var("EXT_SUFFIX"))')"
echo "extension suffix: $EXT"
g++ -O3 -shared -std=c++17 -fPIC \
  $(python3 -m pybind11 --includes) \
  -I/usr/include/eigen3 -I"$HERE" \
  "$HERE/pik_native.cpp" \
  -o "$HERE/pik_native$EXT"
echo "built -> $HERE/pik_native$EXT"
python3 -c 'import sys; sys.path.insert(0, "'"$HERE"'"); import pik_native as pn; print("solve_ccd:", hasattr(pn,"solve_ccd"), "| solve_fabrik:", hasattr(pn,"solve_fabrik"), "| solve_v4:", hasattr(pn,"solve_v4"))'
