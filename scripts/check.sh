#!/usr/bin/env bash
# Run from any directory. Source the ROS workspace before the gate test.
set -euo pipefail
cd "$(dirname "$0")/.."
check_dir=$(mktemp -d)
trap 'rm -r "$check_dir"' EXIT
for test in scan_health pose_search pose_consensus; do
  "${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Wpedantic \
    -Isrc/aisleops_localization/src -Isrc/aisleops_mission/src \
    "tests/$test.cpp" -o "$check_dir/$test"
  "$check_dir/$test"
done
/usr/bin/python3 scripts/test_trajectory.py
/usr/bin/python3 tests/gate_evidence.py
