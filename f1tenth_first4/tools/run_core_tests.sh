#!/usr/bin/env bash
# ROS 없이 알고리즘 핵심부만 빠르게 테스트한다 (PR 올리기 전에 실행)
#   bash tools/run_core_tests.sh
set -euo pipefail
cd "$(dirname "$0")/.."
OUT=$(mktemp -d)
g++ -std=c++17 -Wall -Wextra -Wpedantic -Igap_planner/include \
  gap_planner/test/test_gap_core.cpp -o "$OUT/test_gap_core"
g++ -std=c++17 -Wall -Wextra -Wpedantic -Irace_control/include \
  race_control/test/test_control_core.cpp -o "$OUT/test_control_core"
"$OUT/test_gap_core"
"$OUT/test_control_core"
python3 -m pytest -q lane_planner/test
