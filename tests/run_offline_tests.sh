#!/usr/bin/env bash
# Offline logic tests: real lidar_processor.cpp + navigator.cpp, stubbed Gazebo/MAVLink headers.
# Does NOT test Gazebo, SITL, MAVLink I/O or vehicle dynamics.
set -e
cd "$(dirname "$0")/.."
CXX=${CXX:-g++}; F="-O2 -std=c++17 -Wall -Wextra -Wpedantic -Istubs -Iinclude"
$CXX $F src/lidar_processor.cpp src/navigator.cpp tests/planner_test.cpp -o /tmp/planner_test
$CXX $F src/lidar_processor.cpp src/navigator.cpp tests/closed_loop_sim.cpp -o /tmp/cl_sim
/tmp/planner_test
/tmp/cl_sim 500
echo "--- negative controls (these SHOULD show collisions) ---"
BLIND=1 /tmp/cl_sim 200 || true
MIRROR=1 /tmp/cl_sim 200 || true
