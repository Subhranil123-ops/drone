#!/usr/bin/env bash
set -u

cd "$(dirname "$0")"

export GZ_SIM_RESOURCE_PATH="$HOME/drone-navigation/gazebo/models:$HOME/ardupilot_gazebo/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build"

# SITL TCP endpoint.
export MAVLINK_HOST="${MAVLINK_HOST:-127.0.0.1}"
export MAVLINK_PORT="${MAVLINK_PORT:-5760}"
export GZ_LIDAR_TOPIC="${GZ_LIDAR_TOPIC:-/scan}"

# Autonomous test profile.
export AUTO_ARM="${AUTO_ARM:-1}"
export AUTO_TAKEOFF="${AUTO_TAKEOFF:-1}"
export AUTO_LAND="${AUTO_LAND:-1}"

# Destination is RELATIVE to the startup LOCAL_NED position.
# Change these two values for the event's supplied destination.
export GOAL_NORTH_M="${GOAL_NORTH_M:-20}"
export GOAL_EAST_M="${GOAL_EAST_M:-24}"
export GOAL_ALTITUDE_M="${GOAL_ALTITUDE_M:-3}"

export SAFETY_DISTANCE_M="${SAFETY_DISTANCE_M:-2.2}"
export EMERGENCY_DISTANCE_M="${EMERGENCY_DISTANCE_M:-1.0}"
export MAX_SPEED_MPS="${MAX_SPEED_MPS:-1.2}"
export CONTROL_HZ="${CONTROL_HZ:-10}"

cmake -S . -B build -G Ninja
cmake --build build -j"$(nproc)"

echo
echo "Starting autonomous controller..."
echo "Make sure ArduPilot SITL is already connected to Gazebo on TCP 5760."
echo

exec ./build/drone-navigation
