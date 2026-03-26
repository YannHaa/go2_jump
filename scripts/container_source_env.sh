#!/usr/bin/env bash
set -euo pipefail

safe_source() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

safe_source /opt/ros/humble/setup.bash

if [ -f /workspace/scripts/container_prepare_mujoco.sh ]; then
  safe_source /workspace/scripts/container_prepare_mujoco.sh
fi

if [ -f /workspace/src/unitree_ros2/cyclonedds_ws/install/setup.bash ]; then
  safe_source /workspace/src/unitree_ros2/cyclonedds_ws/install/setup.bash
fi

if [ -f /workspace/install/setup.bash ]; then
  safe_source /workspace/install/setup.bash
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
