#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${GO2_JUMP_IMAGE:-go2-jump-humble:latest}"
TARGET_DISTANCE_M="${1:-0.25}"
TAKEOFF_SPEED_SCALE="${GO2_JUMP_TAKEOFF_SPEED_SCALE:-}"
TAKEOFF_ANGLE_DEG="${GO2_JUMP_TAKEOFF_ANGLE_DEG:-}"
USE_TAKEOFF_SPEED_SCALE_CURVE="${GO2_JUMP_USE_TAKEOFF_SPEED_SCALE_CURVE:-}"

LAUNCH_ARGS=("target_distance_m:=${TARGET_DISTANCE_M}")
if [ -n "${TAKEOFF_ANGLE_DEG}" ]; then
  LAUNCH_ARGS+=("takeoff_angle_deg:=${TAKEOFF_ANGLE_DEG}")
fi
if [ -n "${TAKEOFF_SPEED_SCALE}" ]; then
  LAUNCH_ARGS+=("takeoff_speed_scale:=${TAKEOFF_SPEED_SCALE}")
fi
if [ -n "${USE_TAKEOFF_SPEED_SCALE_CURVE}" ]; then
  LAUNCH_ARGS+=("use_takeoff_speed_scale_curve:=${USE_TAKEOFF_SPEED_SCALE_CURVE}")
fi

docker run --rm --net host \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e ROS_LOG_DIR=/tmp/roslog \
  -v "${ROOT_DIR}:/workspace" \
  -w /workspace \
  "${IMAGE_TAG}" \
  bash -lc "source /workspace/scripts/container_source_env.sh && exec stdbuf -oL -eL ros2 launch go2_jump_bringup sim_jump.launch.py ${LAUNCH_ARGS[*]}"
