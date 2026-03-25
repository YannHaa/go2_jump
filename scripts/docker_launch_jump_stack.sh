#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${GO2_JUMP_IMAGE:-go2-jump-humble:latest}"
TARGET_DISTANCE_M="${1:-0.25}"
source "${ROOT_DIR}/scripts/jump_launch_args.sh"

build_jump_launch_args "${TARGET_DISTANCE_M}" LAUNCH_ARGS

echo "Jump profile: ${GO2_JUMP_EFFECTIVE_PROFILE:-config_default}"
echo "Launching jump stack with arguments: ${LAUNCH_ARGS[*]}"

docker run --rm --net host \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e ROS_LOG_DIR=/tmp/roslog \
  -v "${ROOT_DIR}:/workspace" \
  -w /workspace \
  "${IMAGE_TAG}" \
  bash -lc "source /workspace/scripts/container_source_env.sh && exec stdbuf -oL -eL ros2 launch go2_jump_bringup sim_jump.launch.py ${LAUNCH_ARGS[*]}"
