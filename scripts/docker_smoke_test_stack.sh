#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${GO2_JUMP_IMAGE:-go2-jump-humble:latest}"
TARGET_DISTANCE_M="${1:-0.25}"
ENABLE_LOWCMD_OUTPUT="${GO2_JUMP_ENABLE_LOWCMD_OUTPUT:-false}"
SOLVER_BACKEND="${GO2_JUMP_SOLVER_BACKEND:-reference_preview}"
SIM_CONTAINER_NAME="go2-jump-smoke-sim-$$"
STACK_CONTAINER_NAME="go2-jump-smoke-stack-$$"

cleanup() {
  docker rm -f "${STACK_CONTAINER_NAME}" >/dev/null 2>&1 || true
  docker rm -f "${SIM_CONTAINER_NAME}" >/dev/null 2>&1 || true
}

wait_for_topic() {
  local topic_name="$1"
  local attempts="${2:-20}"
  local sleep_s="${3:-1}"
  local i
  for ((i = 0; i < attempts; ++i)); do
    if docker run --rm --net host \
      --user "$(id -u):$(id -g)" \
      -e HOME=/tmp \
      -e ROS_LOG_DIR=/tmp/roslog \
      -v "${ROOT_DIR}:/workspace" \
      -w /workspace \
      "${IMAGE_TAG}" \
      bash -lc "source /workspace/scripts/container_source_env.sh && timeout 2 ros2 topic echo ${topic_name} --once >/dev/null" >/dev/null 2>&1; then
      return 0
    fi
    sleep "${sleep_s}"
  done
  return 1
}

trap cleanup EXIT INT TERM

echo "[1/4] Starting headless Unitree MuJoCo container"
docker run -d --name "${SIM_CONTAINER_NAME}" --net host --privileged \
  -e UNITREE_MUJOCO_HEADLESS=1 \
  -v "${ROOT_DIR}:/workspace" \
  -w /workspace \
  "${IMAGE_TAG}" \
  bash -lc 'source /workspace/scripts/container_source_env.sh; source /workspace/scripts/container_prepare_unitree_sdk2.sh; source /workspace/scripts/container_prepare_mujoco.sh; unset XAUTHORITY; cd /workspace/src/unitree_mujoco/simulate/build; XVFB_DISPLAY="${UNITREE_MUJOCO_XVFB_DISPLAY:-:199}"; Xvfb "${XVFB_DISPLAY}" -screen 0 1280x720x24 -nolisten tcp -ac >/tmp/unitree_mujoco_xvfb.log 2>&1 & xvfb_pid=$!; trap "kill ${xvfb_pid} 2>/dev/null || true" EXIT; export DISPLAY="${XVFB_DISPLAY}"; sleep 1; exec ./unitree_mujoco -r go2 -s scene.xml' >/dev/null

echo "[2/4] Waiting for /lowstate"
wait_for_topic /lowstate 20 1
echo "       /lowstate is available"

echo "       LowState contact sample:"
docker run --rm --net host \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e ROS_LOG_DIR=/tmp/roslog \
  -v "${ROOT_DIR}:/workspace" \
  -w /workspace \
  "${IMAGE_TAG}" \
  bash -lc 'source /workspace/scripts/container_source_env.sh && timeout 4 ros2 topic echo /lowstate --once'

echo "[3/4] Starting jump stack container"
docker run -d --name "${STACK_CONTAINER_NAME}" --rm --net host \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e ROS_LOG_DIR=/tmp/roslog \
  -v "${ROOT_DIR}:/workspace" \
  -w /workspace \
  "${IMAGE_TAG}" \
  bash -lc "source /workspace/scripts/container_source_env.sh && exec stdbuf -oL -eL ros2 launch go2_jump_bringup sim_jump_mpc.launch.py target_distance_m:=${TARGET_DISTANCE_M} solver_backend:=${SOLVER_BACKEND} enable_lowcmd_output:=${ENABLE_LOWCMD_OUTPUT} auto_start:=true" >/dev/null

echo "[4/4] Waiting for /go2_jump/controller_state"
wait_for_topic /go2_jump/controller_state 20 1
echo "       /go2_jump/controller_state is available"

echo
echo "Controller state sample:"
docker run --rm --net host \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e ROS_LOG_DIR=/tmp/roslog \
  -v "${ROOT_DIR}:/workspace" \
  -w /workspace \
  "${IMAGE_TAG}" \
  bash -lc 'source /workspace/scripts/container_source_env.sh && timeout 4 ros2 topic echo /go2_jump/controller_state --once'

if [ "${ENABLE_LOWCMD_OUTPUT}" = "true" ]; then
  echo
  echo "LowCmd publication rate:"
  docker run --rm --net host \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp \
    -e ROS_LOG_DIR=/tmp/roslog \
    -v "${ROOT_DIR}:/workspace" \
    -w /workspace \
    "${IMAGE_TAG}" \
    bash -lc 'source /workspace/scripts/container_source_env.sh && timeout 4 ros2 topic hz /lowcmd || test $? -eq 124'
fi
