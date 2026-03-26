#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${GO2_JUMP_IMAGE:-go2-jump-humble:latest}"
TARGET_DISTANCE_M="${1:-0.25}"
SOLVER_BACKEND="${GO2_JUMP_SOLVER_BACKEND:-mujoco_native_mpc}"
ENABLE_LOWCMD_OUTPUT="${GO2_JUMP_ENABLE_LOWCMD_OUTPUT:-true}"
AUTO_START="${GO2_JUMP_AUTO_START:-true}"
TRIAL_DURATION_S="${GO2_JUMP_TRIAL_DURATION_S:-6.0}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
REPORT_DIR="${ROOT_DIR}/reports/trials/${TIMESTAMP}_d${TARGET_DISTANCE_M}_${SOLVER_BACKEND}"
REPORT_JSON="${REPORT_DIR}/summary.json"
SIM_LOG="${REPORT_DIR}/sim.log"
STACK_LOG="${REPORT_DIR}/stack.log"
RECORDER_LOG="${REPORT_DIR}/recorder.log"
SIM_CONTAINER_NAME="go2-jump-trial-sim-$$"
STACK_CONTAINER_NAME="go2-jump-trial-stack-$$"
RECORDER_CONTAINER_NAME="go2-jump-trial-recorder-$$"

mkdir -p "${REPORT_DIR}"

cleanup() {
  collect_logs
  docker rm -f "${RECORDER_CONTAINER_NAME}" >/dev/null 2>&1 || true
  docker rm -f "${STACK_CONTAINER_NAME}" >/dev/null 2>&1 || true
  docker rm -f "${SIM_CONTAINER_NAME}" >/dev/null 2>&1 || true
}

collect_logs() {
  docker logs "${RECORDER_CONTAINER_NAME}" >"${RECORDER_LOG}" 2>&1 || true
  docker logs "${STACK_CONTAINER_NAME}" >"${STACK_LOG}" 2>&1 || true
  docker logs "${SIM_CONTAINER_NAME}" >"${SIM_LOG}" 2>&1 || true
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

echo "[1/6] Starting Unitree MuJoCo simulation"
docker run -d --name "${SIM_CONTAINER_NAME}" --net host --privileged \
  -e UNITREE_MUJOCO_HEADLESS=1 \
  -v "${ROOT_DIR}:/workspace" \
  -w /workspace \
  "${IMAGE_TAG}" \
  bash -lc 'source /workspace/scripts/container_source_env.sh; source /workspace/scripts/container_prepare_unitree_sdk2.sh; source /workspace/scripts/container_prepare_mujoco.sh; unset XAUTHORITY; cd /workspace/src/unitree_mujoco/simulate/build; XVFB_DISPLAY="${UNITREE_MUJOCO_XVFB_DISPLAY:-:199}"; Xvfb "${XVFB_DISPLAY}" -screen 0 1280x720x24 -nolisten tcp -ac >/tmp/unitree_mujoco_xvfb.log 2>&1 & xvfb_pid=$!; trap "kill ${xvfb_pid} 2>/dev/null || true" EXIT; export DISPLAY="${XVFB_DISPLAY}"; sleep 1; exec ./unitree_mujoco -r go2 -s scene.xml' >/dev/null

echo "[2/6] Waiting for simulator state topics"
wait_for_topic /lowstate 20 1
wait_for_topic /sportmodestate 20 1

echo "[3/6] Starting trial recorder"
docker run -d --name "${RECORDER_CONTAINER_NAME}" --net host \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e ROS_LOG_DIR=/tmp/roslog \
  -v "${ROOT_DIR}:/workspace" \
  -w /workspace \
  "${IMAGE_TAG}" \
  bash -lc "source /workspace/scripts/container_source_env.sh && exec python3 /workspace/scripts/record_jump_trial.py --duration ${TRIAL_DURATION_S} --output-json /workspace/reports/trials/${TIMESTAMP}_d${TARGET_DISTANCE_M}_${SOLVER_BACKEND}/summary.json" >/dev/null

sleep 1

echo "[4/6] Starting jump stack"
docker run -d --name "${STACK_CONTAINER_NAME}" --net host \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e ROS_LOG_DIR=/tmp/roslog \
  -v "${ROOT_DIR}:/workspace" \
  -w /workspace \
  "${IMAGE_TAG}" \
  bash -lc "source /workspace/scripts/container_source_env.sh && exec stdbuf -oL -eL ros2 launch go2_jump_bringup sim_jump_mpc.launch.py target_distance_m:=${TARGET_DISTANCE_M} solver_backend:=${SOLVER_BACKEND} enable_lowcmd_output:=${ENABLE_LOWCMD_OUTPUT} auto_start:=${AUTO_START}" >/dev/null

echo "[5/6] Waiting for controller topic and trial completion"
wait_for_topic /go2_jump/controller_state 20 1
docker wait "${RECORDER_CONTAINER_NAME}" >/dev/null

echo "[6/6] Collecting logs"
collect_logs
cat "${RECORDER_LOG}"
echo
echo "Artifacts"
echo "  report: ${REPORT_JSON}"
echo "  stack log: ${STACK_LOG}"
echo "  sim log: ${SIM_LOG}"
