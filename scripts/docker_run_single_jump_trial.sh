#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${GO2_JUMP_IMAGE:-go2-jump-humble:latest}"
TARGET_DISTANCE_M="${1:-0.25}"
TAKEOFF_SPEED_SCALE="${GO2_JUMP_TAKEOFF_SPEED_SCALE:-}"
TAKEOFF_ANGLE_DEG="${GO2_JUMP_TAKEOFF_ANGLE_DEG:-}"
USE_TAKEOFF_SPEED_SCALE_CURVE="${GO2_JUMP_USE_TAKEOFF_SPEED_SCALE_CURVE:-}"
TRIAL_TIMEOUT_S="${GO2_JUMP_TRIAL_TIMEOUT_S:-15}"
LOWSTATE_WAIT_S="${GO2_JUMP_LOWSTATE_WAIT_S:-10}"
REPORT_DIR="${ROOT_DIR}/reports/jump_metrics"
REPORT_PATH="${REPORT_DIR}/latest_report.txt"
SIM_LOG_PATH="${REPORT_DIR}/latest_sim.log"
STACK_LOG_PATH="${REPORT_DIR}/latest_stack.log"
TRIAL_TAG="go2_jump_trial_$(date +%Y%m%d_%H%M%S)_$$"
SIM_CONTAINER_NAME="${TRIAL_TAG}_sim"
STACK_CONTAINER_NAME="${TRIAL_TAG}_stack"

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

mkdir -p "${REPORT_DIR}"
rm -f "${REPORT_PATH}" "${SIM_LOG_PATH}" "${STACK_LOG_PATH}"

STALE_CONTAINER_IDS="$(docker ps -aq \
  --filter label=go2_jump.project=go2_jump_ws \
  --filter label=go2_jump.mode=single_trial)"
if [ -n "${STALE_CONTAINER_IDS}" ]; then
  docker stop ${STALE_CONTAINER_IDS} >/dev/null 2>&1 || true
fi

SIM_CONTAINER_ID=""
STACK_CONTAINER_ID=""
SIM_LOG_PID=""
STACK_LOG_PID=""

cleanup() {
  if [ -n "${STACK_LOG_PID}" ]; then
    kill "${STACK_LOG_PID}" 2>/dev/null || true
  fi
  if [ -n "${SIM_LOG_PID}" ]; then
    kill "${SIM_LOG_PID}" 2>/dev/null || true
  fi
  if [ -n "${STACK_CONTAINER_ID}" ]; then
    docker stop "${STACK_CONTAINER_ID}" >/dev/null 2>&1 || true
  fi
  if [ -n "${SIM_CONTAINER_ID}" ]; then
    docker stop "${SIM_CONTAINER_ID}" >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT

SIM_CONTAINER_ID="$(
  docker run -d --rm --name "${SIM_CONTAINER_NAME}" \
    --label go2_jump.project=go2_jump_ws \
    --label go2_jump.mode=single_trial \
    --label go2_jump.role=sim \
    --net host --privileged \
    -v "${ROOT_DIR}:/workspace" \
    -w /workspace \
    "${IMAGE_TAG}" \
    bash -lc '
      source /workspace/scripts/container_source_env.sh
      export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:${LD_LIBRARY_PATH}
      unset XAUTHORITY
      cd /workspace/src/unitree_mujoco/simulate/build
      XVFB_DISPLAY="${UNITREE_MUJOCO_XVFB_DISPLAY:-:199}"
      Xvfb "${XVFB_DISPLAY}" -screen 0 1280x720x24 -nolisten tcp -ac >/tmp/unitree_mujoco_xvfb.log 2>&1 &
      xvfb_pid=$!
      trap "kill ${xvfb_pid} 2>/dev/null || true" EXIT
      export DISPLAY="${XVFB_DISPLAY}"
      sleep 1
      exec stdbuf -oL -eL ./unitree_mujoco -r go2 -s scene.xml
    '
)"

docker logs -f "${SIM_CONTAINER_ID}" >"${SIM_LOG_PATH}" 2>&1 &
SIM_LOG_PID=$!

echo "Started MuJoCo container: ${SIM_CONTAINER_ID}"

set +e
docker run --rm --net host \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp \
    -e ROS_LOG_DIR=/tmp/roslog \
    -e GO2_JUMP_LOWSTATE_WAIT_S="${LOWSTATE_WAIT_S}" \
    -v "${ROOT_DIR}:/workspace" \
    -w /workspace \
    "${IMAGE_TAG}" \
    bash -lc '
      source /workspace/scripts/container_source_env.sh
      python3 - <<'"'"'PY'"'"'
import rclpy
import os
from rclpy.node import Node
from unitree_go.msg import LowState

class Probe(Node):
    def __init__(self):
        super().__init__("wait_lowstate_probe")
        self.ready = False
        self.sub = self.create_subscription(LowState, "/lowstate", self.cb, 10)

    def cb(self, msg):
        if msg.tick > 100:
            self.ready = True

rclpy.init()
node = Probe()
wait_s = float(os.environ.get("GO2_JUMP_LOWSTATE_WAIT_S", "10"))
end_ns = node.get_clock().now().nanoseconds + int(wait_s * 1e9)
while rclpy.ok() and not node.ready and node.get_clock().now().nanoseconds < end_ns:
    rclpy.spin_once(node, timeout_sec=0.1)
if not node.ready:
    raise SystemExit("timed out waiting for steady /lowstate")
node.destroy_node()
rclpy.shutdown()
PY
    '
LOWSTATE_WAIT_EXIT=$?
set -e

if [ "${LOWSTATE_WAIT_EXIT}" -ne 0 ]; then
  echo "Timed out waiting for steady /lowstate from MuJoCo." >&2
  tail -n 120 "${SIM_LOG_PATH}" >&2 || true
  exit "${LOWSTATE_WAIT_EXIT}"
fi

echo "MuJoCo bridge is publishing steady /lowstate. Running trial at ${TARGET_DISTANCE_M} m."

STACK_CONTAINER_ID="$(
  docker run -d --rm --name "${STACK_CONTAINER_NAME}" \
    --label go2_jump.project=go2_jump_ws \
    --label go2_jump.mode=single_trial \
    --label go2_jump.role=stack \
    --net host \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp \
    -e ROS_LOG_DIR=/tmp/roslog \
    -v "${ROOT_DIR}:/workspace" \
    -w /workspace \
    "${IMAGE_TAG}" \
    bash -lc "source /workspace/scripts/container_source_env.sh && exec stdbuf -oL -eL ros2 launch go2_jump_bringup sim_jump.launch.py ${LAUNCH_ARGS[*]}"
)"

docker logs -f "${STACK_CONTAINER_ID}" >"${STACK_LOG_PATH}" 2>&1 &
STACK_LOG_PID=$!

REPORT_READY=0
DEADLINE_EPOCH_S="$(( $(date +%s) + TRIAL_TIMEOUT_S ))"

while [ "$(date +%s)" -lt "${DEADLINE_EPOCH_S}" ]; do
  if [ -s "${REPORT_PATH}" ]; then
    REPORT_READY=1
    break
  fi

  if ! docker ps -q --no-trunc | grep -q "^${STACK_CONTAINER_ID}$"; then
    break
  fi

  sleep 0.2
done

if docker ps -q --no-trunc | grep -q "^${STACK_CONTAINER_ID}$"; then
  docker stop "${STACK_CONTAINER_ID}" >/dev/null 2>&1 || true
fi
STACK_CONTAINER_ID=""

if [ -n "${STACK_LOG_PID}" ]; then
  wait "${STACK_LOG_PID}" 2>/dev/null || true
  STACK_LOG_PID=""
fi

cat "${STACK_LOG_PATH}"

if [ ! -f "${REPORT_PATH}" ]; then
  echo "Trial report was not produced at ${REPORT_PATH}." >&2
  tail -n 120 "${SIM_LOG_PATH}" >&2 || true
  exit 1
fi

if [ "${REPORT_READY}" -ne 1 ]; then
  echo "Trial report was not produced within ${TRIAL_TIMEOUT_S} s." >&2
  exit 1
fi

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
ARCHIVE_PATH="${REPORT_DIR}/trial_${TIMESTAMP}.txt"
cp "${REPORT_PATH}" "${ARCHIVE_PATH}"

echo
echo "Archived trial report: ${ARCHIVE_PATH}"
cat "${REPORT_PATH}"
