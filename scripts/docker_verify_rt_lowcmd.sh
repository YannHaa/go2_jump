#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${GO2_JUMP_IMAGE:-go2-jump-humble:latest}"
VERIFY_DIR="${ROOT_DIR}/reports/verification"
VERIFY_TAG="go2_rt_lowcmd_verify_$(date +%Y%m%d_%H%M%S)_$$"
SIM_CONTAINER_NAME="${VERIFY_TAG}_sim"
SIM_LOG_PATH="${VERIFY_DIR}/${VERIFY_TAG}_sim.log"
VERIFY_LOG_PATH="${VERIFY_DIR}/${VERIFY_TAG}.txt"

mkdir -p "${VERIFY_DIR}"
"${ROOT_DIR}/scripts/apply_local_patches.sh"

if [ ! -x "${ROOT_DIR}/tools/build/verify_rt_lowcmd" ]; then
  echo "Missing /workspace/tools/build/verify_rt_lowcmd." >&2
  echo "Run ./scripts/docker_build_workspace.sh first." >&2
  exit 1
fi

STALE_CONTAINER_IDS="$(docker ps -aq \
  --filter label=go2_jump.project=go2_jump_ws \
  --filter label=go2_jump.mode=verify_rt_lowcmd)"
if [ -n "${STALE_CONTAINER_IDS}" ]; then
  docker stop ${STALE_CONTAINER_IDS} >/dev/null 2>&1 || true
fi

SIM_CONTAINER_ID=""

cleanup() {
  if [ -n "${SIM_CONTAINER_ID}" ]; then
    docker stop "${SIM_CONTAINER_ID}" >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT

SIM_CONTAINER_ID="$(
  docker run -d --rm --name "${SIM_CONTAINER_NAME}" \
    --label go2_jump.project=go2_jump_ws \
    --label go2_jump.mode=verify_rt_lowcmd \
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

echo "Started MuJoCo container: ${SIM_CONTAINER_ID}"
sleep 3

set +e
docker run --rm --net host \
  -v "${ROOT_DIR}:/workspace" \
  -w /workspace \
  "${IMAGE_TAG}" \
  bash -lc '
    source /workspace/scripts/container_source_env.sh
    export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:${LD_LIBRARY_PATH}
    exec stdbuf -oL -eL /workspace/tools/build/verify_rt_lowcmd lo
  ' >"${VERIFY_LOG_PATH}" 2>&1
VERIFY_EXIT=$?
set -e

docker logs "${SIM_CONTAINER_ID}" >"${SIM_LOG_PATH}" 2>&1 || true

cat "${VERIFY_LOG_PATH}"
echo
echo "Verification log: ${VERIFY_LOG_PATH}"
echo "MuJoCo log: ${SIM_LOG_PATH}"

if [ "${VERIFY_EXIT}" -ne 0 ]; then
  echo "Direct rt/lowcmd verification failed." >&2
  exit "${VERIFY_EXIT}"
fi
