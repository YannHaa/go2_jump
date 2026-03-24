#!/usr/bin/env bash
set -euo pipefail

CACHE_DIR="${GO2_JUMP_THIRD_PARTY_CACHE_DIR:-/opt/third_party-cache}"
WORK_DIR="${GO2_JUMP_THIRD_PARTY_WORK_DIR:-/opt/src}"
MUJOCO_VERSION="${GO2_JUMP_MUJOCO_VERSION:-3.3.6}"
UNITREE_SDK2_TAG="${GO2_JUMP_UNITREE_SDK2_TAG:-v2.0.2}"
MUJOCO_ARCHIVE_NAME="mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz"
MUJOCO_URL="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/${MUJOCO_ARCHIVE_NAME}"
UNITREE_SDK2_URL="https://github.com/unitreerobotics/unitree_sdk2/archive/refs/tags/${UNITREE_SDK2_TAG}.tar.gz"

mkdir -p "${CACHE_DIR}" "${WORK_DIR}"

stage_archive_extract() {
  local archive_path="$1"
  local destination_path="$2"
  local tmp_dir
  tmp_dir="$(mktemp -d)"
  tar -xzf "${archive_path}" -C "${tmp_dir}"
  local extracted_dir
  extracted_dir="$(find "${tmp_dir}" -mindepth 1 -maxdepth 1 -type d | head -n 1)"
  rm -rf "${destination_path}"
  mv "${extracted_dir}" "${destination_path}"
  rm -rf "${tmp_dir}"
}

fetch_mujoco_archive() {
  local cache_path="${CACHE_DIR}/${MUJOCO_ARCHIVE_NAME}"
  if [ ! -f "${cache_path}" ]; then
    echo "Downloading MuJoCo ${MUJOCO_VERSION} from official release..."
    curl -L --fail --retry 3 --retry-delay 2 \
      "${MUJOCO_URL}" -o "${cache_path}"
  else
    echo "Using cached MuJoCo archive: ${cache_path}"
  fi
}

fetch_unitree_sdk2_source() {
  local cache_source_dir="${CACHE_DIR}/unitree_sdk2"
  if [ -d "${cache_source_dir}" ]; then
    echo "Using cached Unitree SDK2 source: ${cache_source_dir}"
    rm -rf "${WORK_DIR}/unitree_sdk2"
    cp -a "${cache_source_dir}" "${WORK_DIR}/unitree_sdk2"
    return
  fi

  local archive_cache_path="${CACHE_DIR}/unitree_sdk2-${UNITREE_SDK2_TAG}.tar.gz"
  if [ ! -f "${archive_cache_path}" ]; then
    echo "Downloading Unitree SDK2 ${UNITREE_SDK2_TAG} from official source..."
    curl -L --fail --retry 3 --retry-delay 2 \
      "${UNITREE_SDK2_URL}" -o "${archive_cache_path}"
  else
    echo "Using cached Unitree SDK2 archive: ${archive_cache_path}"
  fi

  stage_archive_extract "${archive_cache_path}" "${WORK_DIR}/unitree_sdk2"
}

fetch_mujoco_archive
fetch_unitree_sdk2_source

echo "Third-party dependencies are ready."
