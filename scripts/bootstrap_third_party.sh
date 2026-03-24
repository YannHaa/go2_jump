#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
THIRD_PARTY_DIR="${ROOT_DIR}/third_party"
MUJOCO_VERSION="${GO2_JUMP_MUJOCO_VERSION:-3.3.6}"
UNITREE_SDK2_TAG="${GO2_JUMP_UNITREE_SDK2_TAG:-v2.0.2}"
MUJOCO_ARCHIVE_NAME="mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz"
MUJOCO_URL="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/${MUJOCO_ARCHIVE_NAME}"
UNITREE_SDK2_URL="https://github.com/unitreerobotics/unitree_sdk2/archive/refs/tags/${UNITREE_SDK2_TAG}.tar.gz"

mkdir -p "${THIRD_PARTY_DIR}"

if [ ! -f "${THIRD_PARTY_DIR}/${MUJOCO_ARCHIVE_NAME}" ]; then
  echo "Downloading MuJoCo ${MUJOCO_VERSION} archive..."
  curl -L --fail --retry 3 --retry-delay 2 \
    "${MUJOCO_URL}" -o "${THIRD_PARTY_DIR}/${MUJOCO_ARCHIVE_NAME}"
else
  echo "MuJoCo archive already cached."
fi

if [ ! -d "${THIRD_PARTY_DIR}/unitree_sdk2" ]; then
  tmp_dir="$(mktemp -d)"
  archive_path="${THIRD_PARTY_DIR}/unitree_sdk2-${UNITREE_SDK2_TAG}.tar.gz"
  if [ ! -f "${archive_path}" ]; then
    echo "Downloading Unitree SDK2 ${UNITREE_SDK2_TAG} source archive..."
    curl -L --fail --retry 3 --retry-delay 2 \
      "${UNITREE_SDK2_URL}" -o "${archive_path}"
  else
    echo "Unitree SDK2 archive already cached."
  fi
  tar -xzf "${archive_path}" -C "${tmp_dir}"
  extracted_dir="$(find "${tmp_dir}" -mindepth 1 -maxdepth 1 -type d | head -n 1)"
  rm -rf "${THIRD_PARTY_DIR}/unitree_sdk2"
  mv "${extracted_dir}" "${THIRD_PARTY_DIR}/unitree_sdk2"
  rm -rf "${tmp_dir}"
else
  echo "Unitree SDK2 source already cached."
fi

echo "Third-party cache is ready under ${THIRD_PARTY_DIR}."
