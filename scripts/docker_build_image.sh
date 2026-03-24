#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${GO2_JUMP_IMAGE:-go2-jump-humble:latest}"

docker build \
  -t "${IMAGE_TAG}" \
  -f "${ROOT_DIR}/docker/Dockerfile.humble" \
  "${ROOT_DIR}"
