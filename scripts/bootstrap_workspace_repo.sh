#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if git -C "${ROOT_DIR}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  git -C "${ROOT_DIR}" submodule sync --recursive
  git -C "${ROOT_DIR}" submodule update --init --recursive
else
  echo "Root workspace is not a git repository yet. Skipping submodule sync." >&2
fi

"${ROOT_DIR}/scripts/apply_local_patches.sh"
