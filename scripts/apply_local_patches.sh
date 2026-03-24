#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

apply_patch_if_needed() {
  local repo_rel="$1"
  local patch_rel="$2"
  local repo_dir="${ROOT_DIR}/${repo_rel}"
  local patch_path="${ROOT_DIR}/${patch_rel}"

  if [ ! -d "${repo_dir}" ]; then
    echo "Missing repository path: ${repo_dir}" >&2
    return 1
  fi

  if ! git -C "${repo_dir}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "Path is not a git repository: ${repo_dir}" >&2
    echo "Run: git -C ${ROOT_DIR} submodule update --init --recursive" >&2
    return 1
  fi

  if [ ! -f "${patch_path}" ]; then
    echo "Missing patch file: ${patch_path}" >&2
    return 1
  fi

  if git -C "${repo_dir}" apply --reverse --check "${patch_path}" >/dev/null 2>&1; then
    echo "Patch already applied: ${patch_rel}"
    return 0
  fi

  if git -C "${repo_dir}" apply --check "${patch_path}" >/dev/null 2>&1; then
    git -C "${repo_dir}" apply "${patch_path}"
    echo "Applied patch: ${patch_rel}"
    return 0
  fi

  echo "Failed to apply patch cleanly: ${patch_rel}" >&2
  echo "Repository status for ${repo_rel}:" >&2
  git -C "${repo_dir}" status --short >&2 || true
  return 1
}

apply_patch_if_needed \
  "src/unitree_mujoco" \
  "patches/unitree_mujoco/0001-go2-sim-compat.patch"
