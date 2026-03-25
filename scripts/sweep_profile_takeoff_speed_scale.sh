#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE_NAME="${1:-aggressive_airborne}"
TARGET_DISTANCE_M="${2:-0.25}"
SCALES_CSV="${3:-}"
TRIALS_PER_SETTING="${4:-${GO2_JUMP_SWEEP_TRIALS:-1}}"

default_scales_for_profile() {
  local profile_name="$1"

  case "${profile_name}" in
    aggressive_airborne)
      printf '%s' "0.94,0.97,1.00,1.03"
      ;;
    conservative_airborne|config_default|yaml_default)
      printf '%s' "1.00,1.03,1.06,1.09"
      ;;
    *)
      printf '%s' "0.98,1.00,1.03,1.06"
      ;;
  esac
}

if [ -z "${SCALES_CSV}" ]; then
  SCALES_CSV="$(default_scales_for_profile "${PROFILE_NAME}")"
fi

echo "Running takeoff-speed sweep for profile='${PROFILE_NAME}' at target=${TARGET_DISTANCE_M} m"
echo "Takeoff speed scale candidates: ${SCALES_CSV}"

GO2_JUMP_PROFILE="${PROFILE_NAME}" \
  "${ROOT_DIR}/scripts/sweep_takeoff_speed_scale.sh" \
  "${TARGET_DISTANCE_M}" "${SCALES_CSV}" "${TRIALS_PER_SETTING}"
