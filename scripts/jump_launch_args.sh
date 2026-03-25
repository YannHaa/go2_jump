#!/usr/bin/env bash

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT_DIR}/scripts/jump_profiles.sh"

append_optional_launch_arg() {
  local array_name="$1"
  local launch_name="$2"
  local launch_value="$3"
  local -n launch_args_array="${array_name}"

  if [ -n "${launch_value}" ]; then
    launch_args_array+=("${launch_name}:=${launch_value}")
  fi
}

build_jump_launch_args() {
  local target_distance_m="$1"
  local array_name="$2"
  local -n launch_args_array="${array_name}"

  apply_jump_profile_defaults
  launch_args_array=("target_distance_m:=${target_distance_m}")

  append_optional_launch_arg "${array_name}" "takeoff_angle_deg" \
    "${GO2_JUMP_TAKEOFF_ANGLE_DEG:-}"
  append_optional_launch_arg "${array_name}" "takeoff_speed_scale" \
    "${GO2_JUMP_TAKEOFF_SPEED_SCALE:-}"
  append_optional_launch_arg "${array_name}" "use_takeoff_speed_scale_curve" \
    "${GO2_JUMP_USE_TAKEOFF_SPEED_SCALE_CURVE:-}"
  append_optional_launch_arg "${array_name}" "push_front_tau_scale" \
    "${GO2_JUMP_PUSH_FRONT_TAU_SCALE:-}"
  append_optional_launch_arg "${array_name}" "push_rear_tau_scale" \
    "${GO2_JUMP_PUSH_REAR_TAU_SCALE:-}"
  append_optional_launch_arg "${array_name}" "landing_front_tau_scale" \
    "${GO2_JUMP_LANDING_FRONT_TAU_SCALE:-}"
  append_optional_launch_arg "${array_name}" "landing_rear_tau_scale" \
    "${GO2_JUMP_LANDING_REAR_TAU_SCALE:-}"
  append_optional_launch_arg "${array_name}" "push_pitch_target_deg" \
    "${GO2_JUMP_PUSH_PITCH_TARGET_DEG:-}"
  append_optional_launch_arg "${array_name}" "push_pitch_compactness_gain" \
    "${GO2_JUMP_PUSH_PITCH_COMPACTNESS_GAIN:-}"
  append_optional_launch_arg "${array_name}" "flight_pitch_target_deg" \
    "${GO2_JUMP_FLIGHT_PITCH_TARGET_DEG:-}"
  append_optional_launch_arg "${array_name}" "flight_pitch_compactness_gain" \
    "${GO2_JUMP_FLIGHT_PITCH_COMPACTNESS_GAIN:-}"
  append_optional_launch_arg "${array_name}" "landing_pitch_target_deg" \
    "${GO2_JUMP_LANDING_PITCH_TARGET_DEG:-}"
  append_optional_launch_arg "${array_name}" "landing_pitch_compactness_gain" \
    "${GO2_JUMP_LANDING_PITCH_COMPACTNESS_GAIN:-}"
}
