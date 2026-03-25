#!/usr/bin/env bash

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT_DIR}/scripts/jump_profiles.sh"

append_optional_launch_arg() {
  local array_name="$1"
  local launch_name="$2"
  local launch_value="$3"
  local -n launch_args_array="${array_name}"
  local normalized_value="${launch_value}"

  if [ -n "${launch_value}" ]; then
    if [[ "${launch_value}" =~ ^-?[0-9]+$ ]]; then
      normalized_value="${launch_value}.0"
    fi
    launch_args_array+=("${launch_name}:=${normalized_value}")
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
  append_optional_launch_arg "${array_name}" "push_pitch_rate_gain" \
    "${GO2_JUMP_PUSH_PITCH_RATE_GAIN:-}"
  append_optional_launch_arg "${array_name}" "push_pitch_correction_limit_rad" \
    "${GO2_JUMP_PUSH_PITCH_CORRECTION_LIMIT_RAD:-}"
  append_optional_launch_arg "${array_name}" "flight_pitch_target_deg" \
    "${GO2_JUMP_FLIGHT_PITCH_TARGET_DEG:-}"
  append_optional_launch_arg "${array_name}" "flight_pitch_compactness_gain" \
    "${GO2_JUMP_FLIGHT_PITCH_COMPACTNESS_GAIN:-}"
  append_optional_launch_arg "${array_name}" "flight_pitch_rate_gain" \
    "${GO2_JUMP_FLIGHT_PITCH_RATE_GAIN:-}"
  append_optional_launch_arg "${array_name}" "flight_pitch_correction_limit_rad" \
    "${GO2_JUMP_FLIGHT_PITCH_CORRECTION_LIMIT_RAD:-}"
  append_optional_launch_arg "${array_name}" "flight_landing_prep_height_m" \
    "${GO2_JUMP_FLIGHT_LANDING_PREP_HEIGHT_M:-}"
  append_optional_launch_arg "${array_name}" \
    "flight_landing_prep_start_descent_speed_mps" \
    "${GO2_JUMP_FLIGHT_LANDING_PREP_START_DESCENT_SPEED_MPS:-}"
  append_optional_launch_arg "${array_name}" \
    "flight_landing_prep_full_descent_speed_mps" \
    "${GO2_JUMP_FLIGHT_LANDING_PREP_FULL_DESCENT_SPEED_MPS:-}"
  append_optional_launch_arg "${array_name}" "flight_landing_prep_max_blend" \
    "${GO2_JUMP_FLIGHT_LANDING_PREP_MAX_BLEND:-}"
  append_optional_launch_arg "${array_name}" "landing_pitch_target_deg" \
    "${GO2_JUMP_LANDING_PITCH_TARGET_DEG:-}"
  append_optional_launch_arg "${array_name}" "landing_pitch_compactness_gain" \
    "${GO2_JUMP_LANDING_PITCH_COMPACTNESS_GAIN:-}"
  append_optional_launch_arg "${array_name}" "landing_pitch_rate_gain" \
    "${GO2_JUMP_LANDING_PITCH_RATE_GAIN:-}"
  append_optional_launch_arg "${array_name}" "landing_pitch_correction_limit_rad" \
    "${GO2_JUMP_LANDING_PITCH_CORRECTION_LIMIT_RAD:-}"
  append_optional_launch_arg "${array_name}" "support_pitch_target_deg" \
    "${GO2_JUMP_SUPPORT_PITCH_TARGET_DEG:-}"
  append_optional_launch_arg "${array_name}" "support_pitch_compactness_gain" \
    "${GO2_JUMP_SUPPORT_PITCH_COMPACTNESS_GAIN:-}"
  append_optional_launch_arg "${array_name}" "support_pitch_rate_gain" \
    "${GO2_JUMP_SUPPORT_PITCH_RATE_GAIN:-}"
  append_optional_launch_arg "${array_name}" "support_pitch_correction_limit_rad" \
    "${GO2_JUMP_SUPPORT_PITCH_CORRECTION_LIMIT_RAD:-}"
  append_optional_launch_arg "${array_name}" "crouch_forward_bias_rad" \
    "${GO2_JUMP_CROUCH_FORWARD_BIAS_RAD:-}"
  append_optional_launch_arg "${array_name}" "push_forward_bias_rad" \
    "${GO2_JUMP_PUSH_FORWARD_BIAS_RAD:-}"
  append_optional_launch_arg "${array_name}" "push_front_compact_delta_rad" \
    "${GO2_JUMP_PUSH_FRONT_COMPACT_DELTA_RAD:-}"
  append_optional_launch_arg "${array_name}" "push_rear_compact_delta_rad" \
    "${GO2_JUMP_PUSH_REAR_COMPACT_DELTA_RAD:-}"
  append_optional_launch_arg "${array_name}" "flight_front_compact_delta_rad" \
    "${GO2_JUMP_FLIGHT_FRONT_COMPACT_DELTA_RAD:-}"
  append_optional_launch_arg "${array_name}" "flight_rear_compact_delta_rad" \
    "${GO2_JUMP_FLIGHT_REAR_COMPACT_DELTA_RAD:-}"
  append_optional_launch_arg "${array_name}" "landing_front_compact_delta_rad" \
    "${GO2_JUMP_LANDING_FRONT_COMPACT_DELTA_RAD:-}"
  append_optional_launch_arg "${array_name}" "landing_rear_compact_delta_rad" \
    "${GO2_JUMP_LANDING_REAR_COMPACT_DELTA_RAD:-}"
  append_optional_launch_arg "${array_name}" "landing_absorption_blend" \
    "${GO2_JUMP_LANDING_ABSORPTION_BLEND:-}"
  append_optional_launch_arg "${array_name}" "support_hip_rad" \
    "${GO2_JUMP_SUPPORT_HIP_RAD:-}"
  append_optional_launch_arg "${array_name}" "support_thigh_rad" \
    "${GO2_JUMP_SUPPORT_THIGH_RAD:-}"
  append_optional_launch_arg "${array_name}" "support_calf_rad" \
    "${GO2_JUMP_SUPPORT_CALF_RAD:-}"
  append_optional_launch_arg "${array_name}" "support_front_compact_delta_rad" \
    "${GO2_JUMP_SUPPORT_FRONT_COMPACT_DELTA_RAD:-}"
  append_optional_launch_arg "${array_name}" "support_rear_compact_delta_rad" \
    "${GO2_JUMP_SUPPORT_REAR_COMPACT_DELTA_RAD:-}"
  append_optional_launch_arg "${array_name}" "landing_kd" \
    "${GO2_JUMP_LANDING_KD:-}"
  append_optional_launch_arg "${array_name}" "support_kp" \
    "${GO2_JUMP_SUPPORT_KP:-}"
  append_optional_launch_arg "${array_name}" "support_kd" \
    "${GO2_JUMP_SUPPORT_KD:-}"
  append_optional_launch_arg "${array_name}" "recovery_kd" \
    "${GO2_JUMP_RECOVERY_KD:-}"
  append_optional_launch_arg "${array_name}" "recovery_release_forward_speed_mps" \
    "${GO2_JUMP_RECOVERY_RELEASE_FORWARD_SPEED_MPS:-}"
  append_optional_launch_arg "${array_name}" "recovery_release_pitch_deg" \
    "${GO2_JUMP_RECOVERY_RELEASE_PITCH_DEG:-}"
  append_optional_launch_arg "${array_name}" "recovery_release_pitch_rate_degps" \
    "${GO2_JUMP_RECOVERY_RELEASE_PITCH_RATE_DEGPS:-}"
  append_optional_launch_arg "${array_name}" "recovery_max_hold_s" \
    "${GO2_JUMP_RECOVERY_MAX_HOLD_S:-}"
  append_optional_launch_arg "${array_name}" "recovery_upright_blend" \
    "${GO2_JUMP_RECOVERY_UPRIGHT_BLEND:-}"
  append_optional_launch_arg "${array_name}" "landing_support_blend" \
    "${GO2_JUMP_LANDING_SUPPORT_BLEND:-}"
  append_optional_launch_arg "${array_name}" "support_relax_duration_s" \
    "${GO2_JUMP_SUPPORT_RELAX_DURATION_S:-}"
  append_optional_launch_arg "${array_name}" "landing_touchdown_reference_blend" \
    "${GO2_JUMP_LANDING_TOUCHDOWN_REFERENCE_BLEND:-}"
  append_optional_launch_arg "${array_name}" "landing_hold_use_touchdown_pose" \
    "${GO2_JUMP_LANDING_HOLD_USE_TOUCHDOWN_POSE:-}"
}
