#!/usr/bin/env bash

set_jump_profile_env_default() {
  local var_name="$1"
  local default_value="$2"

  if [ -z "${!var_name:-}" ]; then
    printf -v "${var_name}" '%s' "${default_value}"
    export "${var_name}"
  fi
}

resolve_jump_profile_name() {
  printf '%s' "${GO2_JUMP_PROFILE:-config_default}"
}

apply_jump_profile_defaults() {
  local profile_name
  profile_name="$(resolve_jump_profile_name)"

  GO2_JUMP_EFFECTIVE_PROFILE="${profile_name}"

  case "${profile_name}" in
    config_default|yaml_default)
      ;;
    conservative_airborne)
      set_jump_profile_env_default "GO2_JUMP_PUSH_FRONT_TAU_SCALE" "0.96"
      set_jump_profile_env_default "GO2_JUMP_PUSH_REAR_TAU_SCALE" "1.12"
      set_jump_profile_env_default "GO2_JUMP_PUSH_PITCH_TARGET_DEG" "-5.0"
      set_jump_profile_env_default "GO2_JUMP_FLIGHT_PITCH_TARGET_DEG" "-2.0"
      ;;
    aggressive_airborne)
      set_jump_profile_env_default "GO2_JUMP_PUSH_FRONT_TAU_SCALE" "0.96"
      set_jump_profile_env_default "GO2_JUMP_PUSH_REAR_TAU_SCALE" "1.12"
      set_jump_profile_env_default "GO2_JUMP_PUSH_PITCH_TARGET_DEG" "-2.0"
      set_jump_profile_env_default "GO2_JUMP_FLIGHT_PITCH_TARGET_DEG" "0.0"
      ;;
    *)
      echo "Unknown GO2_JUMP_PROFILE='${profile_name}'." >&2
      echo "Supported profiles: config_default, yaml_default, conservative_airborne, aggressive_airborne" >&2
      return 1
      ;;
  esac
}
