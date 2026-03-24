#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISTANCES_CSV="${1:-${GO2_JUMP_SWEEP_DISTANCES:-0.15,0.20,0.25,0.30}}"
SCALES_CSV="${2:-${GO2_JUMP_SWEEP_SCALES:-1.00,1.03,1.06,1.09}}"
TRIALS_PER_SETTING="${3:-${GO2_JUMP_SWEEP_TRIALS:-1}}"
OUT_DIR="${ROOT_DIR}/reports/calibration"
RAW_REPORT_PATH="${ROOT_DIR}/reports/jump_metrics/latest_report.txt"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
RAW_CSV_PATH="${OUT_DIR}/speed_scale_sweep_${TIMESTAMP}.csv"
SUMMARY_PATH="${OUT_DIR}/speed_scale_sweep_${TIMESTAMP}_summary.txt"

mkdir -p "${OUT_DIR}"

extract_metric() {
  local key="$1"
  local report_path="$2"
  awk -v key="${key}" '$1 == key ":" {print $2; exit}' "${report_path}"
}

IFS=, read -r -a DISTANCES <<< "${DISTANCES_CSV}"
IFS=, read -r -a SCALES <<< "${SCALES_CSV}"

printf "target_distance_m,takeoff_speed_scale,trial_index,final_forward_displacement_m,distance_error_m,landing_forward_displacement_m,takeoff_forward_displacement_m,airborne_forward_progress_m,post_landing_forward_gain_m,max_airborne_forward_displacement_m,max_height_above_start_m,max_abs_pitch_deg,landing_total_foot_force_est\n" \
  > "${RAW_CSV_PATH}"

for distance in "${DISTANCES[@]}"; do
  for scale in "${SCALES[@]}"; do
    for trial_index in $(seq 1 "${TRIALS_PER_SETTING}"); do
      echo "Running sweep trial: distance=${distance} m scale=${scale} trial=${trial_index}/${TRIALS_PER_SETTING}"
      GO2_JUMP_USE_TAKEOFF_SPEED_SCALE_CURVE=false \
      GO2_JUMP_TAKEOFF_SPEED_SCALE="${scale}" \
        "${ROOT_DIR}/scripts/docker_run_single_jump_trial.sh" "${distance}"

      final_forward_displacement_m="$(extract_metric "final_forward_displacement_m" "${RAW_REPORT_PATH}")"
      distance_error_m="$(extract_metric "distance_error_m" "${RAW_REPORT_PATH}")"
      landing_forward_displacement_m="$(extract_metric "landing_forward_displacement_m" "${RAW_REPORT_PATH}")"
      takeoff_forward_displacement_m="$(extract_metric "takeoff_forward_displacement_m" "${RAW_REPORT_PATH}")"
      airborne_forward_progress_m="$(extract_metric "airborne_forward_progress_m" "${RAW_REPORT_PATH}")"
      post_landing_forward_gain_m="$(extract_metric "post_landing_forward_gain_m" "${RAW_REPORT_PATH}")"
      max_airborne_forward_displacement_m="$(extract_metric "max_airborne_forward_displacement_m" "${RAW_REPORT_PATH}")"
      max_height_above_start_m="$(extract_metric "max_height_above_start_m" "${RAW_REPORT_PATH}")"
      max_abs_pitch_deg="$(extract_metric "max_abs_pitch_deg" "${RAW_REPORT_PATH}")"
      landing_total_foot_force_est="$(extract_metric "landing_total_foot_force_est" "${RAW_REPORT_PATH}")"

      printf "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" \
        "${distance}" "${scale}" "${trial_index}" \
        "${final_forward_displacement_m}" "${distance_error_m}" \
        "${landing_forward_displacement_m}" "${takeoff_forward_displacement_m}" \
        "${airborne_forward_progress_m}" "${post_landing_forward_gain_m}" \
        "${max_airborne_forward_displacement_m}" "${max_height_above_start_m}" \
        "${max_abs_pitch_deg}" "${landing_total_foot_force_est}" \
        >> "${RAW_CSV_PATH}"
    done
  done
done

{
  echo "Raw sweep data: ${RAW_CSV_PATH}"
  echo
  echo "Averages by target distance and takeoff speed scale:"
  awk -F, '
    NR == 1 { next }
    {
      key = $1 FS $2
      count[key] += 1
      final[key] += $4
      error[key] += $5
      landing[key] += $6
      takeoff[key] += $7
      airborne[key] += $8
      post[key] += $9
      airborne_max[key] += $10
      height[key] += $11
      pitch[key] += $12
      landing_force[key] += $13
    }
    END {
      printf "target_m,scale,trials,avg_final_m,avg_error_m,avg_landing_m,avg_takeoff_m,avg_airborne_m,avg_post_landing_m,avg_max_airborne_m,avg_height_m,avg_max_abs_pitch_deg,avg_landing_force_est\n"
      for (key in count) {
        split(key, parts, FS)
        printf "%.3f,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
               parts[1] + 0.0, parts[2] + 0.0, count[key],
               final[key] / count[key], error[key] / count[key],
               landing[key] / count[key], takeoff[key] / count[key],
               airborne[key] / count[key], post[key] / count[key],
               airborne_max[key] / count[key], height[key] / count[key],
               pitch[key] / count[key], landing_force[key] / count[key]
      }
    }
  ' "${RAW_CSV_PATH}" | sort -t, -k1,1n -k2,2n
  echo
  echo "Recommended takeoff_speed_scale by target distance (minimum absolute average final-distance error):"
  awk -F, '
    NR == 1 { next }
    {
      key = $1 FS $2
      count[key] += 1
      final[key] += $4
      landing[key] += $6
      airborne[key] += $8
      post[key] += $9
      pitch[key] += $12
    }
    END {
      for (key in count) {
        split(key, parts, FS)
        target = parts[1] + 0.0
        scale = parts[2] + 0.0
        avg_final = final[key] / count[key]
        avg_landing = landing[key] / count[key]
        avg_airborne = airborne[key] / count[key]
        avg_post = post[key] / count[key]
        avg_pitch = pitch[key] / count[key]
        abs_error = avg_final - target
        if (abs_error < 0.0) {
          abs_error = -abs_error
        }
        if (!(target in best_abs_error) || abs_error < best_abs_error[target]) {
          best_abs_error[target] = abs_error
          best_scale[target] = scale
          best_final[target] = avg_final
          best_landing[target] = avg_landing
          best_airborne[target] = avg_airborne
          best_post[target] = avg_post
          best_pitch[target] = avg_pitch
        }
      }

      printf "target_m,best_scale,avg_final_m,avg_landing_m,avg_airborne_m,avg_post_landing_m,avg_max_abs_pitch_deg,abs_error_m\n"
      for (target in best_scale) {
        printf "%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
               target + 0.0, best_scale[target] + 0.0,
               best_final[target], best_landing[target],
               best_airborne[target], best_post[target],
               best_pitch[target], best_abs_error[target]
      }
    }
  ' "${RAW_CSV_PATH}" | sort -t, -k1,1n
  echo
  echo "Recommended takeoff_speed_scale by target distance (minimum absolute average landing-distance error):"
  awk -F, '
    NR == 1 { next }
    {
      key = $1 FS $2
      count[key] += 1
      final[key] += $4
      landing[key] += $6
      airborne[key] += $8
      post[key] += $9
      pitch[key] += $12
    }
    END {
      for (key in count) {
        split(key, parts, FS)
        target = parts[1] + 0.0
        scale = parts[2] + 0.0
        avg_final = final[key] / count[key]
        avg_landing = landing[key] / count[key]
        avg_airborne = airborne[key] / count[key]
        avg_post = post[key] / count[key]
        avg_pitch = pitch[key] / count[key]
        abs_error = avg_landing - target
        if (abs_error < 0.0) {
          abs_error = -abs_error
        }
        if (!(target in best_abs_error) || abs_error < best_abs_error[target]) {
          best_abs_error[target] = abs_error
          best_scale[target] = scale
          best_final[target] = avg_final
          best_landing[target] = avg_landing
          best_airborne[target] = avg_airborne
          best_post[target] = avg_post
          best_pitch[target] = avg_pitch
        }
      }

      printf "target_m,best_scale,avg_final_m,avg_landing_m,avg_airborne_m,avg_post_landing_m,avg_max_abs_pitch_deg,landing_abs_error_m\n"
      for (target in best_scale) {
        printf "%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
               target + 0.0, best_scale[target] + 0.0,
               best_final[target], best_landing[target],
               best_airborne[target], best_post[target],
               best_pitch[target], best_abs_error[target]
      }
    }
  ' "${RAW_CSV_PATH}" | sort -t, -k1,1n
} | tee "${SUMMARY_PATH}"

echo
echo "Wrote sweep summary to ${SUMMARY_PATH}"
