#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET_DISTANCE_M="${1:-${GO2_JUMP_PITCH_CAPTURE_SWEEP_DISTANCE:-0.25}}"
FLIGHT_SCALES_CSV="${2:-${GO2_JUMP_SWEEP_FLIGHT_PITCH_CAPTURE_SCALES:-0.60,0.80,1.00}}"
LANDING_SCALES_CSV="${3:-${GO2_JUMP_SWEEP_LANDING_PITCH_CAPTURE_SCALES:-0.80,1.00,1.20}}"
SUPPORT_SCALES_CSV="${4:-${GO2_JUMP_SWEEP_SUPPORT_PITCH_CAPTURE_SCALES:-0.90,1.15}}"
FADE_DURATIONS_CSV="${5:-${GO2_JUMP_SWEEP_SUPPORT_PITCH_CAPTURE_FADE_S:-0.08,0.12,0.16}}"
TRIALS_PER_SETTING="${6:-${GO2_JUMP_SWEEP_TRIALS:-1}}"
FINAL_ERROR_GATE_M="${GO2_JUMP_PITCH_CAPTURE_FINAL_ERROR_GATE_M:-0.06}"
OUT_DIR="${ROOT_DIR}/reports/calibration"
RAW_REPORT_PATH="${ROOT_DIR}/reports/jump_metrics/latest_report.txt"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
RAW_CSV_PATH="${OUT_DIR}/pitch_capture_sweep_${TIMESTAMP}.csv"
AVG_CSV_PATH="${OUT_DIR}/pitch_capture_sweep_${TIMESTAMP}_avg.csv"
SUMMARY_PATH="${OUT_DIR}/pitch_capture_sweep_${TIMESTAMP}_summary.txt"

mkdir -p "${OUT_DIR}"

extract_metric() {
  local key="$1"
  local report_path="$2"
  awk -v key="${key}" '$1 == key ":" {print $2; exit}' "${report_path}"
}

IFS=, read -r -a FLIGHT_SCALES <<< "${FLIGHT_SCALES_CSV}"
IFS=, read -r -a LANDING_SCALES <<< "${LANDING_SCALES_CSV}"
IFS=, read -r -a SUPPORT_SCALES <<< "${SUPPORT_SCALES_CSV}"
IFS=, read -r -a FADE_DURATIONS <<< "${FADE_DURATIONS_CSV}"

printf "target_distance_m,flight_pitch_capture_gain_scale,landing_pitch_capture_gain_scale,support_pitch_capture_gain_scale,support_pitch_capture_fade_duration_s,trial_index,final_forward_displacement_m,distance_error_m,landing_forward_displacement_m,airborne_forward_progress_m,post_landing_forward_gain_m,landing_pitch_deg,final_pitch_deg,max_abs_pitch_deg,measured_flight_time_s\n" \
  > "${RAW_CSV_PATH}"

for flight_scale in "${FLIGHT_SCALES[@]}"; do
  for landing_scale in "${LANDING_SCALES[@]}"; do
    for support_scale in "${SUPPORT_SCALES[@]}"; do
      for fade_duration in "${FADE_DURATIONS[@]}"; do
        for trial_index in $(seq 1 "${TRIALS_PER_SETTING}"); do
          echo "Running pitch-capture sweep trial: target=${TARGET_DISTANCE_M} m flight=${flight_scale} landing=${landing_scale} support=${support_scale} fade=${fade_duration} s trial=${trial_index}/${TRIALS_PER_SETTING}"
          GO2_JUMP_FLIGHT_PITCH_CAPTURE_GAIN_SCALE="${flight_scale}" \
          GO2_JUMP_LANDING_PITCH_CAPTURE_GAIN_SCALE="${landing_scale}" \
          GO2_JUMP_SUPPORT_PITCH_CAPTURE_GAIN_SCALE="${support_scale}" \
          GO2_JUMP_SUPPORT_PITCH_CAPTURE_FADE_DURATION_S="${fade_duration}" \
            "${ROOT_DIR}/scripts/docker_run_single_jump_trial.sh" "${TARGET_DISTANCE_M}"

          final_forward_displacement_m="$(extract_metric "final_forward_displacement_m" "${RAW_REPORT_PATH}")"
          distance_error_m="$(extract_metric "distance_error_m" "${RAW_REPORT_PATH}")"
          landing_forward_displacement_m="$(extract_metric "landing_forward_displacement_m" "${RAW_REPORT_PATH}")"
          airborne_forward_progress_m="$(extract_metric "airborne_forward_progress_m" "${RAW_REPORT_PATH}")"
          post_landing_forward_gain_m="$(extract_metric "post_landing_forward_gain_m" "${RAW_REPORT_PATH}")"
          landing_pitch_deg="$(extract_metric "landing_pitch_deg" "${RAW_REPORT_PATH}")"
          final_pitch_deg="$(extract_metric "final_pitch_deg" "${RAW_REPORT_PATH}")"
          max_abs_pitch_deg="$(extract_metric "max_abs_pitch_deg" "${RAW_REPORT_PATH}")"
          measured_flight_time_s="$(extract_metric "measured_flight_time_s" "${RAW_REPORT_PATH}")"

          printf "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" \
            "${TARGET_DISTANCE_M}" "${flight_scale}" "${landing_scale}" \
            "${support_scale}" "${fade_duration}" "${trial_index}" \
            "${final_forward_displacement_m}" "${distance_error_m}" \
            "${landing_forward_displacement_m}" "${airborne_forward_progress_m}" \
            "${post_landing_forward_gain_m}" "${landing_pitch_deg}" \
            "${final_pitch_deg}" "${max_abs_pitch_deg}" \
            "${measured_flight_time_s}" >> "${RAW_CSV_PATH}"
        done
      done
    done
  done
done

awk -F, -v target="${TARGET_DISTANCE_M}" '
  NR == 1 { next }
  {
    key = $2 FS $3 FS $4 FS $5
    count[key] += 1
    final[key] += $7
    landing[key] += $9
    airborne[key] += $10
    post[key] += $11
    landing_pitch[key] += $12
    final_pitch[key] += $13
    max_pitch[key] += $14
    flight_time[key] += $15
  }
  END {
    printf "flight_pitch_capture_gain_scale,landing_pitch_capture_gain_scale,support_pitch_capture_gain_scale,support_pitch_capture_fade_duration_s,trials,avg_final_m,avg_abs_final_error_m,avg_landing_m,avg_airborne_m,avg_post_landing_m,avg_landing_pitch_deg,avg_final_pitch_deg,avg_max_abs_pitch_deg,avg_measured_flight_time_s\n"
    for (key in count) {
      split(key, parts, FS)
      avg_final = final[key] / count[key]
      abs_error = avg_final - target
      if (abs_error < 0.0) {
        abs_error = -abs_error
      }
      printf "%.3f,%.3f,%.3f,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
             parts[1] + 0.0, parts[2] + 0.0, parts[3] + 0.0, parts[4] + 0.0,
             count[key], avg_final, abs_error, landing[key] / count[key],
             airborne[key] / count[key], post[key] / count[key],
             landing_pitch[key] / count[key], final_pitch[key] / count[key],
             max_pitch[key] / count[key], flight_time[key] / count[key]
    }
  }
' "${RAW_CSV_PATH}" > "${AVG_CSV_PATH}"

{
  echo "Raw sweep data: ${RAW_CSV_PATH}"
  echo "Averaged sweep data: ${AVG_CSV_PATH}"
  echo
  echo "Top settings by average landing pitch (less nose-down is better):"
  {
    head -n 1 "${AVG_CSV_PATH}"
    tail -n +2 "${AVG_CSV_PATH}" | sort -t, -k11,11gr -k9,9gr -k10,10g | head -n 10
  }
  echo
  echo "Best setting within average final-error gate <= ${FINAL_ERROR_GATE_M} m:"
  FILTERED_ROWS="$(awk -F, -v gate="${FINAL_ERROR_GATE_M}" 'NR > 1 && ($7 + 0.0) <= gate { print }' "${AVG_CSV_PATH}")"
  if [ -n "${FILTERED_ROWS}" ]; then
    echo "$(head -n 1 "${AVG_CSV_PATH}")"
    printf "%s\n" "${FILTERED_ROWS}" | sort -t, -k11,11gr -k9,9gr -k10,10g | head -n 1
  else
    echo "No setting satisfied the final-error gate."
  fi
} | tee "${SUMMARY_PATH}"

echo
echo "Wrote pitch-capture sweep summary to ${SUMMARY_PATH}"
