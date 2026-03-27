#include "go2_jump_core/jump_task.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace go2_jump_core {

namespace {

constexpr double kPi = 3.14159265358979323846;

double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double Smooth01(double value) {
  const double x = Clamp(value, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

double Lerp(double a, double b, double alpha) {
  return a + (b - a) * alpha;
}

double DistanceDemandAlpha(const JumpTaskSpec& task) {
  return Clamp((task.objective.target_distance_m - 0.18) / 0.18, 0.0, 1.2);
}

std::string MakeTaskId(const JumpObjective& objective) {
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(3);
  stream << "jump_d" << objective.target_distance_m << "_a"
         << objective.takeoff_angle_deg;
  return stream.str();
}

}  // namespace

const char* PhaseName(JumpPhase phase) {
  switch (phase) {
    case JumpPhase::kCrouch:
      return "crouch";
    case JumpPhase::kPush:
      return "push";
    case JumpPhase::kFlight:
      return "flight";
    case JumpPhase::kLanding:
      return "landing";
    case JumpPhase::kSettle:
      return "settle";
  }
  return "unknown";
}

JumpTaskSpec BuildJumpTaskSpec(const JumpObjective& objective,
                               const JumpTaskConfig& config) {
  JumpTaskSpec task{};
  task.objective = objective;
  task.objective.target_distance_m =
      Clamp(task.objective.target_distance_m, 0.05, 0.80);
  task.objective.takeoff_angle_deg =
      Clamp(task.objective.takeoff_angle_deg, 20.0, 65.0);
  task.objective.takeoff_speed_scale =
      Clamp(task.objective.takeoff_speed_scale, 0.85, 1.30);
  task.objective.max_motion_duration_s =
      Clamp(task.objective.max_motion_duration_s, 0.60, 2.00);
  task.task_id = MakeTaskId(task.objective);

  const double angle_rad = task.objective.takeoff_angle_deg * kPi / 180.0;
  const double ballistic_denominator = std::max(std::sin(2.0 * angle_rad), 0.2);
  const double ballistic_speed =
      std::sqrt(task.objective.target_distance_m * config.gravity_mps2 /
                ballistic_denominator);

  task.target_takeoff_speed_mps =
      ballistic_speed * task.objective.takeoff_speed_scale;
  task.target_takeoff_velocity_x_mps =
      task.target_takeoff_speed_mps * std::cos(angle_rad);
  task.target_takeoff_velocity_z_mps =
      task.target_takeoff_speed_mps * std::sin(angle_rad);
  task.estimated_apex_height_m =
      (task.target_takeoff_velocity_z_mps * task.target_takeoff_velocity_z_mps) /
      (2.0 * std::max(config.gravity_mps2, 1e-6));
  task.estimated_flight_time_s = Clamp(
      2.0 * task.target_takeoff_velocity_z_mps / config.gravity_mps2 +
          config.flight_margin_s,
      0.18, 0.60);

  task.crouch_duration_s = Clamp(
      config.crouch_duration_base_s +
          config.crouch_duration_gain_s_per_m * task.objective.target_distance_m,
      0.16, 0.40);
  task.push_duration_s = Clamp(
      config.push_duration_base_s +
          config.push_duration_gain_s_per_m * task.objective.target_distance_m,
      0.08, 0.24);
  task.landing_duration_s = Clamp(
      config.landing_duration_base_s +
          config.landing_duration_gain_s_per_m * task.objective.target_distance_m,
      0.12, 0.30);
  task.settle_duration_s = Clamp(
      config.settle_duration_base_s +
          config.settle_duration_gain_s_per_m * task.objective.target_distance_m,
      0.18, 0.40);
  task.total_motion_duration_s =
      task.crouch_duration_s + task.push_duration_s + task.estimated_flight_time_s +
      task.landing_duration_s + task.settle_duration_s;
  task.horizon_duration_s = Clamp(
      config.horizon_duration_base_s +
          config.horizon_duration_gain_s_per_m * task.objective.target_distance_m,
      0.60, task.objective.max_motion_duration_s);
  return task;
}

JumpReferenceProfile BuildJumpReferenceProfile(const JumpTaskSpec& task,
                                               const JumpTaskConfig& config) {
  JumpReferenceProfile profile{};
  const double demand_alpha = DistanceDemandAlpha(task);

  profile.effective_takeoff_pitch_deg = Clamp(
      task.objective.target_takeoff_pitch_deg + 2.5 * demand_alpha, -8.0, 4.0);
  profile.push_forward_velocity_mps =
      task.target_takeoff_velocity_x_mps * (1.0 + 0.03 * demand_alpha);
  profile.push_vertical_velocity_mps =
      task.target_takeoff_velocity_z_mps * (1.0 - 0.05 * demand_alpha);
  profile.flight_forward_velocity_mps =
      task.target_takeoff_velocity_x_mps * (1.0 + 0.02 * demand_alpha);
  profile.crouch_height_offset_m = config.crouch_height_offset_m - 0.008 * demand_alpha;
  profile.push_height_offset_m = config.push_height_offset_m - 0.006 * demand_alpha;
  profile.flight_height_offset_m = config.flight_height_offset_m - 0.006 * demand_alpha;
  profile.landing_height_offset_m =
      config.landing_height_offset_m - 0.003 * demand_alpha;
  profile.leg_retraction_scale = Clamp(1.0 - 0.08 * demand_alpha, 0.85, 1.0);
  profile.landing_brace_scale = 1.0 + 0.05 * demand_alpha;
  return profile;
}

JumpReferenceSample SampleJumpReference(const JumpTaskSpec& task,
                                        const JumpTaskConfig& config,
                                        double elapsed_s) {
  JumpReferenceSample sample{};
  const auto profile = BuildJumpReferenceProfile(task, config);
  const double t = std::max(0.0, elapsed_s);
  const double crouch_end = task.crouch_duration_s;
  const double push_end = crouch_end + task.push_duration_s;
  const double flight_end = push_end + task.estimated_flight_time_s;
  const double landing_end = flight_end + task.landing_duration_s;

  if (t < crouch_end) {
    const double alpha = Smooth01(t / std::max(task.crouch_duration_s, 1e-6));
    sample.phase = JumpPhase::kCrouch;
    sample.time_in_phase_s = t;
    sample.desired_body_pitch_deg =
        Lerp(0.0, 0.6 * profile.effective_takeoff_pitch_deg, alpha);
    sample.desired_body_height_offset_m =
        Lerp(0.0, profile.crouch_height_offset_m, alpha);
    return sample;
  }

  if (t < push_end) {
    const double alpha =
        Smooth01((t - crouch_end) / std::max(task.push_duration_s, 1e-6));
    sample.phase = JumpPhase::kPush;
    sample.time_in_phase_s = t - crouch_end;
    sample.desired_forward_velocity_mps =
        Lerp(0.0, profile.push_forward_velocity_mps, alpha);
    sample.desired_vertical_velocity_mps =
        Lerp(0.0, profile.push_vertical_velocity_mps, alpha);
    sample.desired_body_pitch_deg =
        Lerp(0.6 * profile.effective_takeoff_pitch_deg,
             profile.effective_takeoff_pitch_deg, alpha);
    sample.desired_body_height_offset_m =
        Lerp(profile.crouch_height_offset_m, profile.push_height_offset_m, alpha);
    return sample;
  }

  if (t < flight_end) {
    const double alpha =
        (t - push_end) / std::max(task.estimated_flight_time_s, 1e-6);
    const double descent_alpha = Smooth01(std::max(0.0, (alpha - 0.5) / 0.5));
    sample.phase = JumpPhase::kFlight;
    sample.time_in_phase_s = t - push_end;
    sample.desired_forward_velocity_mps = profile.flight_forward_velocity_mps;
    sample.desired_vertical_velocity_mps =
        profile.push_vertical_velocity_mps -
        9.81 * (t - push_end);
    sample.desired_body_pitch_deg =
        Lerp(profile.effective_takeoff_pitch_deg,
             task.objective.target_landing_pitch_deg, descent_alpha);
    sample.desired_body_height_offset_m = Lerp(
        profile.flight_height_offset_m, profile.landing_height_offset_m,
        descent_alpha);
    sample.leg_retraction_ratio = Smooth01(alpha) * profile.leg_retraction_scale;
    sample.landing_brace_factor =
        Clamp(descent_alpha * profile.landing_brace_scale, 0.0, 1.2);
    return sample;
  }

  if (t < landing_end) {
    const double alpha =
        Smooth01((t - flight_end) / std::max(task.landing_duration_s, 1e-6));
    sample.phase = JumpPhase::kLanding;
    sample.time_in_phase_s = t - flight_end;
    sample.desired_body_pitch_deg =
        Lerp(task.objective.target_landing_pitch_deg, 0.0, alpha);
    sample.desired_body_height_offset_m =
        Lerp(profile.landing_height_offset_m, 0.0, alpha);
    sample.landing_brace_factor = Lerp(
        Clamp(profile.landing_brace_scale, 0.8, 1.2), 0.3, alpha);
    return sample;
  }

  const double alpha = Smooth01(
      (t - landing_end) / std::max(task.settle_duration_s, 1e-6));
  sample.phase = JumpPhase::kSettle;
  sample.time_in_phase_s = t - landing_end;
  sample.desired_body_pitch_deg = Lerp(0.0, 0.0, alpha);
  sample.desired_body_height_offset_m = 0.0;
  sample.landing_brace_factor = Lerp(0.3, 0.0, alpha);
  return sample;
}

}  // namespace go2_jump_core
