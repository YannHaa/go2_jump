#pragma once

#include <string>

namespace go2_jump_core {

enum class JumpPhase {
  kCrouch,
  kPush,
  kFlight,
  kLanding,
  kSettle,
};

const char* PhaseName(JumpPhase phase);

struct JumpObjective {
  double target_distance_m{0.25};
  double takeoff_angle_deg{34.0};
  double takeoff_speed_scale{1.0};
  double target_takeoff_pitch_deg{-4.0};
  double target_landing_pitch_deg{0.0};
  double max_motion_duration_s{1.20};
};

struct JumpTaskConfig {
  double gravity_mps2{9.81};
  double crouch_duration_base_s{0.22};
  double crouch_duration_gain_s_per_m{0.18};
  double push_duration_base_s{0.12};
  double push_duration_gain_s_per_m{0.10};
  double landing_duration_base_s{0.18};
  double landing_duration_gain_s_per_m{0.05};
  double settle_duration_base_s{0.24};
  double settle_duration_gain_s_per_m{0.08};
  double horizon_duration_base_s{0.80};
  double horizon_duration_gain_s_per_m{0.25};
  double flight_margin_s{0.04};
  double crouch_height_offset_m{-0.08};
  double push_height_offset_m{0.03};
  double flight_height_offset_m{0.09};
  double landing_height_offset_m{-0.03};
};

struct JumpTaskSpec {
  std::string task_id;
  JumpObjective objective{};
  double target_takeoff_speed_mps{0.0};
  double target_takeoff_velocity_x_mps{0.0};
  double target_takeoff_velocity_z_mps{0.0};
  double estimated_apex_height_m{0.0};
  double estimated_flight_time_s{0.0};
  double crouch_duration_s{0.0};
  double push_duration_s{0.0};
  double landing_duration_s{0.0};
  double settle_duration_s{0.0};
  double horizon_duration_s{0.0};
  double total_motion_duration_s{0.0};
};

struct JumpReferenceSample {
  JumpPhase phase{JumpPhase::kCrouch};
  double time_in_phase_s{0.0};
  double desired_forward_velocity_mps{0.0};
  double desired_vertical_velocity_mps{0.0};
  double desired_body_pitch_deg{0.0};
  double desired_body_height_offset_m{0.0};
  double leg_retraction_ratio{0.0};
  double landing_brace_factor{0.0};
};

struct JumpReferenceProfile {
  double effective_takeoff_pitch_deg{0.0};
  double push_forward_velocity_mps{0.0};
  double push_vertical_velocity_mps{0.0};
  double flight_forward_velocity_mps{0.0};
  double crouch_height_offset_m{0.0};
  double push_height_offset_m{0.0};
  double flight_height_offset_m{0.0};
  double landing_height_offset_m{0.0};
  double leg_retraction_scale{1.0};
  double landing_brace_scale{1.0};
};

JumpTaskSpec BuildJumpTaskSpec(const JumpObjective& objective,
                               const JumpTaskConfig& config);

JumpReferenceProfile BuildJumpReferenceProfile(const JumpTaskSpec& task,
                                               const JumpTaskConfig& config);

JumpReferenceSample SampleJumpReference(const JumpTaskSpec& task,
                                        const JumpTaskConfig& config,
                                        double elapsed_s);

}  // namespace go2_jump_core
