#pragma once

#include <array>
#include <string>
#include <vector>

#include "go2_jump_core/jump_task.hpp"

namespace go2_jump_mpc {

constexpr std::size_t kControlledJointCount = 12;
constexpr std::size_t kFullLowCmdMotorCount = 20;

struct RobotObservation {
  bool lowstate_received{false};
  bool sportstate_received{false};
  std::array<double, kControlledJointCount> q{};
  std::array<double, kControlledJointCount> dq{};
  std::array<double, 4> foot_force_est{};
  std::array<bool, 4> foot_contact{};
  bool contact_signal_valid{false};
  std::array<double, 3> body_rpy{};
  std::array<double, 3> body_velocity{};
  std::array<double, 3> position{};
};

struct PreviewStep {
  double time_from_now_s{0.0};
  go2_jump_core::JumpPhase phase{go2_jump_core::JumpPhase::kCrouch};
  double desired_forward_velocity_mps{0.0};
  double desired_vertical_velocity_mps{0.0};
  double desired_body_pitch_deg{0.0};
  double desired_body_height_offset_m{0.0};
  double landing_brace_factor{0.0};
};

struct WholeBodyMpcCommand {
  bool valid{false};
  bool backend_ready{false};
  bool lowcmd_enabled{false};
  std::string backend_name{"reference_preview"};
  go2_jump_core::JumpPhase phase{go2_jump_core::JumpPhase::kCrouch};
  bool contact_signal_valid{false};
  bool contact_override{false};
  std::array<bool, 4> foot_contact{};
  int contact_count{0};
  double desired_forward_velocity_mps{0.0};
  double desired_vertical_velocity_mps{0.0};
  double desired_body_pitch_deg{0.0};
  double desired_body_height_offset_m{0.0};
  std::array<double, kControlledJointCount> q_ref{};
  std::array<double, kControlledJointCount> dq_ref{};
  std::array<double, kControlledJointCount> tau_ff{};
  double uniform_kp{0.0};
  double uniform_kd{0.0};
  std::vector<PreviewStep> preview;
};

struct WholeBodyMpcConfig {
  double control_dt_s{0.005};
  int horizon_steps{40};
  bool enable_lowcmd_output{false};
  bool auto_start{true};
  std::string solver_backend{"reference_preview"};

  double default_kp{42.0};
  double default_kd{4.5};
  double flight_kp{22.0};
  double flight_kd{2.5};
  double landing_kp{58.0};
  double landing_kd{6.0};
  double settle_kp{45.0};
  double settle_kd{5.0};
  double max_feedforward_torque_nm{18.0};
  double contact_force_threshold_n{12.0};
  int flight_contact_count_max{1};
  int touchdown_contact_count_threshold{2};
  double min_flight_time_before_touchdown_s{0.03};
  double settle_vertical_velocity_threshold_mps{0.20};

  go2_jump_core::JumpTaskConfig reference_config{};

  std::array<double, kControlledJointCount> stand_pose{
      -0.0, 0.70, -1.40, 0.0, 0.70, -1.40,
      -0.0, 0.98, -1.72, 0.0, 0.98, -1.72,
  };
  std::array<double, kControlledJointCount> crouch_pose{
      -0.02, 1.12, -2.12, 0.02, 1.12, -2.12,
      -0.02, 1.28, -2.34, 0.02, 1.28, -2.34,
  };
  std::array<double, kControlledJointCount> push_pose{
      -0.01, 0.74, -1.04, 0.01, 0.74, -1.04,
      -0.01, 0.84, -1.12, 0.01, 0.84, -1.12,
  };
  std::array<double, kControlledJointCount> flight_pose{
      -0.04, 0.95, -1.58, 0.04, 0.95, -1.58,
      -0.02, 0.88, -1.48, 0.02, 0.88, -1.48,
  };
  std::array<double, kControlledJointCount> landing_pose{
      -0.05, 1.12, -2.05, 0.05, 1.12, -2.05,
      -0.02, 1.02, -1.92, 0.02, 1.02, -1.92,
  };
  std::array<double, kControlledJointCount> settle_pose{
      -0.01, 0.82, -1.56, 0.01, 0.82, -1.56,
      -0.01, 0.98, -1.82, 0.01, 0.98, -1.82,
  };
};

class WholeBodyMpc {
 public:
  explicit WholeBodyMpc(WholeBodyMpcConfig config);

  void SetTask(const go2_jump_core::JumpTaskSpec& task);
  bool HasTask() const;

  WholeBodyMpcCommand Solve(const RobotObservation& observation,
                            double task_elapsed_s) const;

 private:
  std::array<double, kControlledJointCount> BuildPoseForSample(
      const go2_jump_core::JumpReferenceSample& sample) const;
  std::array<double, kControlledJointCount> BuildFeedforwardForSample(
      const go2_jump_core::JumpTaskSpec& task,
      const go2_jump_core::JumpReferenceSample& sample) const;
  double GainsForPhase(go2_jump_core::JumpPhase phase, bool derivative) const;
  go2_jump_core::JumpReferenceSample ApplyContactOverrides(
      const go2_jump_core::JumpReferenceSample& planned_sample,
      const RobotObservation& observation,
      double task_elapsed_s) const;

  WholeBodyMpcConfig config_;
  go2_jump_core::JumpTaskSpec task_{};
  bool have_task_{false};
};

}  // namespace go2_jump_mpc
