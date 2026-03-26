#pragma once

#include <array>
#include <memory>
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
  std::array<double, 4> raw_foot_force_est{};
  std::array<double, 4> foot_force_est{};
  std::array<bool, 4> foot_contact{};
  bool contact_signal_valid{false};
  std::array<double, 3> body_rpy{};
  std::array<double, 3> body_velocity{};
  std::array<double, 3> body_angular_velocity{};
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

struct PhaseEventState {
  bool contact_signal_valid{false};
  bool takeoff_latched{false};
  bool touchdown_latched{false};
  bool settle_latched{false};
  double takeoff_time_s{-1.0};
  double touchdown_time_s{-1.0};
  double settle_time_s{-1.0};
  double takeoff_candidate_start_s{-1.0};
  double touchdown_candidate_start_s{-1.0};
  double settle_candidate_start_s{-1.0};
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
  double native_backend_update_interval_s{0.03};
  bool auto_start_require_full_contact{true};
  double auto_start_stance_dwell_s{0.15};
  double auto_start_max_planar_speed_mps{0.20};
  double auto_start_max_vertical_speed_mps{0.12};

  double default_kp{42.0};
  double default_kd{4.5};
  double push_kp{26.0};
  double push_kd{2.8};
  double flight_kp{22.0};
  double flight_kd{2.5};
  double landing_kp{58.0};
  double landing_kd{6.0};
  double settle_kp{45.0};
  double settle_kd{5.0};
  double max_feedforward_torque_nm{18.0};
  double contact_force_threshold_n{8.0};
  double contact_release_threshold_n{3.0};
  double contact_filter_alpha{0.35};
  int contact_stable_cycles{2};
  double min_contact_signal_force_n{1.0};
  int flight_contact_count_max{1};
  int touchdown_contact_count_threshold{2};
  double late_takeoff_window_s{0.30};
  double min_flight_time_before_touchdown_s{0.03};
  double takeoff_latch_dwell_s{0.02};
  double touchdown_latch_dwell_s{0.02};
  double settle_latch_dwell_s{0.04};
  double settle_vertical_velocity_threshold_mps{0.20};
  std::string mujoco_model_path{};
  int mujoco_rollout_steps{18};
  int mujoco_rollout_substeps{2};

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
  ~WholeBodyMpc();

  void SetTask(const go2_jump_core::JumpTaskSpec& task);
  bool HasTask() const;

  WholeBodyMpcCommand Solve(const RobotObservation& observation,
                            double task_elapsed_s);

 private:
  struct MujocoBackend;

  WholeBodyMpcCommand SolveReferencePreview(const RobotObservation& observation,
                                            double task_elapsed_s);
  WholeBodyMpcCommand SolveMujocoSampling(const RobotObservation& observation,
                                          double task_elapsed_s);
  std::array<double, kControlledJointCount> BuildPoseForSample(
      const go2_jump_core::JumpReferenceSample& sample) const;
  std::array<double, kControlledJointCount> BuildFeedforwardForSample(
      const go2_jump_core::JumpTaskSpec& task,
      const go2_jump_core::JumpReferenceSample& sample) const;
  double GainsForPhase(go2_jump_core::JumpPhase phase, bool derivative) const;
  PhaseEventState BuildExecutionPhaseEventState(
      const RobotObservation& observation, double task_elapsed_s);
  void UpdatePhaseEventState(PhaseEventState& state, int contact_count,
                             bool contact_signal_valid,
                             double body_vertical_velocity_mps,
                             double task_elapsed_s) const;
  go2_jump_core::JumpReferenceSample ApplyContactOverrides(
      const go2_jump_core::JumpReferenceSample& planned_sample,
      const RobotObservation& observation, double task_elapsed_s);
  go2_jump_core::JumpReferenceSample ApplyContactOverrides(
      const go2_jump_core::JumpReferenceSample& planned_sample,
      const PhaseEventState& state, int contact_count,
      double body_vertical_velocity_mps, double task_elapsed_s) const;

  WholeBodyMpcConfig config_;
  go2_jump_core::JumpTaskSpec task_{};
  bool have_task_{false};
  PhaseEventState execution_phase_state_{};
  std::unique_ptr<MujocoBackend> mujoco_backend_;
};

}  // namespace go2_jump_mpc
