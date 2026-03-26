#include "go2_jump_mpc/whole_body_mpc.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace go2_jump_mpc {

namespace {

constexpr double kPi = 3.14159265358979323846;

double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double Lerp(double a, double b, double alpha) {
  return a + (b - a) * Clamp(alpha, 0.0, 1.0);
}

std::array<double, kControlledJointCount> BlendPose(
    const std::array<double, kControlledJointCount>& a,
    const std::array<double, kControlledJointCount>& b,
    double alpha) {
  std::array<double, kControlledJointCount> out{};
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = Lerp(a[i], b[i], alpha);
  }
  return out;
}

int CountContacts(const std::array<bool, 4>& foot_contact) {
  return static_cast<int>(std::count(foot_contact.begin(), foot_contact.end(), true));
}

}  // namespace

WholeBodyMpc::WholeBodyMpc(WholeBodyMpcConfig config)
    : config_(std::move(config)) {}

void WholeBodyMpc::SetTask(const go2_jump_core::JumpTaskSpec& task) {
  task_ = task;
  have_task_ = true;
}

bool WholeBodyMpc::HasTask() const { return have_task_; }

WholeBodyMpcCommand WholeBodyMpc::Solve(const RobotObservation& observation,
                                        double task_elapsed_s) const {
  WholeBodyMpcCommand command{};
  if (!have_task_ || !observation.lowstate_received) {
    return command;
  }

  command.valid = true;
  command.backend_name = config_.solver_backend;
  command.backend_ready = (config_.solver_backend == "reference_preview");
  command.lowcmd_enabled = config_.enable_lowcmd_output;
  command.contact_signal_valid = observation.contact_signal_valid;
  command.foot_contact = observation.foot_contact;
  command.contact_count = CountContacts(observation.foot_contact);

  const auto planned_sample = go2_jump_core::SampleJumpReference(
      task_, config_.reference_config, task_elapsed_s);
  const auto current_sample =
      ApplyContactOverrides(planned_sample, observation, task_elapsed_s);
  command.contact_override = (current_sample.phase != planned_sample.phase);
  command.phase = current_sample.phase;
  command.desired_forward_velocity_mps =
      current_sample.desired_forward_velocity_mps;
  command.desired_vertical_velocity_mps =
      current_sample.desired_vertical_velocity_mps;
  command.desired_body_pitch_deg = current_sample.desired_body_pitch_deg;
  command.desired_body_height_offset_m =
      current_sample.desired_body_height_offset_m;
  command.q_ref = BuildPoseForSample(current_sample);
  command.dq_ref.fill(0.0);
  command.tau_ff = BuildFeedforwardForSample(task_, current_sample);
  command.uniform_kp = GainsForPhase(current_sample.phase, false);
  command.uniform_kd = GainsForPhase(current_sample.phase, true);

  command.preview.reserve(std::max(config_.horizon_steps, 1));
  for (int step = 0; step < std::max(config_.horizon_steps, 1); ++step) {
    const double time_from_now = step * config_.control_dt_s;
    const auto sample = go2_jump_core::SampleJumpReference(
        task_, config_.reference_config, task_elapsed_s + time_from_now);
    command.preview.push_back(PreviewStep{
        time_from_now,
        sample.phase,
        sample.desired_forward_velocity_mps,
        sample.desired_vertical_velocity_mps,
        sample.desired_body_pitch_deg,
        sample.desired_body_height_offset_m,
        sample.landing_brace_factor,
    });
  }

  return command;
}

std::array<double, kControlledJointCount> WholeBodyMpc::BuildPoseForSample(
    const go2_jump_core::JumpReferenceSample& sample) const {
  std::array<double, kControlledJointCount> pose = config_.stand_pose;
  switch (sample.phase) {
    case go2_jump_core::JumpPhase::kCrouch:
      pose = BlendPose(config_.stand_pose, config_.crouch_pose, 0.9);
      break;
    case go2_jump_core::JumpPhase::kPush:
      pose = config_.push_pose;
      break;
    case go2_jump_core::JumpPhase::kFlight:
      pose = BlendPose(config_.flight_pose, config_.landing_pose,
                       sample.landing_brace_factor);
      break;
    case go2_jump_core::JumpPhase::kLanding:
      pose = config_.landing_pose;
      break;
    case go2_jump_core::JumpPhase::kSettle:
      pose = BlendPose(config_.landing_pose, config_.settle_pose, 0.8);
      break;
  }

  const double pitch_rad = sample.desired_body_pitch_deg * kPi / 180.0;
  const double hip_delta = Clamp(0.18 * pitch_rad, -0.10, 0.10);
  const double thigh_delta = Clamp(0.28 * pitch_rad, -0.14, 0.14);
  const double calf_delta = Clamp(-0.40 * pitch_rad, -0.18, 0.18);

  pose[0] -= hip_delta;
  pose[3] += hip_delta;
  pose[6] -= 0.5 * hip_delta;
  pose[9] += 0.5 * hip_delta;

  pose[1] += thigh_delta;
  pose[4] += thigh_delta;
  pose[7] -= thigh_delta;
  pose[10] -= thigh_delta;

  pose[2] += calf_delta;
  pose[5] += calf_delta;
  pose[8] -= 0.6 * calf_delta;
  pose[11] -= 0.6 * calf_delta;

  return pose;
}

std::array<double, kControlledJointCount> WholeBodyMpc::BuildFeedforwardForSample(
    const go2_jump_core::JumpTaskSpec& task,
    const go2_jump_core::JumpReferenceSample& sample) const {
  std::array<double, kControlledJointCount> tau{};
  const double push_scale = Clamp(
      task.target_takeoff_speed_mps / 2.2, 0.0, 1.0);
  const double landing_scale = Clamp(sample.landing_brace_factor, 0.0, 1.0);

  if (sample.phase == go2_jump_core::JumpPhase::kPush) {
    const double thigh_tau =
        Clamp(4.0 + 5.0 * push_scale, 0.0, config_.max_feedforward_torque_nm);
    const double calf_tau =
        Clamp(7.0 + 7.0 * push_scale, 0.0, config_.max_feedforward_torque_nm);
    for (std::size_t idx : {1u, 4u, 7u, 10u}) {
      tau[idx] = thigh_tau;
    }
    for (std::size_t idx : {2u, 5u, 8u, 11u}) {
      tau[idx] = calf_tau;
    }
  } else if (sample.phase == go2_jump_core::JumpPhase::kLanding) {
    const double thigh_tau =
        Clamp(-2.0 - 4.0 * landing_scale, -config_.max_feedforward_torque_nm, 0.0);
    const double calf_tau =
        Clamp(-3.0 - 6.0 * landing_scale, -config_.max_feedforward_torque_nm, 0.0);
    for (std::size_t idx : {1u, 4u, 7u, 10u}) {
      tau[idx] = thigh_tau;
    }
    for (std::size_t idx : {2u, 5u, 8u, 11u}) {
      tau[idx] = calf_tau;
    }
  }

  return tau;
}

double WholeBodyMpc::GainsForPhase(go2_jump_core::JumpPhase phase,
                                   bool derivative) const {
  switch (phase) {
    case go2_jump_core::JumpPhase::kCrouch:
    case go2_jump_core::JumpPhase::kPush:
      return derivative ? config_.default_kd : config_.default_kp;
    case go2_jump_core::JumpPhase::kFlight:
      return derivative ? config_.flight_kd : config_.flight_kp;
    case go2_jump_core::JumpPhase::kLanding:
      return derivative ? config_.landing_kd : config_.landing_kp;
    case go2_jump_core::JumpPhase::kSettle:
      return derivative ? config_.settle_kd : config_.settle_kp;
  }
  return derivative ? config_.default_kd : config_.default_kp;
}

go2_jump_core::JumpReferenceSample WholeBodyMpc::ApplyContactOverrides(
    const go2_jump_core::JumpReferenceSample& planned_sample,
    const RobotObservation& observation,
    double task_elapsed_s) const {
  auto sample = planned_sample;
  if (!observation.contact_signal_valid) {
    return sample;
  }

  const int contact_count = CountContacts(observation.foot_contact);
  const double push_end = task_.crouch_duration_s + task_.push_duration_s;
  const double flight_elapsed_s = std::max(0.0, task_elapsed_s - push_end);

  if (planned_sample.phase == go2_jump_core::JumpPhase::kPush &&
      task_elapsed_s >= task_.crouch_duration_s + 0.35 * task_.push_duration_s &&
      contact_count <= config_.flight_contact_count_max) {
    sample.phase = go2_jump_core::JumpPhase::kFlight;
    sample.time_in_phase_s = flight_elapsed_s;
    sample.leg_retraction_ratio = std::max(sample.leg_retraction_ratio, 0.2);
  }

  if (planned_sample.phase == go2_jump_core::JumpPhase::kFlight &&
      flight_elapsed_s >= config_.min_flight_time_before_touchdown_s &&
      contact_count >= config_.touchdown_contact_count_threshold) {
    sample.phase = go2_jump_core::JumpPhase::kLanding;
    sample.time_in_phase_s = 0.0;
    sample.desired_vertical_velocity_mps =
        std::min(sample.desired_vertical_velocity_mps, 0.0);
    sample.desired_body_pitch_deg = task_.objective.target_landing_pitch_deg;
    sample.desired_body_height_offset_m =
        config_.reference_config.landing_height_offset_m;
    sample.landing_brace_factor = 1.0;
  }

  if (planned_sample.phase == go2_jump_core::JumpPhase::kLanding &&
      contact_count == 4 &&
      std::abs(observation.body_velocity[2]) <=
          config_.settle_vertical_velocity_threshold_mps &&
      task_elapsed_s >=
          push_end + task_.estimated_flight_time_s + 0.5 * task_.landing_duration_s) {
    sample.phase = go2_jump_core::JumpPhase::kSettle;
    sample.time_in_phase_s = 0.0;
    sample.desired_body_pitch_deg = 0.0;
    sample.desired_body_height_offset_m = 0.0;
    sample.landing_brace_factor = 0.2;
  }

  return sample;
}

}  // namespace go2_jump_mpc
