#include "go2_jump_mpc/unitree_crc.hpp"
#include "go2_jump_mpc/whole_body_mpc.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "go2_jump_msgs/msg/jump_task.hpp"
#include "go2_jump_msgs/msg/jump_controller_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

namespace {

constexpr double kRadToDeg = 57.29577951308232;

template <std::size_t N>
std::array<double, N> ReadPoseParameter(rclcpp::Node& node, const std::string& name,
                                        const std::array<double, N>& defaults) {
  const auto values = node.declare_parameter(name, std::vector<double>(defaults.begin(), defaults.end()));
  if (values.size() != N) {
    RCLCPP_WARN(node.get_logger(),
                "Parameter %s expected %zu values, got %zu. Using defaults.",
                name.c_str(), N, values.size());
    return defaults;
  }

  std::array<double, N> out{};
  std::copy(values.begin(), values.end(), out.begin());
  return out;
}

go2_jump_core::JumpTaskSpec TaskFromMsg(const go2_jump_msgs::msg::JumpTask& msg) {
  go2_jump_core::JumpTaskSpec task{};
  task.task_id = msg.task_id;
  task.objective.target_distance_m = msg.target_distance_m;
  task.objective.takeoff_angle_deg = msg.takeoff_angle_deg;
  task.objective.takeoff_speed_scale = 1.0;
  task.objective.target_takeoff_pitch_deg = msg.target_takeoff_pitch_deg;
  task.objective.target_landing_pitch_deg = msg.target_landing_pitch_deg;
  task.objective.max_motion_duration_s = msg.max_motion_duration_s;
  task.target_takeoff_speed_mps = msg.target_takeoff_speed_mps;
  task.target_takeoff_velocity_x_mps = msg.target_takeoff_velocity_x_mps;
  task.target_takeoff_velocity_z_mps = msg.target_takeoff_velocity_z_mps;
  task.estimated_flight_time_s = msg.estimated_flight_time_s;
  task.crouch_duration_s = msg.crouch_duration_s;
  task.push_duration_s = msg.push_duration_s;
  task.landing_duration_s = msg.landing_duration_s;
  task.settle_duration_s = msg.settle_duration_s;
  task.horizon_duration_s = msg.horizon_duration_s;
  task.total_motion_duration_s = msg.crouch_duration_s + msg.push_duration_s +
                                 msg.estimated_flight_time_s + msg.landing_duration_s +
                                 msg.settle_duration_s;
  return task;
}

go2_jump_msgs::msg::JumpControllerState ToDebugMessage(
    const go2_jump_core::JumpTaskSpec& task,
    const go2_jump_mpc::RobotObservation& observation,
    const go2_jump_mpc::WholeBodyMpcCommand& command, const rclcpp::Time& stamp,
    double task_elapsed_s) {
  go2_jump_msgs::msg::JumpControllerState msg;
  msg.header.stamp = stamp;
  msg.task_id = task.task_id;
  msg.phase = go2_jump_core::PhaseName(command.phase);
  msg.backend_name = command.backend_name;
  msg.target_distance_m = task.objective.target_distance_m;
  msg.task_elapsed_s = task_elapsed_s;
  msg.desired_forward_velocity_mps = command.desired_forward_velocity_mps;
  msg.desired_vertical_velocity_mps = command.desired_vertical_velocity_mps;
  msg.desired_body_pitch_deg = command.desired_body_pitch_deg;
  msg.body_pitch_deg = observation.body_rpy[1] * kRadToDeg;
  msg.forward_velocity_mps = observation.body_velocity[0];
  msg.vertical_velocity_mps = observation.body_velocity[2];
  msg.contact_count = static_cast<uint8_t>(command.contact_count);
  msg.contact_signal_valid = command.contact_signal_valid;
  msg.contact_override = command.contact_override;
  msg.backend_ready = command.backend_ready;
  msg.lowcmd_enabled = command.lowcmd_enabled;
  msg.preview_points = static_cast<uint32_t>(command.preview.size());
  for (std::size_t i = 0; i < msg.foot_force_est.size(); ++i) {
    msg.foot_force_est[i] = observation.foot_force_est[i];
    msg.foot_contact[i] = command.foot_contact[i];
  }
  return msg;
}

}  // namespace

class WholeBodyMpcNode : public rclcpp::Node {
 public:
  WholeBodyMpcNode() : Node("go2_jump_mpc_node") {
    config_.control_dt_s = declare_parameter("control_dt_s", config_.control_dt_s);
    config_.horizon_steps = declare_parameter("horizon_steps", config_.horizon_steps);
    config_.enable_lowcmd_output =
        declare_parameter("enable_lowcmd_output", config_.enable_lowcmd_output);
    config_.auto_start = declare_parameter("auto_start", config_.auto_start);
    config_.solver_backend =
        declare_parameter("solver_backend", config_.solver_backend);
    config_.default_kp = declare_parameter("default_kp", config_.default_kp);
    config_.default_kd = declare_parameter("default_kd", config_.default_kd);
    config_.flight_kp = declare_parameter("flight_kp", config_.flight_kp);
    config_.flight_kd = declare_parameter("flight_kd", config_.flight_kd);
    config_.landing_kp = declare_parameter("landing_kp", config_.landing_kp);
    config_.landing_kd = declare_parameter("landing_kd", config_.landing_kd);
    config_.settle_kp = declare_parameter("settle_kp", config_.settle_kp);
    config_.settle_kd = declare_parameter("settle_kd", config_.settle_kd);
    config_.max_feedforward_torque_nm = declare_parameter(
        "max_feedforward_torque_nm", config_.max_feedforward_torque_nm);
    config_.contact_force_threshold_n = declare_parameter(
        "contact_force_threshold_n", config_.contact_force_threshold_n);
    config_.contact_release_threshold_n = declare_parameter(
        "contact_release_threshold_n", config_.contact_release_threshold_n);
    config_.contact_filter_alpha = declare_parameter(
        "contact_filter_alpha", config_.contact_filter_alpha);
    config_.contact_stable_cycles = declare_parameter(
        "contact_stable_cycles", config_.contact_stable_cycles);
    config_.min_contact_signal_force_n = declare_parameter(
        "min_contact_signal_force_n", config_.min_contact_signal_force_n);
    config_.flight_contact_count_max = declare_parameter(
        "flight_contact_count_max", config_.flight_contact_count_max);
    config_.touchdown_contact_count_threshold = declare_parameter(
        "touchdown_contact_count_threshold",
        config_.touchdown_contact_count_threshold);
    config_.min_flight_time_before_touchdown_s = declare_parameter(
        "min_flight_time_before_touchdown_s",
        config_.min_flight_time_before_touchdown_s);
    config_.settle_vertical_velocity_threshold_mps = declare_parameter(
        "settle_vertical_velocity_threshold_mps",
        config_.settle_vertical_velocity_threshold_mps);
    config_.mujoco_model_path = declare_parameter(
        "mujoco_model_path", config_.mujoco_model_path);
    config_.mujoco_rollout_steps = declare_parameter(
        "mujoco_rollout_steps", config_.mujoco_rollout_steps);
    config_.mujoco_rollout_substeps = declare_parameter(
        "mujoco_rollout_substeps", config_.mujoco_rollout_substeps);

    config_.reference_config.gravity_mps2 = declare_parameter(
        "gravity_mps2", config_.reference_config.gravity_mps2);
    config_.reference_config.crouch_height_offset_m = declare_parameter(
        "crouch_height_offset_m", config_.reference_config.crouch_height_offset_m);
    config_.reference_config.push_height_offset_m = declare_parameter(
        "push_height_offset_m", config_.reference_config.push_height_offset_m);
    config_.reference_config.flight_height_offset_m = declare_parameter(
        "flight_height_offset_m", config_.reference_config.flight_height_offset_m);
    config_.reference_config.landing_height_offset_m = declare_parameter(
        "landing_height_offset_m", config_.reference_config.landing_height_offset_m);

    config_.stand_pose = ReadPoseParameter(*this, "stand_pose", config_.stand_pose);
    config_.crouch_pose = ReadPoseParameter(*this, "crouch_pose", config_.crouch_pose);
    config_.push_pose = ReadPoseParameter(*this, "push_pose", config_.push_pose);
    config_.flight_pose = ReadPoseParameter(*this, "flight_pose", config_.flight_pose);
    config_.landing_pose = ReadPoseParameter(*this, "landing_pose", config_.landing_pose);
    config_.settle_pose = ReadPoseParameter(*this, "settle_pose", config_.settle_pose);

    controller_ = std::make_unique<go2_jump_mpc::WholeBodyMpc>(config_);
    InitLowCmdTemplate();

    task_sub_ = create_subscription<go2_jump_msgs::msg::JumpTask>(
        "/go2_jump/task", 10,
        [this](const go2_jump_msgs::msg::JumpTask::SharedPtr msg) {
          OnTask(*msg);
        });
    low_state_sub_ = create_subscription<unitree_go::msg::LowState>(
        "/lowstate", 50,
        [this](const unitree_go::msg::LowState::SharedPtr msg) {
          OnLowState(*msg);
        });
    sport_state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
        "/sportmodestate", 50,
        [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
          OnSportState(*msg);
        });
    low_cmd_pub_ =
        create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
    debug_pub_ = create_publisher<go2_jump_msgs::msg::JumpControllerState>(
        "/go2_jump/controller_state", 20);

    const auto period = std::chrono::duration<double>(config_.control_dt_s);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { ControlTick(); });

    RCLCPP_INFO(
        get_logger(),
        "Whole-body MPC node initialized: backend=%s, horizon_steps=%d, dt=%.4f s, lowcmd_output=%s",
        config_.solver_backend.c_str(), config_.horizon_steps, config_.control_dt_s,
        config_.enable_lowcmd_output ? "true" : "false");
  }

 private:
  void InitLowCmdTemplate() {
    low_cmd_template_.head[0] = 0xFE;
    low_cmd_template_.head[1] = 0xEF;
    low_cmd_template_.level_flag = 0xFF;
    low_cmd_template_.gpio = 0;
    for (std::size_t i = 0; i < go2_jump_mpc::kFullLowCmdMotorCount; ++i) {
      low_cmd_template_.motor_cmd[i].mode = 0x01;
      low_cmd_template_.motor_cmd[i].q = go2_jump_mpc::kPosStop;
      low_cmd_template_.motor_cmd[i].dq = go2_jump_mpc::kVelStop;
      low_cmd_template_.motor_cmd[i].kp = 0.0;
      low_cmd_template_.motor_cmd[i].kd = 0.0;
      low_cmd_template_.motor_cmd[i].tau = 0.0;
    }
  }

  void OnTask(const go2_jump_msgs::msg::JumpTask& msg) {
    const auto task = TaskFromMsg(msg);
    const bool task_changed = (!have_task_ || task.task_id != active_task_.task_id);
    active_task_ = task;
    controller_->SetTask(active_task_);
    have_task_ = true;
    if (task_changed) {
      task_started_ = false;
      last_phase_name_.clear();
      RCLCPP_INFO(
          get_logger(),
          "Received JumpTask task_id=%s distance=%.3f takeoff_speed=%.3f flight=%.3f",
          active_task_.task_id.c_str(), active_task_.objective.target_distance_m,
          active_task_.target_takeoff_speed_mps,
          active_task_.estimated_flight_time_s);
    }
  }

  void OnLowState(const unitree_go::msg::LowState& msg) {
    observation_.lowstate_received = true;
    for (std::size_t i = 0; i < go2_jump_mpc::kControlledJointCount; ++i) {
      observation_.q[i] = msg.motor_state[i].q;
      observation_.dq[i] = msg.motor_state[i].dq;
    }
    const double alpha = std::clamp(config_.contact_filter_alpha, 0.0, 1.0);
    const double release_threshold =
        std::min(config_.contact_release_threshold_n,
                 config_.contact_force_threshold_n);
    const int stable_cycles = std::max(config_.contact_stable_cycles, 1);
    bool have_contact_proxy_signal = false;
    for (std::size_t i = 0; i < observation_.foot_force_est.size(); ++i) {
      const double raw_force =
          std::max(0.0, static_cast<double>(msg.foot_force_est[i]));
      observation_.raw_foot_force_est[i] = raw_force;
      filtered_contact_force_[i] =
          alpha * raw_force + (1.0 - alpha) * filtered_contact_force_[i];
      observation_.foot_force_est[i] = filtered_contact_force_[i];

      if (raw_force >= config_.min_contact_signal_force_n ||
          filtered_contact_force_[i] >= config_.min_contact_signal_force_n) {
        have_contact_proxy_signal = true;
      }

      const bool candidate_contact = observation_.foot_contact[i]
                                         ? filtered_contact_force_[i] >=
                                               release_threshold
                                         : filtered_contact_force_[i] >=
                                               config_.contact_force_threshold_n;
      if (candidate_contact == observation_.foot_contact[i]) {
        pending_contact_state_[i] = candidate_contact;
        pending_contact_ticks_[i] = 0;
      } else if (candidate_contact != pending_contact_state_[i]) {
        pending_contact_state_[i] = candidate_contact;
        pending_contact_ticks_[i] = 1;
      } else {
        ++pending_contact_ticks_[i];
        if (pending_contact_ticks_[i] >= stable_cycles) {
          observation_.foot_contact[i] = candidate_contact;
          pending_contact_ticks_[i] = 0;
        }
      }
    }
    observation_.contact_signal_valid =
        observation_.contact_signal_valid || have_contact_proxy_signal;
    observation_.body_rpy[0] = msg.imu_state.rpy[0];
    observation_.body_rpy[1] = msg.imu_state.rpy[1];
    observation_.body_rpy[2] = msg.imu_state.rpy[2];
    observation_.body_angular_velocity[0] = msg.imu_state.gyroscope[0];
    observation_.body_angular_velocity[1] = msg.imu_state.gyroscope[1];
    observation_.body_angular_velocity[2] = msg.imu_state.gyroscope[2];
  }

  void OnSportState(const unitree_go::msg::SportModeState& msg) {
    observation_.sportstate_received = true;
    for (std::size_t i = 0; i < observation_.body_velocity.size(); ++i) {
      observation_.body_velocity[i] = msg.velocity[i];
      observation_.position[i] = msg.position[i];
    }
  }

  void StartTaskIfNeeded() {
    if (!have_task_ || !config_.auto_start || task_started_) {
      return;
    }
    task_started_ = true;
    task_start_time_ = now();
    RCLCPP_INFO(get_logger(), "Starting MPC task execution.");
  }

  void ControlTick() {
    StartTaskIfNeeded();
    if (!task_started_ || !controller_->HasTask()) {
      return;
    }
    if (!observation_.lowstate_received) {
      return;
    }

    const double task_elapsed_s = (now() - task_start_time_).seconds();
    const auto command = controller_->Solve(observation_, task_elapsed_s);
    if (!command.valid) {
      return;
    }

    debug_pub_->publish(
        ToDebugMessage(active_task_, observation_, command, now(), task_elapsed_s));

    const std::string phase_name = go2_jump_core::PhaseName(command.phase);
    if (phase_name != last_phase_name_) {
      last_phase_name_ = phase_name;
      RCLCPP_INFO(
          get_logger(),
          "MPC phase=%s elapsed=%.3f backend=%s preview_points=%zu lowcmd=%s contacts=%d signal=%s override=%s",
          phase_name.c_str(), task_elapsed_s, command.backend_name.c_str(),
          command.preview.size(), command.lowcmd_enabled ? "true" : "false",
          command.contact_count, command.contact_signal_valid ? "true" : "false",
          command.contact_override ? "true" : "false");
    }

    if (!command.lowcmd_enabled) {
      return;
    }

    auto low_cmd = low_cmd_template_;
    for (std::size_t i = 0; i < go2_jump_mpc::kControlledJointCount; ++i) {
      low_cmd.motor_cmd[i].mode = 0x01;
      low_cmd.motor_cmd[i].q = command.q_ref[i];
      low_cmd.motor_cmd[i].dq = command.dq_ref[i];
      low_cmd.motor_cmd[i].kp = command.uniform_kp;
      low_cmd.motor_cmd[i].kd = command.uniform_kd;
      low_cmd.motor_cmd[i].tau = command.tau_ff[i];
    }
    go2_jump_mpc::FillLowCmdCrc(low_cmd);
    low_cmd_pub_->publish(low_cmd);
  }

  go2_jump_mpc::WholeBodyMpcConfig config_{};
  go2_jump_mpc::RobotObservation observation_{};
  std::array<double, 4> filtered_contact_force_{};
  std::array<bool, 4> pending_contact_state_{};
  std::array<int, 4> pending_contact_ticks_{};
  go2_jump_core::JumpTaskSpec active_task_{};
  bool have_task_{false};
  bool task_started_{false};
  std::string last_phase_name_;
  rclcpp::Time task_start_time_{0, 0, RCL_ROS_TIME};
  unitree_go::msg::LowCmd low_cmd_template_{};
  std::unique_ptr<go2_jump_mpc::WholeBodyMpc> controller_;

  rclcpp::Subscription<go2_jump_msgs::msg::JumpTask>::SharedPtr task_sub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_state_sub_;
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_pub_;
  rclcpp::Publisher<go2_jump_msgs::msg::JumpControllerState>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WholeBodyMpcNode>());
  rclcpp::shutdown();
  return 0;
}
