#include "go2_jump_controller/unitree_crc.hpp"
#include "go2_jump_planner/jump_plan.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

namespace {

constexpr std::size_t kControlledJointCount = 12;
constexpr double kRadToDeg = 57.29577951308232;
constexpr double kCompactnessToCalfScale = 1.6;

double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

std::chrono::nanoseconds ToDuration(double seconds) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
}

std::string FormatDouble(double value, int precision = 4) {
  if (!std::isfinite(value)) {
    return "nan";
  }

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

struct JumpTrialMetrics {
  bool active{false};
  bool reported{false};
  bool baseline_captured{false};
  bool takeoff_detected{false};
  bool landing_detected{false};

  std::size_t sample_count{0};

  std::array<double, 3> baseline_position{};

  double latest_forward_displacement_m{0.0};
  double max_forward_displacement_m{0.0};
  double max_airborne_forward_displacement_m{0.0};
  double max_lateral_drift_m{0.0};
  double max_height_above_start_m{0.0};
  double peak_forward_speed_mps{0.0};
  double peak_upward_speed_mps{0.0};
  double peak_descent_speed_mps{0.0};
  double max_total_foot_force_est{0.0};
  double min_roll_rad{std::numeric_limits<double>::infinity()};
  double max_roll_rad{-std::numeric_limits<double>::infinity()};
  double min_pitch_rad{std::numeric_limits<double>::infinity()};
  double max_pitch_rad{-std::numeric_limits<double>::infinity()};
  double max_abs_roll_rad{0.0};
  double max_abs_pitch_rad{0.0};
  double max_joint_tracking_error_rad{0.0};
  double max_joint_speed_radps{0.0};

  double takeoff_time_s{std::numeric_limits<double>::quiet_NaN()};
  double landing_time_s{std::numeric_limits<double>::quiet_NaN()};
  double takeoff_forward_displacement_m{
      std::numeric_limits<double>::quiet_NaN()};
  double takeoff_height_above_start_m{
      std::numeric_limits<double>::quiet_NaN()};
  double landing_forward_displacement_m{std::numeric_limits<double>::quiet_NaN()};
  double landing_height_above_start_m{
      std::numeric_limits<double>::quiet_NaN()};
  double landing_forward_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double landing_total_foot_force_est{
      std::numeric_limits<double>::quiet_NaN()};
  double final_forward_displacement_m{std::numeric_limits<double>::quiet_NaN()};
  double takeoff_pitch_rad{std::numeric_limits<double>::quiet_NaN()};
  double landing_pitch_rad{std::numeric_limits<double>::quiet_NaN()};
  double final_pitch_rad{std::numeric_limits<double>::quiet_NaN()};
};

struct StartupReadiness {
  bool ready{false};
  double max_pose_error_rad{0.0};
  double body_speed_mps{0.0};
  double abs_roll_deg{0.0};
  double abs_pitch_deg{0.0};
};

}  // namespace

class JumpControllerNode : public rclcpp::Node {
 public:
  JumpControllerNode() : Node("go2_jump_controller") {
    planner_config_.target_distance_m =
        this->declare_parameter("target_distance_m", 0.25);
    planner_config_.takeoff_angle_deg =
        this->declare_parameter("takeoff_angle_deg", 45.0);
    planner_config_.takeoff_speed_scale =
        this->declare_parameter("takeoff_speed_scale",
                                planner_config_.takeoff_speed_scale);
    planner_config_.use_takeoff_speed_scale_curve =
        this->declare_parameter("use_takeoff_speed_scale_curve",
                                planner_config_.use_takeoff_speed_scale_curve);
    planner_config_.takeoff_speed_scale_distance_points_m =
        this->declare_parameter("takeoff_speed_scale_distance_points_m",
                                planner_config_.takeoff_speed_scale_distance_points_m);
    planner_config_.takeoff_speed_scale_values =
        this->declare_parameter("takeoff_speed_scale_values",
                                planner_config_.takeoff_speed_scale_values);

    planner_config_.stand_hip_rad =
        this->declare_parameter("stand_hip_rad", planner_config_.stand_hip_rad);
    planner_config_.stand_thigh_rad = this->declare_parameter(
        "stand_thigh_rad", planner_config_.stand_thigh_rad);
    planner_config_.stand_calf_rad =
        this->declare_parameter("stand_calf_rad", planner_config_.stand_calf_rad);
    planner_config_.stand_front_compact_delta_rad = this->declare_parameter(
        "stand_front_compact_delta_rad",
        planner_config_.stand_front_compact_delta_rad);
    planner_config_.stand_rear_compact_delta_rad = this->declare_parameter(
        "stand_rear_compact_delta_rad",
        planner_config_.stand_rear_compact_delta_rad);

    planner_config_.crouch_hip_rad =
        this->declare_parameter("crouch_hip_rad", planner_config_.crouch_hip_rad);
    planner_config_.crouch_thigh_rad = this->declare_parameter(
        "crouch_thigh_rad", planner_config_.crouch_thigh_rad);
    planner_config_.crouch_calf_rad = this->declare_parameter(
        "crouch_calf_rad", planner_config_.crouch_calf_rad);
    planner_config_.crouch_front_compact_delta_rad = this->declare_parameter(
        "crouch_front_compact_delta_rad",
        planner_config_.crouch_front_compact_delta_rad);
    planner_config_.crouch_rear_compact_delta_rad = this->declare_parameter(
        "crouch_rear_compact_delta_rad",
        planner_config_.crouch_rear_compact_delta_rad);

    planner_config_.push_hip_rad =
        this->declare_parameter("push_hip_rad", planner_config_.push_hip_rad);
    planner_config_.push_thigh_rad =
        this->declare_parameter("push_thigh_rad", planner_config_.push_thigh_rad);
    planner_config_.push_calf_rad =
        this->declare_parameter("push_calf_rad", planner_config_.push_calf_rad);
    planner_config_.push_front_compact_delta_rad = this->declare_parameter(
        "push_front_compact_delta_rad",
        planner_config_.push_front_compact_delta_rad);
    planner_config_.push_rear_compact_delta_rad = this->declare_parameter(
        "push_rear_compact_delta_rad",
        planner_config_.push_rear_compact_delta_rad);

    planner_config_.flight_hip_rad =
        this->declare_parameter("flight_hip_rad", planner_config_.flight_hip_rad);
    planner_config_.flight_thigh_rad = this->declare_parameter(
        "flight_thigh_rad", planner_config_.flight_thigh_rad);
    planner_config_.flight_calf_rad = this->declare_parameter(
        "flight_calf_rad", planner_config_.flight_calf_rad);
    planner_config_.flight_front_compact_delta_rad = this->declare_parameter(
        "flight_front_compact_delta_rad",
        planner_config_.flight_front_compact_delta_rad);
    planner_config_.flight_rear_compact_delta_rad = this->declare_parameter(
        "flight_rear_compact_delta_rad",
        planner_config_.flight_rear_compact_delta_rad);

    planner_config_.landing_hip_rad = this->declare_parameter(
        "landing_hip_rad", planner_config_.landing_hip_rad);
    planner_config_.landing_thigh_rad = this->declare_parameter(
        "landing_thigh_rad", planner_config_.landing_thigh_rad);
    planner_config_.landing_calf_rad = this->declare_parameter(
        "landing_calf_rad", planner_config_.landing_calf_rad);
    planner_config_.landing_front_compact_delta_rad = this->declare_parameter(
        "landing_front_compact_delta_rad",
        planner_config_.landing_front_compact_delta_rad);
    planner_config_.landing_rear_compact_delta_rad = this->declare_parameter(
        "landing_rear_compact_delta_rad",
        planner_config_.landing_rear_compact_delta_rad);

    planner_config_.crouch_duration_s = this->declare_parameter(
        "crouch_duration_s", planner_config_.crouch_duration_s);
    planner_config_.push_duration_s =
        this->declare_parameter("push_duration_s", planner_config_.push_duration_s);
    planner_config_.landing_duration_s = this->declare_parameter(
        "landing_duration_s", planner_config_.landing_duration_s);
    planner_config_.recovery_duration_s = this->declare_parameter(
        "recovery_duration_s", planner_config_.recovery_duration_s);

    planner_config_.push_thigh_tau_ff = this->declare_parameter(
        "push_thigh_tau_ff", planner_config_.push_thigh_tau_ff);
    planner_config_.push_calf_tau_ff = this->declare_parameter(
        "push_calf_tau_ff", planner_config_.push_calf_tau_ff);
    planner_config_.landing_thigh_tau_ff = this->declare_parameter(
        "landing_thigh_tau_ff", planner_config_.landing_thigh_tau_ff);
    planner_config_.landing_calf_tau_ff = this->declare_parameter(
        "landing_calf_tau_ff", planner_config_.landing_calf_tau_ff);

    control_dt_s_ = this->declare_parameter("control_dt_s", 0.002);
    start_delay_s_ = this->declare_parameter("start_delay_s", 1.0);
    auto_start_ = this->declare_parameter("auto_start", true);
    hold_kp_ = this->declare_parameter("hold_kp", 55.0);
    flight_kp_ = this->declare_parameter("flight_kp", 28.0);
    kd_ = this->declare_parameter("kd", 3.5);
    push_front_tau_scale_ = this->declare_parameter("push_front_tau_scale", 1.0);
    push_rear_tau_scale_ = this->declare_parameter("push_rear_tau_scale", 1.0);
    landing_front_tau_scale_ =
        this->declare_parameter("landing_front_tau_scale", 1.0);
    landing_rear_tau_scale_ =
        this->declare_parameter("landing_rear_tau_scale", 1.0);
    push_pitch_target_deg_ = this->declare_parameter("push_pitch_target_deg", -5.0);
    push_pitch_compactness_gain_ =
        this->declare_parameter("push_pitch_compactness_gain", 0.35);
    push_pitch_rate_gain_ =
        this->declare_parameter("push_pitch_rate_gain", 0.03);
    push_pitch_correction_limit_rad_ =
        this->declare_parameter("push_pitch_correction_limit_rad", 0.08);
    flight_pitch_target_deg_ =
        this->declare_parameter("flight_pitch_target_deg", -2.0);
    flight_pitch_compactness_gain_ =
        this->declare_parameter("flight_pitch_compactness_gain", 0.55);
    flight_pitch_rate_gain_ =
        this->declare_parameter("flight_pitch_rate_gain", 0.05);
    flight_pitch_correction_limit_rad_ =
        this->declare_parameter("flight_pitch_correction_limit_rad", 0.18);
    landing_pitch_target_deg_ =
        this->declare_parameter("landing_pitch_target_deg", -8.0);
    landing_pitch_compactness_gain_ =
        this->declare_parameter("landing_pitch_compactness_gain", 0.40);
    landing_pitch_rate_gain_ =
        this->declare_parameter("landing_pitch_rate_gain", 0.04);
    landing_pitch_correction_limit_rad_ =
        this->declare_parameter("landing_pitch_correction_limit_rad", 0.12);
    startup_pose_tolerance_rad_ =
        this->declare_parameter("startup_pose_tolerance_rad", 0.20);
    startup_body_speed_tolerance_mps_ =
        this->declare_parameter("startup_body_speed_tolerance_mps", 0.30);
    startup_tilt_tolerance_deg_ =
        this->declare_parameter("startup_tilt_tolerance_deg", 20.0);

    report_path_ = this->declare_parameter("report_path", std::string{});
    takeoff_height_threshold_m_ =
        this->declare_parameter("takeoff_height_threshold_m", 0.03);
    landing_height_threshold_m_ =
        this->declare_parameter("landing_height_threshold_m", 0.02);
    landing_speed_threshold_mps_ =
        this->declare_parameter("landing_speed_threshold_mps", 0.35);

    plan_ = go2_jump_planner::MakeJumpPlan(planner_config_);
    active_target_pose_ = plan_.stand_pose;
    InitializeLowCmd();

    low_cmd_pub_ =
        this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
    low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
        "/lowstate", 10,
        [this](const unitree_go::msg::LowState::SharedPtr msg) {
          latest_low_state_ = *msg;
          have_low_state_ = true;
          if (!first_low_state_seen_) {
            first_low_state_seen_ = true;
            first_low_state_time_ = this->now();
            RCLCPP_INFO(this->get_logger(),
                        "Received first /lowstate. Waiting %.2f s before auto-start.",
                        start_delay_s_);
          }
        });

    sport_state_sub_ =
        this->create_subscription<unitree_go::msg::SportModeState>(
            "/sportmodestate", 10,
            [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
              latest_sport_state_ = *msg;
              have_sport_state_ = true;
            });

    jump_target_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/jump_target_distance", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          const double clamped_distance = Clamp(msg->data, 0.05, 0.60);
          if (std::abs(clamped_distance - planner_config_.target_distance_m) <
              1e-6) {
            return;
          }

          planner_config_.target_distance_m = clamped_distance;
          plan_ = go2_jump_planner::MakeJumpPlan(planner_config_);
          RCLCPP_INFO(this->get_logger(),
                      "Updated jump target to %.3f m, planned takeoff speed %.3f m/s.",
                      plan_.target_distance_m, plan_.takeoff_speed_mps);
        });

    timer_ = this->create_wall_timer(ToDuration(control_dt_s_),
                                     [this]() { ControlTick(); });

    LogPlan();
  }

 private:
  void InitializeLowCmd() {
    low_cmd_.head[0] = 0xFE;
    low_cmd_.head[1] = 0xEF;
    low_cmd_.level_flag = 0xFF;
    low_cmd_.gpio = 0;
    for (int i = 0; i < 20; ++i) {
      low_cmd_.motor_cmd[i].mode = 0x01;
      low_cmd_.motor_cmd[i].q = go2_jump_controller::kPosStop;
      low_cmd_.motor_cmd[i].kp = 0.0;
      low_cmd_.motor_cmd[i].dq = go2_jump_controller::kVelStop;
      low_cmd_.motor_cmd[i].kd = 0.0;
      low_cmd_.motor_cmd[i].tau = 0.0;
    }
  }

  void ResetTrialMetrics() {
    trial_metrics_ = JumpTrialMetrics{};
    trial_metrics_.active = true;
  }

  void StartJump(const rclcpp::Time& now) {
    jump_started_ = true;
    jump_start_time_ = now;
    last_phase_.clear();
    ResetTrialMetrics();
    RCLCPP_INFO(this->get_logger(), "Starting jump sequence.");
  }

  double MaxPoseErrorTo(const std::array<double, 12>& pose) const {
    double max_error = 0.0;
    for (std::size_t i = 0; i < kControlledJointCount; ++i) {
      max_error = std::max(
          max_error, std::abs(static_cast<double>(latest_low_state_.motor_state[i].q) -
                                  pose[i]));
    }
    return max_error;
  }

  StartupReadiness ComputeStartupReadiness() const {
    StartupReadiness readiness{};
    if (!have_low_state_) {
      return readiness;
    }

    readiness.max_pose_error_rad = MaxPoseErrorTo(plan_.stand_pose);
    readiness.abs_roll_deg =
        std::abs(static_cast<double>(latest_low_state_.imu_state.rpy[0])) *
        kRadToDeg;
    readiness.abs_pitch_deg =
        std::abs(static_cast<double>(latest_low_state_.imu_state.rpy[1])) *
        kRadToDeg;

    if (have_sport_state_) {
      const double vx = latest_sport_state_.velocity[0];
      const double vy = latest_sport_state_.velocity[1];
      const double vz = latest_sport_state_.velocity[2];
      readiness.body_speed_mps = std::sqrt(vx * vx + vy * vy + vz * vz);
    }

    readiness.ready =
        readiness.max_pose_error_rad <= startup_pose_tolerance_rad_ &&
        readiness.body_speed_mps <= startup_body_speed_tolerance_mps_ &&
        readiness.abs_roll_deg <= startup_tilt_tolerance_deg_ &&
        readiness.abs_pitch_deg <= startup_tilt_tolerance_deg_;
    return readiness;
  }

  void MaybeLogStartupWaiting(const rclcpp::Time& now,
                              const StartupReadiness& readiness) {
    if (last_startup_wait_log_time_.nanoseconds() != 0 &&
        (now - last_startup_wait_log_time_).seconds() < 1.0) {
      return;
    }

    last_startup_wait_log_time_ = now;
    RCLCPP_INFO(
        this->get_logger(),
        "Waiting for startup readiness: pose_err=%.3f rad (<= %.3f), body_speed=%.3f m/s (<= %.3f), roll=%.2f deg, pitch=%.2f deg (<= %.2f), sport_state=%s",
        readiness.max_pose_error_rad, startup_pose_tolerance_rad_,
        readiness.body_speed_mps, startup_body_speed_tolerance_mps_,
        readiness.abs_roll_deg, readiness.abs_pitch_deg,
        startup_tilt_tolerance_deg_, have_sport_state_ ? "true" : "false");
  }

  void CaptureBaselineIfReady() {
    if (trial_metrics_.baseline_captured || !have_sport_state_) {
      return;
    }

    for (std::size_t i = 0; i < trial_metrics_.baseline_position.size(); ++i) {
      trial_metrics_.baseline_position[i] = latest_sport_state_.position[i];
    }
    trial_metrics_.baseline_captured = true;
  }

  void ApplyPitchCompactnessCorrection(std::array<double, 12>& pose,
                                       double pitch_target_deg,
                                       double compactness_gain,
                                       double pitch_rate_gain,
                                       double correction_limit_rad) const {
    if (!have_low_state_ || correction_limit_rad <= 0.0) {
      return;
    }

    const double pitch_rad =
        static_cast<double>(latest_low_state_.imu_state.rpy[1]);
    const double pitch_rate_radps =
        static_cast<double>(latest_low_state_.imu_state.gyroscope[1]);
    const double pitch_target_rad = pitch_target_deg / kRadToDeg;
    const double pitch_error_rad = pitch_target_rad - pitch_rad;
    const double compactness_correction = Clamp(
        compactness_gain * pitch_error_rad - pitch_rate_gain * pitch_rate_radps,
        -correction_limit_rad, correction_limit_rad);

    pose[1] += compactness_correction;
    pose[2] -= kCompactnessToCalfScale * compactness_correction;
    pose[4] += compactness_correction;
    pose[5] -= kCompactnessToCalfScale * compactness_correction;

    pose[7] -= compactness_correction;
    pose[8] += kCompactnessToCalfScale * compactness_correction;
    pose[10] -= compactness_correction;
    pose[11] += kCompactnessToCalfScale * compactness_correction;
  }

  double ComputeTotalEstimatedFootForce() const {
    if (!have_low_state_) {
      return 0.0;
    }

    double total_force = 0.0;
    for (const auto value : latest_low_state_.foot_force_est) {
      total_force += std::max(0.0, static_cast<double>(value));
    }
    return total_force;
  }

  void UpdateTrialMetrics(double elapsed_s,
                          const std::array<double, 12>& target_pose,
                          const std::string& phase) {
    if (!trial_metrics_.active || !have_low_state_) {
      return;
    }

    CaptureBaselineIfReady();
    ++trial_metrics_.sample_count;

    const double roll_rad =
        static_cast<double>(latest_low_state_.imu_state.rpy[0]);
    const double pitch_rad =
        static_cast<double>(latest_low_state_.imu_state.rpy[1]);
    trial_metrics_.min_roll_rad = std::min(trial_metrics_.min_roll_rad, roll_rad);
    trial_metrics_.max_roll_rad = std::max(trial_metrics_.max_roll_rad, roll_rad);
    trial_metrics_.min_pitch_rad =
        std::min(trial_metrics_.min_pitch_rad, pitch_rad);
    trial_metrics_.max_pitch_rad =
        std::max(trial_metrics_.max_pitch_rad, pitch_rad);
    trial_metrics_.max_abs_roll_rad =
        std::max(trial_metrics_.max_abs_roll_rad, std::abs(roll_rad));
    trial_metrics_.max_abs_pitch_rad =
        std::max(trial_metrics_.max_abs_pitch_rad, std::abs(pitch_rad));

    for (std::size_t i = 0; i < kControlledJointCount; ++i) {
      const double joint_q = latest_low_state_.motor_state[i].q;
      const double joint_dq = latest_low_state_.motor_state[i].dq;
      trial_metrics_.max_joint_tracking_error_rad =
          std::max(trial_metrics_.max_joint_tracking_error_rad,
                   std::abs(target_pose[i] - joint_q));
      trial_metrics_.max_joint_speed_radps =
          std::max(trial_metrics_.max_joint_speed_radps, std::abs(joint_dq));
    }

    if (!have_sport_state_ || !trial_metrics_.baseline_captured) {
      return;
    }

    const double x =
        latest_sport_state_.position[0] - trial_metrics_.baseline_position[0];
    const double y =
        latest_sport_state_.position[1] - trial_metrics_.baseline_position[1];
    const double z =
        latest_sport_state_.position[2] - trial_metrics_.baseline_position[2];
    const double vx = latest_sport_state_.velocity[0];
    const double vz = latest_sport_state_.velocity[2];
    const double total_foot_force_est = ComputeTotalEstimatedFootForce();

    trial_metrics_.latest_forward_displacement_m = x;
    trial_metrics_.max_forward_displacement_m =
        std::max(trial_metrics_.max_forward_displacement_m, x);
    trial_metrics_.max_lateral_drift_m =
        std::max(trial_metrics_.max_lateral_drift_m, std::abs(y));
    trial_metrics_.max_height_above_start_m =
        std::max(trial_metrics_.max_height_above_start_m, z);
    trial_metrics_.peak_forward_speed_mps =
        std::max(trial_metrics_.peak_forward_speed_mps, vx);
    trial_metrics_.peak_upward_speed_mps =
        std::max(trial_metrics_.peak_upward_speed_mps, vz);
    trial_metrics_.peak_descent_speed_mps =
        std::max(trial_metrics_.peak_descent_speed_mps, -vz);
    trial_metrics_.max_total_foot_force_est =
        std::max(trial_metrics_.max_total_foot_force_est, total_foot_force_est);

    if (!trial_metrics_.takeoff_detected &&
        z >= takeoff_height_threshold_m_ && vz > 0.0) {
      trial_metrics_.takeoff_detected = true;
      trial_metrics_.takeoff_time_s = elapsed_s;
      trial_metrics_.takeoff_forward_displacement_m = x;
      trial_metrics_.takeoff_height_above_start_m = z;
      trial_metrics_.takeoff_pitch_rad = pitch_rad;
    }

    if (trial_metrics_.takeoff_detected && !trial_metrics_.landing_detected) {
      trial_metrics_.max_airborne_forward_displacement_m =
          std::max(trial_metrics_.max_airborne_forward_displacement_m, x);
    }

    const bool landing_or_recovery_phase =
        phase == "landing" || phase == "recovery" || phase == "complete";

    if (trial_metrics_.takeoff_detected && !trial_metrics_.landing_detected &&
        elapsed_s > trial_metrics_.takeoff_time_s + 0.05 &&
        z <= landing_height_threshold_m_ &&
        (vz <= 0.0 || std::abs(vz) <= landing_speed_threshold_mps_ ||
         landing_or_recovery_phase)) {
      trial_metrics_.landing_detected = true;
      trial_metrics_.landing_time_s = elapsed_s;
      trial_metrics_.landing_forward_displacement_m = x;
      trial_metrics_.landing_height_above_start_m = z;
      trial_metrics_.landing_forward_speed_mps = vx;
      trial_metrics_.landing_total_foot_force_est = total_foot_force_est;
      trial_metrics_.landing_pitch_rad = pitch_rad;
    }
  }

  std::string BuildTrialReport() const {
    const double measured_flight_time_s =
        (std::isfinite(trial_metrics_.takeoff_time_s) &&
         std::isfinite(trial_metrics_.landing_time_s))
            ? (trial_metrics_.landing_time_s - trial_metrics_.takeoff_time_s)
            : std::numeric_limits<double>::quiet_NaN();
    const double distance_error_m =
        std::isfinite(trial_metrics_.final_forward_displacement_m)
            ? (trial_metrics_.final_forward_displacement_m - plan_.target_distance_m)
            : std::numeric_limits<double>::quiet_NaN();
    const double airborne_forward_progress_m =
        (std::isfinite(trial_metrics_.landing_forward_displacement_m) &&
         std::isfinite(trial_metrics_.takeoff_forward_displacement_m))
            ? (trial_metrics_.landing_forward_displacement_m -
               trial_metrics_.takeoff_forward_displacement_m)
            : std::numeric_limits<double>::quiet_NaN();
    const double post_landing_forward_gain_m =
        (std::isfinite(trial_metrics_.final_forward_displacement_m) &&
         std::isfinite(trial_metrics_.landing_forward_displacement_m))
            ? (trial_metrics_.final_forward_displacement_m -
               trial_metrics_.landing_forward_displacement_m)
            : std::numeric_limits<double>::quiet_NaN();

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4);
    stream << "jump_trial_report:\n";
    stream << "  target_distance_m: " << plan_.target_distance_m << "\n";
    stream << "  ballistic_takeoff_speed_mps: "
           << plan_.ballistic_takeoff_speed_mps << "\n";
    stream << "  takeoff_speed_scale_mode: "
           << (plan_.using_takeoff_speed_scale_curve ? "curve" : "manual")
           << "\n";
    stream << "  takeoff_speed_scale: " << plan_.takeoff_speed_scale << "\n";
    stream << "  planned_takeoff_speed_mps: " << plan_.takeoff_speed_mps << "\n";
    stream << "  planned_flight_time_s: " << plan_.estimated_flight_time_s << "\n";
    stream << "  takeoff_forward_displacement_m: "
           << FormatDouble(trial_metrics_.takeoff_forward_displacement_m) << "\n";
    stream << "  takeoff_height_above_start_m: "
           << FormatDouble(trial_metrics_.takeoff_height_above_start_m) << "\n";
    stream << "  final_forward_displacement_m: "
           << FormatDouble(trial_metrics_.final_forward_displacement_m) << "\n";
    stream << "  landing_forward_displacement_m: "
           << FormatDouble(trial_metrics_.landing_forward_displacement_m) << "\n";
    stream << "  airborne_forward_progress_m: "
           << FormatDouble(airborne_forward_progress_m) << "\n";
    stream << "  post_landing_forward_gain_m: "
           << FormatDouble(post_landing_forward_gain_m) << "\n";
    stream << "  max_forward_displacement_m: "
           << FormatDouble(trial_metrics_.max_forward_displacement_m) << "\n";
    stream << "  max_airborne_forward_displacement_m: "
           << FormatDouble(trial_metrics_.max_airborne_forward_displacement_m) << "\n";
    stream << "  distance_error_m: " << FormatDouble(distance_error_m) << "\n";
    stream << "  max_lateral_drift_m: "
           << FormatDouble(trial_metrics_.max_lateral_drift_m) << "\n";
    stream << "  max_height_above_start_m: "
           << FormatDouble(trial_metrics_.max_height_above_start_m) << "\n";
    stream << "  peak_forward_speed_mps: "
           << FormatDouble(trial_metrics_.peak_forward_speed_mps) << "\n";
    stream << "  peak_upward_speed_mps: "
           << FormatDouble(trial_metrics_.peak_upward_speed_mps) << "\n";
    stream << "  peak_descent_speed_mps: "
           << FormatDouble(trial_metrics_.peak_descent_speed_mps) << "\n";
    stream << "  takeoff_time_s: " << FormatDouble(trial_metrics_.takeoff_time_s)
           << "\n";
    stream << "  landing_detected: "
           << (trial_metrics_.landing_detected ? "true" : "false") << "\n";
    stream << "  landing_time_s: " << FormatDouble(trial_metrics_.landing_time_s)
           << "\n";
    stream << "  landing_height_above_start_m: "
           << FormatDouble(trial_metrics_.landing_height_above_start_m) << "\n";
    stream << "  landing_forward_speed_mps: "
           << FormatDouble(trial_metrics_.landing_forward_speed_mps) << "\n";
    stream << "  landing_total_foot_force_est: "
           << FormatDouble(trial_metrics_.landing_total_foot_force_est) << "\n";
    stream << "  measured_flight_time_s: "
           << FormatDouble(measured_flight_time_s) << "\n";
    stream << "  max_total_foot_force_est: "
           << FormatDouble(trial_metrics_.max_total_foot_force_est) << "\n";
    stream << "  max_abs_roll_deg: "
           << FormatDouble(trial_metrics_.max_abs_roll_rad * kRadToDeg) << "\n";
    stream << "  min_roll_deg: "
           << FormatDouble(trial_metrics_.min_roll_rad * kRadToDeg) << "\n";
    stream << "  max_roll_deg: "
           << FormatDouble(trial_metrics_.max_roll_rad * kRadToDeg) << "\n";
    stream << "  max_abs_pitch_deg: "
           << FormatDouble(trial_metrics_.max_abs_pitch_rad * kRadToDeg) << "\n";
    stream << "  min_pitch_deg: "
           << FormatDouble(trial_metrics_.min_pitch_rad * kRadToDeg) << "\n";
    stream << "  max_pitch_deg: "
           << FormatDouble(trial_metrics_.max_pitch_rad * kRadToDeg) << "\n";
    stream << "  takeoff_pitch_deg: "
           << FormatDouble(trial_metrics_.takeoff_pitch_rad * kRadToDeg) << "\n";
    stream << "  landing_pitch_deg: "
           << FormatDouble(trial_metrics_.landing_pitch_rad * kRadToDeg) << "\n";
    stream << "  final_pitch_deg: "
           << FormatDouble(trial_metrics_.final_pitch_rad * kRadToDeg) << "\n";
    stream << "  max_joint_tracking_error_rad: "
           << FormatDouble(trial_metrics_.max_joint_tracking_error_rad) << "\n";
    stream << "  max_joint_speed_radps: "
           << FormatDouble(trial_metrics_.max_joint_speed_radps) << "\n";
    stream << "  sample_count: " << trial_metrics_.sample_count << "\n";
    return stream.str();
  }

  void WriteTrialReport(const std::string& report) {
    if (report_path_.empty()) {
      return;
    }

    try {
      const std::filesystem::path path(report_path_);
      if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
      }

      std::ofstream output(path, std::ios::out | std::ios::trunc);
      output << report;
      output.close();
      RCLCPP_INFO(this->get_logger(), "Wrote jump trial report to %s",
                  report_path_.c_str());
    } catch (const std::exception& error) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to write jump trial report to %s: %s",
                  report_path_.c_str(), error.what());
    }
  }

  void FinalizeTrialReport(double elapsed_s) {
    if (!trial_metrics_.active || trial_metrics_.reported) {
      return;
    }

    if (have_sport_state_ && trial_metrics_.baseline_captured) {
      trial_metrics_.final_forward_displacement_m =
          latest_sport_state_.position[0] - trial_metrics_.baseline_position[0];
      if (!trial_metrics_.landing_detected) {
        trial_metrics_.landing_forward_displacement_m =
            trial_metrics_.final_forward_displacement_m;
      }
    }

    if (trial_metrics_.takeoff_detected && !trial_metrics_.landing_detected) {
      trial_metrics_.landing_time_s = elapsed_s;
      trial_metrics_.landing_forward_displacement_m =
          trial_metrics_.final_forward_displacement_m;
      trial_metrics_.landing_height_above_start_m = 0.0;
      trial_metrics_.landing_forward_speed_mps = have_sport_state_
                                                     ? latest_sport_state_.velocity[0]
                                                     : std::numeric_limits<double>::quiet_NaN();
      trial_metrics_.landing_total_foot_force_est =
          ComputeTotalEstimatedFootForce();
    }

    if (have_low_state_) {
      trial_metrics_.final_pitch_rad =
          static_cast<double>(latest_low_state_.imu_state.rpy[1]);
    }

    trial_metrics_.active = false;
    trial_metrics_.reported = true;

    const std::string report = BuildTrialReport();
    RCLCPP_INFO_STREAM(this->get_logger(), "\n" << report);
    WriteTrialReport(report);
  }

  void ControlTick() {
    if (!have_low_state_) {
      return;
    }

    const auto now = this->now();
    if (!jump_started_) {
      PublishPose(plan_.stand_pose, hold_kp_, kd_, std::array<double, 12>{});
      if (auto_start_ && first_low_state_seen_ &&
          (now - first_low_state_time_).seconds() >= start_delay_s_) {
        const auto readiness = ComputeStartupReadiness();
        if (readiness.ready) {
          StartJump(now);
        } else {
          MaybeLogStartupWaiting(now, readiness);
        }
      }
      return;
    }

    const double elapsed = (now - jump_start_time_).seconds();
    const double crouch_end = plan_.crouch_duration_s;
    const double push_end = crouch_end + plan_.push_duration_s;
    const double flight_end = push_end + plan_.flight_duration_s;
    const double landing_end = flight_end + plan_.landing_duration_s;
    const double recovery_end = landing_end + plan_.recovery_duration_s;

    std::array<double, 12> target_pose = plan_.stand_pose;
    std::array<double, 12> tau_ff{};
    double kp = hold_kp_;
    std::string phase = "stand";

    if (elapsed < crouch_end) {
      phase = "crouch";
      target_pose = go2_jump_planner::InterpolatePose(
          plan_.stand_pose, plan_.crouch_pose, elapsed / plan_.crouch_duration_s);
    } else if (elapsed < push_end) {
      phase = "push";
      kp = hold_kp_ + 5.0;
      target_pose = go2_jump_planner::InterpolatePose(
          plan_.crouch_pose, plan_.push_pose,
          (elapsed - crouch_end) / plan_.push_duration_s);
      ApplyPitchCompactnessCorrection(target_pose, push_pitch_target_deg_,
                                      push_pitch_compactness_gain_,
                                      push_pitch_rate_gain_,
                                      push_pitch_correction_limit_rad_);
      ApplyLegTorque(plan_.push_thigh_tau_ff * push_front_tau_scale_,
                     plan_.push_calf_tau_ff * push_front_tau_scale_,
                     plan_.push_thigh_tau_ff * push_rear_tau_scale_,
                     plan_.push_calf_tau_ff * push_rear_tau_scale_, tau_ff);
    } else if (elapsed < flight_end) {
      phase = "flight";
      kp = flight_kp_;
      target_pose = plan_.flight_pose;
      ApplyPitchCompactnessCorrection(target_pose, flight_pitch_target_deg_,
                                      flight_pitch_compactness_gain_,
                                      flight_pitch_rate_gain_,
                                      flight_pitch_correction_limit_rad_);
    } else if (elapsed < landing_end) {
      phase = "landing";
      target_pose = go2_jump_planner::InterpolatePose(
          plan_.flight_pose, plan_.landing_pose,
          (elapsed - flight_end) / plan_.landing_duration_s);
      ApplyPitchCompactnessCorrection(target_pose, landing_pitch_target_deg_,
                                      landing_pitch_compactness_gain_,
                                      landing_pitch_rate_gain_,
                                      landing_pitch_correction_limit_rad_);
      ApplyLegTorque(-plan_.landing_thigh_tau_ff * landing_front_tau_scale_,
                     -plan_.landing_calf_tau_ff * landing_front_tau_scale_,
                     -plan_.landing_thigh_tau_ff * landing_rear_tau_scale_,
                     -plan_.landing_calf_tau_ff * landing_rear_tau_scale_,
                     tau_ff);
    } else if (elapsed < recovery_end) {
      phase = "recovery";
      target_pose = go2_jump_planner::InterpolatePose(
          plan_.landing_pose, plan_.stand_pose,
          (elapsed - landing_end) / plan_.recovery_duration_s);
    } else {
      phase = "complete";
      target_pose = plan_.stand_pose;
    }

    if (phase != last_phase_) {
      last_phase_ = phase;
      RCLCPP_INFO(this->get_logger(), "Jump phase -> %s", phase.c_str());
    }

    PublishPose(target_pose, kp, kd_, tau_ff);
    UpdateTrialMetrics(elapsed, target_pose, phase);

    if (elapsed >= recovery_end) {
      FinalizeTrialReport(elapsed);
    }
  }

  void ApplyLegTorque(double front_thigh_tau, double front_calf_tau,
                      double rear_thigh_tau, double rear_calf_tau,
                      std::array<double, 12>& tau_ff) const {
    tau_ff[1] = front_thigh_tau;
    tau_ff[2] = front_calf_tau;
    tau_ff[4] = front_thigh_tau;
    tau_ff[5] = front_calf_tau;
    tau_ff[7] = rear_thigh_tau;
    tau_ff[8] = rear_calf_tau;
    tau_ff[10] = rear_thigh_tau;
    tau_ff[11] = rear_calf_tau;
  }

  void PublishPose(const std::array<double, 12>& pose, double kp, double kd,
                   const std::array<double, 12>& tau_ff) {
    active_target_pose_ = pose;
    for (std::size_t i = 0; i < kControlledJointCount; ++i) {
      low_cmd_.motor_cmd[i].mode = 0x01;
      low_cmd_.motor_cmd[i].q = pose[i];
      low_cmd_.motor_cmd[i].dq = 0.0;
      low_cmd_.motor_cmd[i].kp = kp;
      low_cmd_.motor_cmd[i].kd = kd;
      low_cmd_.motor_cmd[i].tau = tau_ff[i];
    }

    go2_jump_controller::FillLowCmdCrc(low_cmd_);
    low_cmd_pub_->publish(low_cmd_);
  }

  void LogPlan() const {
    RCLCPP_INFO(this->get_logger(),
                "Jump plan distance=%.3f m, scale_mode=%s, ballistic_takeoff=%.3f m/s, speed_scale=%.3f, takeoff_speed=%.3f m/s, flight=%.3f s",
                plan_.target_distance_m,
                plan_.using_takeoff_speed_scale_curve ? "curve" : "manual",
                plan_.ballistic_takeoff_speed_mps,
                plan_.takeoff_speed_scale, plan_.takeoff_speed_mps,
                plan_.estimated_flight_time_s);
  }

  go2_jump_planner::JumpPlannerConfig planner_config_{};
  go2_jump_planner::JumpPlan plan_{};
  JumpTrialMetrics trial_metrics_{};

  double control_dt_s_{0.002};
  double start_delay_s_{1.0};
  double hold_kp_{55.0};
  double flight_kp_{28.0};
  double kd_{3.5};
  double push_front_tau_scale_{1.0};
  double push_rear_tau_scale_{1.0};
  double landing_front_tau_scale_{1.0};
  double landing_rear_tau_scale_{1.0};
  double push_pitch_target_deg_{-5.0};
  double push_pitch_compactness_gain_{0.35};
  double push_pitch_rate_gain_{0.03};
  double push_pitch_correction_limit_rad_{0.08};
  double flight_pitch_target_deg_{-2.0};
  double flight_pitch_compactness_gain_{0.55};
  double flight_pitch_rate_gain_{0.05};
  double flight_pitch_correction_limit_rad_{0.18};
  double landing_pitch_target_deg_{-8.0};
  double landing_pitch_compactness_gain_{0.40};
  double landing_pitch_rate_gain_{0.04};
  double landing_pitch_correction_limit_rad_{0.12};
  double startup_pose_tolerance_rad_{0.20};
  double startup_body_speed_tolerance_mps_{0.30};
  double startup_tilt_tolerance_deg_{20.0};
  double takeoff_height_threshold_m_{0.03};
  double landing_height_threshold_m_{0.02};
  double landing_speed_threshold_mps_{0.35};
  bool auto_start_{true};

  bool have_low_state_{false};
  bool have_sport_state_{false};
  bool first_low_state_seen_{false};
  bool jump_started_{false};

  std::string report_path_;
  std::string last_phase_;

  rclcpp::Time first_low_state_time_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time jump_start_time_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time last_startup_wait_log_time_{0, 0, RCL_SYSTEM_TIME};

  std::array<double, 12> active_target_pose_{};

  unitree_go::msg::LowCmd low_cmd_{};
  unitree_go::msg::LowState latest_low_state_{};
  unitree_go::msg::SportModeState latest_sport_state_{};

  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_pub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr
      sport_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr jump_target_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JumpControllerNode>());
  rclcpp::shutdown();
  return 0;
}
