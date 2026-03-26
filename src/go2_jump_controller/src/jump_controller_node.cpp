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
constexpr std::size_t kLegCount = 4;
constexpr double kRadToDeg = 57.29577951308232;
constexpr double kCompactnessToCalfScale = 1.6;
constexpr std::array<std::size_t, kLegCount> kHipJointIndices{{0, 3, 6, 9}};
constexpr std::array<std::size_t, kLegCount> kThighJointIndices{{1, 4, 7, 10}};
constexpr std::array<std::size_t, kLegCount> kCalfJointIndices{{2, 5, 8, 11}};
constexpr std::array<double, kLegCount> kLegHipXOffsetsM{{0.1934, 0.1934, -0.1934, -0.1934}};

double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double SmoothClamp01(double value) {
  const double x = Clamp(value, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
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
  double recovery_release_time_s{std::numeric_limits<double>::quiet_NaN()};
  double landing_forward_displacement_m{std::numeric_limits<double>::quiet_NaN()};
  double landing_height_above_start_m{
      std::numeric_limits<double>::quiet_NaN()};
  double landing_forward_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double landing_total_foot_force_est{
      std::numeric_limits<double>::quiet_NaN()};
  double recovery_release_forward_displacement_m{
      std::numeric_limits<double>::quiet_NaN()};
  double recovery_release_forward_speed_mps{
      std::numeric_limits<double>::quiet_NaN()};
  double final_forward_displacement_m{std::numeric_limits<double>::quiet_NaN()};
  double takeoff_pitch_rad{std::numeric_limits<double>::quiet_NaN()};
  double landing_pitch_rad{std::numeric_limits<double>::quiet_NaN()};
  double recovery_release_pitch_rad{std::numeric_limits<double>::quiet_NaN()};
  double final_pitch_rad{std::numeric_limits<double>::quiet_NaN()};
};

struct StartupReadiness {
  bool ready{false};
  double max_pose_error_rad{0.0};
  double body_speed_mps{0.0};
  double abs_roll_deg{0.0};
  double abs_pitch_deg{0.0};
};

struct SagittalLegKinematics {
  double x_m{0.0};
  double z_m{0.0};
  double dx_dthigh{0.0};
  double dx_dcalf{0.0};
  double dz_dthigh{0.0};
  double dz_dcalf{0.0};
};

struct SagittalFootPoint {
  double x_m{0.0};
  double z_m{0.0};
};

struct ContactWrenchTargets {
  double total_fx_n{0.0};
  double total_fz_n{0.0};
  double pitch_moment_nm{0.0};
};

enum class RuntimePhase {
  kStand,
  kCrouch,
  kPush,
  kFlight,
  kLanding,
  kRecovery,
  kComplete,
};

const char* PhaseName(RuntimePhase phase) {
  switch (phase) {
    case RuntimePhase::kStand:
      return "stand";
    case RuntimePhase::kCrouch:
      return "crouch";
    case RuntimePhase::kPush:
      return "push";
    case RuntimePhase::kFlight:
      return "flight";
    case RuntimePhase::kLanding:
      return "landing";
    case RuntimePhase::kRecovery:
      return "recovery";
    case RuntimePhase::kComplete:
      return "complete";
  }
  return "unknown";
}

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
    planner_config_.crouch_forward_bias_rad = this->declare_parameter(
        "crouch_forward_bias_rad", planner_config_.crouch_forward_bias_rad);

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
    planner_config_.push_forward_bias_rad = this->declare_parameter(
        "push_forward_bias_rad", planner_config_.push_forward_bias_rad);

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
    planner_config_.landing_absorption_blend = this->declare_parameter(
        "landing_absorption_blend", planner_config_.landing_absorption_blend);
    planner_config_.leg_link_length_m = this->declare_parameter(
        "leg_link_length_m", planner_config_.leg_link_length_m);
    planner_config_.landing_capture_time_constant_s = this->declare_parameter(
        "landing_capture_time_constant_s",
        planner_config_.landing_capture_time_constant_s);
    planner_config_.landing_capture_rear_ratio = this->declare_parameter(
        "landing_capture_rear_ratio",
        planner_config_.landing_capture_rear_ratio);
    planner_config_.landing_capture_limit_m = this->declare_parameter(
        "landing_capture_limit_m", planner_config_.landing_capture_limit_m);
    planner_config_.landing_extension_m = this->declare_parameter(
        "landing_extension_m", planner_config_.landing_extension_m);
    planner_config_.support_capture_ratio = this->declare_parameter(
        "support_capture_ratio", planner_config_.support_capture_ratio);
    planner_config_.support_hip_rad = this->declare_parameter(
        "support_hip_rad", planner_config_.support_hip_rad);
    planner_config_.support_thigh_rad = this->declare_parameter(
        "support_thigh_rad", planner_config_.support_thigh_rad);
    planner_config_.support_calf_rad = this->declare_parameter(
        "support_calf_rad", planner_config_.support_calf_rad);
    planner_config_.support_front_compact_delta_rad = this->declare_parameter(
        "support_front_compact_delta_rad",
        planner_config_.support_front_compact_delta_rad);
    planner_config_.support_rear_compact_delta_rad = this->declare_parameter(
        "support_rear_compact_delta_rad",
        planner_config_.support_rear_compact_delta_rad);

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
    flight_landing_prep_height_m_ =
        this->declare_parameter("flight_landing_prep_height_m", 0.14);
    flight_landing_prep_start_descent_speed_mps_ =
        this->declare_parameter("flight_landing_prep_start_descent_speed_mps",
                                0.10);
    flight_landing_prep_full_descent_speed_mps_ =
        this->declare_parameter("flight_landing_prep_full_descent_speed_mps",
                                1.20);
    flight_landing_prep_max_blend_ =
        this->declare_parameter("flight_landing_prep_max_blend", 0.0);
    landing_pitch_target_deg_ =
        this->declare_parameter("landing_pitch_target_deg", -8.0);
    landing_pitch_compactness_gain_ =
        this->declare_parameter("landing_pitch_compactness_gain", 0.40);
    landing_pitch_rate_gain_ =
        this->declare_parameter("landing_pitch_rate_gain", 0.04);
    landing_pitch_correction_limit_rad_ =
        this->declare_parameter("landing_pitch_correction_limit_rad", 0.12);
    support_pitch_target_deg_ =
        this->declare_parameter("support_pitch_target_deg", -2.0);
    support_pitch_compactness_gain_ =
        this->declare_parameter("support_pitch_compactness_gain", 0.65);
    support_pitch_rate_gain_ =
        this->declare_parameter("support_pitch_rate_gain", 0.06);
    support_pitch_correction_limit_rad_ =
        this->declare_parameter("support_pitch_correction_limit_rad", 0.18);
    pitch_capture_foot_delta_gain_m_per_rad_ = this->declare_parameter(
        "pitch_capture_foot_delta_gain_m_per_rad", 0.035);
    pitch_capture_foot_delta_rate_gain_m_per_radps_ = this->declare_parameter(
        "pitch_capture_foot_delta_rate_gain_m_per_radps", 0.008);
    pitch_capture_foot_delta_limit_m_ = this->declare_parameter(
        "pitch_capture_foot_delta_limit_m", 0.03);
    pitch_capture_front_z_scale_ = this->declare_parameter(
        "pitch_capture_front_z_scale", 0.75);
    pitch_capture_rear_z_scale_ = this->declare_parameter(
        "pitch_capture_rear_z_scale", 0.35);
    flight_pitch_capture_gain_scale_ = this->declare_parameter(
        "flight_pitch_capture_gain_scale", 0.80);
    landing_pitch_capture_gain_scale_ = this->declare_parameter(
        "landing_pitch_capture_gain_scale", 1.00);
    support_pitch_capture_gain_scale_ = this->declare_parameter(
        "support_pitch_capture_gain_scale", 1.15);
    support_pitch_capture_fade_duration_s_ = this->declare_parameter(
        "support_pitch_capture_fade_duration_s", 0.12);
    takeoff_wait_timeout_s_ =
        this->declare_parameter("takeoff_wait_timeout_s", 0.18);
    landing_wait_timeout_s_ =
        this->declare_parameter("landing_wait_timeout_s", 0.30);
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
    landing_kp_ = this->declare_parameter("landing_kp", hold_kp_);
    landing_kd_ = this->declare_parameter("landing_kd", kd_ + 1.5);
    recovery_kp_ = this->declare_parameter("recovery_kp", hold_kp_);
    recovery_kd_ = this->declare_parameter("recovery_kd", kd_ + 1.0);
    recovery_min_hold_s_ =
        this->declare_parameter("recovery_min_hold_s", 0.06);
    recovery_max_hold_s_ =
        this->declare_parameter("recovery_max_hold_s", 0.30);
    recovery_release_forward_speed_mps_ =
        this->declare_parameter("recovery_release_forward_speed_mps", 0.18);
    recovery_release_pitch_deg_ =
        this->declare_parameter("recovery_release_pitch_deg", 18.0);
    recovery_release_pitch_rate_degps_ =
        this->declare_parameter("recovery_release_pitch_rate_degps", 45.0);
    recovery_upright_blend_ =
        this->declare_parameter("recovery_upright_blend", 0.35);
    landing_support_blend_ =
        this->declare_parameter("landing_support_blend", 0.55);
    support_relax_duration_s_ =
        this->declare_parameter("support_relax_duration_s", 0.0);
    landing_touchdown_reference_blend_ =
        this->declare_parameter("landing_touchdown_reference_blend", 0.0);
    support_kp_ = this->declare_parameter("support_kp", landing_kp_);
    support_kd_ = this->declare_parameter("support_kd", landing_kd_ + 0.5);
    landing_hold_use_touchdown_pose_ =
        this->declare_parameter("landing_hold_use_touchdown_pose", false);
    use_centroidal_wbc_ =
        this->declare_parameter("use_centroidal_wbc", false);
    robot_mass_kg_ = this->declare_parameter("robot_mass_kg", 15.0);
    leg_link_length_m_ = planner_config_.leg_link_length_m;
    foot_contact_force_threshold_ = this->declare_parameter(
        "foot_contact_force_threshold", 25.0);
    takeoff_contact_force_threshold_ = this->declare_parameter(
        "takeoff_contact_force_threshold", 15.0);
    landing_contact_force_threshold_ = this->declare_parameter(
        "landing_contact_force_threshold", 30.0);
    foot_force_est_scale_ =
        this->declare_parameter("foot_force_est_scale", 0.1);
    foot_force_filter_alpha_ =
        this->declare_parameter("foot_force_filter_alpha", 0.15);
    takeoff_total_force_threshold_n_ = this->declare_parameter(
        "takeoff_total_force_threshold_n", 40.0);
    landing_total_force_threshold_n_ = this->declare_parameter(
        "landing_total_force_threshold_n", 80.0);
    wbc_friction_coeff_ =
        this->declare_parameter("wbc_friction_coeff", 0.7);
    wbc_max_leg_normal_force_n_ = this->declare_parameter(
        "wbc_max_leg_normal_force_n", 180.0);
    wbc_push_velocity_gain_ =
        this->declare_parameter("wbc_push_velocity_gain", 10.0);
    wbc_push_vertical_velocity_gain_ =
        this->declare_parameter("wbc_push_vertical_velocity_gain", 12.0);
    wbc_landing_velocity_gain_ =
        this->declare_parameter("wbc_landing_velocity_gain", 8.0);
    wbc_landing_height_gain_ =
        this->declare_parameter("wbc_landing_height_gain", 40.0);
    wbc_pitch_gain_ =
        this->declare_parameter("wbc_pitch_gain", 85.0);
    wbc_pitch_rate_gain_ =
        this->declare_parameter("wbc_pitch_rate_gain", 10.0);
    wbc_pitch_moment_limit_nm_ = this->declare_parameter(
        "wbc_pitch_moment_limit_nm", 55.0);
    wbc_tau_blend_ =
        this->declare_parameter("wbc_tau_blend", 1.0);

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
          for (const auto value : latest_low_state_.foot_force_est) {
            if (std::abs(static_cast<double>(value)) > 1e-3) {
              contact_signal_seen_ = true;
              break;
            }
          }
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
    runtime_phase_ = RuntimePhase::kCrouch;
    phase_start_elapsed_s_ = 0.0;
    recovery_release_elapsed_s_ = std::numeric_limits<double>::quiet_NaN();
    landing_hold_pose_captured_ = false;
    landing_hold_pose_ = plan_.landing_pose;
    support_hold_pose_captured_ = false;
    support_hold_pose_ = plan_.support_pose;
    recovery_release_start_pose_ = plan_.support_pose;
    recovery_release_start_pose_captured_ = false;
    last_phase_.clear();
    ResetTrialMetrics();
    CaptureBaselineIfReady();
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

  SagittalFootPoint ForwardKinematicsFromPose(
      const std::array<double, 12>& pose, std::size_t thigh_index,
      std::size_t calf_index) const {
    SagittalFootPoint foot_point{};
    if (thigh_index >= pose.size() || calf_index >= pose.size()) {
      return foot_point;
    }

    const double thigh_rad = pose[thigh_index];
    const double calf_rad = pose[calf_index];
    const double leg_angle = thigh_rad + calf_rad;
    const double l1 = leg_link_length_m_;
    const double l2 = leg_link_length_m_;
    foot_point.x_m =
        l1 * std::sin(thigh_rad) + l2 * std::sin(leg_angle);
    foot_point.z_m =
        -(l1 * std::cos(thigh_rad) + l2 * std::cos(leg_angle));
    return foot_point;
  }

  bool SolveSagittalLegIk(double x_m, double z_m, double& thigh_rad,
                          double& calf_rad) const {
    const double l1 = leg_link_length_m_;
    const double l2 = leg_link_length_m_;
    const double min_radius_m = 1e-5;
    const double max_radius_m = std::max(min_radius_m, l1 + l2 - 1e-5);
    const double radius_m = Clamp(std::sqrt(x_m * x_m + z_m * z_m),
                                  min_radius_m, max_radius_m);
    const double cos_calf = Clamp((radius_m * radius_m - l1 * l1 - l2 * l2) /
                                      (2.0 * l1 * l2),
                                  -1.0, 1.0);
    calf_rad = -std::acos(cos_calf);
    const double k1 = l1 + l2 * std::cos(calf_rad);
    const double k2 = l2 * std::sin(calf_rad);
    thigh_rad = std::atan2(x_m, -z_m) - std::atan2(k2, k1);
    return std::isfinite(thigh_rad) && std::isfinite(calf_rad);
  }

  void ApplySymmetricLegConfiguration(std::array<double, 12>& pose,
                                      double front_thigh_rad,
                                      double front_calf_rad,
                                      double rear_thigh_rad,
                                      double rear_calf_rad) const {
    pose[1] = front_thigh_rad;
    pose[2] = front_calf_rad;
    pose[4] = front_thigh_rad;
    pose[5] = front_calf_rad;
    pose[7] = rear_thigh_rad;
    pose[8] = rear_calf_rad;
    pose[10] = rear_thigh_rad;
    pose[11] = rear_calf_rad;
  }

  void ApplyPitchCaptureFootPlacementCorrection(
      std::array<double, 12>& pose, double pitch_target_deg,
      double gain_scale) const {
    if (!have_low_state_ || gain_scale <= 1e-6 ||
        pitch_capture_foot_delta_limit_m_ <= 1e-6) {
      return;
    }

    const double pitch_rad =
        static_cast<double>(latest_low_state_.imu_state.rpy[1]);
    const double pitch_rate_radps =
        static_cast<double>(latest_low_state_.imu_state.gyroscope[1]);
    const double pitch_target_rad = pitch_target_deg / kRadToDeg;
    const double pitch_error_rad = pitch_target_rad - pitch_rad;
    const double foot_delta_m = Clamp(
        gain_scale *
            (pitch_capture_foot_delta_gain_m_per_rad_ * pitch_error_rad -
             pitch_capture_foot_delta_rate_gain_m_per_radps_ *
                 pitch_rate_radps),
        -pitch_capture_foot_delta_limit_m_, pitch_capture_foot_delta_limit_m_);
    if (std::abs(foot_delta_m) <= 1e-6) {
      return;
    }

    auto front_foot =
        ForwardKinematicsFromPose(pose, kThighJointIndices[0], kCalfJointIndices[0]);
    auto rear_foot =
        ForwardKinematicsFromPose(pose, kThighJointIndices[2], kCalfJointIndices[2]);
    const double rear_x_ratio =
        Clamp(planner_config_.landing_capture_rear_ratio, 0.0, 1.5);
    front_foot.x_m += foot_delta_m;
    rear_foot.x_m -= rear_x_ratio * foot_delta_m;
    front_foot.z_m -= pitch_capture_front_z_scale_ * foot_delta_m;
    rear_foot.z_m += pitch_capture_rear_z_scale_ * foot_delta_m;

    double front_thigh_rad = pose[1];
    double front_calf_rad = pose[2];
    double rear_thigh_rad = pose[7];
    double rear_calf_rad = pose[8];
    if (!SolveSagittalLegIk(front_foot.x_m, front_foot.z_m, front_thigh_rad,
                            front_calf_rad) ||
        !SolveSagittalLegIk(rear_foot.x_m, rear_foot.z_m, rear_thigh_rad,
                            rear_calf_rad)) {
      return;
    }

    ApplySymmetricLegConfiguration(pose, front_thigh_rad, front_calf_rad,
                                   rear_thigh_rad, rear_calf_rad);
  }

  double CurrentHeightAboveBaselineM() const {
    if (!have_sport_state_ || !trial_metrics_.baseline_captured) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    return latest_sport_state_.position[2] - trial_metrics_.baseline_position[2];
  }

  double ComputeFlightLandingPrepBlend() const {
    if (!have_sport_state_ || !trial_metrics_.baseline_captured ||
        !trial_metrics_.takeoff_detected) {
      return 0.0;
    }

    const double relative_height_m = CurrentHeightAboveBaselineM();
    if (!std::isfinite(relative_height_m)) {
      return 0.0;
    }

    const double vz = latest_sport_state_.velocity[2];
    if (vz >= -flight_landing_prep_start_descent_speed_mps_) {
      return 0.0;
    }

    const double descent_speed_window_mps =
        std::max(flight_landing_prep_full_descent_speed_mps_ -
                     flight_landing_prep_start_descent_speed_mps_,
                 1e-3);
    const double descent_alpha = Clamp(
        (-vz - flight_landing_prep_start_descent_speed_mps_) /
            descent_speed_window_mps,
        0.0, 1.0);
    const double prep_height_window_m =
        std::max(flight_landing_prep_height_m_ - landing_height_threshold_m_,
                 1e-3);
    const double height_alpha = Clamp(
        (flight_landing_prep_height_m_ - relative_height_m) /
            prep_height_window_m,
        0.0, 1.0);

    const double prep_signal =
        SmoothClamp01(0.35 * descent_alpha + 0.65 * height_alpha);
    return Clamp(flight_landing_prep_max_blend_ * prep_signal, 0.0, 1.0);
  }

  void TransitionToPhase(RuntimePhase phase, double elapsed_s) {
    if (runtime_phase_ == phase) {
      return;
    }

    if (phase == RuntimePhase::kLanding && have_low_state_) {
      for (std::size_t i = 0; i < kControlledJointCount; ++i) {
        landing_hold_pose_[i] = latest_low_state_.motor_state[i].q;
      }
      landing_hold_pose_captured_ = true;
    }
    if (phase == RuntimePhase::kLanding) {
      support_hold_pose_ =
          ComputeSupportHoldPose(CurrentLandingReferencePose());
      support_hold_pose_captured_ = true;
    }

    runtime_phase_ = phase;
    phase_start_elapsed_s_ = elapsed_s;
    if (phase == RuntimePhase::kRecovery) {
      recovery_release_elapsed_s_ = std::numeric_limits<double>::quiet_NaN();
    }
  }

  double ComputeTotalEstimatedFootForce() const {
    if (!have_low_state_) {
      return 0.0;
    }

    double total_force = 0.0;
    for (const auto value : filtered_foot_force_est_n_) {
      total_force += value;
    }
    return total_force;
  }

  std::array<double, kLegCount> ReadRawEstimatedFootForces() const {
    std::array<double, kLegCount> foot_forces{};
    if (!have_low_state_) {
      return foot_forces;
    }

    for (std::size_t i = 0; i < foot_forces.size(); ++i) {
      foot_forces[i] = std::max(
          0.0,
          static_cast<double>(latest_low_state_.foot_force_est[i]) *
              foot_force_est_scale_);
    }
    return foot_forces;
  }

  void UpdateFilteredFootForces() {
    const auto raw_forces = ReadRawEstimatedFootForces();
    const double alpha = Clamp(foot_force_filter_alpha_, 0.0, 1.0);
    for (std::size_t i = 0; i < filtered_foot_force_est_n_.size(); ++i) {
      filtered_foot_force_est_n_[i] =
          alpha * raw_forces[i] + (1.0 - alpha) * filtered_foot_force_est_n_[i];
    }
  }

  std::array<double, kLegCount> CurrentEstimatedFootForces() const {
    return filtered_foot_force_est_n_;
  }

  std::array<bool, kLegCount> CurrentFootContacts(double threshold) const {
    std::array<bool, kLegCount> contacts{};
    if (!have_low_state_) {
      return contacts;
    }

    const auto foot_forces = CurrentEstimatedFootForces();
    for (std::size_t i = 0; i < contacts.size(); ++i) {
      contacts[i] = foot_forces[i] >= threshold;
    }
    return contacts;
  }

  int ContactCount(const std::array<bool, kLegCount>& contacts) const {
    int active_contact_count = 0;
    for (const bool in_contact : contacts) {
      active_contact_count += in_contact ? 1 : 0;
    }
    return active_contact_count;
  }

  SagittalLegKinematics ComputeSagittalLegKinematics(
      std::size_t leg_index) const {
    SagittalLegKinematics kinematics{};
    if (!have_low_state_ || leg_index >= kLegCount) {
      return kinematics;
    }

    const double thigh_q = latest_low_state_.motor_state[kThighJointIndices[leg_index]].q;
    const double calf_q = latest_low_state_.motor_state[kCalfJointIndices[leg_index]].q;
    const double leg_angle = thigh_q + calf_q;

    const double l1 = leg_link_length_m_;
    const double l2 = leg_link_length_m_;
    const double sin_thigh = std::sin(thigh_q);
    const double cos_thigh = std::cos(thigh_q);
    const double sin_leg = std::sin(leg_angle);
    const double cos_leg = std::cos(leg_angle);

    kinematics.x_m =
        kLegHipXOffsetsM[leg_index] + l1 * sin_thigh + l2 * sin_leg;
    kinematics.z_m = -(l1 * cos_thigh + l2 * cos_leg);
    kinematics.dx_dthigh = l1 * cos_thigh + l2 * cos_leg;
    kinematics.dx_dcalf = l2 * cos_leg;
    kinematics.dz_dthigh = l1 * sin_thigh + l2 * sin_leg;
    kinematics.dz_dcalf = l2 * sin_leg;
    return kinematics;
  }

  void AccumulateLegForceTorques(std::size_t leg_index, double fx_n, double fz_n,
                                 std::array<double, 12>& tau_ff) const {
    if (leg_index >= kLegCount) {
      return;
    }

    const auto kinematics = ComputeSagittalLegKinematics(leg_index);
    tau_ff[kThighJointIndices[leg_index]] +=
        kinematics.dx_dthigh * fx_n + kinematics.dz_dthigh * fz_n;
    tau_ff[kCalfJointIndices[leg_index]] +=
        kinematics.dx_dcalf * fx_n + kinematics.dz_dcalf * fz_n;
  }

  ContactWrenchTargets ComputeDesiredContactWrench(double elapsed_s) const {
    ContactWrenchTargets targets{};
    if (!have_sport_state_) {
      return targets;
    }

    const double gravity_force_n = robot_mass_kg_ * planner_config_.gravity_mps2;
    const double z = latest_sport_state_.position[2];
    const double vz = latest_sport_state_.velocity[2];
    const double vx = latest_sport_state_.velocity[0];
    const double pitch_rad =
        have_low_state_ ? static_cast<double>(latest_low_state_.imu_state.rpy[1]) : 0.0;
    const double pitch_rate_radps =
        have_low_state_ ? static_cast<double>(latest_low_state_.imu_state.gyroscope[1]) : 0.0;
    const double baseline_z = trial_metrics_.baseline_captured
                                  ? trial_metrics_.baseline_position[2]
                                  : z;

    if (runtime_phase_ == RuntimePhase::kPush) {
      const double push_alpha = Clamp(
          (elapsed_s - plan_.crouch_duration_s) / plan_.push_duration_s, 0.0, 1.0);
      const double velocity_blend = SmoothClamp01(push_alpha);
      const double desired_vx =
          plan_.takeoff_velocity_x_mps * velocity_blend;
      const double desired_vz =
          plan_.takeoff_velocity_z_mps * velocity_blend;
      const double desired_pitch_rad =
          push_pitch_target_deg_ / kRadToDeg;

      const double commanded_ax =
          wbc_push_velocity_gain_ * (desired_vx - vx);
      const double commanded_az =
          wbc_push_vertical_velocity_gain_ * (desired_vz - vz);
      targets.total_fx_n = robot_mass_kg_ * commanded_ax;
      targets.total_fz_n = gravity_force_n + robot_mass_kg_ * commanded_az;
      targets.pitch_moment_nm = Clamp(
          wbc_pitch_gain_ * (desired_pitch_rad - pitch_rad) -
              wbc_pitch_rate_gain_ * pitch_rate_radps,
          -wbc_pitch_moment_limit_nm_, wbc_pitch_moment_limit_nm_);
      return targets;
    }

    if (runtime_phase_ == RuntimePhase::kLanding ||
        runtime_phase_ == RuntimePhase::kRecovery) {
      const double desired_pitch_deg =
          runtime_phase_ == RuntimePhase::kLanding ? landing_pitch_target_deg_
                                                   : support_pitch_target_deg_;
      const double desired_pitch_rad = desired_pitch_deg / kRadToDeg;
      const double commanded_ax =
          -wbc_landing_velocity_gain_ * vx;
      const double commanded_az =
          wbc_landing_height_gain_ * (baseline_z - z) -
          wbc_landing_velocity_gain_ * vz;
      targets.total_fx_n = robot_mass_kg_ * commanded_ax;
      targets.total_fz_n = gravity_force_n + robot_mass_kg_ * commanded_az;
      targets.pitch_moment_nm = Clamp(
          wbc_pitch_gain_ * (desired_pitch_rad - pitch_rad) -
              wbc_pitch_rate_gain_ * pitch_rate_radps,
          -wbc_pitch_moment_limit_nm_, wbc_pitch_moment_limit_nm_);
    }

    return targets;
  }

  void ApplyCentroidalWbcTorques(double elapsed_s,
                                 std::array<double, 12>& tau_ff) const {
    if (!use_centroidal_wbc_ || !have_low_state_ || !have_sport_state_) {
      return;
    }

    std::array<bool, kLegCount> active_contacts{};
    if (runtime_phase_ == RuntimePhase::kPush) {
      active_contacts.fill(true);
    } else {
      active_contacts = CurrentFootContacts(foot_contact_force_threshold_);
      if ((runtime_phase_ == RuntimePhase::kLanding ||
           runtime_phase_ == RuntimePhase::kRecovery) &&
          ContactCount(active_contacts) == 0 &&
          ComputeTotalEstimatedFootForce() >= landing_total_force_threshold_n_) {
        active_contacts.fill(true);
      }
    }

    const int active_leg_count = ContactCount(active_contacts);
    if (active_leg_count <= 0) {
      return;
    }

    auto targets = ComputeDesiredContactWrench(elapsed_s);
    const double total_force_limit_n =
        wbc_max_leg_normal_force_n_ * static_cast<double>(active_leg_count);
    targets.total_fz_n =
        Clamp(targets.total_fz_n, 0.0, std::max(0.0, total_force_limit_n));
    const double max_total_fx_n =
        wbc_friction_coeff_ * std::max(targets.total_fz_n, 0.0);
    targets.total_fx_n =
        Clamp(targets.total_fx_n, -max_total_fx_n, max_total_fx_n);

    double x_moment_denominator = 0.0;
    for (std::size_t leg_index = 0; leg_index < kLegCount; ++leg_index) {
      if (!active_contacts[leg_index]) {
        continue;
      }
      x_moment_denominator +=
          ComputeSagittalLegKinematics(leg_index).x_m *
          ComputeSagittalLegKinematics(leg_index).x_m;
    }
    x_moment_denominator = std::max(x_moment_denominator, 1e-6);

    const double base_leg_fx =
        targets.total_fx_n / static_cast<double>(active_leg_count);
    const double base_leg_fz =
        targets.total_fz_n / static_cast<double>(active_leg_count);

    for (std::size_t leg_index = 0; leg_index < kLegCount; ++leg_index) {
      if (!active_contacts[leg_index]) {
        continue;
      }

      const auto kinematics = ComputeSagittalLegKinematics(leg_index);
      double leg_fx = base_leg_fx;
      double leg_fz =
          base_leg_fz - targets.pitch_moment_nm * kinematics.x_m / x_moment_denominator;
      leg_fz = Clamp(leg_fz, 0.0, wbc_max_leg_normal_force_n_);
      const double friction_limit_n = wbc_friction_coeff_ * leg_fz;
      leg_fx = Clamp(leg_fx, -friction_limit_n, friction_limit_n);
      AccumulateLegForceTorques(
          leg_index, wbc_tau_blend_ * leg_fx, wbc_tau_blend_ * leg_fz, tau_ff);
    }
  }

  double CurrentPlanarBodySpeedMps() const {
    if (!have_sport_state_) {
      return 0.0;
    }

    const double vx = latest_sport_state_.velocity[0];
    const double vy = latest_sport_state_.velocity[1];
    return std::sqrt(vx * vx + vy * vy);
  }

  double CurrentAbsPitchDeg() const {
    if (!have_low_state_) {
      return 0.0;
    }

    return std::abs(static_cast<double>(latest_low_state_.imu_state.rpy[1])) *
           kRadToDeg;
  }

  double CurrentAbsPitchRateDegps() const {
    if (!have_low_state_) {
      return 0.0;
    }

    return std::abs(static_cast<double>(latest_low_state_.imu_state.gyroscope[1])) *
           kRadToDeg;
  }

  std::array<double, 12> CurrentLandingReferencePose() const {
    if (!landing_hold_pose_captured_) {
      return plan_.landing_pose;
    }
    const double touchdown_blend =
        landing_hold_use_touchdown_pose_ ? 1.0 : landing_touchdown_reference_blend_;
    return go2_jump_planner::InterpolatePose(plan_.landing_pose, landing_hold_pose_,
                                             touchdown_blend);
  }

  std::array<double, 12> CurrentRecoveryTargetPose() const {
    return go2_jump_planner::InterpolatePose(plan_.crouch_pose, plan_.stand_pose,
                                             recovery_upright_blend_);
  }

  std::array<double, 12> ComputeRecoverySupportPose(
      double elapsed_s, const std::array<double, 12>& support_hold_pose,
      const std::array<double, 12>& recovery_target_pose) const {
    if (support_relax_duration_s_ <= 1e-6) {
      return support_hold_pose;
    }

    const double time_in_recovery_s =
        std::max(0.0, elapsed_s - phase_start_elapsed_s_);
    const double relax_alpha =
        Clamp(time_in_recovery_s / support_relax_duration_s_, 0.0, 1.0);
    return go2_jump_planner::InterpolatePose(support_hold_pose,
                                             recovery_target_pose, relax_alpha);
  }

  std::array<double, 12> ComputeSupportHoldPose(
      const std::array<double, 12>& landing_reference_pose) const {
    return go2_jump_planner::InterpolatePose(landing_reference_pose,
                                             plan_.support_pose,
                                             landing_support_blend_);
  }

  bool RecoveryReadyToStand() const {
    if (!have_low_state_) {
      return true;
    }

    if (CurrentAbsPitchDeg() > recovery_release_pitch_deg_) {
      return false;
    }
    if (CurrentAbsPitchRateDegps() > recovery_release_pitch_rate_degps_) {
      return false;
    }

    if (!have_sport_state_) {
      return true;
    }

    const double planar_speed_mps = CurrentPlanarBodySpeedMps();
    const double vertical_speed_mps = std::abs(latest_sport_state_.velocity[2]);
    return planar_speed_mps <= recovery_release_forward_speed_mps_ &&
           vertical_speed_mps <= landing_speed_threshold_mps_;
  }

  void CaptureRecoveryReleaseState(double elapsed_s) {
    if (!trial_metrics_.active ||
        std::isfinite(trial_metrics_.recovery_release_time_s)) {
      return;
    }

    trial_metrics_.recovery_release_time_s = elapsed_s;
    if (have_sport_state_ && trial_metrics_.baseline_captured) {
      trial_metrics_.recovery_release_forward_displacement_m =
          latest_sport_state_.position[0] - trial_metrics_.baseline_position[0];
      trial_metrics_.recovery_release_forward_speed_mps =
          latest_sport_state_.velocity[0];
    }
    if (have_low_state_) {
      trial_metrics_.recovery_release_pitch_rad =
          static_cast<double>(latest_low_state_.imu_state.rpy[1]);
    }
  }

  void MaybeStartRecoveryRelease(double elapsed_s) {
    if (std::isfinite(recovery_release_elapsed_s_)) {
      return;
    }

    const double time_in_recovery_s = elapsed_s - phase_start_elapsed_s_;
    if (time_in_recovery_s < recovery_min_hold_s_) {
      return;
    }

    if (RecoveryReadyToStand() || time_in_recovery_s >= recovery_max_hold_s_) {
      recovery_release_elapsed_s_ = elapsed_s;
      CaptureRecoveryReleaseState(elapsed_s);
    }
  }

  void UpdateRuntimePhase(double elapsed_s) {
    const double planned_push_end_s = plan_.crouch_duration_s + plan_.push_duration_s;
    const double max_push_end_s = planned_push_end_s + takeoff_wait_timeout_s_;
    const double max_flight_end_s =
        max_push_end_s + plan_.estimated_flight_time_s + landing_wait_timeout_s_;

    switch (runtime_phase_) {
      case RuntimePhase::kStand:
        break;
      case RuntimePhase::kCrouch:
        if (elapsed_s >= plan_.crouch_duration_s) {
          TransitionToPhase(RuntimePhase::kPush, elapsed_s);
        }
        break;
      case RuntimePhase::kPush:
        if ((trial_metrics_.takeoff_detected &&
             elapsed_s >= plan_.crouch_duration_s) ||
            elapsed_s >= max_push_end_s) {
          TransitionToPhase(RuntimePhase::kFlight, elapsed_s);
        }
        break;
      case RuntimePhase::kFlight:
        if (trial_metrics_.landing_detected || elapsed_s >= max_flight_end_s) {
          TransitionToPhase(RuntimePhase::kLanding, elapsed_s);
        }
        break;
      case RuntimePhase::kLanding:
        if (elapsed_s - phase_start_elapsed_s_ >= plan_.landing_duration_s) {
          TransitionToPhase(RuntimePhase::kRecovery, elapsed_s);
        }
        break;
      case RuntimePhase::kRecovery:
        MaybeStartRecoveryRelease(elapsed_s);
        if (std::isfinite(recovery_release_elapsed_s_) &&
            elapsed_s - recovery_release_elapsed_s_ >= plan_.recovery_duration_s) {
          TransitionToPhase(RuntimePhase::kComplete, elapsed_s);
        }
        break;
      case RuntimePhase::kComplete:
        break;
    }
  }

  void UpdateTrialMetrics(double elapsed_s,
                          const std::array<double, 12>& target_pose) {
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

    const bool takeoff_by_contact =
        contact_signal_seen_ && elapsed_s >= plan_.crouch_duration_s &&
        total_foot_force_est <= takeoff_total_force_threshold_n_ && vz > 0.0;
    const bool takeoff_by_height =
        !contact_signal_seen_ && z >= takeoff_height_threshold_m_ && vz > 0.0;

    if (!trial_metrics_.takeoff_detected &&
        (takeoff_by_contact || takeoff_by_height)) {
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

    const bool landing_by_contact =
        contact_signal_seen_ &&
        total_foot_force_est >= landing_total_force_threshold_n_;
    const bool landing_by_height =
        !contact_signal_seen_ && z <= landing_height_threshold_m_ &&
        (vz <= 0.0 || std::abs(vz) <= landing_speed_threshold_mps_);
    const bool landing_by_timeout =
        elapsed_s >= plan_.crouch_duration_s + plan_.push_duration_s +
                         plan_.estimated_flight_time_s +
                         landing_wait_timeout_s_;

    if (trial_metrics_.takeoff_detected && !trial_metrics_.landing_detected &&
        elapsed_s > trial_metrics_.takeoff_time_s + 0.05 &&
        (landing_by_contact || landing_by_height || landing_by_timeout)) {
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
    const double push_extension_after_plan_s =
        std::isfinite(trial_metrics_.takeoff_time_s)
            ? (trial_metrics_.takeoff_time_s -
               (plan_.crouch_duration_s + plan_.push_duration_s))
            : std::numeric_limits<double>::quiet_NaN();
    const double measured_flight_time_s =
        (std::isfinite(trial_metrics_.takeoff_time_s) &&
         std::isfinite(trial_metrics_.landing_time_s))
            ? (trial_metrics_.landing_time_s - trial_metrics_.takeoff_time_s)
            : std::numeric_limits<double>::quiet_NaN();
    const double flight_extension_after_plan_s =
        std::isfinite(measured_flight_time_s)
            ? (measured_flight_time_s - plan_.estimated_flight_time_s)
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
    const double support_hold_forward_gain_m =
        (std::isfinite(trial_metrics_.recovery_release_forward_displacement_m) &&
         std::isfinite(trial_metrics_.landing_forward_displacement_m))
            ? (trial_metrics_.recovery_release_forward_displacement_m -
               trial_metrics_.landing_forward_displacement_m)
            : std::numeric_limits<double>::quiet_NaN();
    const double release_to_complete_forward_gain_m =
        (std::isfinite(trial_metrics_.final_forward_displacement_m) &&
         std::isfinite(trial_metrics_.recovery_release_forward_displacement_m))
            ? (trial_metrics_.final_forward_displacement_m -
               trial_metrics_.recovery_release_forward_displacement_m)
            : std::numeric_limits<double>::quiet_NaN();
    const double airborne_completion_ratio =
        std::isfinite(airborne_forward_progress_m) && plan_.target_distance_m > 1e-6
            ? (airborne_forward_progress_m / plan_.target_distance_m)
            : std::numeric_limits<double>::quiet_NaN();
    const double post_landing_completion_ratio =
        std::isfinite(post_landing_forward_gain_m) && plan_.target_distance_m > 1e-6
            ? (post_landing_forward_gain_m / plan_.target_distance_m)
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
    stream << "  planned_takeoff_velocity_x_mps: "
           << plan_.takeoff_velocity_x_mps << "\n";
    stream << "  planned_takeoff_velocity_z_mps: "
           << plan_.takeoff_velocity_z_mps << "\n";
    stream << "  planned_touchdown_velocity_z_mps: "
           << plan_.touchdown_velocity_z_mps << "\n";
    stream << "  planned_apex_height_above_takeoff_m: "
           << plan_.apex_height_above_takeoff_m << "\n";
    stream << "  planned_landing_capture_offset_m: "
           << plan_.landing_capture_offset_m << "\n";
    stream << "  planned_flight_time_s: " << plan_.estimated_flight_time_s << "\n";
    stream << "  use_centroidal_wbc: "
           << (use_centroidal_wbc_ ? "true" : "false") << "\n";
    stream << "  foot_contact_force_threshold: "
           << foot_contact_force_threshold_ << "\n";
    stream << "  foot_force_est_scale: " << foot_force_est_scale_ << "\n";
    stream << "  foot_force_filter_alpha: " << foot_force_filter_alpha_ << "\n";
    stream << "  takeoff_total_force_threshold_n: "
           << takeoff_total_force_threshold_n_ << "\n";
    stream << "  landing_total_force_threshold_n: "
           << landing_total_force_threshold_n_ << "\n";
    stream << "  push_front_tau_scale: " << push_front_tau_scale_ << "\n";
    stream << "  push_rear_tau_scale: " << push_rear_tau_scale_ << "\n";
    stream << "  landing_front_tau_scale: " << landing_front_tau_scale_ << "\n";
    stream << "  landing_rear_tau_scale: " << landing_rear_tau_scale_ << "\n";
    stream << "  landing_support_blend: " << landing_support_blend_ << "\n";
    stream << "  support_relax_duration_s: " << support_relax_duration_s_
           << "\n";
    stream << "  landing_touchdown_reference_blend: "
           << landing_touchdown_reference_blend_ << "\n";
    stream << "  push_pitch_target_deg: " << push_pitch_target_deg_ << "\n";
    stream << "  push_pitch_compactness_gain: " << push_pitch_compactness_gain_
           << "\n";
    stream << "  flight_pitch_target_deg: " << flight_pitch_target_deg_ << "\n";
    stream << "  flight_pitch_compactness_gain: "
           << flight_pitch_compactness_gain_ << "\n";
    stream << "  flight_landing_prep_height_m: "
           << flight_landing_prep_height_m_ << "\n";
    stream << "  flight_landing_prep_start_descent_speed_mps: "
           << flight_landing_prep_start_descent_speed_mps_ << "\n";
    stream << "  flight_landing_prep_full_descent_speed_mps: "
           << flight_landing_prep_full_descent_speed_mps_ << "\n";
    stream << "  flight_landing_prep_max_blend: "
           << flight_landing_prep_max_blend_ << "\n";
    stream << "  landing_pitch_target_deg: " << landing_pitch_target_deg_ << "\n";
    stream << "  landing_pitch_compactness_gain: "
           << landing_pitch_compactness_gain_ << "\n";
    stream << "  support_pitch_target_deg: " << support_pitch_target_deg_
           << "\n";
    stream << "  support_pitch_compactness_gain: "
           << support_pitch_compactness_gain_ << "\n";
    stream << "  support_pitch_rate_gain: " << support_pitch_rate_gain_ << "\n";
    stream << "  support_pitch_correction_limit_rad: "
           << support_pitch_correction_limit_rad_ << "\n";
    stream << "  pitch_capture_foot_delta_gain_m_per_rad: "
           << pitch_capture_foot_delta_gain_m_per_rad_ << "\n";
    stream << "  pitch_capture_foot_delta_rate_gain_m_per_radps: "
           << pitch_capture_foot_delta_rate_gain_m_per_radps_ << "\n";
    stream << "  pitch_capture_foot_delta_limit_m: "
           << pitch_capture_foot_delta_limit_m_ << "\n";
    stream << "  pitch_capture_front_z_scale: "
           << pitch_capture_front_z_scale_ << "\n";
    stream << "  pitch_capture_rear_z_scale: "
           << pitch_capture_rear_z_scale_ << "\n";
    stream << "  flight_pitch_capture_gain_scale: "
           << flight_pitch_capture_gain_scale_ << "\n";
    stream << "  landing_pitch_capture_gain_scale: "
           << landing_pitch_capture_gain_scale_ << "\n";
    stream << "  support_pitch_capture_gain_scale: "
           << support_pitch_capture_gain_scale_ << "\n";
    stream << "  support_pitch_capture_fade_duration_s: "
           << support_pitch_capture_fade_duration_s_ << "\n";
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
    stream << "  airborne_completion_ratio: "
           << FormatDouble(airborne_completion_ratio) << "\n";
    stream << "  post_landing_forward_gain_m: "
           << FormatDouble(post_landing_forward_gain_m) << "\n";
    stream << "  support_hold_forward_gain_m: "
           << FormatDouble(support_hold_forward_gain_m) << "\n";
    stream << "  release_to_complete_forward_gain_m: "
           << FormatDouble(release_to_complete_forward_gain_m) << "\n";
    stream << "  post_landing_completion_ratio: "
           << FormatDouble(post_landing_completion_ratio) << "\n";
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
    stream << "  push_extension_after_plan_s: "
           << FormatDouble(push_extension_after_plan_s) << "\n";
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
    stream << "  recovery_release_time_s: "
           << FormatDouble(trial_metrics_.recovery_release_time_s) << "\n";
    stream << "  recovery_release_forward_displacement_m: "
           << FormatDouble(trial_metrics_.recovery_release_forward_displacement_m)
           << "\n";
    stream << "  recovery_release_forward_speed_mps: "
           << FormatDouble(trial_metrics_.recovery_release_forward_speed_mps)
           << "\n";
    stream << "  measured_flight_time_s: "
           << FormatDouble(measured_flight_time_s) << "\n";
    stream << "  flight_extension_after_plan_s: "
           << FormatDouble(flight_extension_after_plan_s) << "\n";
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
    stream << "  recovery_release_pitch_deg: "
           << FormatDouble(trial_metrics_.recovery_release_pitch_rad * kRadToDeg)
           << "\n";
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

    UpdateFilteredFootForces();

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
    UpdateRuntimePhase(elapsed);

    std::array<double, 12> target_pose = plan_.stand_pose;
    std::array<double, 12> tau_ff{};
    double kp = hold_kp_;
    double kd = kd_;
    const std::string phase = PhaseName(runtime_phase_);
    const std::array<double, 12> landing_reference_pose =
        CurrentLandingReferencePose();
    const std::array<double, 12> recovery_target_pose =
        CurrentRecoveryTargetPose();
    const std::array<double, 12>& support_hold_pose =
        support_hold_pose_captured_ ? support_hold_pose_ : landing_reference_pose;
    const std::array<double, 12> relaxed_support_pose =
        ComputeRecoverySupportPose(elapsed, support_hold_pose,
                                   recovery_target_pose);

    if (runtime_phase_ == RuntimePhase::kRecovery &&
        std::isfinite(recovery_release_elapsed_s_) &&
        !recovery_release_start_pose_captured_) {
      recovery_release_start_pose_ = relaxed_support_pose;
      recovery_release_start_pose_captured_ = true;
    }

    if (runtime_phase_ == RuntimePhase::kCrouch) {
      target_pose = go2_jump_planner::InterpolatePose(
          plan_.stand_pose, plan_.crouch_pose, elapsed / plan_.crouch_duration_s);
    } else if (runtime_phase_ == RuntimePhase::kPush) {
      kp = hold_kp_ + 5.0;
      target_pose = go2_jump_planner::InterpolatePose(
          plan_.crouch_pose, plan_.push_pose,
          (elapsed - plan_.crouch_duration_s) / plan_.push_duration_s);
      ApplyPitchCompactnessCorrection(target_pose, push_pitch_target_deg_,
                                      push_pitch_compactness_gain_,
                                      push_pitch_rate_gain_,
                                      push_pitch_correction_limit_rad_);
      ApplyLegTorque(plan_.push_thigh_tau_ff * push_front_tau_scale_,
                     plan_.push_calf_tau_ff * push_front_tau_scale_,
                     plan_.push_thigh_tau_ff * push_rear_tau_scale_,
                     plan_.push_calf_tau_ff * push_rear_tau_scale_, tau_ff);
      ApplyCentroidalWbcTorques(elapsed, tau_ff);
    } else if (runtime_phase_ == RuntimePhase::kFlight) {
      kp = flight_kp_;
      const double landing_prep_blend = ComputeFlightLandingPrepBlend();
      target_pose = go2_jump_planner::InterpolatePose(
          plan_.flight_pose, plan_.landing_pose, landing_prep_blend);
      const double pitch_target_deg =
          flight_pitch_target_deg_ +
          (landing_pitch_target_deg_ - flight_pitch_target_deg_) *
              landing_prep_blend;
      const double compactness_gain =
          flight_pitch_compactness_gain_ +
          (landing_pitch_compactness_gain_ - flight_pitch_compactness_gain_) *
              landing_prep_blend;
      const double pitch_rate_gain =
          flight_pitch_rate_gain_ +
          (landing_pitch_rate_gain_ - flight_pitch_rate_gain_) *
              landing_prep_blend;
      const double correction_limit_rad =
          flight_pitch_correction_limit_rad_ +
          (landing_pitch_correction_limit_rad_ -
           flight_pitch_correction_limit_rad_) *
              landing_prep_blend;
      ApplyPitchCompactnessCorrection(target_pose, pitch_target_deg,
                                      compactness_gain, pitch_rate_gain,
                                      correction_limit_rad);
      ApplyPitchCaptureFootPlacementCorrection(
          target_pose, pitch_target_deg,
          flight_pitch_capture_gain_scale_ * landing_prep_blend);
    } else if (runtime_phase_ == RuntimePhase::kLanding) {
      kp = landing_kp_;
      kd = landing_kd_;
      const double landing_alpha = Clamp(
          (elapsed - phase_start_elapsed_s_) / plan_.landing_duration_s, 0.0,
          1.0);
      target_pose = go2_jump_planner::InterpolatePose(
          landing_reference_pose, support_hold_pose, landing_alpha);
      const double pitch_target_deg =
          landing_pitch_target_deg_ +
          (support_pitch_target_deg_ - landing_pitch_target_deg_) *
              landing_alpha;
      const double compactness_gain =
          landing_pitch_compactness_gain_ +
          (support_pitch_compactness_gain_ - landing_pitch_compactness_gain_) *
              landing_alpha;
      const double pitch_rate_gain =
          landing_pitch_rate_gain_ +
          (support_pitch_rate_gain_ - landing_pitch_rate_gain_) * landing_alpha;
      const double correction_limit_rad =
          landing_pitch_correction_limit_rad_ +
          (support_pitch_correction_limit_rad_ -
           landing_pitch_correction_limit_rad_) *
              landing_alpha;
      ApplyPitchCompactnessCorrection(target_pose, pitch_target_deg,
                                      compactness_gain, pitch_rate_gain,
                                      correction_limit_rad);
      const double capture_gain_scale =
          landing_pitch_capture_gain_scale_ +
          (support_pitch_capture_gain_scale_ -
           landing_pitch_capture_gain_scale_) *
              landing_alpha;
      ApplyPitchCaptureFootPlacementCorrection(target_pose, pitch_target_deg,
                                               capture_gain_scale);
      ApplyLegTorque(-plan_.landing_thigh_tau_ff * landing_front_tau_scale_,
                     -plan_.landing_calf_tau_ff * landing_front_tau_scale_,
                     -plan_.landing_thigh_tau_ff * landing_rear_tau_scale_,
                     -plan_.landing_calf_tau_ff * landing_rear_tau_scale_,
                     tau_ff);
      ApplyCentroidalWbcTorques(elapsed, tau_ff);
    } else if (runtime_phase_ == RuntimePhase::kRecovery) {
      if (std::isfinite(recovery_release_elapsed_s_)) {
        kp = recovery_kp_;
        kd = recovery_kd_;
        const double recovery_alpha =
            (elapsed - recovery_release_elapsed_s_) / plan_.recovery_duration_s;
        target_pose = go2_jump_planner::InterpolatePose(
            recovery_release_start_pose_captured_ ? recovery_release_start_pose_
                                                  : relaxed_support_pose,
            recovery_target_pose, recovery_alpha);
        const double correction_fade =
            1.0 - Clamp(recovery_alpha, 0.0, 1.0);
        ApplyPitchCompactnessCorrection(target_pose, support_pitch_target_deg_,
                                        support_pitch_compactness_gain_ *
                                            correction_fade,
                                        support_pitch_rate_gain_ * correction_fade,
                                        support_pitch_correction_limit_rad_ *
                                            correction_fade);
        ApplyPitchCaptureFootPlacementCorrection(
            target_pose, support_pitch_target_deg_,
            support_pitch_capture_gain_scale_ * correction_fade);
      } else {
        kp = support_kp_;
        kd = support_kd_;
        target_pose = relaxed_support_pose;
        const double time_in_recovery_s =
            std::max(0.0, elapsed - phase_start_elapsed_s_);
        double support_capture_fade = 1.0;
        if (support_pitch_capture_fade_duration_s_ > 1e-6) {
          support_capture_fade =
              1.0 -
              Clamp(time_in_recovery_s / support_pitch_capture_fade_duration_s_,
                    0.0, 1.0);
        }
        ApplyPitchCompactnessCorrection(target_pose, support_pitch_target_deg_,
                                        support_pitch_compactness_gain_,
                                        support_pitch_rate_gain_,
                                        support_pitch_correction_limit_rad_);
        ApplyPitchCaptureFootPlacementCorrection(
            target_pose, support_pitch_target_deg_,
            support_pitch_capture_gain_scale_ * support_capture_fade);
        ApplyCentroidalWbcTorques(elapsed, tau_ff);
      }
    } else if (runtime_phase_ == RuntimePhase::kComplete) {
      target_pose = recovery_target_pose;
    }

    if (phase != last_phase_) {
      last_phase_ = phase;
      RCLCPP_INFO(this->get_logger(), "Jump phase -> %s", phase.c_str());
    }

    PublishPose(target_pose, kp, kd, tau_ff);
    UpdateTrialMetrics(elapsed, target_pose);
    UpdateRuntimePhase(elapsed);

    if (runtime_phase_ == RuntimePhase::kComplete) {
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
                "Jump plan distance=%.3f m, scale_mode=%s, ballistic_takeoff=%.3f m/s, speed_scale=%.3f, takeoff_speed=%.3f m/s, takeoff_vx=%.3f m/s, takeoff_vz=%.3f m/s, apex=%.3f m, capture_offset=%.3f m, flight=%.3f s, centroidal_wbc=%s",
                plan_.target_distance_m,
                plan_.using_takeoff_speed_scale_curve ? "curve" : "manual",
                plan_.ballistic_takeoff_speed_mps,
                plan_.takeoff_speed_scale, plan_.takeoff_speed_mps,
                plan_.takeoff_velocity_x_mps, plan_.takeoff_velocity_z_mps,
                plan_.apex_height_above_takeoff_m,
                plan_.landing_capture_offset_m,
                plan_.estimated_flight_time_s,
                use_centroidal_wbc_ ? "true" : "false");
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
  double flight_landing_prep_height_m_{0.14};
  double flight_landing_prep_start_descent_speed_mps_{0.10};
  double flight_landing_prep_full_descent_speed_mps_{1.20};
  double flight_landing_prep_max_blend_{0.0};
  double landing_pitch_target_deg_{-8.0};
  double landing_pitch_compactness_gain_{0.40};
  double landing_pitch_rate_gain_{0.04};
  double landing_pitch_correction_limit_rad_{0.12};
  double support_pitch_target_deg_{-2.0};
  double support_pitch_compactness_gain_{0.65};
  double support_pitch_rate_gain_{0.06};
  double support_pitch_correction_limit_rad_{0.18};
  double pitch_capture_foot_delta_gain_m_per_rad_{0.035};
  double pitch_capture_foot_delta_rate_gain_m_per_radps_{0.008};
  double pitch_capture_foot_delta_limit_m_{0.03};
  double pitch_capture_front_z_scale_{0.75};
  double pitch_capture_rear_z_scale_{0.35};
  double flight_pitch_capture_gain_scale_{0.80};
  double landing_pitch_capture_gain_scale_{1.00};
  double support_pitch_capture_gain_scale_{1.15};
  double support_pitch_capture_fade_duration_s_{0.12};
  double startup_pose_tolerance_rad_{0.20};
  double startup_body_speed_tolerance_mps_{0.30};
  double startup_tilt_tolerance_deg_{20.0};
  double takeoff_height_threshold_m_{0.03};
  double landing_height_threshold_m_{0.02};
  double landing_speed_threshold_mps_{0.35};
  double takeoff_wait_timeout_s_{0.18};
  double landing_wait_timeout_s_{0.30};
  double landing_kp_{55.0};
  double landing_kd_{5.0};
  double recovery_kp_{55.0};
  double recovery_kd_{4.5};
  double recovery_min_hold_s_{0.06};
  double recovery_max_hold_s_{0.30};
  double recovery_release_forward_speed_mps_{0.18};
  double recovery_release_pitch_deg_{18.0};
  double recovery_release_pitch_rate_degps_{45.0};
  double recovery_upright_blend_{0.35};
  double landing_support_blend_{0.55};
  double support_relax_duration_s_{0.0};
  double landing_touchdown_reference_blend_{0.0};
  double support_kp_{55.0};
  double support_kd_{5.5};
  bool use_centroidal_wbc_{false};
  double robot_mass_kg_{15.0};
  double leg_link_length_m_{0.213};
  double foot_contact_force_threshold_{25.0};
  double takeoff_contact_force_threshold_{15.0};
  double landing_contact_force_threshold_{30.0};
  double foot_force_est_scale_{0.1};
  double foot_force_filter_alpha_{0.15};
  double takeoff_total_force_threshold_n_{40.0};
  double landing_total_force_threshold_n_{80.0};
  double wbc_friction_coeff_{0.7};
  double wbc_max_leg_normal_force_n_{180.0};
  double wbc_push_velocity_gain_{10.0};
  double wbc_push_vertical_velocity_gain_{12.0};
  double wbc_landing_velocity_gain_{8.0};
  double wbc_landing_height_gain_{40.0};
  double wbc_pitch_gain_{85.0};
  double wbc_pitch_rate_gain_{10.0};
  double wbc_pitch_moment_limit_nm_{55.0};
  double wbc_tau_blend_{1.0};
  bool landing_hold_use_touchdown_pose_{false};
  bool auto_start_{true};

  bool have_low_state_{false};
  bool have_sport_state_{false};
  bool first_low_state_seen_{false};
  bool jump_started_{false};
  bool contact_signal_seen_{false};

  std::string report_path_;
  std::string last_phase_;
  RuntimePhase runtime_phase_{RuntimePhase::kStand};
  double phase_start_elapsed_s_{0.0};
  double recovery_release_elapsed_s_{
      std::numeric_limits<double>::quiet_NaN()};
  std::array<double, 12> landing_hold_pose_{};
  bool landing_hold_pose_captured_{false};
  std::array<double, 12> support_hold_pose_{};
  bool support_hold_pose_captured_{false};
  std::array<double, 12> recovery_release_start_pose_{};
  bool recovery_release_start_pose_captured_{false};

  rclcpp::Time first_low_state_time_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time jump_start_time_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time last_startup_wait_log_time_{0, 0, RCL_SYSTEM_TIME};

  std::array<double, 12> active_target_pose_{};
  std::array<double, kLegCount> filtered_foot_force_est_n_{};

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
