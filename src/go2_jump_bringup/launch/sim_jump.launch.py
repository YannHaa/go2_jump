from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("go2_jump_bringup"), "config", "jump_params.yaml"]
    )

    target_distance = LaunchConfiguration("target_distance_m")
    takeoff_angle = LaunchConfiguration("takeoff_angle_deg")
    takeoff_speed_scale = LaunchConfiguration("takeoff_speed_scale")
    use_takeoff_speed_scale_curve = LaunchConfiguration(
        "use_takeoff_speed_scale_curve"
    )
    use_centroidal_wbc = LaunchConfiguration("use_centroidal_wbc")
    robot_mass_kg = LaunchConfiguration("robot_mass_kg")
    leg_link_length_m = LaunchConfiguration("leg_link_length_m")
    foot_contact_force_threshold = LaunchConfiguration(
        "foot_contact_force_threshold"
    )
    takeoff_contact_force_threshold = LaunchConfiguration(
        "takeoff_contact_force_threshold"
    )
    landing_contact_force_threshold = LaunchConfiguration(
        "landing_contact_force_threshold"
    )
    foot_force_est_scale = LaunchConfiguration("foot_force_est_scale")
    foot_force_filter_alpha = LaunchConfiguration("foot_force_filter_alpha")
    takeoff_total_force_threshold_n = LaunchConfiguration(
        "takeoff_total_force_threshold_n"
    )
    landing_total_force_threshold_n = LaunchConfiguration(
        "landing_total_force_threshold_n"
    )
    wbc_friction_coeff = LaunchConfiguration("wbc_friction_coeff")
    wbc_max_leg_normal_force_n = LaunchConfiguration(
        "wbc_max_leg_normal_force_n"
    )
    wbc_push_velocity_gain = LaunchConfiguration("wbc_push_velocity_gain")
    wbc_push_vertical_velocity_gain = LaunchConfiguration(
        "wbc_push_vertical_velocity_gain"
    )
    wbc_landing_velocity_gain = LaunchConfiguration(
        "wbc_landing_velocity_gain"
    )
    wbc_landing_height_gain = LaunchConfiguration("wbc_landing_height_gain")
    wbc_pitch_gain = LaunchConfiguration("wbc_pitch_gain")
    wbc_pitch_rate_gain = LaunchConfiguration("wbc_pitch_rate_gain")
    wbc_pitch_moment_limit_nm = LaunchConfiguration(
        "wbc_pitch_moment_limit_nm"
    )
    wbc_tau_blend = LaunchConfiguration("wbc_tau_blend")
    auto_start = LaunchConfiguration("auto_start")
    push_front_tau_scale = LaunchConfiguration("push_front_tau_scale")
    push_rear_tau_scale = LaunchConfiguration("push_rear_tau_scale")
    landing_front_tau_scale = LaunchConfiguration("landing_front_tau_scale")
    landing_rear_tau_scale = LaunchConfiguration("landing_rear_tau_scale")
    push_pitch_target_deg = LaunchConfiguration("push_pitch_target_deg")
    push_pitch_compactness_gain = LaunchConfiguration(
        "push_pitch_compactness_gain"
    )
    push_pitch_rate_gain = LaunchConfiguration("push_pitch_rate_gain")
    push_pitch_correction_limit_rad = LaunchConfiguration(
        "push_pitch_correction_limit_rad"
    )
    flight_pitch_target_deg = LaunchConfiguration("flight_pitch_target_deg")
    flight_pitch_compactness_gain = LaunchConfiguration(
        "flight_pitch_compactness_gain"
    )
    flight_pitch_rate_gain = LaunchConfiguration("flight_pitch_rate_gain")
    flight_pitch_correction_limit_rad = LaunchConfiguration(
        "flight_pitch_correction_limit_rad"
    )
    flight_landing_prep_height_m = LaunchConfiguration(
        "flight_landing_prep_height_m"
    )
    flight_landing_prep_start_descent_speed_mps = LaunchConfiguration(
        "flight_landing_prep_start_descent_speed_mps"
    )
    flight_landing_prep_full_descent_speed_mps = LaunchConfiguration(
        "flight_landing_prep_full_descent_speed_mps"
    )
    flight_landing_prep_max_blend = LaunchConfiguration(
        "flight_landing_prep_max_blend"
    )
    landing_pitch_target_deg = LaunchConfiguration("landing_pitch_target_deg")
    landing_pitch_compactness_gain = LaunchConfiguration(
        "landing_pitch_compactness_gain"
    )
    landing_pitch_rate_gain = LaunchConfiguration("landing_pitch_rate_gain")
    landing_pitch_correction_limit_rad = LaunchConfiguration(
        "landing_pitch_correction_limit_rad"
    )
    support_pitch_target_deg = LaunchConfiguration("support_pitch_target_deg")
    support_pitch_compactness_gain = LaunchConfiguration(
        "support_pitch_compactness_gain"
    )
    support_pitch_rate_gain = LaunchConfiguration("support_pitch_rate_gain")
    support_pitch_correction_limit_rad = LaunchConfiguration(
        "support_pitch_correction_limit_rad"
    )
    pitch_capture_foot_delta_gain_m_per_rad = LaunchConfiguration(
        "pitch_capture_foot_delta_gain_m_per_rad"
    )
    pitch_capture_foot_delta_rate_gain_m_per_radps = LaunchConfiguration(
        "pitch_capture_foot_delta_rate_gain_m_per_radps"
    )
    pitch_capture_foot_delta_limit_m = LaunchConfiguration(
        "pitch_capture_foot_delta_limit_m"
    )
    pitch_capture_front_z_scale = LaunchConfiguration(
        "pitch_capture_front_z_scale"
    )
    pitch_capture_rear_z_scale = LaunchConfiguration(
        "pitch_capture_rear_z_scale"
    )
    flight_pitch_capture_gain_scale = LaunchConfiguration(
        "flight_pitch_capture_gain_scale"
    )
    landing_pitch_capture_gain_scale = LaunchConfiguration(
        "landing_pitch_capture_gain_scale"
    )
    support_pitch_capture_gain_scale = LaunchConfiguration(
        "support_pitch_capture_gain_scale"
    )
    support_pitch_capture_fade_duration_s = LaunchConfiguration(
        "support_pitch_capture_fade_duration_s"
    )
    crouch_forward_bias_rad = LaunchConfiguration("crouch_forward_bias_rad")
    push_forward_bias_rad = LaunchConfiguration("push_forward_bias_rad")
    push_front_compact_delta_rad = LaunchConfiguration(
        "push_front_compact_delta_rad"
    )
    push_rear_compact_delta_rad = LaunchConfiguration(
        "push_rear_compact_delta_rad"
    )
    flight_front_compact_delta_rad = LaunchConfiguration(
        "flight_front_compact_delta_rad"
    )
    flight_rear_compact_delta_rad = LaunchConfiguration(
        "flight_rear_compact_delta_rad"
    )
    landing_front_compact_delta_rad = LaunchConfiguration(
        "landing_front_compact_delta_rad"
    )
    landing_rear_compact_delta_rad = LaunchConfiguration(
        "landing_rear_compact_delta_rad"
    )
    landing_absorption_blend = LaunchConfiguration("landing_absorption_blend")
    landing_capture_time_constant_s = LaunchConfiguration(
        "landing_capture_time_constant_s"
    )
    landing_capture_rear_ratio = LaunchConfiguration(
        "landing_capture_rear_ratio"
    )
    landing_capture_limit_m = LaunchConfiguration("landing_capture_limit_m")
    landing_extension_m = LaunchConfiguration("landing_extension_m")
    support_capture_ratio = LaunchConfiguration("support_capture_ratio")
    support_hip_rad = LaunchConfiguration("support_hip_rad")
    support_thigh_rad = LaunchConfiguration("support_thigh_rad")
    support_calf_rad = LaunchConfiguration("support_calf_rad")
    support_front_compact_delta_rad = LaunchConfiguration(
        "support_front_compact_delta_rad"
    )
    support_rear_compact_delta_rad = LaunchConfiguration(
        "support_rear_compact_delta_rad"
    )
    landing_kd = LaunchConfiguration("landing_kd")
    support_kp = LaunchConfiguration("support_kp")
    support_kd = LaunchConfiguration("support_kd")
    recovery_kd = LaunchConfiguration("recovery_kd")
    recovery_release_forward_speed_mps = LaunchConfiguration(
        "recovery_release_forward_speed_mps"
    )
    recovery_release_pitch_deg = LaunchConfiguration("recovery_release_pitch_deg")
    recovery_release_pitch_rate_degps = LaunchConfiguration(
        "recovery_release_pitch_rate_degps"
    )
    recovery_max_hold_s = LaunchConfiguration("recovery_max_hold_s")
    recovery_upright_blend = LaunchConfiguration("recovery_upright_blend")
    landing_support_blend = LaunchConfiguration("landing_support_blend")
    support_relax_duration_s = LaunchConfiguration("support_relax_duration_s")
    landing_touchdown_reference_blend = LaunchConfiguration(
        "landing_touchdown_reference_blend"
    )
    landing_hold_use_touchdown_pose = LaunchConfiguration(
        "landing_hold_use_touchdown_pose"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("target_distance_m", default_value="0.25"),
            DeclareLaunchArgument("takeoff_angle_deg", default_value="35.0"),
            DeclareLaunchArgument("takeoff_speed_scale", default_value="1.06"),
            DeclareLaunchArgument(
                "use_takeoff_speed_scale_curve", default_value="true"
            ),
            DeclareLaunchArgument("use_centroidal_wbc", default_value="false"),
            DeclareLaunchArgument("robot_mass_kg", default_value="15.0"),
            DeclareLaunchArgument("leg_link_length_m", default_value="0.213"),
            DeclareLaunchArgument(
                "foot_contact_force_threshold", default_value="25.0"
            ),
            DeclareLaunchArgument(
                "takeoff_contact_force_threshold", default_value="15.0"
            ),
            DeclareLaunchArgument(
                "landing_contact_force_threshold", default_value="30.0"
            ),
            DeclareLaunchArgument("foot_force_est_scale", default_value="0.1"),
            DeclareLaunchArgument("foot_force_filter_alpha", default_value="0.15"),
            DeclareLaunchArgument(
                "takeoff_total_force_threshold_n", default_value="40.0"
            ),
            DeclareLaunchArgument(
                "landing_total_force_threshold_n", default_value="80.0"
            ),
            DeclareLaunchArgument("wbc_friction_coeff", default_value="0.70"),
            DeclareLaunchArgument(
                "wbc_max_leg_normal_force_n", default_value="180.0"
            ),
            DeclareLaunchArgument(
                "wbc_push_velocity_gain", default_value="10.0"
            ),
            DeclareLaunchArgument(
                "wbc_push_vertical_velocity_gain", default_value="12.0"
            ),
            DeclareLaunchArgument(
                "wbc_landing_velocity_gain", default_value="8.0"
            ),
            DeclareLaunchArgument(
                "wbc_landing_height_gain", default_value="40.0"
            ),
            DeclareLaunchArgument("wbc_pitch_gain", default_value="85.0"),
            DeclareLaunchArgument("wbc_pitch_rate_gain", default_value="10.0"),
            DeclareLaunchArgument(
                "wbc_pitch_moment_limit_nm", default_value="55.0"
            ),
            DeclareLaunchArgument("wbc_tau_blend", default_value="1.0"),
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument("push_front_tau_scale", default_value="0.96"),
            DeclareLaunchArgument("push_rear_tau_scale", default_value="1.12"),
            DeclareLaunchArgument("landing_front_tau_scale", default_value="0.90"),
            DeclareLaunchArgument("landing_rear_tau_scale", default_value="1.10"),
            DeclareLaunchArgument("push_pitch_target_deg", default_value="-5.0"),
            DeclareLaunchArgument(
                "push_pitch_compactness_gain", default_value="0.35"
            ),
            DeclareLaunchArgument("push_pitch_rate_gain", default_value="0.03"),
            DeclareLaunchArgument(
                "push_pitch_correction_limit_rad", default_value="0.08"
            ),
            DeclareLaunchArgument("flight_pitch_target_deg", default_value="-2.0"),
            DeclareLaunchArgument(
                "flight_pitch_compactness_gain", default_value="0.55"
            ),
            DeclareLaunchArgument("flight_pitch_rate_gain", default_value="0.05"),
            DeclareLaunchArgument(
                "flight_pitch_correction_limit_rad", default_value="0.18"
            ),
            DeclareLaunchArgument(
                "flight_landing_prep_height_m", default_value="0.14"
            ),
            DeclareLaunchArgument(
                "flight_landing_prep_start_descent_speed_mps",
                default_value="0.10",
            ),
            DeclareLaunchArgument(
                "flight_landing_prep_full_descent_speed_mps",
                default_value="1.20",
            ),
            DeclareLaunchArgument(
                "flight_landing_prep_max_blend", default_value="0.00"
            ),
            DeclareLaunchArgument("landing_pitch_target_deg", default_value="-8.0"),
            DeclareLaunchArgument(
                "landing_pitch_compactness_gain", default_value="0.40"
            ),
            DeclareLaunchArgument("landing_pitch_rate_gain", default_value="0.04"),
            DeclareLaunchArgument(
                "landing_pitch_correction_limit_rad", default_value="0.12"
            ),
            DeclareLaunchArgument("support_pitch_target_deg", default_value="-2.0"),
            DeclareLaunchArgument(
                "support_pitch_compactness_gain", default_value="0.65"
            ),
            DeclareLaunchArgument("support_pitch_rate_gain", default_value="0.06"),
            DeclareLaunchArgument(
                "support_pitch_correction_limit_rad", default_value="0.18"
            ),
            DeclareLaunchArgument(
                "pitch_capture_foot_delta_gain_m_per_rad",
                default_value="0.035",
            ),
            DeclareLaunchArgument(
                "pitch_capture_foot_delta_rate_gain_m_per_radps",
                default_value="0.008",
            ),
            DeclareLaunchArgument(
                "pitch_capture_foot_delta_limit_m", default_value="0.03"
            ),
            DeclareLaunchArgument(
                "pitch_capture_front_z_scale", default_value="0.75"
            ),
            DeclareLaunchArgument(
                "pitch_capture_rear_z_scale", default_value="0.35"
            ),
            DeclareLaunchArgument(
                "flight_pitch_capture_gain_scale", default_value="0.80"
            ),
            DeclareLaunchArgument(
                "landing_pitch_capture_gain_scale", default_value="1.00"
            ),
            DeclareLaunchArgument(
                "support_pitch_capture_gain_scale", default_value="1.15"
            ),
            DeclareLaunchArgument(
                "support_pitch_capture_fade_duration_s", default_value="0.12"
            ),
            DeclareLaunchArgument("crouch_forward_bias_rad", default_value="0.08"),
            DeclareLaunchArgument("push_forward_bias_rad", default_value="0.05"),
            DeclareLaunchArgument(
                "push_front_compact_delta_rad", default_value="-0.18"
            ),
            DeclareLaunchArgument(
                "push_rear_compact_delta_rad", default_value="0.04"
            ),
            DeclareLaunchArgument(
                "flight_front_compact_delta_rad", default_value="0.20"
            ),
            DeclareLaunchArgument(
                "flight_rear_compact_delta_rad", default_value="-0.10"
            ),
            DeclareLaunchArgument(
                "landing_front_compact_delta_rad", default_value="-0.08"
            ),
            DeclareLaunchArgument(
                "landing_rear_compact_delta_rad", default_value="0.14"
            ),
            DeclareLaunchArgument("landing_absorption_blend", default_value="0.45"),
            DeclareLaunchArgument(
                "landing_capture_time_constant_s", default_value="0.06"
            ),
            DeclareLaunchArgument(
                "landing_capture_rear_ratio", default_value="0.60"
            ),
            DeclareLaunchArgument("landing_capture_limit_m", default_value="0.08"),
            DeclareLaunchArgument("landing_extension_m", default_value="0.015"),
            DeclareLaunchArgument("support_capture_ratio", default_value="0.60"),
            DeclareLaunchArgument("support_hip_rad", default_value="0.036"),
            DeclareLaunchArgument("support_thigh_rad", default_value="1.126"),
            DeclareLaunchArgument("support_calf_rad", default_value="-2.215"),
            DeclareLaunchArgument(
                "support_front_compact_delta_rad", default_value="0.164"
            ),
            DeclareLaunchArgument(
                "support_rear_compact_delta_rad", default_value="-0.164"
            ),
            DeclareLaunchArgument("landing_kd", default_value="5.5"),
            DeclareLaunchArgument("support_kp", default_value="58.0"),
            DeclareLaunchArgument("support_kd", default_value="6.5"),
            DeclareLaunchArgument("recovery_kd", default_value="5.0"),
            DeclareLaunchArgument(
                "recovery_release_forward_speed_mps", default_value="0.08"
            ),
            DeclareLaunchArgument("recovery_release_pitch_deg", default_value="10.0"),
            DeclareLaunchArgument(
                "recovery_release_pitch_rate_degps", default_value="35.0"
            ),
            DeclareLaunchArgument("recovery_max_hold_s", default_value="0.55"),
            DeclareLaunchArgument("recovery_upright_blend", default_value="0.35"),
            DeclareLaunchArgument("landing_support_blend", default_value="0.40"),
            DeclareLaunchArgument("support_relax_duration_s", default_value="0.00"),
            DeclareLaunchArgument(
                "landing_touchdown_reference_blend", default_value="0.80"
            ),
            DeclareLaunchArgument(
                "landing_hold_use_touchdown_pose", default_value="false"
            ),
            Node(
                package="go2_jump_planner",
                executable="jump_target_node",
                name="go2_jump_planner",
                output="screen",
                parameters=[
                    config_path,
                    {
                        "target_distance_m": target_distance,
                        "takeoff_angle_deg": takeoff_angle,
                        "takeoff_speed_scale": takeoff_speed_scale,
                        "use_takeoff_speed_scale_curve": use_takeoff_speed_scale_curve,
                        "crouch_forward_bias_rad": crouch_forward_bias_rad,
                        "push_forward_bias_rad": push_forward_bias_rad,
                        "push_front_compact_delta_rad": push_front_compact_delta_rad,
                        "push_rear_compact_delta_rad": push_rear_compact_delta_rad,
                        "flight_front_compact_delta_rad": flight_front_compact_delta_rad,
                        "flight_rear_compact_delta_rad": flight_rear_compact_delta_rad,
                        "landing_front_compact_delta_rad": landing_front_compact_delta_rad,
                        "landing_rear_compact_delta_rad": landing_rear_compact_delta_rad,
                        "landing_absorption_blend": landing_absorption_blend,
                        "leg_link_length_m": leg_link_length_m,
                        "landing_capture_time_constant_s": landing_capture_time_constant_s,
                        "landing_capture_rear_ratio": landing_capture_rear_ratio,
                        "landing_capture_limit_m": landing_capture_limit_m,
                        "landing_extension_m": landing_extension_m,
                        "support_capture_ratio": support_capture_ratio,
                        "support_hip_rad": support_hip_rad,
                        "support_thigh_rad": support_thigh_rad,
                        "support_calf_rad": support_calf_rad,
                        "support_front_compact_delta_rad": support_front_compact_delta_rad,
                        "support_rear_compact_delta_rad": support_rear_compact_delta_rad,
                    }
                ],
            ),
            Node(
                package="go2_jump_controller",
                executable="jump_controller_node",
                name="go2_jump_controller",
                output="screen",
                parameters=[
                    config_path,
                    {
                        "auto_start": auto_start,
                        "target_distance_m": target_distance,
                        "takeoff_angle_deg": takeoff_angle,
                        "takeoff_speed_scale": takeoff_speed_scale,
                        "use_takeoff_speed_scale_curve": use_takeoff_speed_scale_curve,
                        "use_centroidal_wbc": use_centroidal_wbc,
                        "robot_mass_kg": robot_mass_kg,
                        "leg_link_length_m": leg_link_length_m,
                        "foot_contact_force_threshold": foot_contact_force_threshold,
                        "takeoff_contact_force_threshold": takeoff_contact_force_threshold,
                        "landing_contact_force_threshold": landing_contact_force_threshold,
                        "foot_force_est_scale": foot_force_est_scale,
                        "foot_force_filter_alpha": foot_force_filter_alpha,
                        "takeoff_total_force_threshold_n": takeoff_total_force_threshold_n,
                        "landing_total_force_threshold_n": landing_total_force_threshold_n,
                        "wbc_friction_coeff": wbc_friction_coeff,
                        "wbc_max_leg_normal_force_n": wbc_max_leg_normal_force_n,
                        "wbc_push_velocity_gain": wbc_push_velocity_gain,
                        "wbc_push_vertical_velocity_gain": wbc_push_vertical_velocity_gain,
                        "wbc_landing_velocity_gain": wbc_landing_velocity_gain,
                        "wbc_landing_height_gain": wbc_landing_height_gain,
                        "wbc_pitch_gain": wbc_pitch_gain,
                        "wbc_pitch_rate_gain": wbc_pitch_rate_gain,
                        "wbc_pitch_moment_limit_nm": wbc_pitch_moment_limit_nm,
                        "wbc_tau_blend": wbc_tau_blend,
                        "push_front_tau_scale": push_front_tau_scale,
                        "push_rear_tau_scale": push_rear_tau_scale,
                        "landing_front_tau_scale": landing_front_tau_scale,
                        "landing_rear_tau_scale": landing_rear_tau_scale,
                        "push_pitch_target_deg": push_pitch_target_deg,
                        "push_pitch_compactness_gain": push_pitch_compactness_gain,
                        "push_pitch_rate_gain": push_pitch_rate_gain,
                        "push_pitch_correction_limit_rad": push_pitch_correction_limit_rad,
                        "flight_pitch_target_deg": flight_pitch_target_deg,
                        "flight_pitch_compactness_gain": flight_pitch_compactness_gain,
                        "flight_pitch_rate_gain": flight_pitch_rate_gain,
                        "flight_pitch_correction_limit_rad": flight_pitch_correction_limit_rad,
                        "flight_landing_prep_height_m": flight_landing_prep_height_m,
                        "flight_landing_prep_start_descent_speed_mps": flight_landing_prep_start_descent_speed_mps,
                        "flight_landing_prep_full_descent_speed_mps": flight_landing_prep_full_descent_speed_mps,
                        "flight_landing_prep_max_blend": flight_landing_prep_max_blend,
                        "landing_pitch_target_deg": landing_pitch_target_deg,
                        "landing_pitch_compactness_gain": landing_pitch_compactness_gain,
                        "landing_pitch_rate_gain": landing_pitch_rate_gain,
                        "landing_pitch_correction_limit_rad": landing_pitch_correction_limit_rad,
                        "support_pitch_target_deg": support_pitch_target_deg,
                        "support_pitch_compactness_gain": support_pitch_compactness_gain,
                        "support_pitch_rate_gain": support_pitch_rate_gain,
                        "support_pitch_correction_limit_rad": support_pitch_correction_limit_rad,
                        "pitch_capture_foot_delta_gain_m_per_rad": pitch_capture_foot_delta_gain_m_per_rad,
                        "pitch_capture_foot_delta_rate_gain_m_per_radps": pitch_capture_foot_delta_rate_gain_m_per_radps,
                        "pitch_capture_foot_delta_limit_m": pitch_capture_foot_delta_limit_m,
                        "pitch_capture_front_z_scale": pitch_capture_front_z_scale,
                        "pitch_capture_rear_z_scale": pitch_capture_rear_z_scale,
                        "flight_pitch_capture_gain_scale": flight_pitch_capture_gain_scale,
                        "landing_pitch_capture_gain_scale": landing_pitch_capture_gain_scale,
                        "support_pitch_capture_gain_scale": support_pitch_capture_gain_scale,
                        "support_pitch_capture_fade_duration_s": support_pitch_capture_fade_duration_s,
                        "crouch_forward_bias_rad": crouch_forward_bias_rad,
                        "push_forward_bias_rad": push_forward_bias_rad,
                        "push_front_compact_delta_rad": push_front_compact_delta_rad,
                        "push_rear_compact_delta_rad": push_rear_compact_delta_rad,
                        "flight_front_compact_delta_rad": flight_front_compact_delta_rad,
                        "flight_rear_compact_delta_rad": flight_rear_compact_delta_rad,
                        "landing_front_compact_delta_rad": landing_front_compact_delta_rad,
                        "landing_rear_compact_delta_rad": landing_rear_compact_delta_rad,
                        "landing_absorption_blend": landing_absorption_blend,
                        "support_hip_rad": support_hip_rad,
                        "support_thigh_rad": support_thigh_rad,
                        "support_calf_rad": support_calf_rad,
                        "support_front_compact_delta_rad": support_front_compact_delta_rad,
                        "support_rear_compact_delta_rad": support_rear_compact_delta_rad,
                        "leg_link_length_m": leg_link_length_m,
                        "landing_capture_time_constant_s": landing_capture_time_constant_s,
                        "landing_capture_rear_ratio": landing_capture_rear_ratio,
                        "landing_capture_limit_m": landing_capture_limit_m,
                        "landing_extension_m": landing_extension_m,
                        "support_capture_ratio": support_capture_ratio,
                        "landing_kd": landing_kd,
                        "support_kp": support_kp,
                        "support_kd": support_kd,
                        "recovery_kd": recovery_kd,
                        "recovery_release_forward_speed_mps": recovery_release_forward_speed_mps,
                        "recovery_release_pitch_deg": recovery_release_pitch_deg,
                        "recovery_release_pitch_rate_degps": recovery_release_pitch_rate_degps,
                        "recovery_max_hold_s": recovery_max_hold_s,
                        "recovery_upright_blend": recovery_upright_blend,
                        "landing_support_blend": landing_support_blend,
                        "support_relax_duration_s": support_relax_duration_s,
                        "landing_touchdown_reference_blend": landing_touchdown_reference_blend,
                        "landing_hold_use_touchdown_pose": landing_hold_use_touchdown_pose,
                    },
                ],
            ),
        ]
    )
