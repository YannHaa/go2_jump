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
    auto_start = LaunchConfiguration("auto_start")

    return LaunchDescription(
        [
            DeclareLaunchArgument("target_distance_m", default_value="0.25"),
            DeclareLaunchArgument("takeoff_angle_deg", default_value="45.0"),
            DeclareLaunchArgument("takeoff_speed_scale", default_value="1.06"),
            DeclareLaunchArgument(
                "use_takeoff_speed_scale_curve", default_value="true"
            ),
            DeclareLaunchArgument("auto_start", default_value="true"),
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
                    },
                ],
            ),
        ]
    )
