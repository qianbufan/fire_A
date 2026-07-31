import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def float_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def int_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def bool_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def generate_launch_description():
    robot_share = get_package_share_directory("robot")
    demo_share = get_package_share_directory("fire_swing_demo")
    urdf_file = os.path.join(robot_share, "urdf", "robot.urdf")
    rviz_file = os.path.join(demo_share, "rviz", "horizontal_disk.rviz")

    arguments = [
        DeclareLaunchArgument("disk_radius_m", default_value="0.50"),
        DeclareLaunchArgument("surface_frequency_hz", default_value="0.10"),
        DeclareLaunchArgument("wrist2_frequency_hz", default_value="4.0"),
        DeclareLaunchArgument("wrist2_amplitude_deg", default_value="14.0"),
        DeclareLaunchArgument("wrist2_phase_deg", default_value="0.0"),
        DeclareLaunchArgument("backward_sign", default_value="-1.0"),
        DeclareLaunchArgument("lateral_sign", default_value="1.0"),
        DeclareLaunchArgument("center_offset_x_m", default_value="0.3"),
        DeclareLaunchArgument("center_offset_y_m", default_value="0.5"),
        DeclareLaunchArgument("center_offset_z_m", default_value="-0.6"),
        DeclareLaunchArgument("base_initial_deg", default_value="0.0"),
        DeclareLaunchArgument("shoulder_initial_deg", default_value="0.0"),
        DeclareLaunchArgument("elbow_initial_deg", default_value="0.0"),
        DeclareLaunchArgument("wrist1_initial_deg", default_value="0.0"),
        DeclareLaunchArgument("wrist2_center_deg", default_value="0.0"),
        DeclareLaunchArgument("wrist3_initial_deg", default_value="0.0"),
        DeclareLaunchArgument("publish_rate_hz", default_value="125.0"),
        DeclareLaunchArgument("ik_iterations", default_value="6"),
        DeclareLaunchArgument("ik_damping", default_value="0.02"),
        DeclareLaunchArgument("ik_gain", default_value="0.85"),
        DeclareLaunchArgument("max_ik_step_deg", default_value="8.0"),
        DeclareLaunchArgument("trail_points", default_value="6000"),
        DeclareLaunchArgument("show_trail", default_value="true"),
        DeclareLaunchArgument("show_target_disk", default_value="true"),
    ]

    robot_description = ParameterValue(
        Command(["xacro ", urdf_file]), value_type=str
    )
    float_names = [
        "disk_radius_m",
        "surface_frequency_hz",
        "wrist2_frequency_hz",
        "wrist2_amplitude_deg",
        "wrist2_phase_deg",
        "backward_sign",
        "lateral_sign",
        "center_offset_x_m",
        "center_offset_y_m",
        "center_offset_z_m",
        "base_initial_deg",
        "shoulder_initial_deg",
        "elbow_initial_deg",
        "wrist1_initial_deg",
        "wrist2_center_deg",
        "wrist3_initial_deg",
        "publish_rate_hz",
        "ik_damping",
        "ik_gain",
        "max_ik_step_deg",
    ]
    parameters = {name: float_parameter(name) for name in float_names}
    parameters.update(
        {
            "ik_iterations": int_parameter("ik_iterations"),
            "trail_points": int_parameter("trail_points"),
            "show_trail": bool_parameter("show_trail"),
            "show_target_disk": bool_parameter("show_target_disk"),
        }
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="fire_swing_demo",
            executable="horizontal_disk_sweep.py",
            name="horizontal_disk_sweep_publisher",
            output="screen",
            parameters=[parameters],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_file],
        ),
    ]
    return LaunchDescription(arguments + nodes)
