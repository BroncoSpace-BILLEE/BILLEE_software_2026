from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Launch arguments (same as teleop.launch.py) ---
    max_linear_speed = LaunchConfiguration("max_linear_speed")
    max_angular_speed = LaunchConfiguration("max_angular_speed")
    joy_dev = LaunchConfiguration("joy_dev")

    declare_args = [
        DeclareLaunchArgument(
            "max_linear_speed",
            default_value="0.4",
            description="Maximum linear speed for manual control (m/s).",
        ),
        DeclareLaunchArgument(
            "max_angular_speed",
            default_value="1.2",
            description="Maximum angular speed for manual control (rad/s).",
        ),
        DeclareLaunchArgument(
            "joy_dev",
            default_value="/dev/input/js0",
            description="Joystick device for controlling both chassis and arm.",
        ),
    ]

    # --- Joy Linux Nodes ---
    joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_node",
        output="screen",
        parameters=[{"dev": joy_dev}],
    )

    # --- Manual Control (listen on /joy_chassis) ---
    manual_control_node = Node(
        package="chassis_teleop",
        executable="manual_control",
        name="manual_control",
        output="screen",
        parameters=[
            {
                "max_linear_speed": max_linear_speed,
                "max_angular_speed": max_angular_speed,
                "control_method": "triggers",  # Use triggers for chassis control
            }
        ],
    )

    # --- Diff Drive Controller ---
    diff_drive_controller_node = Node(
        package="diff_drive_controller",
        executable="diff_drive_controller",
        name="diff_drive_controller",
        output="screen",
    )

    # --- Arm Teleop all_arm.launch.py is launched on the jetson

    return LaunchDescription(
        declare_args
        + [
            joy_linux_node,
            manual_control_node,
            diff_drive_controller_node,
        ]
    )
