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
    arm_joy_dev = LaunchConfiguration("arm_joy_dev")
    chassis_joy_dev = LaunchConfiguration("chassis_joy_dev")

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
            "arm_joy_dev",
            default_value="/dev/input/js0",
            description="Joystick device for arm controller.",
        ),
        DeclareLaunchArgument(
            "chassis_joy_dev",
            default_value="/dev/input/js1",
            description="Joystick device for chassis controller.",
        ),
    ]

    # --- Joy Linux Nodes ---
    # Arm controller joystick on default topic: /joy
    joy_linux_arm_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux_arm_node",
        output="screen",
        parameters=[{"dev": arm_joy_dev}],
    )

    # Chassis controller joystick on: /joy_chassis
    joy_linux_chassis_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux_chassis_node",
        output="screen",
        parameters=[{"dev": chassis_joy_dev}],
        remappings=[("joy", "/joy_chassis")],
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
            }
        ],
        remappings=[("joy", "/joy_chassis")],
    )

    # --- Diff Drive Controller ---
    diff_drive_controller_node = Node(
        package="diff_drive_controller",
        executable="diff_drive_controller",
        name="diff_drive_controller",
        output="screen",
        remappings=[("joy", "/joy_chassis")],
    )

    return LaunchDescription(
        declare_args
        + [
            joy_linux_arm_node,
            joy_linux_chassis_node,
            manual_control_node,
            diff_drive_controller_node,
        ]
    )
