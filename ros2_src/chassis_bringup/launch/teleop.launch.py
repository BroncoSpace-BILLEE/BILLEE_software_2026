from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Launch arguments ---
    max_linear_speed = LaunchConfiguration("max_linear_speed")
    max_angular_speed = LaunchConfiguration("max_angular_speed")

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
    ]


    # --- Joy Linux Node ---
    joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux_node",
        output="screen",
    )

    # --- Manual Control ---
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
    )

    # --- Diff Drive Controller ---
    diff_drive_controller_node = Node(
        package="diff_drive_controller",
        executable="diff_drive_controller",
        name="diff_drive_controller",
        output="screen",
    )

    return LaunchDescription(
        declare_args
        + [
            joy_linux_node,
            manual_control_node,
            diff_drive_controller_node,
        ]
    )
