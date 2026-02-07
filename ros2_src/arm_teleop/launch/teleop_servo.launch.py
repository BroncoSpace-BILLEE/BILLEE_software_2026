#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- Launch args ----
    joy_dev = LaunchConfiguration("joy_dev")
    deadzone = LaunchConfiguration("deadzone")
    autorepeat_rate = LaunchConfiguration("autorepeat_rate")
    start_servo = LaunchConfiguration("start_servo")
    servo_start_delay_sec = LaunchConfiguration("servo_start_delay_sec")

    # ---- Include MoveIt Servo launch ----
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("arm_moveit_config"),
                    "launch",
                    "servo.launch.py",
                ]
            )
        ),
        # If your servo.launch.py has args, add them here.
        # launch_arguments={"planning_group_name": "arm"}.items(),
    )

    # ---- Joystick driver (joy_linux) ----
    joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux_node",
        output="screen",
        parameters=[
            {
                "dev": joy_dev,
                "deadzone": deadzone,
                "autorepeat_rate": autorepeat_rate,
            }
        ],
    )

    # ---- Teleop mapper: joy -> TwistStamped for Servo ----
    # Assumes your script is installed as a console_script executable named "joy_to_servo_twist"
    joy_to_servo = Node(
        package="arm_teleop",
        executable="joy_to_servo_twist",
        name="joy_to_servo_twist",
        output="screen",
        # If your node has params, add them here.
        # parameters=[{"frame_id": "base_link"}],
        # Remaps if needed:
        # remappings=[("/joy", "/joy"), ("/servo_node/delta_twist_cmds", "/servo_node/delta_twist_cmds")],
    )

        # ---- Optional: arm Servo via Trigger service ----
    # Delay a bit so servo_node is up and its service exists.
    from launch.actions import ExecuteProcess
    from launch.conditions import IfCondition

    start_servo_timer = TimerAction(
        period=servo_start_delay_sec,
        actions=[
            ExecuteProcess(
                condition=IfCondition(start_servo),
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/servo_node/start_servo",
                    "std_srvs/srv/Trigger",
                    "{}",
                ],
                output="screen",
            )
        ],
    )


    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joy_dev",
                default_value="/dev/input/js0",
                description="Joystick device path for joy_linux.",
            ),
            DeclareLaunchArgument(
                "deadzone",
                default_value="0.05",
                description="Deadzone for joystick axes.",
            ),
            DeclareLaunchArgument(
                "autorepeat_rate",
                default_value="20.0",
                description="Autorepeat rate (Hz) for joystick messages.",
            ),
            DeclareLaunchArgument(
                "start_servo",
                default_value="true",
                description="If true, call /servo_node/start_servo after launch.",
            ),
            DeclareLaunchArgument(
                "servo_start_delay_sec",
                default_value="2.0",
                description="Delay (seconds) before calling /servo_node/start_servo.",
            ),
            servo_launch,
            joy_node,
            joy_to_servo,
            start_servo_timer,
        ]
    )
