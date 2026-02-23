from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Launch arguments ---
    description_pkg = LaunchConfiguration("description_pkg")
    xacro_file = LaunchConfiguration("xacro_file")
    can_interface = LaunchConfiguration("can_interface")

    declare_args = [
        DeclareLaunchArgument(
            "description_pkg",
            default_value="chassis_description",
            description="Package that contains the robot URDF/Xacro in its share/ directory.",
        ),
        DeclareLaunchArgument(
            "xacro_file",
            default_value="urdf/chassis_model.xacro",
            description="Path to the robot Xacro (relative to the description package share).",
        ),
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="CAN bus interface name for ODrive controllers.",
        ),
    ]

    xacro_path = PathJoinSubstitution([
        FindPackageShare(description_pkg),
        xacro_file,
    ])

    # --- Build robot_description from Xacro (real hardware) ---
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("chassis_bringup"),
        "config",
        "controllers.yaml",
    ])

    robot_description = ParameterValue(
        Command([
            "xacro", " ",
            xacro_path, " ",
            "controllers_yaml:=", controllers_yaml, " ",
            "use_sim:=false", " ",
            "can_interface:=", can_interface,
        ]),
        value_type=str,
    )

    # --- Publish TF and robot_description ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # --- ros2_control_node (replaces the Gazebo plugin on real hardware) ---
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml,
        ],
        output="screen",
        # Allow running without a PREEMPT_RT kernel
        arguments=["--ros-args", "-p", "thread_priority:=0"],
    )

    # --- Spawn controllers (after controller_manager is up) ---
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_diff = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    spawn_controllers = TimerAction(
        period=3.0,
        actions=[spawn_jsb, spawn_diff],
    )

    return LaunchDescription(
        declare_args
        + [
            robot_state_publisher,
            ros2_control_node,
            spawn_controllers,
        ]
    )
