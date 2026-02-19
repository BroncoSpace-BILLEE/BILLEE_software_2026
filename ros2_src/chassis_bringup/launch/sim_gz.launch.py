from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

desc_share = get_package_share_directory("chassis_description")  # .../share/chassis_description
share_root = os.path.dirname(desc_share)                         # .../share
pkg_share = get_package_share_directory('chassis_bringup')

set_env = [
    SetEnvironmentVariable("IGN_GAZEBO_MODEL_PATH", share_root),
    SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", share_root),
    # Also set newer names (harmless on Fortress)
    SetEnvironmentVariable("GZ_SIM_MODEL_PATH", share_root),
    SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", share_root),
]

def generate_launch_description():
    # --- Launch arguments ---
    description_pkg = LaunchConfiguration("description_pkg")
    xacro_file = LaunchConfiguration("xacro_file")
    entity_name = LaunchConfiguration("entity_name")
    set_env_bridge = SetEnvironmentVariable("GZ_VERSION", "fortress")
    # Pass-through argument to ros_gz_sim's gz_sim.launch.py
    # Examples:
    #   "empty.sdf"
    #   "-r -v 4 empty.sdf"
    #   "-r -v 4 /absolute/path/to/world.sdf"
    gz_args = LaunchConfiguration("gz_args")

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")

    bridge_yaml = os.path.join(pkg_share, 'config', 'bridge.yaml')
    world_path = os.path.join(pkg_share, 'worlds', 'offroad_test.world.sdf')

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
            "entity_name",
            default_value="chassis",
            description="Name of the spawned entity in Gazebo.",
        ),
        DeclareLaunchArgument(
            "gz_args",
            default_value=f"-r -v 4 {world_path}",
            description="Arguments passed to Gazebo Sim (via ros_gz_sim gz_sim.launch.py).",
        ),
        DeclareLaunchArgument("x", default_value="0.0", description="Spawn X (m)."),
        DeclareLaunchArgument("y", default_value="0.0", description="Spawn Y (m)."),
        DeclareLaunchArgument("z", default_value="2", description="Spawn Z (m)."),
        DeclareLaunchArgument("yaw", default_value="0.0", description="Spawn yaw (rad)."),
    ]
    xacro_path = PathJoinSubstitution([
    FindPackageShare(description_pkg),
    xacro_file,
    ])

    # --- Build robot_description from Xacro ---

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("chassis_bringup"),
        "config",
        "controllers.yaml",
    ])

    

    # 1) Launch Gazebo Sim (Fortress) with a specific world
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 {world_path}',  # -r = run, -v 4 = verbose
        }.items()
    )

    robot_description = ParameterValue(
    Command([
        "xacro", " ",
        xacro_path, " ",
        "controllers_yaml:=", controllers_yaml
    ]),
    value_type=str,
    )

  

    # --- Publish TF and robot_description ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_yaml, 'use_sim_time': True}],
        arguments=['--ros-args', '--log-level', 'debug'],
    )


    # --- Spawn the robot from /robot_description into Gazebo ---
    spawn_entity = Node(
    package="ros_gz_sim",
    executable="create",
    output="screen",
    arguments=[
        "-world", "empty",
        "-name", entity_name,
        "-topic", "robot_description",
        "-x", x, "-y", y, "-z", z,
        "-Y", yaw,
    ],
    )

    # Delay RSP so it receives valid /clock before publishing /tf_static
    robot_state_publisher_delayed = TimerAction(
        period=1.0, actions=[robot_state_publisher]
    )

    spawn_entity_delayed = TimerAction(period=2.0, actions=[spawn_entity])


    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_diff = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", 
                   "--controller-manager", 
                   "/controller_manager"
                   ],
        output="screen",
    )   

    spawn_controllers = TimerAction(
        period=3.0,
        actions=[spawn_jsb, spawn_diff],
    )

    # Ignition Fortress depth/camera sensors ignore <ignition_frame_id>,
    # so they publish with the scoped frame "chassis/base_footprint/depth_cam_sensor".
    # The depth sensor outputs points in the SDF link frame convention
    # (X-forward, Y-left, Z-up), matching camera_link, not the optical frame.
    gz_depth_frame_bridge = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gz_depth_frame_bridge",
        output="screen",
        arguments=[
            "--frame-id", "camera_link",
            "--child-frame-id", "chassis/base_footprint/depth_cam_sensor",
        ],
        parameters=[{"use_sim_time": True}],
    )


    return LaunchDescription(
        declare_args
        + set_env
        + [set_env_bridge]
        + [
            gz_sim_launch,
            bridge,
            robot_state_publisher_delayed,
            spawn_entity_delayed,
            spawn_controllers,
            gz_depth_frame_bridge,
        ]
    )
