"""
nav2_gps.launch.py
──────────────────
Nav2 GPS waypoint navigation with rolling costmaps.

Brings up:
    1. state_estimation (dual-EKF + navsat_transform) – provides map→odom→base_footprint
     TF chain and /odometry/filtered, /odometry/global topics
  2. Nav2 navigation stack (no AMCL / map_server) configured for:
     • rolling global + local costmaps (no prior map required)
     • depth-camera obstacle detection (/depth/points)
     • GPS waypoint following via FollowGPSWaypoints action
     • Regulated Pure Pursuit controller

Usage
─────
  ros2 launch chassis_bringup nav2_gps.launch.py

Send GPS waypoints with:
  ros2 action send_goal /follow_gps_waypoints nav2_msgs/action/FollowGPSWaypoints \\
      "{ gps_poses: [ { ... }, ... ] }"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # ── Package paths ────────────────────────────────────────────
    chassis_bringup_dir = get_package_share_directory('chassis_bringup')
    state_estimation_dir = get_package_share_directory('state_estimation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # ── Launch arguments ─────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    nav2_params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')
    use_composition = LaunchConfiguration('use_composition')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock.',
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 lifecycle nodes.',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            chassis_bringup_dir, 'config', 'nav2_gps_waypoint.yaml'
        ),
        description='Full path to Nav2 parameter YAML file.',
    )
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for Nav2 nodes.',
    )
    declare_use_composition = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup (single-process).',
    )

    # ── Rewrite use_sim_time into every node's params ────────────
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True,
    )

    # ── 1. State estimation (EKF local + navsat + EKF global) ────
    state_estimation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(state_estimation_dir, 'launch', 'state_estimation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ── 2. Nav2 lifecycle-managed nodes ──────────────────────────
    # Remap /cmd_vel → diff_drive_controller input topic
    nav2_nodes = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/diff_drive_controller/cmd_vel_unstamped'),

            # --- Controller server ---
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),

            # --- Smoother server ---
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),

            # --- Planner server ---
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),

            # --- Behavior server (recoveries) ---
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),

            # --- BT Navigator ---
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),

            # --- Waypoint follower (handles FollowGPSWaypoints) ---
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),

            # --- Velocity smoother ---
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),

            # --- Lifecycle manager for all Nav2 nodes ---
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': [
                        'controller_server',
                        'smoother_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'velocity_smoother',
                    ]},
                ],
            ),
        ]
    )

    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time,
        declare_autostart,
        declare_params_file,
        declare_log_level,
        declare_use_composition,
        # Launch state estimation
        state_estimation_launch,
        # Launch Nav2 stack
        nav2_nodes,
    ])
