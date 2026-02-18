"""
state_estimation.launch.py
──────────────────────────
Dual-EKF + navsat_transform pipeline for nav2-ready state estimation.

Nodes launched
──────────────
1. ekf_local  – fuses wheel odom + IMU → /odometry/filtered, odom→base_link TF
2. navsat_transform – converts GPS NavSatFix → /odometry/gps
3. ekf_global – fuses wheel odom + IMU + GPS odom → /odometry/global, map→odom TF
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('state_estimation')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock.',
    )

    # ── Paths to parameter files ──────────────────────────────────
    ekf_local_yaml = os.path.join(pkg_dir, 'config', 'ekf_local.yaml')
    ekf_global_yaml = os.path.join(pkg_dir, 'config', 'ekf_global.yaml')
    navsat_yaml = os.path.join(pkg_dir, 'config', 'navsat.yaml')

    # ── 1. EKF Local (odom frame) ────────────────────────────────
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[ekf_local_yaml, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ],
    )

    # ── 2. NavSat Transform ──────────────────────────────────────
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[navsat_yaml, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu',              '/imu/data'),
            ('gps/fix',          '/fix'),
            ('odometry/filtered', '/odometry/filtered'),
            ('odometry/gps',     '/odometry/gps'),
            ('gps/filtered',     '/gps/filtered'),
        ],
    )

    # ── 3. EKF Global (map frame) ────────────────────────────────
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[ekf_global_yaml, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', '/odometry/global'),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        ekf_local,
        navsat_transform,
        ekf_global,
    ])
