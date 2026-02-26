#!/usr/bin/env python3
"""
all_arm.launch.py

Launches the complete arm teleop system:

  1. canopen_control — shoulder motor (CANopen node 1, joy axis 1, velocity mode)
  2. canopen_control — elbow motor   (CANopen node 2, joy axis 4, velocity mode)
  3. wrist_teleop   — differential wrist via RoboClaw (D-pad)
  4. gripper_base   — gripper-base via RoboClaw (left stick X + A/B buttons)

No joy node is launched here; one must already be running.

Parameters exposed:
  can_interface           – CAN bus interface          (default can3)
  shoulder_max_velocity   – shoulder max vel pulses/s  (default 500000)
  elbow_max_velocity      – elbow max vel pulses/s     (default 240000)
  wrist_speed             – wrist speed scaling 0–1    (default 0.5)
  wrist_port              – wrist RoboClaw serial port (default /dev/ttyACM0)
  wrist_address           – wrist RoboClaw address     (default 128)
  gripper_speed           – gripper speed scaling 0–1  (default 0.5)
  gripper_port            – gripper RoboClaw serial port (default /dev/ttyACM1)
  gripper_address         – gripper RoboClaw address   (default 129)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch arguments ────────────────────────────────────────────────

    # CAN bus
    can_interface_arg = DeclareLaunchArgument(
        'can_interface', default_value='can3',
        description='socketCAN interface name')

    # Shoulder (CANopen node 1)
    shoulder_max_vel_arg = DeclareLaunchArgument(
        'shoulder_max_velocity', default_value='500000',
        description='Shoulder motor max velocity (pulses/s)')

    # Elbow (CANopen node 2)
    elbow_max_vel_arg = DeclareLaunchArgument(
        'elbow_max_velocity', default_value='240000',
        description='Elbow motor max velocity (pulses/s)')

    # Wrist RoboClaw
    wrist_speed_arg = DeclareLaunchArgument(
        'wrist_speed', default_value='0.5',
        description='Wrist speed scaling (0.0–1.0)')
    wrist_port_arg = DeclareLaunchArgument(
        'wrist_port', default_value='/dev/ttyACM0',
        description='Serial port for wrist RoboClaw')
    wrist_address_arg = DeclareLaunchArgument(
        'wrist_address', default_value='128',
        description='Wrist RoboClaw serial address')

    # Gripper-base RoboClaw
    gripper_speed_arg = DeclareLaunchArgument(
        'gripper_speed', default_value='0.5',
        description='Gripper-base speed scaling (0.0–1.0)')
    gripper_port_arg = DeclareLaunchArgument(
        'gripper_port', default_value='/dev/ttyACM1',
        description='Serial port for gripper-base RoboClaw')
    gripper_address_arg = DeclareLaunchArgument(
        'gripper_address', default_value='129',
        description='Gripper-base RoboClaw serial address')

    # ── Locate launch files ─────────────────────────────────────────────

    canopen_launch = PathJoinSubstitution([
        FindPackageShare('canopen_control'), 'launch', 'canopen_control.launch.py'
    ])
    wrist_launch = PathJoinSubstitution([
        FindPackageShare('arm_teleop_simple'), 'launch', 'wrist_teleop.launch.py'
    ])
    gripper_launch = PathJoinSubstitution([
        FindPackageShare('arm_teleop_simple'), 'launch', 'gripper_base.launch.py'
    ])

    # ── Shoulder — CANopen node 1, left stick Y (axis 1) ───────────────

    shoulder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(canopen_launch),
        launch_arguments={
            'can_interface': LaunchConfiguration('can_interface'),
            'canopen_node_id': '1',
            'operating_mode': 'velocity',
            'max_velocity': LaunchConfiguration('shoulder_max_velocity'),
            'joy_axis': '1',
            'launch_joy': 'false',
        }.items(),
    )

    # ── Elbow — CANopen node 2, right stick Y (axis 4) ─────────────────

    elbow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(canopen_launch),
        launch_arguments={
            'can_interface': LaunchConfiguration('can_interface'),
            'canopen_node_id': '2',
            'operating_mode': 'velocity',
            'max_velocity': LaunchConfiguration('elbow_max_velocity'),
            'joy_axis': '4',
            'launch_joy': 'false',
        }.items(),
    )

    # ── Wrist — differential RoboClaw (D-pad) ──────────────────────────

    wrist = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(wrist_launch),
        launch_arguments={
            'speed': LaunchConfiguration('wrist_speed'),
            'port': LaunchConfiguration('wrist_port'),
            'address': LaunchConfiguration('wrist_address'),
        }.items(),
    )

    # ── Gripper-base — RoboClaw (left stick X + A/B buttons) ───────────

    gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gripper_launch),
        launch_arguments={
            'speed': LaunchConfiguration('gripper_speed'),
            'port': LaunchConfiguration('gripper_port'),
            'address': LaunchConfiguration('gripper_address'),
        }.items(),
    )

    # ── Assemble ────────────────────────────────────────────────────────

    return LaunchDescription([
        # Arguments
        can_interface_arg,
        shoulder_max_vel_arg,
        elbow_max_vel_arg,
        wrist_speed_arg,
        wrist_port_arg,
        wrist_address_arg,
        gripper_speed_arg,
        gripper_port_arg,
        gripper_address_arg,
        # Launches
        shoulder,
        elbow,
        wrist,
        gripper,
    ])
