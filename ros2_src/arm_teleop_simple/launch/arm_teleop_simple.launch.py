#!/usr/bin/env python3
"""
arm_teleop_simple.launch.py

Launches:
  1. canopen_control node 1 – motor 1 (shoulder), velocity mode, listening on /motor1/joy
  2. canopen_control node 2 – motor 2 (elbow),    velocity mode, listening on /motor2/joy
  3. planar_ik_teleop       – IK solver: /joy → /motor1/joy, /motor2/joy

Both motors share the same CAN bus.  Each canopen_control instance is
remapped so it subscribes to its own synthetic Joy topic instead of /joy.
The IK node reads the real /joy and publishes computed velocities.

NOTE: joy_linux_node must be launched separately (e.g. via chassis_bringup).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ────────────────────────────────────────────────

    # CAN bus
    can_interface_arg = DeclareLaunchArgument(
        'can_interface', default_value='can3',
        description='socketCAN interface name')

    # CANopen node IDs
    motor1_node_id_arg = DeclareLaunchArgument(
        'motor1_canopen_id', default_value='1',
        description='CANopen Node ID for motor 1 (shoulder)')
    motor2_node_id_arg = DeclareLaunchArgument(
        'motor2_canopen_id', default_value='2',
        description='CANopen Node ID for motor 2 (elbow)')

    # Motor velocity limits (pulses/s)
    motor1_max_vel_arg = DeclareLaunchArgument(
        'motor1_max_velocity', default_value='1000',
        description='Max velocity for motor 1 (pulses/s)')
    motor2_max_vel_arg = DeclareLaunchArgument(
        'motor2_max_velocity', default_value='1000',
        description='Max velocity for motor 2 (pulses/s)')

    # Motor profile parameters
    profile_acceleration_arg = DeclareLaunchArgument(
        'profile_acceleration', default_value='1000000',
        description='Profile acceleration (pulses/s²)')
    profile_deceleration_arg = DeclareLaunchArgument(
        'profile_deceleration', default_value='1000000',
        description='Profile deceleration (pulses/s²)')

    # Arm geometry
    link1_length_arg = DeclareLaunchArgument(
        'link1_length', default_value='1.0',
        description='Length of link 1 in metres (base to elbow)')
    link2_length_arg = DeclareLaunchArgument(
        'link2_length', default_value='1.0',
        description='Length of link 2 in metres (elbow to end-effector)')

    # Initial joint angles (radians) – defaults avoid the q2=0 singularity
    q1_init_arg = DeclareLaunchArgument(
        'q1_init', default_value='0.7854',
        description='Initial angle of joint 1 in radians (~45° shoulder)')
    q2_init_arg = DeclareLaunchArgument(
        'q2_init', default_value='-1.5708',
        description='Initial angle of joint 2 in radians (~-90° elbow)')

    # Cartesian speed
    max_cartesian_vel_arg = DeclareLaunchArgument(
        'max_cartesian_vel', default_value='0.5',
        description='Max end-effector Cartesian velocity (m/s) at full stick deflection')

    # IK control rate
    publish_hz_arg = DeclareLaunchArgument(
        'publish_hz', default_value='20.0',
        description='IK control loop rate (Hz)')

    # Joystick axis mapping
    joy_axis_x_arg = DeclareLaunchArgument(
        'joy_axis_x', default_value='1',
        description='Joystick axis for forward/backward (default: left stick Y)')
    joy_axis_y_arg = DeclareLaunchArgument(
        'joy_axis_y', default_value='4',
        description='Joystick axis for up/down (default: right stick Y)')

    # ── Nodes ───────────────────────────────────────────────────────────

    # 1) canopen_control – Motor 1 (shoulder)
    #    Remapped: subscribes to /motor1/joy instead of /joy
    #    joy_axis=0 because the IK node puts the command in axes[0]
    motor1_node = Node(
        package='canopen_control',
        executable='canopen_control',
        name='canopen_motor1',
        output='screen',
        remappings=[('joy', '/motor1/joy')],
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'canopen_node_id': LaunchConfiguration('motor1_canopen_id'),
            'max_velocity': LaunchConfiguration('motor1_max_velocity'),
            'joy_axis': 0,  # IK node puts the value in axes[0]
            'operating_mode': 'velocity',
            'profile_acceleration': LaunchConfiguration('profile_acceleration'),
            'profile_deceleration': LaunchConfiguration('profile_deceleration'),
        }],
    )

    # 2) canopen_control – Motor 2 (elbow)
    motor2_node = Node(
        package='canopen_control',
        executable='canopen_control',
        name='canopen_motor2',
        output='screen',
        remappings=[('joy', '/motor2/joy')],
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'canopen_node_id': LaunchConfiguration('motor2_canopen_id'),
            'max_velocity': LaunchConfiguration('motor2_max_velocity'),
            'joy_axis': 0,
            'operating_mode': 'velocity',
            'profile_acceleration': LaunchConfiguration('profile_acceleration'),
            'profile_deceleration': LaunchConfiguration('profile_deceleration'),
        }],
    )

    # 3) Planar IK teleop node
    ik_node = Node(
        package='arm_teleop_simple',
        executable='planar_ik_teleop',
        name='planar_ik_teleop',
        output='screen',
        parameters=[{
            'link1_length': LaunchConfiguration('link1_length'),
            'link2_length': LaunchConfiguration('link2_length'),
            'q1_init': LaunchConfiguration('q1_init'),
            'q2_init': LaunchConfiguration('q2_init'),
            'max_cartesian_vel': LaunchConfiguration('max_cartesian_vel'),
            'publish_hz': LaunchConfiguration('publish_hz'),
            'joy_topic': '/joy',
            'motor1_joy_topic': '/motor1/joy',
            'motor2_joy_topic': '/motor2/joy',
            'joy_axis_x': LaunchConfiguration('joy_axis_x'),
            'joy_axis_y': LaunchConfiguration('joy_axis_y'),
        }],
    )

    return LaunchDescription([
        # Arguments
        can_interface_arg,
        motor1_node_id_arg,
        motor2_node_id_arg,
        motor1_max_vel_arg,
        motor2_max_vel_arg,
        profile_acceleration_arg,
        profile_deceleration_arg,
        link1_length_arg,
        link2_length_arg,
        q1_init_arg,
        q2_init_arg,
        max_cartesian_vel_arg,
        publish_hz_arg,
        joy_axis_x_arg,
        joy_axis_y_arg,
        # Nodes
        motor1_node,
        motor2_node,
        ik_node,
    ])
