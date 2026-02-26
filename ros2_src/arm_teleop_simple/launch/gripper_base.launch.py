#!/usr/bin/env python3
"""
gripper_base.launch.py

Launches the gripper-base teleop system:

  1. gripper_base_teleop node – reads /joy, publishes /gripper_base_joy
  2. simple_roboclaw node     – drives two motors from /gripper_base_joy

Channel 1 (M1): simple joint controlled by left stick X (axis 0)
Channel 2 (M2): button-driven — A (btn 0) backward, B (btn 1) forward

No joy node is launched here; one must already be running.

Parameters exposed:
  speed   – motor speed scaling (0.0–1.0, default 0.5)
  port    – serial port for the RoboClaw (default /dev/ttyACM0)
  address – RoboClaw address (default 128)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ────────────────────────────────────────────────
    speed_arg = DeclareLaunchArgument(
        'speed',
        default_value='0.5',
        description='Motor speed scaling factor (0.0–1.0)')

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for the gripper-base RoboClaw')

    address_arg = DeclareLaunchArgument(
        'address',
        default_value='128',
        description='RoboClaw serial address')

    # ── Nodes ───────────────────────────────────────────────────────────

    gripper_base_teleop_node = Node(
        package='arm_teleop_simple',
        executable='gripper_base_teleop',
        name='gripper_base_teleop',
        output='screen',
        parameters=[{
            'axis_joint': 0,          # Left stick X
            'button_backward': 0,     # A button
            'button_forward': 1,      # B button
            'speed': LaunchConfiguration('speed'),
            'deadzone': 0.05,
            'publish_hz': 20.0,
        }],
    )

    # simple_roboclaw with joy topic remapped to gripper_base_joy
    simple_roboclaw_node = Node(
        package='simple_roboclaw',
        executable='simple_roboclaw',
        name='gripper_base_roboclaw',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'address': LaunchConfiguration('address'),
            'axis_left': 0,    # index into synthetic Joy.axes → M1 (joint)
            'axis_right': 1,   # index into synthetic Joy.axes → M2 (gripper)
        }],
        remappings=[
            ('joy', 'gripper_base_joy'),
        ],
    )

    return LaunchDescription([
        speed_arg,
        port_arg,
        address_arg,
        gripper_base_teleop_node,
        simple_roboclaw_node,
    ])
