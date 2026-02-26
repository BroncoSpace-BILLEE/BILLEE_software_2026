#!/usr/bin/env python3
"""
wrist_teleop.launch.py

Launches the differential wrist teleop system:

  1. wrist_teleop node    – reads /joy D-pad axes, publishes /wrist/joy
  2. simple_roboclaw node – drives two motors from /wrist/joy (remapped)

No joy node is launched here; one must already be running.

Parameters exposed:
  speed       – wrist speed scaling (0.0–1.0, default 0.5)
  port        – serial port for the RoboClaw (default /dev/ttyACM0)
  address     – RoboClaw address (default 128)
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
        description='Wrist speed scaling factor (0.0–1.0)')

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for the wrist RoboClaw')

    address_arg = DeclareLaunchArgument(
        'address',
        default_value='128',
        description='RoboClaw serial address')

    # ── Nodes ───────────────────────────────────────────────────────────

    wrist_teleop_node = Node(
        package='arm_teleop_simple',
        executable='wrist_teleop',
        name='wrist_teleop',
        output='screen',
        parameters=[{
            'axis_pitch': 7,       # D-pad up/down
            'axis_roll': 6,        # D-pad left/right
            'speed': LaunchConfiguration('speed'),
            'deadzone': 0.05,
            'publish_hz': 20.0,
        }],
    )

    # simple_roboclaw subscribes to "joy" by default.
    # We remap its joy subscription to /wrist/joy so it receives
    # the synthetic Joy messages from wrist_teleop.
    simple_roboclaw_node = Node(
        package='simple_roboclaw',
        executable='simple_roboclaw',
        name='wrist_roboclaw',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'address': LaunchConfiguration('address'),
            'axis_left': 0,    # index into synthetic Joy.axes → M1
            'axis_right': 1,   # index into synthetic Joy.axes → M2
        }],
        remappings=[
            ('joy', 'wrist_joy'),
        ],
    )

    return LaunchDescription([
        speed_arg,
        port_arg,
        address_arg,
        wrist_teleop_node,
        simple_roboclaw_node,
    ])
