#!/usr/bin/env python3
"""
Launch file for ODrive CAN node.

This launch file starts the ODrive SocketCAN communication node with configurable
parameters for CAN interface, node ID, joystick input, and topic remappings.

Usage:
  ros2 launch odrive odrive.launch.py
  ros2 launch odrive odrive.launch.py can_interface:=can1 node_id:=1
  ros2 launch odrive odrive.launch.py joy_axis:=0 joy_scale:=2.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for ODrive CAN node."""

    # Declare launch arguments
    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="can2",
        description="CAN network interface name (e.g., can0, can1)"
    )

    node_id_arg = DeclareLaunchArgument(
        "node_id",
        default_value="0",
        description="ODrive node ID on the CAN bus (0-63)"
    )

    joy_axis_arg = DeclareLaunchArgument(
        "joy_axis",
        default_value="1",
        description="Joystick axis index to use for velocity control (0-based)"
    )

    joy_scale_arg = DeclareLaunchArgument(
        "joy_scale",
        default_value="1.0",
        description="Scale factor applied to joystick axis value (velocity = axis * scale)"
    )

    # Create the ODrive CAN node
    odrive_node = Node(
        package="odrive",
        executable="odrive",
        name="odrive_can_node",
        output="screen",
        parameters=[
            {"can_interface": LaunchConfiguration("can_interface")},
            {"node_id": LaunchConfiguration("node_id")},
            {"joy_axis": LaunchConfiguration("joy_axis")},
            {"joy_scale": LaunchConfiguration("joy_scale")},
        ],
    )
    joy_node = Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_node',
            parameters=[{
                'device_id': 0, 
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),



    return LaunchDescription([
        can_interface_arg,
        node_id_arg,
        joy_axis_arg,
        joy_scale_arg,
        odrive_node,
        joy_node,
    ])
