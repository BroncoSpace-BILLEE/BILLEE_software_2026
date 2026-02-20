#!/usr/bin/env python3
"""
Launch file for ODrive CAN node.

This launch file starts the ODrive SocketCAN communication node with configurable
parameters for CAN interface, node ID, and cmd_vel input.

Usage:
  ros2 launch odrive odrive.launch.py
  ros2 launch odrive odrive.launch.py can_interface:=can1 node_id:=1
    ros2 launch odrive odrive.launch.py cmd_vel_scale:=2.0
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

    cmd_vel_topic_arg = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel",
        description="Topic name for geometry_msgs/Twist velocity commands"
    )

    cmd_vel_scale_arg = DeclareLaunchArgument(
        "cmd_vel_scale",
        default_value="1.0",
        description="Scale factor applied to commanded velocity"
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
            {"cmd_vel_topic": LaunchConfiguration("cmd_vel_topic")},
            {"cmd_vel_scale": LaunchConfiguration("cmd_vel_scale")},
        ],
    )

    return LaunchDescription([
        can_interface_arg,
        node_id_arg,
        cmd_vel_topic_arg,
        cmd_vel_scale_arg,
        odrive_node,
    ])
