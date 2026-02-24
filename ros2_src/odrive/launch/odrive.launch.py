#!/usr/bin/env python3
"""
Launch file for multiple ODrive CAN nodes.

This launch file starts one ODrive SocketCAN communication node per motor with
shared CAN interface and cmd_vel scaling but unique name, node ID, and
joint_state_topic.

Usage:
    ros2 launch odrive odrive.launch.py
    ros2 launch odrive odrive.launch.py can_interface:=can1
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

    cmd_vel_scale_arg = DeclareLaunchArgument(
        "cmd_vel_scale",
        default_value="1.0",
        description="Scale factor applied to commanded velocity"
    )

    odrive_nodes = [
        {"name": "odrive_l1", "node_id": 3, "joint_state_topic": "/joint_wheel_l1"},
        {"name": "odrive_l2", "node_id": 7, "joint_state_topic": "/joint_wheel_l2"},
        {"name": "odrive_l3", "node_id": 2, "joint_state_topic": "/joint_wheel_l3"},
        {"name": "odrive_r1", "node_id": 6, "joint_state_topic": "/joint_wheel_r1"},
        {"name": "odrive_r2", "node_id": 4, "joint_state_topic": "/joint_wheel_r2"},
        {"name": "odrive_r3", "node_id": 5, "joint_state_topic": "/joint_wheel_r3"},
    ]

    nodes = [
        Node(
            package="odrive",
            executable="odrive",
            name=odrive_config["name"],
            output="screen",
            parameters=[
                {"can_interface": LaunchConfiguration("can_interface")},
                {"node_id": odrive_config["node_id"]},
                {"joint_state_topic": odrive_config["joint_state_topic"]},
                {"cmd_vel_scale": LaunchConfiguration("cmd_vel_scale")},
            ],
        )
        for odrive_config in odrive_nodes
    ]

    return LaunchDescription([
        can_interface_arg,
        cmd_vel_scale_arg,
        *nodes,
    ])
