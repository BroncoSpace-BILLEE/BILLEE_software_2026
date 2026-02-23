#!/usr/bin/env python3
"""
Launch file for ODrive CAN node with multiple nodes.

This launch file starts multiple ODrive SocketCAN communication nodes with configurable
parameters for CAN interface, node IDs, and cmd_vel input. 
It launches 6 nodes with different node_ids and joint_state_topics.

Usage:
  ros2 launch odrive odrive.launch.py
  ros2 launch odrive odrive.launch.py can_interface:=can1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import GroupAction


def generate_launch_description():
    """Generate launch description for multiple ODrive CAN nodes."""

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

    # Define node IDs and joint topics for 6 nodes with dynamically generated topic names
    node_configs = [
        {"node_id": 7, "joint_state_topic": "/joint_wheel_l1"},
        {"node_id": 6, "joint_state_topic": "/joint_wheel_r1"},
        {"node_id": 3, "joint_state_topic": "/joint_wheel_l2"},
        {"node_id": 4, "joint_state_topic": "/joint_wheel_r2"},
        {"node_id": 2, "joint_state_topic": "/joint_wheel_l3"},
        {"node_id": 5, "joint_state_topic": "/joint_wheel_r3"},
    ]

    # Create a list of nodes for each configuration
    odrive_nodes = [
        Node(
            package="odrive",
            executable="odrive",
            name=f"odrive_can_node_{config['node_id']}",
            output="screen",
            parameters=[
                {"can_interface": LaunchConfiguration("can_interface")},
                {"node_id": config["node_id"]},
                {"joint_state_topic": config["joint_state_topic"]},
                {"cmd_vel_scale": LaunchConfiguration("cmd_vel_scale")},
            ],
        )
        for config in node_configs
    ]

    # Add LogInfo to print out which node is being launched
    node_logs = [
        LogInfo(
            condition=None,  # This will log regardless of any conditions
            msg=f"Launching ODrive node {config['node_id']} with topic {config['joint_state_topic']}"
        )
        for config in node_configs
    ]

    # Grouping all node launch actions to ensure they run concurrently
    launch_group = GroupAction(
        actions=node_logs + odrive_nodes
    )

    return LaunchDescription([
        can_interface_arg,
        cmd_vel_scale_arg,
        launch_group,  # Ensure all nodes are launched together
    ])

