#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    
    launch_joy_arg = DeclareLaunchArgument(
        'launch_joy',
        default_value='true',
        description='Launch joy_node (set to False if already running)'
    )
    
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='socketCAN interface name ($ls /dev/ | grep can)'
    )
    
    canopen_node_id_arg = DeclareLaunchArgument(
        'canopen_node_id',
        default_value='1',
        description='CANopen Node ID (see: "3.2 Node ID set up" for more info)'
    )
    
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='1000',
        description='Maximum velocity in pulses/s (see drive EDS for more info)'
    )
    
    joy_axis_arg = DeclareLaunchArgument(
        'joy_axis',
        default_value='1',
        description='Joystick axis to use (0=left_x, 1=left_y, 2=right_x, 3=right_y)'
    )
    
    
    
    return LaunchDescription([
        launch_joy_arg,
        can_interface_arg,
        canopen_node_id_arg,
        max_velocity_arg,
        joy_axis_arg,
        
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
            condition=IfCondition(LaunchConfiguration('launch_joy'))
        ),
        
        Node(
            package='canopen_control',
            executable='canopen_control',
            name='canopen_control',
            output='screen',
            parameters=[{
                'can_interface': LaunchConfiguration('can_interface'),
                'canopen_node_id': LaunchConfiguration('canopen_node_id'),
                'max_velocity': LaunchConfiguration('max_velocity'),
                'joy_axis': LaunchConfiguration('joy_axis'),
                'target_velocity_index': LaunchConfiguration('target_velocity_index'),
                'target_velocity_subindex': LaunchConfiguration('target_velocity_subindex'),
            }]
        ),
    ])