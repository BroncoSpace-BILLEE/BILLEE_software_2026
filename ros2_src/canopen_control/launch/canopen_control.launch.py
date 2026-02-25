#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    
    launch_joy_arg = DeclareLaunchArgument(
        'launch_joy',
        default_value='false',
        description='Launch joy_node (set to False if already running)'
    )
    
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can3',
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
    
    operating_mode_arg = DeclareLaunchArgument(
        'operating_mode',
        default_value='position',
        description='Motor operating mode: "position" (Profile Position, CiA402 mode 1) or "velocity" (Profile Velocity, CiA402 mode 3)'
    )
    
    max_position_arg = DeclareLaunchArgument(
        'max_position',
        default_value='100000',
        description='Position increment per joystick callback at full stick deflection (pulses). Controls how fast the target position changes.'
    )
    
    profile_velocity_arg = DeclareLaunchArgument(
        'profile_velocity',
        default_value='10000',
        description='Profile velocity in pulses/s (speed during position moves, or initial velocity in velocity mode)'
    )
    
    profile_acceleration_arg = DeclareLaunchArgument(
        'profile_acceleration',
        default_value='1000000',
        description='Profile acceleration in pulses/s²'
    )
    
    profile_deceleration_arg = DeclareLaunchArgument(
        'profile_deceleration',
        default_value='1000000',
        description='Profile deceleration in pulses/s²'
    )
    
    return LaunchDescription([
        launch_joy_arg,
        can_interface_arg,
        canopen_node_id_arg,
        max_velocity_arg,
        joy_axis_arg,
        operating_mode_arg,
        max_position_arg,
        profile_velocity_arg,
        profile_acceleration_arg,
        profile_deceleration_arg,
        
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
                'operating_mode': LaunchConfiguration('operating_mode'),
                'max_position': LaunchConfiguration('max_position'),
                'profile_velocity': LaunchConfiguration('profile_velocity'),
                'profile_acceleration': LaunchConfiguration('profile_acceleration'),
                'profile_deceleration': LaunchConfiguration('profile_deceleration'),
            }]
        ),
    ])