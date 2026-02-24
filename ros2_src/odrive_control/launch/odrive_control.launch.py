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
    
    odrive_node_id_arg = DeclareLaunchArgument(
        'node_id',
        default_value='1',
        description='ODrive axis node ID'
    )
    
    can_interface_arg = DeclareLaunchArgument(
        'interface',
        default_value='can0',
        description='socketCAN interface name (e.g., can0, can1)'
    )
    
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='10.0',
        description='Maximum velocity command (in units configured on ODrive)'
    )
    
    joy_axis_vel_arg = DeclareLaunchArgument(
        'joy_axis_vel',
        default_value='1',
        description='Joystick axis to use for velocity (0=left_x, 1=left_y, 2=right_x, 3=right_y)'
    )
    
    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='2',
        description='ODrive control mode (0=VOLTAGE, 1=TORQUE, 2=VELOCITY, 3=POSITION)'
    )
    
    input_mode_arg = DeclareLaunchArgument(
        'input_mode',
        default_value='1',
        description='ODrive input mode (1=PASSTHROUGH, 2=VEL_RAMP, 3=POS_FILTER)'
    )
    
    axis_idle_on_shutdown_arg = DeclareLaunchArgument(
        'axis_idle_on_shutdown',
        default_value='true',
        description='Set ODrive to IDLE state when node shuts down'
    )
    
    return LaunchDescription([
        launch_joy_arg,
        odrive_node_id_arg,
        can_interface_arg,
        max_velocity_arg,
        joy_axis_vel_arg,
        control_mode_arg,
        input_mode_arg,
        axis_idle_on_shutdown_arg,
        
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
            package='odrive_control',
            executable='joy_to_control_message',
            name='joy_to_control_message',
            output='screen',
            parameters=[{
                'node_id': LaunchConfiguration('node_id'),
                'joy_axis_vel': LaunchConfiguration('joy_axis_vel'),
                'max_velocity': LaunchConfiguration('max_velocity'),
                'control_mode': LaunchConfiguration('control_mode'),
                'input_mode': LaunchConfiguration('input_mode'),
            }]
        ),
    ])
