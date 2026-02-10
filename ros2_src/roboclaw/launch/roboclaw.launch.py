
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
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for RoboClaw'
    )
    
    
    return LaunchDescription([
        launch_joy_arg,
        port_arg,
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_node',
            parameters=[{
                'device_id': 0, #TODO: fill out
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
            condition=IfCondition(LaunchConfiguration('launch_joy'))
        ),
        
        Node(
            package='roboclaw',
            executable='roboclaw',
            name='roboclaw',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'), #TODO: configure in tuning software
                'address': 128,
                'axis_left': 1,
                'axis_right': 3,
            }]
        ),
    ])