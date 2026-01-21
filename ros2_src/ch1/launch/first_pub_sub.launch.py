from launch import LaunchDescription
from launch_ros.actions import Node

#naming the different nodes and remapping the topics to match the publisher and subscriber

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ch1',
            executable='first_publisher',
            name='shrek',
            remappings=[('/points', '/shrek_points')],
            output='screen',
        ),
        Node(
            package='ch1',
            executable='first_subscriber',
            name='donkey',
            remappings=[('/some_points', '/shrek_points')],
            output='screen',
        ),
    ])