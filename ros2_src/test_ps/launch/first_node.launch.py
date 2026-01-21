from launch import LaunchDescription
from launch_ros.actions import Node

# Launch description function
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ch1',
            executable='first_node', #name of executable in setup.py
            name='first_node', #name of node to start
            output='screen', #show output in terminal
        ),
    ])