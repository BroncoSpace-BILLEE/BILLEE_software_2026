import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # FIX: We explicitly tell it the package_name to avoid the auto-suffix issue
    moveit_config = (
        MoveItConfigsBuilder("arm", package_name="arm_moveit_config")
        # IMPORTANT: Verify 'config/...' matches the actual filename in your folder!
        # If your URDF is named 'panda.urdf.xacro', change it here.
        .robot_description(file_path="config/arm_model.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_model.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # Get path to servo.yaml
    # We still need this to load the specific servo parameters file
    pkg_share = get_package_share_directory("arm_moveit_config")
    servo_yaml = os.path.join(pkg_share, "config", "servo.yaml")

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        # We put the node in the 'arm' namespace as you requested
        namespace="arm",
        parameters=[
            servo_yaml,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True}
        ],
        output="screen",
    )

    return LaunchDescription([servo_node])