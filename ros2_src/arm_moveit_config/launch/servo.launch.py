#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import launch_ros

from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def _make_servo_container(context, *args, **kwargs):
    moveit_config_package = LaunchConfiguration("moveit_config_package").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    servo_yaml = LaunchConfiguration("servo_yaml").perform(context)
    planning_group_name = LaunchConfiguration("planning_group_name").perform(context)
    accel_filter_update_period = float(
        LaunchConfiguration("accel_filter_update_period").perform(context)
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name=robot_name, package_name=moveit_config_package)
        .to_moveit_configs()
    )

    servo_params = {
        "moveit_servo": ParameterBuilder(moveit_config_package).yaml(servo_yaml).to_dict()
    }

    common_parameters = [
        servo_params,
        {"planning_group_name": planning_group_name},
        {"update_period": accel_filter_update_period},
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ]

    container = launch_ros.actions.ComposableNodeContainer(
        name="moveit_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=common_parameters,
            )
        ],
    )

    return [container]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "moveit_config_package",
                default_value="arm_moveit_config",
                description="MoveIt config package name.",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="arm",
                description="Robot name used by MoveItConfigsBuilder.",
            ),
            DeclareLaunchArgument(
                "servo_yaml",
                default_value="config/servo.yaml",
                description="Servo YAML path, relative to the moveit_config_package share directory.",
            ),
            DeclareLaunchArgument(
                "planning_group_name",
                default_value="arm_no_wrist",
                description="Planning group to servo.",
            ),
            DeclareLaunchArgument(
                "accel_filter_update_period",
                default_value="0.01",
                description="Servo acceleration filter update period (seconds).",
            ),
            OpaqueFunction(function=_make_servo_container),
        ]
    )
