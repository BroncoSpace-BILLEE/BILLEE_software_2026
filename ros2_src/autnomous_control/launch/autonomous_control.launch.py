from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    detection_source = LaunchConfiguration("detection_source")
    
    declare_detection_source = DeclareLaunchArgument(
        "detection_source",
        default_value="zed",
        description="Object detection source: 'zed' or 'yolov8'",
    )


    return LaunchDescription(
        [
            declare_detection_source,
            Node(
                package="autnomous_control",
                executable="autonomous_control",
                name="simple_autonomy",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": "/diff_drive_controller/cmd_vel_unstamped",
                        "object_detected_topic": "/object_detected",
                        "publish_hz": 20.0,
                        "forward_speed": 0.4,
                        "turn_speed": 0.8,
                        "avoid_forward_speed": 0.3,
                        "avoid_forward_duration": 5.0,
                        "turn_duration": 3.0,
                    }
                ],
            )
        ]
    )
