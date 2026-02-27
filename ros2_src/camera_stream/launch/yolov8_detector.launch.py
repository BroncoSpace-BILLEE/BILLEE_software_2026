from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('camera_stream')
    default_model_path = PathJoinSubstitution([package_share, 'models', ''])
    
    return LaunchDescription(
        [
            Node(
                package="camera_stream",
                executable="yolov8_detector",
                name="yolov8_object_detector",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/camera/video/compressed",
                        "detection_topic": "/object_detected",
                        "publish_hz": 10.0,
                        "confidence_threshold": 0.5,
                        "model_path": "src/models/yolov8s.pt",
                        "model_size": "s",
                        "enable_gpu": True,
                        "target_classes": [
                            'person',
                            'laptop'
                        ],
                    }
                ],
            )
        ]
    )
