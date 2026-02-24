from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # ---- Shared video parameters (override via launch arguments) ----
    num_cameras = LaunchConfiguration("num_cameras")
    encoding = LaunchConfiguration("encoding")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fps = LaunchConfiguration("fps")
    bitrate = LaunchConfiguration("bitrate")
    gop = LaunchConfiguration("gop")
    use_pts = LaunchConfiguration("use_pts")
    flip_method = LaunchConfiguration("flip_method")

    # Change these to match your package/executable names
    pkg = "camera_stream"
    exe = "csi_compressed_video_pub"

    # We'll use a Python expression to expand the number of cameras
    # Since LaunchConfiguration is evaluated at runtime, we need a different approach
    # We'll create nodes for a reasonable max and rely on the parameter
    import os
    num_cams = int(os.environ.get('NUM_CAMERAS', '3'))
    
    nodes = []
    for i in range(num_cams):
        nodes.append(
            Node(
                package=pkg,
                executable=exe,
                name=f"csi_video_pub_{i}",
                output="screen",
                parameters=[
                    {
                        "sensor_id": i,
                        "topic": f"/cam{i}/video",
                        "frame_id": f"cam{i}_optical",
                        "encoding": encoding,
                        "width": width,
                        "height": height,
                        "fps": fps,
                        "bitrate": bitrate,
                        "gop": gop,
                        "use_pts": use_pts,
                        "flip_method": flip_method,
                    }
                ],
            )
        )

    return LaunchDescription(
        [
            # ---- Launch args ----
            DeclareLaunchArgument(
                "num_cameras",
                default_value=TextSubstitution(text="3"),
                description="Number of cameras to launch (0-6)",
            ),
            DeclareLaunchArgument(
                "encoding",
                default_value=TextSubstitution(text="h264"),
                description="Video encoding: h264 or h265 (h264 recommended for compatibility)",
            ),
            DeclareLaunchArgument("width", default_value=TextSubstitution(text="1280")),
            DeclareLaunchArgument("height", default_value=TextSubstitution(text="720")),
            DeclareLaunchArgument("fps", default_value=TextSubstitution(text="30")),
            DeclareLaunchArgument(
                "bitrate",
                default_value=TextSubstitution(text="3000000"),
                description="Per-camera bitrate in bits/sec",
            ),
            DeclareLaunchArgument(
                "gop",
                default_value=TextSubstitution(text="30"),
                description="Keyframe interval in frames (30 = ~1s at 30fps)",
            ),
            DeclareLaunchArgument(
                "use_pts",
                default_value=TextSubstitution(text="false"),
                description="Use GStreamer PTS for timestamps (true/false)",
            ),
            DeclareLaunchArgument(
                "flip_method",
                default_value=TextSubstitution(text="0"),
                description="nvvidconv flip-method (0..7). 0 = no flip",
            ),
            *nodes,
        ]
    )
