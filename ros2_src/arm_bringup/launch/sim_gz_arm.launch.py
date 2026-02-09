from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction
import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

desc_share = get_package_share_directory("arm_description")
  # .../share/chassis_description
share_root = os.path.dirname(desc_share)                         # .../share

set_env = [
    SetEnvironmentVariable("IGN_GAZEBO_MODEL_PATH", share_root),
    SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", share_root),
    # Also set newer names (harmless on Fortress)
    SetEnvironmentVariable("GZ_SIM_MODEL_PATH", share_root),
    SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", share_root),
]

def _make_fixed_base_sdf(context, *args, **kwargs):
    # --- adjust these paths ---
    desc_pkg = "arm_description"  # or wherever your xacro lives
    xacro_relpath = os.path.join("urdf", "arm_model.xacro")
    xacro_path = os.path.join(get_package_share_directory(desc_pkg), xacro_relpath)
    
    # Output files (per-launch temp)
    tmpdir = tempfile.mkdtemp(prefix="arm_gz_")
    urdf_path = os.path.join(tmpdir, "arm.urdf")
    sdf_path  = os.path.join(tmpdir, "arm.sdf")

    # 1) xacro -> URDF
    os.system(f'ros2 run xacro xacro -o "{urdf_path}" "{xacro_path}"')

    # 2) URDF -> SDF
    os.system(f'ign sdf -p "{urdf_path}" > "{sdf_path}"')

    # 3) Inject a fixed joint: world -> base_link
    #    (keeps model dynamic but pins base)
    with open(sdf_path, "r", encoding="utf-8") as f:
        sdf = f.read()

    # --- 1) inject the world fixed joint (you already do something like this) ---
    world_joint = """
    <joint name="world_fixed" type="fixed">
      <parent>world</parent>
      <child>base_link</child>
    </joint>
"""

    # --- 2) inject gz_ros2_control plugin ---
    # Make controllers.yaml an absolute path
    controllers_yaml = os.path.join(
        get_package_share_directory("arm_bringup"),
        "config",
        "controllers.yaml",
    )

    gz_ros2_control_plugin = f"""
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>{controllers_yaml}</parameters>
    </plugin>
"""

    import re

    # Insert BOTH right after the opening <model ...> tag
    injection = world_joint + gz_ros2_control_plugin
    sdf2 = re.sub(r"(<model[^>]*>\s*)", r"\1" + injection, sdf, count=1)

    with open(sdf_path, "w", encoding="utf-8") as f:
        f.write(sdf2)

    # Export for spawn_entity Node arguments
    context.launch_configurations["arm_sdf_path"] = sdf_path
    return []

def generate_launch_description():
    # --- Launch arguments ---
    description_pkg = LaunchConfiguration("description_pkg")
    xacro_file = LaunchConfiguration("xacro_file")
    entity_name = LaunchConfiguration("entity_name")
    set_env_bridge = SetEnvironmentVariable("GZ_VERSION", "fortress")
    # Pass-through argument to ros_gz_sim's gz_sim.launch.py
    # Examples:
    #   "empty.sdf"
    #   "-r -v 4 empty.sdf"
    #   "-r -v 4 /absolute/path/to/world.sdf"
    gz_args = LaunchConfiguration("gz_args")

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")

    arm_moveit_share = FindPackageShare("arm_moveit_config")

    set_sim_time = SetParameter(name="use_sim_time", value=True)

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([arm_moveit_share, "/launch/move_group.launch.py",])
    )

    #servo = IncludeLaunchDescription(
    #PythonLaunchDescriptionSource([arm_moveit_share, "/launch/servo.launch.py"])
    #)

    #start_servo = TimerAction(period=5.0, actions=[servo])
    
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([arm_moveit_share, "/launch/moveit_rviz.launch.py"])
    )

    declare_args = [
        DeclareLaunchArgument(
            "description_pkg",
            default_value="arm_description",
            description="Package that contains the robot URDF/Xacro in its share/ directory.",
        ),
        DeclareLaunchArgument(
            "xacro_file",
            default_value="urdf/arm_model.xacro",
            description="Path to the robot Xacro (relative to the description package share).",
        ),
        DeclareLaunchArgument(
            "entity_name",
            default_value="arm",
            description="Name of the spawned entity in Gazebo.",
        ),
        DeclareLaunchArgument(
            "gz_args",
            default_value="-r -v 4 empty.sdf",
            description="Arguments passed to Gazebo Sim (via ros_gz_sim gz_sim.launch.py).",
        ),
        DeclareLaunchArgument("x", default_value="0.0", description="Spawn X (m)."),
        DeclareLaunchArgument("y", default_value="0.0", description="Spawn Y (m)."),
        DeclareLaunchArgument("z", default_value="2", description="Spawn Z (m)."),
        DeclareLaunchArgument("yaw", default_value="0.0", description="Spawn yaw (rad)."),
    ]
    xacro_path = PathJoinSubstitution([
    FindPackageShare(description_pkg),
    xacro_file,
    ])

    # --- Build robot_description from Xacro ---

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("arm_bringup"),
        "config",
        "controllers.yaml",
    ])

    robot_description = ParameterValue(
    Command([
        "xacro", " ",
        xacro_path, " ",
        "controllers_yaml:=", controllers_yaml
    ]),
    value_type=str,
    )

    # --- Launch Gazebo Sim using ros_gz_sim's included launch file ---
    gz_sim_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare("ros_gz_sim"),
            "launch",
            "gz_sim.launch.py"
        ])
    ),
    launch_arguments={
        "gz_args": gz_args,
        "on_exit_shutdown": "true",
    }.items(),
    )

    # --- Publish TF and robot_description ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # --- Bridge /clock so ROS nodes use simulation time (critical for controllers / Nav2) ---
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=[
            # 1. The Topic to bridge
            "/world/empty/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            
            # 2. Force the remapping using explicit flags
            "--ros-args",
            "-r",
            "/world/empty/clock:=/clock"
        ]
    )

    # --- Spawn the robot from /robot_description into Gazebo ---
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", entity_name,
            "-file",  LaunchConfiguration("arm_sdf_path"),
            "-x", x, "-y", y, "-z", z,
            "-Y", yaw,
        ],
    )

    spawn_jsb = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "joint_state_broadcaster",
        "--controller-manager", "/controller_manager",
        "--controller-manager-timeout", "60",
    ],
    output="screen",
    )

    spawn_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_no_wrist_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

    spawn_wrist = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wrist_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

    spawn_gripper = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

 
    actions = []
    actions += declare_args
    actions += set_env
    actions += [set_env_bridge]
    actions += [
        OpaqueFunction(function=_make_fixed_base_sdf),  # must be in the action list
        set_sim_time,
        gz_sim_launch,
        clock_bridge,
        robot_state_publisher,
        spawn_entity,
        spawn_jsb,
        spawn_arm,
        spawn_wrist,
        spawn_gripper,
        move_group,
        #start_servo,
        moveit_rviz,
    ]

    return LaunchDescription(actions)
