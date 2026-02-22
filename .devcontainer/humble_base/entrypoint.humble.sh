#!/usr/bin/env bash
set -e

# Source ROS 2
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi

# Source overlay workspace if it exists
if [ -f "/home/ros_user/ros2_ws/install/setup.bash" ]; then
  source /home/ros_user/ros2_ws/install/setup.bash
fi


