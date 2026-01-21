#!/usr/bin/env bash
set -e

# Source ROS 2
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi

# Source overlay workspace if it exists
if [ -f "/ws/install/setup.bash" ]; then
  source /ws/install/setup.bash
fi

exec "$@"

