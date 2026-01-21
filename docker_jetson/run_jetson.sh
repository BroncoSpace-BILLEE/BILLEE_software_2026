#!/usr/bin/env bash
set -e

xhost +local:root || true

# Directory containing this script (docker_jetson/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Repo root is the parent of docker_jetson/
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Host workspace folder (sibling of docker_jetson/)
HOST_WS="${REPO_ROOT}/ros2_src"

docker run -it --rm \
  --mount type=bind,source="${HOST_WS}",target=/home/ros_user/ros2_ws/src \
  --net=host \
  --ipc=host \
  --runtime nvidia \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /tmp/argus_socket:/tmp/argus_socket \
  my_isaac_ros_jetson:humble

