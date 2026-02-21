#!/usr/bin/env bash
set -euo pipefail

# Allow local root in X 
xhost +local:root >/dev/null 2>&1 || true

# Directory containing this script (docker_jetson/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Repo root is the parent of docker_jetson/
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Host workspace folder (sibling of docker_jetson/)
HOST_WS="${REPO_ROOT}/ros2_src"

CONTAINER_NAME="billee_ros"
IMAGE="my_isaac_ros_jetson:humble"

# if the can hat is connected then pass it to the container
if [ -e /dev/can0 ]; then
    DEV_ARG="--device=/dev/can0:/dev/can0"
fi


# If the container is already running, just open a new shell in it
if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  exec docker exec -it "${CONTAINER_NAME}" bash
fi

# If a container with that name exists but is stopped, start it and open a shell
if docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  docker start "${CONTAINER_NAME}" >/dev/null
  exec docker exec -it "${CONTAINER_NAME}" bash
fi

# Otherwise, create and run it
exec docker run -it \
  --name "${CONTAINER_NAME}" \
  --mount "type=bind,source=${HOST_WS},target=/home/ros_user/ros2_ws/src" \
  --device=/dev/video0 \
  --network=host \
  --ipc=host \
  ${DEV_ARG} \
  --runtime nvidia \
  -e "DISPLAY=${DISPLAY}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /tmp/argus_socket:/tmp/argus_socket \
  "${IMAGE}" \
  bash

