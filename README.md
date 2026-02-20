# BILLEE_software_2026
ROS packages developed for BILLEE 2025-26 season

## Images
VScode devcontainers (under .devcontainer folder):
1. humble base
    - OSRF humble image
3. Isaac Dev
   - NVIDIA ISAAC ROS humble image for x86-64 devices

Jetson containers (under docker_jetson folder):
1. ISAAC ROS container
    - open with run_jetson.sh (if container is already running opens new terminal in it)
    - meant to run on a Jetson Orin AGX
  
## Requirements
for all devcontainers:
1. Ubuntu 24+ base OS on device
2. Docker Engine (use convienence script)
3. VScode with Remote Development Extension group installed

for isaac_dev devcontainer:
1. NVIDIA GPU 3000 series+ (ex - 3060ti) with drivers installed
2. NVIDA Container Toolkit
3. At least 80GB of free storage space
