#!/bin/bash

# Download dependencies for zed SDK installation RUN file
sudo apt-get update -y || true
sudo apt-get install --no-install-recommends lsb-release wget less udev sudo zstd build-essential cmake libpng-dev libgomp1 -y

# Download zed SDK installation RUN file to /tmp directory
cd /tmp
URL=https://download.stereolabs.com/zedsdk/5.0/cu12/ubuntu24
wget -q --no-check-certificate -O ZED_SDK_Linux.run ${URL}
SIZE=$(du -sb './ZED_SDK_Linux.run' | awk '{ print $1 }')
if ((SIZE<1024)) ; then
   echo "ERROR: ZED_SDK_Linux.run size is not valid: ${SIZE}B!!!";
   echo " * Verify the ZED SDK download link validity: " ${URL} ;
   exit 1
fi

sudo chmod 777 ./ZED_SDK_Linux.run
sudo ./ZED_SDK_Linux.run silent skip_od_module skip_python skip_drivers


# Install zed-ros2-wrapper dependencies
sudo apt-get update && sudo apt-get install -y \
   ros-humble-zed-msgs \
   ros-humble-nmea-msgs \
   ros-humble-geographic-msgs \
   ros-humble-robot-localization \
   #ros-humble-point-cl#!/bin/bash

# Download dependencies for zed SDK installation RUN file
sudo apt-get update -y || true
sudo apt-get install --no-install-recommends lsb-release wget less udev sudo zstd build-essential cmake libpng-dev libgomp1 -y

# Download zed SDK installation RUN file to /tmp directory
cd /tmp
URL=https://download.stereolabs.com/zedsdk/5.1/cu12/ubuntu24
wget -q --no-check-certificate -O ZED_SDK_Linux.run ${URL}
SIZE=$(du -sb './ZED_SDK_Linux.run' | awk '{ print $1 }')
if ((SIZE<1024)) ; then
   echo "ERROR: ZED_SDK_Linux.run size is not valid: ${SIZE}B!!!";
   echo " * Verify the ZED SDK download link validity: " ${URL} ;
   exit 1
fi

sudo chmod 777 ./ZED_SDK_Linux.run
sudo ./ZED_SDK_Linux.run silent skip_od_module skip_python skip_drivers


# Install zed-ros2-wrapper dependencies
sudo apt-get update && sudo apt-get install -y \
   ros-humble-zed-msgs \
   ros-humble-nmea-msgs \
   ros-humble-geographic-msgs \
   ros-humble-robot-localization \
   #ros-humble-point-cloud-transport \
   #ros-humble-point-cloud-transport-plugins \
   #ros-humble-draco-point-cloud-transport \
   #ros-humble-zlib-point-cloud-transport \
   #ros-humble-zstd-point-cloud-transport \
   #ros-humble-point-cloud-transport \

# Uncomment the point cloud transport packages above if you want to use point cloud compression.

# Install base Isaac ROS Nitros packages
sudo apt-get update && sudo apt-get install -y \
   ros-humble-isaac-ros-common \
   ros-humble-isaac-ros-nitros \
   ros-humble-isaac-ros-managed-nitros \
   ros-humble-isaac-ros-nitros-image-type

# Cleanup
sudo rm -rf /usr/local/zed/resources/*
rm -rf ZED_SDK_Linux.run
sudo rm -rf /var/lib/apt/lists/*oud-transport \
   #ros-humble-point-cloud-transport-plugins \
   #ros-humble-draco-point-cloud-transport \
   #ros-humble-zlib-point-cloud-transport \
   #ros-humble-zstd-point-cloud-transport \
   #ros-humble-point-cloud-transport \

# Uncomment the point cloud transport packages above if you want to use point cloud compression.

# Install base Isaac ROS Nitros packages
sudo apt-get update && sudo apt-get install -y \
   ros-humble-isaac-ros-common \
   ros-humble-isaac-ros-nitros \
   ros-humble-isaac-ros-managed-nitros \
   ros-humble-isaac-ros-nitros-image-type

# Cleanup
sudo rm -rf /usr/local/zed/resources/*
rm -rf ZED_SDK_Linux.run
sudo rm -rf /var/lib/apt/lists/*