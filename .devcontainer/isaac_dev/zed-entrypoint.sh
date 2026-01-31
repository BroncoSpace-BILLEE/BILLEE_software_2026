#!/bin/bash

# Enable ZED if installed
echo "Checking ZED SDK..."

ZED_FOLDER=/usr/local/zed/

if [ -d "${ZED_FOLDER}" ]; then
   echo " * ${ZED_FOLDER} exists. Changing permissions"
   chown -R ${USERNAME}:${USERNAME} ${ZED_FOLDER}
   echo " * ZED SDK available:"
   ${ZED_FOLDER}tools/ZED_Explorer -v
else
   echo " * $ZED_FOLDER does not exist. ZED SDK not installed"
   exit 1
fi

exec "$@"