#! /bin/bash

if [ $# -ne 1 ]; then
  echo "Usage: $0 <input_bag_folder_path>"
  exit 1
fi

INPUT_BAG_FOLDER=$1

# ROS2 환경 설정
ROS_DIR=/opt/ros/humble
ROS_WS_DIR=/home/$USER/Canlab_ROS2

# ROS 설치 확인 및 소스 설정
if [ -d ${ROS_DIR} ]; then
  source ${ROS_DIR}/setup.bash
else
  echo "Abort!"
  echo "ROS not found"
  exit 1
fi

if [ -d ${ROS_WS_DIR} ]; then
  cd ${ROS_WS_DIR}
  source ${ROS_WS_DIR}/install/setup.bash
else
  echo "Abort!"
  echo "No such directory"
  exit 1
fi

ros2 run gps_extractor extractor ${INPUT_BAG_FOLDER}

if [ $? -eq 1 ]; then
  echo "Failed to execute gps_extractor"
  exit 1
fi

echo "gps_extractor executed successfully"

