#!/bin/bash

if [ $# -ne 7 ]; then
  echo "Usage: $0 <bag_path1> <duration1> <topic1> <bag_path2> <duration2> <topic2> <output_dir>"
  exit 1
fi

BAG_PATH1=$1
DURATION1=$2
TOPIC1=$3
BAG_PATH2=$4
DURATION2=$5
TOPIC2=$6
OUTPUT_DIR=$7

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


ros2 run image_extractor extractor ${BAG_PATH1} ${DURATION1} ${TOPIC1} ${BAG_PATH2} ${DURATION2} ${TOPIC2} ${OUTPUT_DIR}

if [ $? -eq 1 ]; then
  echo "Failed to execute image_extractor"
  exit 1
fi

echo "image_extractor executed successfully, output saved to ${OUTPUT_DIR}"
