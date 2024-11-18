#! /bin/bash

PASSWD=1

ROS1_DIR=/opt/ros/noetic
ROS2_DIR=/opt/ros/galactic

if [ -d ${ROS1_DIR} ]; then	
	source ${ROS1_DIR}/setup.bash
else
	echo "Abort!"
	echo "ROS not found"
	exit 1
fi

if [ -d ${ROS2_DIR} ]; then	
	source ${ROS2_DIR}/setup.bash
else
	echo "Abort!"
	echo "ROS not found"
	exit 1
fi


ros2 run ros1_bridge dynamic_bridge

if [ $? -eq 1 ]; then
	echo "App failed"
	exit 1
fi

