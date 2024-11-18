#! /bin/bash

PASSWD=1

ROS1_DIR=/opt/ros/noetic

if [ -d ${ROS1_DIR} ]; then	
	source ${ROS1_DIR}/setup.bash
else
	echo "Abort!"
	echo "ROS not found"
	exit 1
fi

roslaunch rosbridge_server rosbridge_websocket.launch

if [ $? -eq 1 ]; then
	echo "App failed"
	exit 1
fi

