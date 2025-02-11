#! /bin/bash

PASSWD=1

ROS_DIR=/opt/ros/humble
ROS_WS_DIR=/home/$USER/Canlab_ROS2

if [ `whoami` == "root" ]; then
	$ROS_WS_DIR/set_max_freq.sh
else
	echo ${PASSWD} | sudo -S $ROS_WS_DIR/set_max_freq.sh
fi

#C-CAN, AD-CAN enable / bitrate 500000
ccan=`ifconfig -a can0 | awk 'NR < 2 {print $2}' | grep UP`
if [ $? != 0 ]; then
	sudo ip link set can0 up type can bitrate 500000
fi
adcan=`ifconfig -a can1 | awk 'NR < 2 {print $2}' | grep UP`
if [ $? != 0 ]; then
	sudo ip link set can1 up type can bitrate 500000
fi
# PCAN USB verify
#ccan=`ifconfig -a can4 | awk 'NR < 2 {print $2}' | grep UP`
#if [ $? != 0 ]; then
#	sudo ip link set can4 up type can bitrate 500000
#fi
#adcan=`ifconfig -a can5 | awk 'NR < 2 {print $2}' | grep UP`
#if [ $? != 0 ]; then
#	sudo ip link set can5 up type can bitrate 500000
#fi
#mrr=`ifconfig -a can7 | awk 'NR < 2 {print $2}' | grep UP`
#if [ $? != 0 ]; then
#	sudo ip link set can7 up type can bitrate 500000
#fi
#srr_f=`ifconfig -a can8 | awk 'NR < 2 {print $2}' | grep UP`
#if [ $? != 0 ]; then
#	sudo ip link set can8 up type can bitrate 500000 dbitrate 2000000 fd on
#fi
#srr_r=`ifconfig -a can9 | awk 'NR < 2 {print $2}' | grep UP`
#if [ $? != 0 ]; then
#	sudo ip link set can9 up type can bitrate 500000 dbitrate 2000000 fd on
#fi

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

if [ -d ${ROS_WS_DIR} ]; then
	cd ${ROS_WS_DIR}
	ros2 launch clapp clapp_all_launch.py
	
	if [ $? -eq 1 ]; then
		echo "App launch failed"
		exit 1
	fi
else
	echo "Abort!"
	echo "No app"
	exit 1
fi
