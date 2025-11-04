#!/bin/bash

ROS_DIR=/opt/ros/humble
BAG_DIR=/home/$USER/BAG

if [ -d ${ROS_DIR} ]; then
	source ${ROS_DIR}/setup.bash
else
	echo "Abort!"
	echo "ROS not found"
	exit 1
fi

rm -rf $BAG_DIR

`ros2 bag record -d 30 -o $BAG_DIR /diagnostics_agg /planning/scenario_planning/trajectory /control/command/control_cmd /perception/object_recognition/objects /sensing/lidar/concatenated/pointcloud /can0_rx /can0_tx /sensing/radar/mrr /sensing/radar/srr/fr /sensing/radar/srr/fl /sensing/radar/srr/rr /sensing/radar/srr/rl /sensing/gnss/bestpos /sensing/gnss/bestutm /sensing/gnss/bestvel /sensing/gnss/insstdev /sensing/gnss/inspvax /sensing/gnss/odom /clpe/cam_0/compressed /clpe/cam_1/compressed /clpe/cam_2/compressed /clpe/cam_4/compressed /clpe/cam_5/compressed /clpe/cam_6/compressed /sensing/gnss/inspva /tf`
