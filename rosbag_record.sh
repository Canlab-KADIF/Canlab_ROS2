#!/bin/bash

ROS_DIR=/opt/ros/humble
BAG_DIR=/home/$USER/BAG
filename=_$(date +"%Y-%m-%d-%H-%M-%S")

if [ -d ${ROS_DIR} ]; then
	source ${ROS_DIR}/setup.bash
else
	echo "Abort!"
	echo "ROS not found"
	exit 1
fi

rm -rf $BAG_DIR

: <<'END'
cat <<EOF
************************************
*                                  *
*  CREATE SAVE BAG FOLDER          *
*                                  *
************************************
EOF

#read name

#if [ -d ${BAG_DIR} ]; then
	#echo "ALREADY EXIST FOLDER"
	#echo "DELETE BAG FOLDER"
	#echo "DELETE BAG FOLDER? (Y/N)"
	#read delete
	
	#if [ $delete == "Y" -o $delete == "y" ]; then
	#	rm -rf $BAG_DIR
	#else
	#	echo rename or move BAG FOLDER
	#	exit 1
	#fi
#else
	#`mkdir ${BAG_DIR}`
	#echo "CREATE FOLDER"
#fi

cat <<EOF
************************************
*                                  *
*  SET DURATION TIME               *
*  1m 30s                          *
*                                  *
************************************
EOF

#read duration

cat <<EOF
************************************
*                                  *
*  IF YOU WANT EXIT Rosbag         *
*  Enter Ctrl + C		    *
*                                  *
************************************
EOF
END

`ros2 bag record -d 90 -a -o $BAG_DIR`
