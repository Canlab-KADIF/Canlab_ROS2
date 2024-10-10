#!/bin/bash

cat <<EOF
************************************
*                                  *
*  CREATE SAVE BAG FOLDER NAME     *
*                                  *
************************************
EOF

read name

BAG_DIR=/home/$USER/$name/
filename=_$(date +"%Y-%m-%d-%H-%M-%S")

if [ -d ${BAG_DIR} ]; then
	echo "ALREADY EXIST FOLDER"
else
	`mkdir ${BAG_DIR}`
	echo "CREATE FOLDER"
fi

cat <<EOF
************************************
*                                  *
*  SET DURATION TIME               *
*  Must be set in seconds          *
*  ex)335 for 5 minutes 35 seconds *
*                                  *
************************************
EOF

read duration

cat <<EOF
************************************
*                                  *
*  IF YOU WANT EXIT Rosbag         *
*  Enter Ctrl + C		    *
*                                  *
************************************
EOF

cd ${BAG_DIR}

`ros2 bag record -d $duration -a -o $filename`
