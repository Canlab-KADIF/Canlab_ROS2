#! /bin/bash

PASSWD=1

if [ `whoami` == "root" ]; then
	killall -2 clpe_ros
	killall -2 nodelet
	killall -2 ros2
else
	echo ${PASSWD} | sudo -S killall -2 clpe_ros
	echo ${PASSWD} | sudo -S killall -2 nodelet
	echo ${PASSWD} | sudo -S killall -2 ros2
fi

echo ' '
