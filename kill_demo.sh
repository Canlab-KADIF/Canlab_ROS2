#! /bin/bash

PASSWD='1'

if [ `whoami` == "root" ]; then
	killall -2 clpe_ros
	killall -2 trigger_topic_pub
	#killall -2 nodelet
	bag=`ps -aux | grep "ros2 bag" | grep -v "color=auto"`
	if [ $? == 0 ]; then
		kill `ps -aux | grep "ros2 bag" | grep -v "color=auto" | awk '{print $2}'` 2>/dev/null
	fi
	#launch=`ps -aux | grep "ros2 launch" | grep -v "color=auto"`
	#if [ $? == 0 ]; then
	#	kill `ps -aux | grep "ros2 launch" | grep -v "color=auto" | awk '{print $2}'` 2>/dev/null
	#fi
else
	echo ${PASSWD} | sudo -S killall -2 clpe_ros
	echo ${PASSWD} | sudo -S killall -2 trigger_topic_pub
	#echo ${PASSWD} | sudo -S killall -2 nodelet
	bag=`ps -aux | grep "ros2 bag" | grep -v "color=auto"`
	if [ $? == 0 ]; then
		echo ${PASSWD} | sudo -S kill `ps -aux | grep "ros2 bag" | grep -v "color=auto" | awk '{print $2}'` 2>/dev/null
	fi
	#launch=`ps -aux | grep "ros2 launch" | grep -v "color=auto"`
	#if [ $? == 0 ]; then
	#	echo ${PASSWD} | sudo -S kill `ps -aux | grep "ros2 launch" | grep -v "color=auto" | awk '{print $2}'` 2>/dev/null
	#fi
fi

echo ' '
