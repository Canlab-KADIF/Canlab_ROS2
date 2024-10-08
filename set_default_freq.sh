#! /bin/bash


cpus=`ls /sys/bus/cpu/devices/`

for cpu in ${cpus}; do
	echo schedutil > /sys/bus/cpu/devices/${cpu}/cpufreq/scaling_governor
done

echo 1420800 > /sys/bus/cpu/devices/cpu0/cpufreq/scaling_max_freq
