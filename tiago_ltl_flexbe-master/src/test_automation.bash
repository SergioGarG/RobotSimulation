#!/usr/bin/env bash
CATKIN_SHELL=bash

export PATH=$ROS_ROOT/bin:$PATH
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311



data_record='test6'
nb_test=$1
cpt=0
if ! pgrep -x rosmaster > /dev/null
then
	roslaunch tiago_ltl_flexbe pick_test.launch rviz:=false &
	sleep 10
	rosparam set simulation_status loading
fi

while [ "$cpt" -lt "$nb_test" ]
do
	while [ "$(rosparam get simulation_status)" != "ready" ]
	do
		sleep 1
	done
	./pick_test.py $data_record &
	rosparam set robot_status busy
	while [ "$(rosparam get robot_status)" != "ready" ]
	do
		sleep 1
	done
	let "cpt = cpt + 1"
done
