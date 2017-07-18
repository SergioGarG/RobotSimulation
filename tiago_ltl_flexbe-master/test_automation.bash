#!/usr/bin/env bash
CATKIN_SHELL=bash

export PATH=$ROS_ROOT/bin:$PATH
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

data_record='test'
nb_test=$1

for i in `seq 1 $nb_test`
do
	roslaunch tiago_ltl_flexbe pick_test.launch rviz:=false &
	rosparam set simulation_ready false
	while [ $(rosparam get simulation_ready)!='true' ]
	do
		sleep 1
	done
	echo '-------Simulation ready----------'
	./pick_test.py $data_record &
	rosparam set robot_status busy
	while [ $(rosparam get robot_status)!=ready ]
	do
		sleep 1
	done
	./pick_test.py $data_record &
	rosparam set robot_status busy
	while [ $(rosparam get robot_status)!=ready ]
	do
		sleep 1
	done
	killall -9 roscore
	killall -9 roslaunch
	killall -9 rosmaster
	killall -9 rosout
	killall -9 gzserver
	killall -9 gzclient
done
