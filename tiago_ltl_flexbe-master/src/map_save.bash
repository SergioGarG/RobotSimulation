#!/usr/bin/env bash


CATKIN_SHELL=bash

rosservice call /pal_map_manager/save_map "directory ''"

workspace='tiago_public_ws'

if [ ! -e ~/$workspace/tiago_maps ]
then
	mkdir ~/$workspace/tiago_maps
fi

map=$(rosparam get /world_name)

if [ -e ~/$workspace/tiago_maps/$map ]
then
	rm -r ~/$workspace/tiago_maps/$map/config
	cp -R ~/.pal/tiago_maps/config ~/$workspace/tiago_maps/$map
else
	mkdir ~/$workspace/tiago_maps/$map
	cp -R ~/.pal/tiago_maps/config ~/$workspace/tiago_maps/$map
fi
	
