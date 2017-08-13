#!/usr/bin/env bash


CATKIN_SHELL=bash

workspace='tiago_public_ws'

rm -r ~/.pal/tiago_maps/config

if [ -e ~/$workspace/tiago_maps/$1 ]
then
	cp -R ~/$workspace/tiago_maps/$1/config ~/.pal/tiago_maps
else
	echo 'Bug : required map does not exist'
	return 1
fi
	
