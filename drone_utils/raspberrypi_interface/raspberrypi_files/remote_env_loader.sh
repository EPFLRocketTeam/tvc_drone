#!/bin/bash

export ROS_IP=192.168.0.196
export ROS_MASTER_URI=http://192.168.0.196:11311
#export ROSLAUNCH_SSH_UNKNOWN=1

source /home/pi/catkin_ws/devel/setup.bash

exec "$@"
