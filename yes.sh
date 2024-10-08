#!/usr/bin/env sh
cd /home/ubuntu/share/catkin_ws
source ./devel/setup.bash
roslaunch realsense2_camera rs_t265.launch
