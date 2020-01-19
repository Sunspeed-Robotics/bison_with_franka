#!/bin/bash


source /opt/ros/kinetic/setup.bash

source /home/bison/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.1.30:11311
export ROS_IP=192.168.1.30
echo "ROS environment is Ready"
sleep 5

roslaunch bison_driver bison_driver.launch 
