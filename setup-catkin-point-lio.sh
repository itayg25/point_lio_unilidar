#!/bin/bash

# Source the ROS setup script
source /opt/ros/noetic/setup.bash

cd /catkin_ws/unilidar_sdk/unitree_lidar_ros/catkin_point_lio_unilidar

catkin_make
