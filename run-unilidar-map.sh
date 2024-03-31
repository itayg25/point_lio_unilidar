#!/bin/bash
# Start process 1 in the background
(
cd /catkin_ws/unilidar_sdk/unitree_lidar_ros
source devel/setup.bash
roslaunch unitree_lidar_ros run_without_rviz.launch
) &

sleep 5
# Start process 2 in the background
(
cd /catkin_ws/unilidar_sdk/unitree_lidar_ros/catkin_point_lio_unilidar
source devel/setup.bash
roslaunch point_lio_unilidar mapping_unilidar_no_rviz.launch 
) &

# Wait for both processes to finish
wait
