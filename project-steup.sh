#!/bin/bash

git clone https://github.com/unitreerobotics/unilidar_sdk.git

cd unilidar_sdk/unitree_lidar_ros

catkin_make

mkdir -p catkin_point_lio_unilidar/src

cd catkin_point_lio_unilidar/src

git clone https://github.com/unitreerobotics/point_lio_unilidar.git

cd ..

catkin_make

