#!/usr/bin/env bash

# Items to edit for specific situation
# ~/cartographer_latest/src/cartographer_ros/cartographer_ros/launch/my_robot_2d_localization.launch
# ~/cartographer_latest/src/cartographer_ros/cartographer_ros/configuration_files/realtime_2d.lua
source ~/cartographer_latest/devel_isolated/setup.bash
roslaunch cartographer_ros my_robot_2d_localization_cut.launch &

# Store background PID, which is the cartographer launch above
bg_pid=$!

# Trapping SIGINTs so we can send them back to $bg_pid
trap "kill -2 $bg_pid" 2

sleep 2

rosservice call /finish_trajectory "trajectory_id: 1"
rosrun cartographer_ros cartographer_start_trajectory -configuration_directory '/home/csm/cartographer_latest/install_isolated/share/cartographer_ros/configuration_files' -configuration_basename 'realtime_2d.lua' -initial_pose '{to_trajectory_id = 0,relative_pose = {translation={-0.5 , 15, 0},rotation = {0.0, 0.0, 0.1}},timestamp = 0}'

wait $bg_pid
