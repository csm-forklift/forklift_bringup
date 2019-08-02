#!/usr/bin/env bash

# Items to edit for specific situation
# ~/cartographer_latest/src/cartographer_ros/cartographer_ros/launch/my_robot_2d_localization.launch
# ~/cartographer_latest/src/cartographer_ros/cartographer_ros/configuration_files/realtime_2d.lua
source ~/cartographer_latest/devel_isolated/setup.bash
roslaunch cartographer_ros my_robot_2d_localization_cut.launch > /dev/null &

# Store background PID, which is the cartographer launch above
bg_pid=$!

# Trapping SIGINTs so we can send them back to $bg_pid
trap "kill -2 $bg_pid" 2

sleep 2

# before curve starting position to new origin: translation={-2, -8, 0},rotation={0.0, 0.0, 1.57079632679}

rosservice call /finish_trajectory "trajectory_id: 1"
# Starting point for demo
#rosrun cartographer_ros cartographer_start_trajectory -configuration_directory '/home/csm/cartographer_latest/install_isolated/share/cartographer_ros/configuration_files' -configuration_basename 'realtime_2d.lua' -initial_pose '{to_trajectory_id = 0,relative_pose = {translation={5, 16, 0},rotation = {0.0, 0.0, 1.571}},timestamp = 0}' > /dev/null &
# Test starting point
rosrun cartographer_ros cartographer_start_trajectory -configuration_directory '/home/csm/cartographer_latest/install_isolated/share/cartographer_ros/configuration_files' -configuration_basename 'realtime_2d.lua' -initial_pose '{to_trajectory_id = 0,relative_pose = {translation={8, -4, 0},rotation = {0.0, 0.0, 1.571}}, timestamp = 0}' > /dev/null &
#{translation={-1.9, 0, 0},rotation = {0.0, 0.0, 3.14159265359}}

# Store background PID for rosrun command
bg_pid2=$!

# Trapping SIGINTS so we can send them back to $bg_pid2
trap "kill -2 $bg_pid2" 2

wait $bg_pid
wait $bg_pid2
