#!/usr/bin/env bash

# Items to edit for specific situation
# ~/cartographer_ws/src/cartographer_ros/cartographer_ros/launch/my_robot_2d_localization.launch
# ~/cartographer_ws/src/cartographer_ros/cartographer_ros/configuration_files/realtime_2d.lua
# remove the output from this file by adding > /dev/null before the '&' symbols at the end of the launch line and the rosrun line.
source ~/cartographer_ws/devel_isolated/setup.bash
roslaunch cartographer_ros forklift_2d_localization.launch &

# Store background PID, which is the cartographer launch above
bg_pid=$!

# Trapping SIGINTs so we can send them back to $bg_pid
trap "kill -2 $bg_pid" 2

sleep 2

# before curve starting position to new origin: translation={-2, -8, 0},rotation={0.0, 0.0, 1.57079632679}
PI=3.141592654
NEG_PI=-3.141592654
PI_DIV_2=`echo "scale=9; $PI/2" | bc` # note the 'angled' apostrophe ` not '
NEG_PI_DIV_2=`echo "scale=9; $NEG_PI/2" | bc`
#===== Official Starting Point - DO NOT CHANGE! =====#
STARTING_POINT="translation={3.3, 14.5, 0},rotation={0.0, 0.0, `echo "scale=9; $NEG_PI_DIV_2+0.1" | bc`}"
#====================================================#
#===== Official Demo Point - DO NOT CHANGE! =====#
#STARTING_POINT="translation={4.8, 1.5, 0.0},rotation={0.0, 0.0, `echo "scale=9; 0.75*$PI+0.03" | bc`}"
#================================================#
#----- Starting point for maneuver
#STARTING_POINT="translation={5.5, 0.0, 0},rotation={0.0, 0.0, `echo "scale=9; (-1.0/4.0)*${PI}+0.07" | bc`}"
#----- Starting point for grasping
#STARTING_POINT="translation={-3.5, -4.0, 0.0},rotation={0.0, 0.0, `echo "scale=9; $NEG_PI_DIV_2+0.0" | bc`}"

rosservice call /finish_trajectory "trajectory_id: 1"
# Starting point for demo
#rosrun cartographer_ros cartographer_start_trajectory -configuration_directory '/home/csm/cartographer_ws/install_isolated/share/cartographer_ros/configuration_files' -configuration_basename 'realtime_2d.lua' -initial_pose '{to_trajectory_id = 0,relative_pose = {translation={4, 16, 0},rotation = {0.0, 0.0, 1.571}},timestamp = 0}' > /dev/null &
# Test starting point
rosrun cartographer_ros cartographer_start_trajectory -configuration_directory "$HOME/cartographer_ws/install_isolated/share/cartographer_ros/configuration_files" -configuration_basename "realtime_2d_forklift.lua" -initial_pose "{to_trajectory_id = 0,relative_pose = {$STARTING_POINT}, timestamp = 0}" &
# {translation={-1.9, 0, 0},rotation = {0.0, 0.0, 3.14159265359}}

# Store background PID for rosrun command
bg_pid2=$!

# Trapping SIGINTS so we can send them back to $bg_pid2
trap "kill -2 $bg_pid2" 2

wait $bg_pid
wait $bg_pid2
