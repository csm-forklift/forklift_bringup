#!/usr/bin/env bash

# This block of code will manually set the IP address for ROS
# copy and paste into "~/.bashrc" to have it load with each terminal

SET_ROS_IP=false

if [ $SET_ROS_IP = true ]; then
    # These variables are used for setting up the ROS IP
    #export IP_ADDRESS="$(hostname -I | cut -d ' ' -f 1)" # take the first IP address available from the hostname
    
    # The individual IP addresses for the roscore and for messages (ROS_IP) can be specified here
    export IP_ADDRESS=192.168.0.1
    export IP_MASTER=$IP_ADDRESS
    export IP_MESSAGE=$IP_ADDRESS
    export ROS_MASTER_URI=http://$IP_MASTER:11311
    export ROS_IP=$IP_MESSAGE
else
    unset ROS_MASTER_URI
    unset ROS_IP
fi
