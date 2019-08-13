#!/bin/bash

# Print out ROS IPs for debugging
echo "ROS_MASTER_URI: ${ROS_MASTER_URI}"
echo "ROS_IP: ${ROS_IP}"

# Setup window rejection for lidar1
./lidar1_setup.sh
