#!/bin/bash

# Print out ROS IPs for debugging
echo "ROS_MASTER_URI: ${ROS_MASTER_URI}"
echo "ROS_IP: ${ROS_IP}"

# Start Canbus reader
slcan_add.sh

# Setup window rejection for lidar0 (64 res)
./lidar0_setup.sh
