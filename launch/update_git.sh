#!/bin/bash

# Pulls all of the forklift_ws github repositories

echo "Updating github repositories"

cd $HOME/forklift_ws/src/forklift_bringup
git pull

cd ../grasping
git pull

cd ../mapping
git pull

cd ../motion_testing
git pull

cd ../ouster_example
git pull

cd ../robot_localization
git pull

cd ../robust_navigation
git pull

cd ../sensors
git pull

cd ../setup_tf
git pull

echo "Finished updating repositories"
