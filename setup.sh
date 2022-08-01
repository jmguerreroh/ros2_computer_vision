#!/bin/bash

# Installing ThirdParty repos
sudo apt update
sudo apt install python3-vcstool python3-pip -y
vcs import < thirdparty.repos

# Building project
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install

# Setup Gazebo to find models - GAZEBO_MODEL_PATH
source /usr/share/gazebo/setup.bash
echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc

# Project's path
source install/setup.bash
echo "source "${PWD}"/install/setup.bash" >> ~/.bashrc
