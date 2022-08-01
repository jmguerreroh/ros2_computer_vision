#!/bin/bash

# Installing ThirdParty repos
sudo apt install python3-vcstool -y
vcs import < thirdparty.repos

# Building project
colcon build --symlink-install

# Setup Gazebo to find models - GAZEBO_MODEL_PATH
source /usr/share/gazebo/setup.bash
echo "source /usr/share/gazebo/setup.bash" >> ~/bashrc

# Project's path
source install/setup.bash
echo "source "${PWD}"/install/setup.bash" >> ~/.bashrc
