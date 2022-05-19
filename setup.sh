#!/bin/bash

# Installing ThirdParty repos
sudo apt install python3-vcstool -y
vcs import < thirdparty.repos
rm -rf thirdparty.repos

# Moving new Tiago launcher to change position using XML file
cp src/computer_vision/launch/tiago_spawn.launch.py src/ThirdParty/tiago_simulation/tiago_gazebo/launch/tiago_spawn.launch.py
rm -rf src/computer_vision/launch/tiago_spawn.launch.py 

# Moving AWS worlds to pal_gazebo_worlds package
cp -r src/aws_worlds/* src/ThirdParty/pal_gazebo_worlds/
rm -rf src/aws_worlds

# Compiling project
colcon build --symlink-install

# Including project's path in ~/.bashrc
#source install/setup.bash
#echo "source "${PWD}"/install/setup.bash" >> ~/.bashrc 

# Removing installation script
rm -rf setup.sh
