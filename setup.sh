#!/bin/bash
sudo apt install python3-vcstool -y
vcs import < thirdparty.repos
cp -r src/aws_scenarios/* src/ThirdParty/pal_gazebo_worlds/
rm -rf src/aws_scenarios
colcon build --symlink-install
#source install/setup.bash
#echo "source "${PWD}"/install/setup.bash" >> ~/.bashrc
