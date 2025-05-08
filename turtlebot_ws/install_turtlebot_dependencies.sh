#!/bin/bash

# update before installing anything
sudo apt update

# install python packages
sudo apt install -y python3-pip
pip install matplotlib

# install helpful tools
sudo apt install -y \
    ros-humble-tf2-tools \
    ros-humble-rqt-* \
    net-tools

# install gazebo
sudo apt install -y \
    ros-humble-gazebo-*
echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc

# install turtlebot dependencies
sudo apt install -y \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-dynamixel-sdk \

source /opt/ros/humble/setup.bash

# create src directory
mkdir src
cd src

# install turtlebot packages
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git

# build 
cd ..
colcon build