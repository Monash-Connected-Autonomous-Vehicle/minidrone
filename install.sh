#! /bin/bash

# install python3 dependencies
echo "Installing Python Dependencies..."
sudo pip3 install -r requirements.txt

# install required ros packages
echo "Installing ROS Dependencies..."
sudo apt install ros-melodic-rplidar-ros ros-melodic-twist-mux