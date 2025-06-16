#!/bin/bash

# Enable command echoing
set -x

# Install curl, Git, and python3-pip
echo "Installing curl, Git, and python3-pip..." && \
sudo apt update && \
sudo apt install -y curl git python3-pip git-lfs net-tools

# Setup the computer to accept software from packages.ros.org
echo "Setting up ROS repository..." && \
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \

# Setup your keys
echo "Setting up keys..." && \
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \

# Update Debian package index
echo "Updating package index..." && \
sudo apt update && \

# Install ROS Noetic with the default configurations
echo "Installing ROS Noetic..." && \
sudo apt install -y ros-noetic-desktop-full && \

# Make ROS environment variables automatically added every time a new shell is launched
echo "Setting up ROS environment variables..." && \
echo "source /opt/ros/noetic/setup.bash" >> $HOME/.bashrc && \
source $HOME/.bashrc && \

# Install additional ROS tools and dependencies
echo "Installing additional ROS tools and dependencies..." && \
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential && \

# Initialize rosdep
echo "Initializing rosdep..." && \
sudo apt install -y python3-rosdep && \
sudo rosdep init && \
rosdep update

echo "ROS Noetic installation and setup complete."

# Replace the current shell with a new shell to persist the environment changes
exec bash
