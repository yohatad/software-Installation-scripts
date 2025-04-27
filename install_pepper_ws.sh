#!/bin/bash

# Enable command echoing
set -x

# Install the NAOqi driver
echo "Installing NAOqi driver..." && \
sudo apt-get install -y ros-noetic-naoqi-driver

# Create ROS workspace and clone repositories
echo "Creating ROS workspace and cloning repositories..." && \
mkdir -p $HOME/workspace/pepper_rob_ws/src && cd $HOME/workspace/pepper_rob_ws/src && \
git clone https://github.com/cssr4Africa/naoqi_dcm_driver.git && \
git clone https://github.com/cssr4Africa/naoqi_driver.git && \
git clone https://github.com/cssr4Africa/pepper_dcm_robot.git && \
git clone https://github.com/ros-naoqi/pepper_virtual.git && \
git clone https://github.com/ros-naoqi/pepper_robot.git && \
git clone https://github.com/ros-naoqi/pepper_moveit_config.git

# Make scripts executable
echo "Making naoqi_driver scripts executable..." && \
chmod +x $HOME/workspace/pepper_rob_ws/src/naoqi_driver/scripts/*

# Build the repository
echo "Building the repository..." && \
cd $HOME/workspace/pepper_rob_ws && catkin_make

# Source the setup file and add to .bashrc
echo "Sourcing the setup file and updating .bashrc..." && \
source devel/setup.bash && \
echo "source $HOME/workspace/pepper_rob_ws/devel/setup.bash" >> $HOME/.bashrc

# Install additional packages
echo "Installing additional packages..." && \
sudo apt-get install -y ros-noetic-joint-trajectory-controller && \
sudo apt-get install -y ros-noetic-ros-controllers && \
sudo apt-get install -y ros-noetic-pepper-meshes

# Install Python 2.7, Pip2, and necessary packages
echo "Installing Python 2.7, Pip2, and necessary packages..." && \
cd $HOME && \
sudo apt install -y python2 libpython2.7 libatlas3-base && \
curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py && \
sudo python2 get-pip.py && \
pip2 install numpy

# Download and extract the Python SDK
echo "Downloading and extracting the Python SDK..." && \
wget -S -L https://community-static.aldebaran.com/resources/2.5.5/sdk-python/pynaoqi-python2.7-2.5.5.5-linux64.tar.gz && \
tar -xvf pynaoqi-python2.7-2.5.5.5-linux64.tar.gz

# Set PYTHONPATH environment variable and apply changes
echo "Setting PYTHONPATH environment variable and applying changes..." && \
echo "export PYTHONPATH=\${PYTHONPATH}:$HOME/pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages" >> $HOME/.bashrc && \
source $HOME/.bashrc

# Install the Gazebo Simulator
echo "Installing the Gazebo Simulator..." && \
mkdir -p $HOME/workspace/pepper_sim_ws/src && \
cd $HOME/workspace/pepper_sim_ws/src && \
git clone -b correct_chain_model_and_gazebo_enabled https://github.com/awesomebytes/pepper_robot && \
git clone -b simulation_that_works https://github.com/awesomebytes/pepper_virtual && \
git clone https://github.com/cssr4africa/gazebo_model_velocity_plugin && \
sudo apt-get install -y ros-noetic-tf2-sensor-msgs ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control ros-noetic-gazebo-plugins ros-noetic-controller-manager ros-noetic-ddynamic-reconfigure-python ros-noetic-pepper-meshes && \
cd $HOME/workspace/pepper_sim_ws && catkin_make

# Change the default Pip version to Python3
echo "Changing the default Pip version to Python3..." && \
sudo rm /usr/local/bin/pip
sudo rm /bin/pip
sudo ln -s /usr/bin/pip3 /usr/bin/pip
hash -r
pip --version

# Replace the current shell with a new shell to persist the environment changes
exec bash
