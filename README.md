# CSSR4Africa Project

![CSSR4Africa Logo](CSSR4AfricaLogo.svg)

The Culturally Sensitive Social Robotics for Africa (CSSR4Africa) project aims to equip robots to interact politely with people in Africa using spatial, non-verbal, and verbal modes of interaction.

## Table of Contents
- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Setting up the Development Environment](#setting-up-the-development-environment)
  - [For the Physical Robot](#for-the-physical-robot)
  - [For the Gazebo Simulator](#for-the-gazebo-simulator)
- [Installing and Running the CSSR4Africa Software](#installing-and-running-the-cssr4africa-software)
  - [Installing for the Physical Robot](#installing-for-the-physical-robot)
  - [Installing for the Simulator Robot](#installing-for-the-simulator-robot)
  - [Running the Software](#running-the-software)
- [References](#references)

## Introduction

This repository contains the source code and installation instructions for the CSSR4Africa project.

## Prerequisites

Please make sure you have a system running Ubuntu 20.04.

## Setting up the Development Environment

### For the Physical Robot

1. **Install the NAOqi Driver and ROS Packages**

    ```bash
    sudo apt-get update
    sudo apt-get install -y ros-noetic-naoqi-driver ros-noetic-joint-trajectory-controller ros-noetic-ros-controllers ros-noetic-pepper-meshes
    ```

2. **Create and Initialize the ROS Workspace**

    ```bash
    mkdir -p $HOME/workspace/pepper_rob_ws/src
    cd $HOME/workspace/pepper_rob_ws/src
    ```

3. **Clone the Required Repositories**

    ```bash
    git clone https://github.com/cssr4africa/naoqi_dcm_driver.git
    git clone https://github.com/cssr4africa/naoqi_driver.git
    git clone https://github.com/cssr4africa/pepper_dcm_robot.git
    git clone https://github.com/ros-naoqi/pepper_virtual.git
    git clone https://github.com/ros-naoqi/pepper_robot.git
    git clone https://github.com/ros-naoqi/pepper_moveit_config.git
    ```

4. **Build the Workspace**

    ```bash
    cd $HOME/workspace/pepper_rob_ws
    catkin_make
    ```

5. **Update the ROS Environment**

    ```bash
    source devel/setup.bash
    echo "source $HOME/workspace/pepper_rob_ws/devel/setup.bash" >> $HOME/.bashrc
    ```

6. **Install and Configure the Python NAOqi SDK**

    ```bash
    cd $HOME
    sudo apt install -y python2 libpython2.7 libatlas3-base
    curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
    sudo python2 get-pip.py
    pip2 install numpy
    wget -S -L https://community-static.aldebaran.com/resources/2.5.5/sdk-python/pynaoqi-python2.7-2.5.5.5-linux64.tar.gz
    tar -xvf pynaoqi-python2.7-2.5.5.5-linux64.tar.gz
    echo "export PYTHONPATH=\${PYTHONPATH}:$HOME/pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages" >> $HOME/.bashrc
    source $HOME/.bashrc
    ```

7. **Bring Up Pepper**

    ```bash
    sudo apt install -y net-tools
    ifconfig
    ```

   ```bash
   roslaunch pepper_dcm_bringup pepper_bringup.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface_name>
   ```

   ```bash
   roslaunch naoqi_driver naoqi_driver.launch nao_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface_name>
   ```

### For the Gazebo Simulator

1. **Install the Gazebo Simulator**

    ```bash
    mkdir -p $HOME/workspace/pepper_sim_ws/src
    cd $HOME/workspace/pepper_sim_ws/src
    git clone -b correct_chain_model_and_gazebo_enabled https://github.com/awesomebytes/pepper_robot
    git clone -b simulation_that_works https://github.com/awesomebytes/pepper_virtual
    git clone https://github.com/cssr4africa/gazebo_model_velocity_plugin
    sudo apt-get install ros-noetic-tf2-sensor-msgs ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control ros-noetic-gazebo-plugins ros-noetic-controller-manager ros-noetic-ddynamic-reconfigure-python ros-noetic-pepper-meshes
    cd .. && catkin_make -DSIMULATOR=ON
    echo "source $HOME/workspace/pepper_sim_ws/devel/setup.bash" >> $HOME/.bashrc
    source $HOME/.bashrc
    ```

2. **Run the Gazebo Simulator**

    ```bash
    roslaunch pepper_gazebo_plugin pepper_gazebo_plugin_in_office_CPU.launch
    rosrun rviz rviz -d `rospack find pepper_gazebo_plugin`/config/pepper_sensors.rviz
    ```

## Installing and Running the CSSR4Africa Software

### Installation for the Physical Robot

1. **Clone and Build the Software**

    ```bash
    cd $HOME/workspace/pepper_rob_ws/src
    git clone https://github.com/cssr4africa/cssr4africa.git
    cd .. && catkin_make
    ```

### Installation for the Simulator Robot

1. **Clone and Build the Software**

    ```bash
    cd $HOME/workspace/pepper_sim_ws/src
    git clone https://github.com/cssr4africa/cssr4africa.git
    cd .. && catkin_make -DSIMULATOR=ON
    ```

### Running the Software

1. **Run Tests on the Physical Robot**

    ```bash
    cd $HOME/workspace/pepper_rob_ws
    source devel/setup.bash
    roslaunch pepper_interface_tests actuatorTestLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface_name>
    roslaunch pepper_interface_tests sensorTestLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface_name>
    ```

2. **Run Tests on the Simulator**

    ```bash
    cd $HOME/workspace/pepper_sim_ws
    source devel/setup.bash
    roslaunch pepper_interface_tests interfaceTestLaunchSimulator.launch
    rosrun pepper_interface_tests sensorTest
    rosrun pepper_interface_tests actuatorTest
    ```

## References

- [Pepper Technical Specifications](http://doc.aldebaran.com/2-5/family/pepper_technical/index_pep.html)
- [Pepper User Guide](http://doc.aldebaran.com/2-5/family/pepper_user_guide/first_conf_pep.html)
- [CSSR4Africa Deliverable D4.1](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.1.pdf)
- [CSSR4Africa Deliverable D5.1](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.1.pdf)
