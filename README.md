# CSSR4Africa Project

<img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:100%; height:auto;">

The CSSR4Africa (Culturally sensitive social robots for Africa) project aims to equip social robots with culturally-sensitive behaviours to engage effectively with people in African contexts. By identifying verbal and non-verbal social and cultural norms prevalent in African countries, the project integrates these behavioural patterns into robots, ensuring interactions align with local expectations. Demonstrations include giving a tour of a university laboratory and assisting visitors with directions at a university reception, showcasing the robots' culturally-aware engagement.

## Table of Contents
- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Setting up the Development Environment](#setting-up-the-development-environment)
  - [For the Physical Robot](#for-the-physical-robot)
  - [For the Gazebo Simulator](#for-the-gazebo-simulator)
- [Installing and Running the CSSR4Africa Software](#installing-and-running-the-cssr4africa-software)
  - [Installing for the Physical Robot](#installation-for-the-physical-robot)
  - [Installing for the Simulator Robot](#installation-for-the-simulator-robot)
  - [Running the Software](#running-the-software)
- [References](#references)

## Introduction

This repository contains the necessary shell scripts and installation instructions for the CSSR4Africa project.

## Prerequisites

Please make sure you have a system running Ubuntu 20.04.

## Setting up the Development Environment

`You can follow one of the two alterantives below to setup the development environment for the CSSR4Africa project. You can follow the step by step guide to install ROS Noetic and the required dependencies or you can use the alternative method to install the required software using the provided shell scripts.`

### Installing Dependencies
1.  **Install Curl, Git, and Python3-pip**

    ```bash
    sudo apt update && sudo apt upgrade -y
    ```

    ```bash
    sudo apt install -y curl git python3-pip net-tools
    ```

### Installing ROS Noetic

1. **Setup your computer to accept software from packages.ros.org.**

    ```bash
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    ```

2. **Setup your keys**

    ```bash
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    ```

3. **Update your package list**

    ```bash
    sudo apt update
    ```

4. **Install ROS Noetic**

    ```bash
    sudo apt install -y ros-noetic-desktop-full
    ```

5. **Add ROS environment variables to your bash session every time a new shell is launched**

    ```bash
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && source ~/.bashrc
    ```

6. **Install dependencies for building packages**

    ```bash
    sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    ```

7. **Initialize rosdep**

    ```bash
    sudo rosdep init
    rosdep update
    ```

### `Alternative Method`
Use the following shell script to install ROS Noetic.
1. **Clone the GitHub repository Software Installation Scripts**

```bash
git clone https://github.com/cssr4africa/software-Installation-scripts.git
```

2. **Navigate to the software-Installation-scripts directory and run the install_ros_noetic.sh script**
```bash
./install_ros_noetic.sh
```
   
### For the Physical Robot

1. **Install the NAOqi Driver and ROS Packages**

    ```bash
    sudo apt-get install -y ros-noetic-naoqi-driver ros-noetic-joint-trajectory-controller ros-noetic-ros-controllers ros-noetic-pepper-meshes
    ```

2. **Create and Initialize the ROS Workspace**

    ```bash
    mkdir -p $HOME/workspace/pepper_rob_ws/src && cd $HOME/workspace/pepper_rob_ws/src

    ```

3. **Clone the Required Repositories**

    ```bash
    git clone https://github.com/cssr4africa/naoqi_dcm_driver.git && \
    git clone https://github.com/cssr4africa/naoqi_driver.git && \
    git clone https://github.com/cssr4africa/pepper_dcm_robot.git && \
    git clone https://github.com/ros-naoqi/pepper_virtual.git && \
    git clone https://github.com/ros-naoqi/pepper_robot.git && \
    git clone https://github.com/ros-naoqi/pepper_moveit_config.git

    ```

4. **Build the Workspace**

    ```bash
    cd $HOME/workspace/pepper_rob_ws && catkin_make

    ```

5. **Update the ROS Environment**

    ```bash
    echo "source $HOME/workspace/pepper_rob_ws/devel/setup.bash" >> $HOME/.bashrc && source devel/setup.bash 
    ```

6. **Install and Configure the Python NAOqi SDK**

    ```bash
    cd $HOME && \
    sudo apt install -y python2 libpython2.7 libatlas3-base && \
    curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py && \
    sudo python2 get-pip.py && \
    pip2 install numpy && \
    wget -S -L https://community-static.aldebaran.com/resources/2.5.5/sdk-python/pynaoqi-python2.7-2.5.5.5-linux64.tar.gz && \
    tar -xvf pynaoqi-python2.7-2.5.5.5-linux64.tar.gz && \
    echo "export PYTHONPATH=\${PYTHONPATH}:$HOME/pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages" >> $HOME/.bashrc && \
    source $HOME/.bashrc
    ```

7. **Bring Up Pepper**

    For bringing up the robot, you need to know the robot IP, the roscore IP, and the network interface name. The robot IP is the IP address of the robot, the roscore IP is the IP address of the computer running the roscore, and the network interface name is the name of the network interface. The network interface name can be found by running the `ifconfig` command below.

    ```bash
    ifconfig
    ```

    On a new terminal launch the pepper_bringup to bring up the robot.

   ```bash
   roslaunch pepper_dcm_bringup pepper_bringup.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface_name>
   ```

   On a new terminal launch the naoqi_driver.

   ```bash
   roslaunch naoqi_driver naoqi_driver.launch nao_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface_name>
   ```

### For the Gazebo Simulator

1. **Install the Gazebo Simulator**

    ```bash
    mkdir -p $HOME/workspace/pepper_sim_ws/src && \
    cd $HOME/workspace/pepper_sim_ws/src && \
    git clone -b correct_chain_model_and_gazebo_enabled https://github.com/awesomebytes/pepper_robot && \
    git clone -b simulation_that_works https://github.com/awesomebytes/pepper_virtual && \
    git clone https://github.com/cssr4africa/gazebo_model_velocity_plugin && \
    sudo apt-get install -y ros-noetic-tf2-sensor-msgs ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control ros-noetic-gazebo-plugins ros-noetic-controller-manager ros-noetic-ddynamic-reconfigure-python ros-noetic-pepper-meshes && \
    cd $HOME/workspace/pepper_sim_ws && catkin_make -DSIMULATOR=ON
    ```

    To automatically source the simulator workspace, add the following lines to your .bashrc file. Note that this will set the simulator workspace as the default ROS environment, which will override the physical robot workspace:
    
    ```bash
    echo "source $HOME/workspace/pepper_sim_ws/devel/setup.bash" >> $HOME/.bashrc && \
    source $HOME/.bashrc
    ```    
2. **Run the Gazebo Simulator**

    ```bash
    roslaunch pepper_gazebo_plugin pepper_gazebo_plugin_in_office_CPU.launch
    ```
    
    To visualize the robot in RViz, run the following command in a new terminal:

    ```bash
    rosrun rviz rviz -d `rospack find pepper_gazebo_plugin`/config/pepper_sensors.rviz
    ```

### `Alternative Method`
`Use the following shell script to setup the workspace for both the physical and simulator enviornment. Inorder to make the simulator as the default workspace, you need to run the above commands to source the simulator workspace.`

**Navigate to the software-Installation-scripts directory and run the install_pepper_ws.sh script**

```bash
./install_pepper_ws.sh
```

## Installing and Running the CSSR4Africa Software

### Installation for the Physical Robot

1. **Clone and Build the Software**

    ```bash
    cd $HOME/workspace/pepper_rob_ws/src && \
    git clone https://github.com/cssr4africa/cssr4africa.git && \
    cd $HOME/workspace/pepper_rob_ws && catkin_make

    ```

### Installation for the Simulator Robot

1. **Clone and Build the Software**

    ```bash
    cd $HOME/workspace/pepper_sim_ws/src && \
    git clone https://github.com/cssr4africa/cssr4africa.git && \
    cd $HOME/workspace/pepper_sim_ws && catkin_make -DSIMULATOR=ON
    ```

### Running the Software

1. **Run Tests on the Physical Robot**

    ```bash
    cd $HOME/workspace/pepper_rob_ws
    source devel/setup.bash
    ```

    1. **Running test on the actuator** 
        
        ```bash
        roslaunch pepper_interface_tests actuatorTestLaunchRobot.launch robot_ip:=<robot_ip> 
        roscore_ip:=<roscore_ip> network_interface:=<network_interface_name>
        ```

        On a new terminal run the actuator test.
        ```bash
        rosrun pepper_interface_tests actuatorTest
        ```

    2. **Running test on the sensor**

        ```bash
        roslaunch pepper_interface_tests sensorTestLaunchRobot.launch robot_ip:=<robot_ip> 
        roscore_ip:=<roscore_ip> network_interface:=<network_interface_name>
        ```

        On a new terminal run the sensor test.
        ```bash
        rosrun pepper_interface_tests sensorTest
        ```

2. **Run Tests on the Simulator**

    ```bash
    cd $HOME/workspace/pepper_sim_ws
    source devel/setup.bash
    roslaunch pepper_interface_tests interfaceTestLaunchSimulator.launch
    ```
    
    On a new terminal run the actuator test.
    ```bash
    rosrun pepper_interface_tests actuatorTest
    ```

    On a new terminal run the sensor test.
    ```bash
    rosrun pepper_interface_tests sensorTest
    ```

## References

- [ROS Noetic Installation](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [Pepper Technical Specifications](http://doc.aldebaran.com/2-5/family/pepper_technical/index_pep.html)
- [Pepper User Guide](http://doc.aldebaran.com/2-5/family/pepper_user_guide/first_conf_pep.html)
- [CSSR4Africa Deliverable D4.1](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.1.pdf)
- [CSSR4Africa Deliverable D5.1](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.1.pdf)
