#!/bin/bash

### Get directory where this script is installed
BASEDIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

[ -d "$HOME/ros2_setup_scripts_ubuntu/" ] || $BASEDIR/setup_ros2.sh

sudo apt install -y python3-rosdep2
sudo apt install -y ros-humble-ros2-control
sudo apt install -y ros-humble-ros2-controllers
sudo apt install -y ros-humble-slam-toolbox
sudo apt install -y ros-humble-navigation2
sudo apt install -y ros-humble-nav2-bringup
sudo apt install -y ros-humble-robot-localization
sudo apt install -y ros-humble-xacro
source /opt/ros/humble/setup.bash
mkdir -p ~/horo_ws/src
cd ~/horo_ws
cp -r $BASEDIR/ros2/* src/
git clone -b ros2 https://github.com/linorobot/ldlidar.git src/ldlidar
rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent --skip-keys micro_ros_agent
colcon build

sudo apt-get -y install ros-humble-teleop-twist-keyboard ros-humble-plotjuggler-ros ros-humble-plotjuggler ros-humble-cartographer
sudo apt install -y ros-humble-rmw-cyclonedds-cpp

echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc

### install Oak-D
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo apt install -y ros-humble-depthai-ros

### Install pip
cd /tmp
wget --no-check-certificate https://bootstrap.pypa.io/get-pip.py
sudo python get-pip.py

### install Gazebo
sudo apt install -y ros-humble-gazebo-ros-pkgs 

### install screen-commander
sudo apt install -y screen
pip install git+https://github.com/hdumcke/screen-commander@main#egg=screen-commander
