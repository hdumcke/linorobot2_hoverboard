#!/bin/bash

### Get directory where this script is installed
BASEDIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

[ -d "$HOME/ros2_setup_scripts_ubuntu/" ] || $BASEDIR/setup_ros2.sh

sudo apt install -y python3-rosdep2
source /opt/ros/humble/setup.bash
mkdir -p ~/linorobot2_ws/src
cd ~/linorobot2_ws
git clone -b $ROS_DISTRO https://github.com/linorobot/linorobot2 src/linorobot2
git clone https://github.com/linorobot/linorobot2_viz src/linorobot2_viz
rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent --skip-keys micro_ros_agent
colcon build

sudo apt-get -y install ros-humble-teleop-twist-keyboard

### install Oak-D
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo apt install -y ros-humble-depthai-ros
