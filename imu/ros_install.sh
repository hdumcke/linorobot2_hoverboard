#!/bin/bash

### Get directory where this script is installed
BASEDIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

sudo apt-get install -y libi2c-dev 

mkdir -p ~/imu_ws/src
cd ~/imu_ws
git clone https://github.com/hiwad-aziz/ros2_mpu6050_driver.git src/ros2_mpu6050_driver
# patch
cp $BASEDIR/mpu6050driver.cpp src/ros2_mpu6050_driver/src/mpu6050driver.cpp
cp $BASEDIR/mpu6050driver_launch.py src/ros2_mpu6050_driver/launch/mpu6050driver_launch.py
rosdep update && rosdep install --from-path src --ignore-src -y
colcon build
