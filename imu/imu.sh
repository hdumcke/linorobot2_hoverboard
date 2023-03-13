#!/bin/bash

source ~/.bashrc
source ~/imu_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch mpu6050driver mpu6050driver_launch.py
