#!/bin/bash

source ~/.bashrc
source ~/imu_ws/install/setup.bash
ros2 launch mpu6050driver mpu6050driver_launch.py
