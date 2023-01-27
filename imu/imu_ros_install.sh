#!/bin/bash

sudo apt-get install -y libi2c-dev 

mkdir -p ~/imu_ws/src
cd ~/imu_ws
git clone https://github.com/hiwad-aziz/ros2_mpu6050_driver.git src/ros2_mpu6050_driver
# rempa topic
sed -i "/parameters=\[parameter_file],/a \ \ \ \ \ \ \ \ ]" src/ros2_mpu6050_driver/launch/mpu6050driver_launch.py
sed -i "/parameters=\[parameter_file],/a \ \ \ \ \ \ \ \ \ \ \ \ ('/imu', '/imu/raw_data')," src/ros2_mpu6050_driver/launch/mpu6050driver_launch.py
sed -i "/parameters=\[parameter_file],/a \ \ \ \ \ \ \ \ remappings=[" src/ros2_mpu6050_driver/launch/mpu6050driver_launch.py
rosdep update && rosdep install --from-path src --ignore-src -y
colcon build
