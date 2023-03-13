#!/bin/bash

source ~/.bashrc
source ~/horo_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
rm -rf ~/.ros/log/
ros2 launch  horo_bringup bringup.launch.py
