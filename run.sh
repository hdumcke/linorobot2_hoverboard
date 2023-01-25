#!/bin/bash

source ~/.bashrc
source ~/linorobot2_ws/install/setup.bash
rm -rf ~/.ros/log/
ros2 launch  linorobot2_hoverboard_bringup bringup.launch.py
