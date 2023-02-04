#!/bin/bash

source ~/.bashrc
source ~/horo_ws/install/setup.bash
rm -rf ~/.ros/log/
ros2 launch  horo_bringup bringup.launch.py
