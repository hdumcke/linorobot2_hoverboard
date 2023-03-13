#!/bin/bash

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch depthai_examples mobile_publisher.launch.py camera_model:=OAK-D-LITE
