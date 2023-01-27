#!/bin/bash

source ~/.bashrc
ros2 launch depthai_examples mobile_publisher.launch.py camera_model:=OAK-D-LITE
