Open Issues
===========

Current main open issue for me is flashing the firmware. It sometimes works but mostly it is not working. I have ordered a new STlink and will see if this will solve the issue

Patrick has changed the firmware, now with speed for each motor as input and wheel encoder data as output. The proxy needs to be adapted as well as the python API.

Linorobot2_hoverboard/ros2/linorobot2_hoverboard_control/linorobot2_hoverboard_control/hoverboard_interface.py must be updated to use wheel encoder information to generate ROS messages for TF and odom

I am currently looking at ROS control if that can be used.
