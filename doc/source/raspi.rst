Raspberry 
=========

This project requires multiple components being installed on the Raspberry:

Operating System
  As we will use ROS2 Humble we requires Ubuntu 22.02 We will opt for the server version but desktop version should also work.
  
HB Proxy
  We requires a proxy that manages the communication with the hoverbaord. This proxy is written in C and is controlled by systemd
  
Python Module
  The hoverbaord hardware is abstracted by a Python API
  
ROS
  ROS2 Humble and ROS2 code that is part of this project needs to be installed
  
Installation
------------

To simplify the installation of the Raspberry Pi we provide a cloid-init based installation procedure.

Prepare a SD card with Ubuntu 22.04

https://cdimage.ubuntu.com/releases/20.04/release/

ubuntu-22.04.1-preinstalled-server-arm64+raspi.img.xz is recommended.

Clone https://github.com/hdumcke/linorobot2_hoverboard on the PC where you have created your SD card. Make sure the SD card is mounted. Run

prepare_sd.py

And answer the questions. At the end eject your SD card, stick it into the Raspberry Pi and boot. At the end the Raspberry Pi will reboot and you should have a complete installation.

During installation the output of the setup script will be written to ~/.setup_err.log and ~/.setup_out.log These file can be used to monitor the installation process 
