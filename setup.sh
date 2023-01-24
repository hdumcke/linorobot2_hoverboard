#!/bin/bash

### Get directory where this script is installed
BASEDIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

############################################
# wait until unattended-upgrade has finished
############################################
tmp=$(ps aux | grep unattended-upgrade | grep -v unattended-upgrade-shutdown | grep python | wc -l)
[ $tmp == "0" ] || echo "waiting for unattended-upgrade to finish"
while [ $tmp != "0" ];do
sleep 10;
echo -n "."
tmp=$(ps aux | grep unattended-upgrade | grep -v unattended-upgrade-shutdown | grep python | wc -l)
done

### Give a meaningfull hostname
grep -q "horo" /etc/hostname || echo "horo" | sudo tee /etc/hostname
grep -q "horo" /etc/hosts || echo "127.0.0.1     horo" | sudo tee -a /etc/hosts


### upgrade Ubuntu and install required packages
echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections
sudo apt update
sudo apt -y upgrade
sudo apt-get -y install g++ dpkg-dev curl python-is-python3

### Install hoverboard proxy
cd $BASEDIR/hoverboard_proxy
./install.sh

### Install pip
cd /tmp
wget --no-check-certificate https://bootstrap.pypa.io/get-pip.py
sudo python get-pip.py

### Install Python module
sudo apt install -y python3-dev
sudo git config --global --add safe.directory $BASEDIR # temporary fix https://bugs.launchpad.net/devstack/+bug/1968798
sudo pip install $BASEDIR/Python_Module


### Install ROS2
[ -d "$HOME/ros2_setup_scripts_ubuntu/" ] || $BASEDIR/setup_ros2.sh

sudo apt install -y python3-rosdep2
source /opt/ros/humble/setup.bash
mkdir -p ~/linorobot2_ws/src
cd ~/linorobot2_ws
cp -r $BASEDIR/ros2/* src/
git clone -b $ROS_DISTRO https://github.com/linorobot/linorobot2 src/linorobot2
touch src/linorobot2/linorobot2_gazebo/AMENT_IGNORE
touch src/linorobot2/linorobot2_navigation/AMENT_IGNORE
touch src/linorobot2/linorobot2_bringup/AMENT_IGNORE
touch src/linorobot2/linorobot2_description/AMENT_IGNORE
touch src/linorobot2_hoverboard_gazebo/AMENT_IGNORE
rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent --skip-keys micro_ros_agent
sudo pip install setuptools==58.2.0 # suppress colcon build warning
colcon build

### Enable bluetooth for joystick
#sudo apt-get -y install joystick
#sudo systemctl enable bluetooth
#sudo adduser $USER bluetooth
