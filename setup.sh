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
sudo apt install -y ros-humble-ros2-control
sudo apt install -y ros-humble-ros2-controllers ros-humble-slam-toolbox
sudo apt install -y ros-humble-navigation2
sudo apt install -y ros-humble-nav2-bringup
sudo apt install -y ros-humble-robot-localization
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source /opt/ros/humble/setup.bash
mkdir -p ~/horo_ws/src
cd ~/horo_ws
cp -r $BASEDIR/ros2/* src/
git clone -b ros2 https://github.com/linorobot/ldlidar.git src/ldlidar
touch src/horo_gazebo/AMENT_IGNORE
rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent --skip-keys micro_ros_agent
sudo pip install setuptools==58.2.0 # suppress colcon build warning
MAKEFLAGS=-j1 colcon build --executor sequential --symlink-install

### Enable UART3
### TXD3 = Pin 7, RXD3 = Pin 29
sudo sed -i "s/console=serial0,115200 //" /boot/firmware/cmdline.txt
grep -q "uart3" /boot/firmware/config.txt || echo "dtoverlay=uart3" | sudo tee -a /boot/firmware/config.txt

# install service
cd ~
sudo cp $BASEDIR/robot.service /etc/systemd/system/
sudo mkdir -p /var/lib/horo/
sudo cp $BASEDIR/run.sh /var/lib/horo/
sudo systemctl daemon-reload
sudo systemctl enable robot

# install OAK-D-LITE
sudo cp $BASEDIR/oak-d-lite/oak-d-lite.service /etc/systemd/system/
sudo cp $BASEDIR/oak-d-lite/oak-d-lite.sh /var/lib/horo/
sudo systemctl enable oak-d-lite
$BASEDIR/oak-d-lite/ros_install.sh

# install imu
sudo cp $BASEDIR/imu/imu.service /etc/systemd/system/
sudo cp $BASEDIR/imu/imu.sh /var/lib/horo/
sudo systemctl enable imu
$BASEDIR/imu/ros_install.sh

# udev rule for lidar
sudo cp ~/horo_ws/src/ldlidar/ldlidar.rules /etc/udev/rules.d/

# install Python modules
sudo pip install simple-pid

### install screen-commander
sudo apt install -y screen
pip install git+https://github.com/hdumcke/screen-commander@main#egg=screen-commander
echo 'export PATH=~/.local/bin:$PATH' >> ~/.bashrc

sudo reboot
