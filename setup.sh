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
grep -q "hover_robot" /etc/hostname || echo "hover_robot" | sudo tee /etc/hostname
grep -q "hover_robot" /etc/hosts || echo "127.0.0.1     hover_robot" | sudo tee -a /etc/hosts


### upgrade Ubuntu and install required packages
echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections
sudo apt update
sudo apt -y upgrade
sudo apt-get -y install g++ dpkg-dev curl python-is-python3 mpg321 python3-tk openssh-server

### Install hoverboard proxy
cd ~/hoverboard_proxy
./install.sh

### Install pip
cd /tmp
wget --no-check-certificate https://bootstrap.pypa.io/get-pip.py
sudo python get-pip.py

### Install Python module
sudo apt install -y python3-dev
sudo git config --global --add safe.directory $BASEDIR # temporary fix https://bugs.launchpad.net/devstack/+bug/1968798
sudo pip install $BASEDIR/Python_Module
