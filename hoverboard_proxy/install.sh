#!/bin/bash
# Install hoverboard-proxy
#

set -x

### Get directory where this script is installed
BASEDIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cd $BASEDIR
make clean
make

sudo mkir -p /var/lib/hoverboard/
sudo cp hoverboard-proxy /var/lib/hoverboard/
sudo cp hoverboard-proxy.service /lib/systemd/system/
sudo systemctl enable  hoverboard-proxy.service
