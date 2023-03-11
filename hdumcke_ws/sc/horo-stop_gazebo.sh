#!/bin/bash

ps aux | grep gazebo | grep horo | awk -F' ' '{print $2}' | xargs -n 1 kill -9
ps aux | grep gzserver | grep horo | awk -F' ' '{print $2}' |  xargs -n 1 kill -9
ps aux | grep gzclient | grep horo | awk -F' ' '{print $2}' | xargs -n 1 kill -9
