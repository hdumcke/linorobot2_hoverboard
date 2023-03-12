#!/bin/bash

ps aux | grep gazebo        | grep -v grep |  awk -F' ' '{print $2}' | xargs -n 1 kill -9
ps aux | grep gzserver      | grep -v grep | awk -F' ' '{print $2}' |  xargs -n 1 kill -9
ps aux | grep gzclient      | grep -v grep | awk -F' ' '{print $2}' | xargs -n 1 kill -9
ps aux | grep launch_params | grep -v grep | awk -F' ' '{print $2}' | xargs -n 1 kill -9
ps aux | grep humble        | grep -v grep | awk -F' ' '{print $2}' | xargs -n 1 kill -9
