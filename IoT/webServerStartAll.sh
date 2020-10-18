#!/bin/bash
echo "killing process"
#killall -9 bash
kill -9 $(ps aux | grep '[w]ebvideoserver' | awk '{print $2}')
kill -9 $(ps aux | grep '[n]ode' | awk '{print $2}')
kill -9 $(ps aux | grep '[n]pm' | awk '{print $2}')


echo "loading ros2 dashing environment"
source /opt/ros/dashing/setup.bash
source ~/dev_ws/install/setup.bash

function roswebserver(){
echo "loading ros2 web video server"
ros2 run webvideoserver webvideoserver
}

function ros2webbridge(){
echo "loading ros2-web-bridge"
node ~/node_modules/ros2-web-bridge/bin/rosbridge.js
}

function newapp(){
echo "runing react App"
cd ~/newapp;npm start
}


roswebserver&ros2webbridge&newapp


echo "finish"
