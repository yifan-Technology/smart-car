#!/bin/bash
echo "killing process"
#killall -9 bash
#kill $(ps aux | grep '[p]ython csp_build.py' | awk '{print $2}')
kill $(ps aux | grep '[r]os2 run webvideoserver webvideoserver' | awk '{print $2}')
kill $(ps aux | grep '[n]ode ~/node_modules/ros2-web-bridge/bin/rosbridge.js' | awk '{print $2}')
kill $(ps aux | grep '[c]d ~/newapp;npm start' | awk '{print $2}')
echo "loading ros2 dashing environment"
source /opt/ros/dashing/setup.bash
source ~/dev_ws/install/setup.bash
#1
echo "loading ros2 web video server"
sleep 1s
{
gnome-terminal -t "roswebserver" -- bash -c "ros2 run webvideoserver webvideoserver;exec bash"
}&
#2
echo "loading ros2-web-bridge"
sleep 1s
{
gnome-terminal -t "ros2webbridge" -- bash -c "node ~/node_modules/ros2-web-bridge/bin/rosbridge.js;exec bash"
}&
#3
echo "runing react App"
sleep 1s
{
gnome-terminal -t "newapp" -- bash -c "cd ~/newapp;npm start;exec bash"
}&

echo "finish"
