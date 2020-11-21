#!/bin/bash
echo "killing process"
#killall -9 bash
kill -9 $(ps aux | grep '[c]olcon' | awk '{print $2}')
kill -9 $(ps aux | grep '[p]lanner' | awk '{print $2}')


function colcon_dwa(){
echo "colconing dwa"
cd ~/dev_ws;
colcon build --symlink-install --packages-select dwa
}

function rundwa(){
echo "running dwa"
ros2 run dwa planner
}

tongjue=$*

echo "loading ros2 dashing environment"
source /opt/ros/dashing/setup.bash
source ~/dev_ws/install/setup.bash


if [[ $tongjue =~ "-a" ]]
then
echo "colcon and run dwa"
colcon_dwa;
rundwa
fi

if [[ $tongjue =~ "-c" ]]
then
echo "only colcon dwa"
colcon_dwa
fi


if [[ $tongjue =~ "-r" ]]
then
echo "only run dwa"
rundwa
fi


echo "finish"
