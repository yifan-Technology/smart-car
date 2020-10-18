#!/bin/bash

echo "yf main start!"


function yfmain(){

cd ~/dev_ws
source  /opt/ros/dashing/setup.bash
source ~/ros2_dashing/install/local_setup.bash
source ~/dev_ws/install/setup.bash
ros2 run yfcam leftcam

}

yfmain

echo "yf main end!"

kill 0
