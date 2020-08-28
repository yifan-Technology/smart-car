#!/bin/bash

echo "yf camera server start!"

function roscore(){
	
	source ~/yf_zed_ws/devel/setup.bash
	roscore

}

function rosBridge(){

	source  /opt/ros/melodic/setup.bash
	source ~/yf_zed_ws/devel/setup.bash
	source  /opt/ros/dashing/setup.bash
	source ~/ros2_dashing/install/local_setup.bash
	ros2 run ros1_bridge dynamic_bridge --bridge-all-topics

}

function zedWrapper(){

	source  /opt/ros/melodic/setup.bash
	source ~/yf_zed_ws/devel/setup.bash
	roslaunch zed_wrapper zed2.launch
	
}

roscore & rosBridge & zedWrapper

echo "yf camera server end!"

kill 0
