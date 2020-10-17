 // eslint-disable-next-line
import React, { Component, useContext, useEffect, useState } from "react";
import PropTypes from "prop-types";
//import Viewer from "react-viewer";
 // eslint-disable-next-line
import { useROS } from './ROS';

export function VideoWindowExsample(props) {
  //const [url] = useROS();
 
  return (
    <div>
      {/* <img src="http://192.168.8.100:8080/stream?topic=/yf_camera/LiveVideo"
       alt = 'LIVE'
       /> */}
	{/*
       <img src="http://192.168.178.51:8080/stream?topic=/webserver/TestVideo2"
       alt = 'LIVE'
       />*/}
       <img src="http://192.168.8.214:8080/stream?topic=/yf_camera/LiveVideo&type=ros_compressed"
       alt = 'LIVE_compressed'
	/>
	<img src="http://192.168.8.214:8080/stream?topic=/dwa/Planner&type=ros_compressed"
       alt = 'planner_compressed'
       />
    </div>
  );
}
VideoWindowExsample.propTypes = {
  videoURL: PropTypes.string,
  // width: PropTypes.number,
  // height: PropTypes.number,
};


export default VideoWindowExsample;
