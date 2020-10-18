 // eslint-disable-next-line
import React, { Component, useContext, useEffect, useState } from "react";
import PropTypes from "prop-types";
//import Viewer from "react-viewer";
 // eslint-disable-next-line
import { useROS } from './ROS';

export function VideoWindowPlanner(props) {
  //const [url] = useROS();
 
  return (
    <div>
	    <img src="http://192.168.8.214:8080/stream?topic=/dwa/Planner&type=ros_compressed"
       alt = 'planner_compressed'
       />
    </div>
  );
}
VideoWindowPlanner.propTypes = {
  videoURL: PropTypes.string,
  // width: PropTypes.number,
  // height: PropTypes.number,
};


export default VideoWindowPlanner;
