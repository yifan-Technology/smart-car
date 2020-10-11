 // eslint-disable-next-line
 import React, { Component, useContext, useEffect, useState } from "react";
 import PropTypes from "prop-types";
 //import Viewer from "react-viewer";
  // eslint-disable-next-line
 import { useROS } from './ROS';
 
 export function VideoWindowExsample(props) {
   // const [ visible, setVisible ] = React.useState(false);
  
   return (
     <div>
       {/* <button onClick={() => { setVisible(true); } }>show</button>
       <Viewer
       visible={visible}
       onClose={() => { setVisible(false); } }
       images={[{
         src: props.videoURL, 
         alt: props.videoURL,
       }]} */}
       {/* // defaultSize={[{
       //   width: props.width, 
       //   height: props.height,
       // }]}
       /> */}
       <img src="/stream?topic=/yf_camera/LiveVideo"
        alt = 'LIVE'
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