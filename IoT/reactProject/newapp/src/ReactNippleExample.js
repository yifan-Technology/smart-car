 // eslint-disable-next-line
import React, { Component, useContext, useEffect, useState } from "react";
import PropTypes from "prop-types";
 // eslint-disable-next-line

import ReactNipple from "react-nipple";
// import DebugView from "react-nipple/lib/DebugView";
import "react-nipple/lib/styles.css";
import { useROS } from './ROS';

export function ReactNippleExample(props) {

    const {pubSollSpeed,isConnected} = useROS();

     // eslint-disable-next-line
    const [mydata, setdata] = useState(undefined);

    const handleJoystickStart = (evt, data) => {
        setdata(data);
    };
    const handleJoystickEnd = (evt, data) => {
        setdata(data);
        if(isConnected){
        pubSollSpeed([0,0,0,0,]);}
        //handleDisconnect();
    };
    const handleJoystickMove = (evt, data) => {
        setdata(data);
        var speed = data.distance*15;
        console.log(speed);
        var vx = speed*Math.sin(data.angle.radian);
        var w = -speed*Math.cos(data.angle.radian)*4;
        if(data.angle.degree>180){
            w = -w;
        } 
        console.log(vx,w);
        var vl= vx - 0.46*w;
        var vr= vx + 0.46*w;
        console.log(vl, vr);
        if(isConnected){
        pubSollSpeed([vl, vr, vl, vr,]);}      
    };
    const handleJoystickDir = (evt, data) => {
        setdata(data);
    };
    const handleJoystickPlain = (evt, data) => {
        setdata(data);
    };
    const handleJoystickShown = (evt, data) => {
        setdata(data);
    };
    const handleJoystickHidden = (evt, data) => {
        setdata(data);
    };
    const handleJoystickPressure = (evt, data) => {
        setdata(data);
    };
        return (
            <div className="NippleExample">
                <h2>{props.title}</h2>
                <ReactNipple
                    className="joystick"
                    options={props.options}
                    style={{
                        outline: `1px dashed ${props.options.color}`,
                        width: props.width,
                        height: props.height,
                        size:props.size,
                    }}
                    onStart={handleJoystickStart}
                    onEnd={handleJoystickEnd}
                    onMove={handleJoystickMove}
                    onDir={handleJoystickDir}
                    onPlain={handleJoystickPlain}
                    onShown={handleJoystickShown}
                    onHidden={handleJoystickHidden}
                    onPressure={handleJoystickPressure}
                />
                {/* <DebugView data={this.state.data} /> */}
            </div>
        );    
}
ReactNippleExample.propTypes = {
    title: PropTypes.string,
    width: PropTypes.number,
    height: PropTypes.number,
    options: PropTypes.object,
    size: PropTypes.number,
  };

export default ReactNippleExample;