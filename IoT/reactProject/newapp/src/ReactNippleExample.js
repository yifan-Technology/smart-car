import React, { Component } from "react";
import PropTypes from "prop-types";
 // eslint-disable-next-line

import ReactNipple from "react-nipple";
// import DebugView from "react-nipple/lib/DebugView";
import "react-nipple/lib/styles.css";
import ROSLIB from 'roslib';


const rosObj = {
    ROS: null,
    url: "ws://192.168.8.100:9090",
    isConnected: false,
    autoconnect: false,
    topics: [],
    listeners: [],
  };
let myros = rosObj;

const handleConnect = () => {
try {
    myros.ROS = new ROSLIB.Ros({
    url : myros.url,
    });

    if (myros.ROS) myros.ROS.on('connection', (error) => {
        console.log('connected!!!');
    });

    if (myros.ROS) myros.ROS.on('error', (error) => {
    console.log(error);
    });
} catch (e) {
    console.log(e);
}
}; 

// eslint-disable-next-line
const handleDisconnect = () => {
    try {
        myros.ROS.close();
        console.log('disconnected!!!');
    } catch (e) {
      console.log(e);
    }
};


function pubSollSpeed(speed) {
    var carpub = new ROSLIB.Topic({
        ros : myros.ROS,
        name : 'soll_speed',
        messageType : 'std_msgs/msg/Float32MultiArray',
    });

    var vel = {
        layout: {
            dim: [
                {
                  label: 'height',
                  size: 2,
                  stride: 2 * 3 * 3,
                },
                {
                  label: 'weight',
                  size: 3,
                  stride: 3 * 3,
                },
                {
                  label: 'channel',
                  size: 3,
                  stride: 3,
                },
            ],
              data_offset: 0,
            },
            data: speed,
    };

    carpub.publish(vel);
  };


export default class ReactNippleExample extends Component {
    static propTypes = {
        title: PropTypes.string,
        width: PropTypes.number,
        height: PropTypes.number,
        options: PropTypes.object,
        size: PropTypes.number,
    };
    state = {
        data: undefined
    };
    render() {
        return (
            <div className="NippleExample">
                <h2>{this.props.title}</h2>
                <ReactNipple
                    className="joystick"
                    options={this.props.options}
                    style={{
                        outline: `1px dashed ${this.props.options.color}`,
                        width: this.props.width,
                        height: this.props.height,
                        size:this.props.size,
                    }}
                    onStart={this.handleJoystickStart}
                    onEnd={this.handleJoystickEnd}
                    onMove={this.handleJoystickMove}
                    onDir={this.handleJoystickDir}
                    onPlain={this.handleJoystickPlain}
                    onShown={this.handleJoystickShown}
                    onHidden={this.handleJoystickHidden}
                    onPressure={this.handleJoystickPressure}
                />
                {/* <DebugView data={this.state.data} /> */}
            </div>
        );
    }

    handleJoystickStart = (evt, data) => {
        this.setState({ data });
        handleConnect();
    };
    handleJoystickEnd = (evt, data) => {
        this.setState({ data });
        pubSollSpeed([0,0,0,0,]);
        //handleDisconnect();
    };
    handleJoystickMove = (evt, data) => {
        this.setState({ data }); 
        var speed = data.distance*7.5;
        console.log(speed);
        var vx = speed*Math.sin(data.angle.radian);
        var w = -speed*Math.cos(data.angle.radian)*4;
        if(data.angle.degree>180){
            w = -w;
        } 
        console.log(vx,w);
        var vl= vx-0.46*w;
        var vr= vx+0.46*w;
        console.log(vl, vr);
        pubSollSpeed([vl, vr, vl, vr,]);      
    };
    handleJoystickDir = (evt, data) => {
        this.setState({ data });
    };
    handleJoystickPlain = (evt, data) => {
        this.setState({ data });
    };
    handleJoystickShown = (evt, data) => {
        this.setState({ data });
    };
    handleJoystickHidden = (evt, data) => {
        this.setState({ data });
    };
    handleJoystickPressure = (evt, data) => {
        this.setState({ data });
    };
}
