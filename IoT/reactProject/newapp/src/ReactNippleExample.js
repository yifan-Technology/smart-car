import React, { Component } from "react";
import PropTypes from "prop-types";
 // eslint-disable-next-line

import ReactNipple from "react-nipple";
import DebugView from "react-nipple/lib/DebugView";
import "react-nipple/lib/styles.css";
import ROSLIB from 'roslib';


const rosObj = {
    ROS: null,
    url: "ws://localhost:9090",
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

const handleDisconnect = () => {
    try {
        myros.ROS.close();
        console.log('disconnected!!!');
    } catch (e) {
      console.log(e);
    }
};

const Listener = () => {
    var newListener = new ROSLIB.Topic({
      ros : myros.ROS,
      name : '/int8',
      messageType : 'std_msgs/Int8',
    });
    newListener.subscribe(function(message) {
        console.log(message.data); 
      });
};


export default class ReactNippleExample extends Component {
    static propTypes = {
        title: PropTypes.string,
        width: PropTypes.number,
        height: PropTypes.number,
        options: PropTypes.object
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
                        height: this.props.height
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
                <DebugView data={this.state.data} />
            </div>
        );
    }

    handleJoystickStart = (evt, data) => {
        this.setState({ data });
        handleConnect();
        Listener();
    };
    handleJoystickEnd = (evt, data) => {
        this.setState({ data });
        handleDisconnect();
    };
    handleJoystickMove = (evt, data) => {
        this.setState({ data });
        Listener();
        
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
