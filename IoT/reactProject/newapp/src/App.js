import React, { Component } from "react";
//import logo from "./logo.svg";
import "./App.css";

import ReactNippleExample from "./ReactNippleExample";
// import { EchoTopic } from './EchoTopic';
// import { ToggleConnect } from './ToggleConnect';
// import { ROS } from './ROS';

class App extends Component {
  render() {
    return (
      <div className="App">
        <div className="App-examples">
          
          <ReactNippleExample
            title="JOY"
            width={400}
            height={400}
            options={{ mode: "static", 
            color: "red",
            position: { top: "50%", left: "50%" },
            size: 200 
          }}
          />
          
          {/* <ROS>
            <ToggleConnect />
            <EchoTopic />
          </ROS> */}
        </div>
      </div>
    );
  }
}

export default App;