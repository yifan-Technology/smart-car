import React, { Component } from "react";
import logo from "./logo.svg";
import "./App.css";

import ReactNippleExample from "./ReactNippleExample";
// import { EchoTopic } from './EchoTopic';
// import { ToggleConnect } from './ToggleConnect';
// import { ROS } from './ROS';

class App extends Component {
  render() {
    return (
      <div className="App">
        <header className="App-header">
          <img src={logo} className="App-logo" alt="logo" />
          <h1 className="App-title">Welcome to React</h1>
        </header>
        <div className="App-examples">
          <ReactNippleExample
            title="Static 2"
            width={250}
            height={300}
            options={{
              mode: "static",
              color: "blue",
              position: { top: "20%", left: "50%" }
            }}
          />
        </div>
      </div>
    );
  }
}

export default App;