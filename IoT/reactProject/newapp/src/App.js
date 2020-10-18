import React, { Component } from "react";
//import logo from "./logo.svg";
import "./App.css";

import ReactNippleExample from "./ReactNippleExample";
// import { EchoTopic } from './EchoTopic';
import { ToggleConnect } from './ToggleConnect';
import { VideoWindowLiveVideo } from './VideoWindowLiveVideo';
import { VideoWindowPlanner } from './VideoWindowPlanner';
import { ROS } from './ROS';
import GridLayout from 'react-grid-layout';

class App extends Component {
  render() {
    const layout = [
      {i: 'a', x: 0.5, y: 3, w: 3, h: 2,static: true},
      {i: 'b', x: 0, y: 6, w: 3, h: 2,static: true},
      {i: 'c', x: 5, y: 0, w: 5, h: 5,static: true},
      {i: 'd', x: 4.5, y: 7, w: 5, h: 5,static: true}
    ];
    return (
      <div className="App">
        <ROS>
        <GridLayout className="layout" layout={layout} cols={12} rowHeight={30} width={1200}>
          <div key="a" className="ToggleConnectLayout">
            <ToggleConnect />
          </div>
          <div key="b" className="ReactNippleLayout">
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
          </div>
          <div key="c" className="VideoWindowLiveVideoLayout">
            <VideoWindowLiveVideo
            />
          </div>
          <div key="d" className="VideoWindowPlannerLayout">
            <VideoWindowPlanner
            />
          </div>
          </GridLayout>
        </ROS>
      </div>
    );
  }
}

export default App;