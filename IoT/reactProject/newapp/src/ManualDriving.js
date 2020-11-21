import React from 'react';
import GridLayout from 'react-grid-layout';
import { ROS } from './ROS';
import ReactNippleExample from "./ReactNippleExample";
import { ToggleConnect } from './ToggleConnect';
import { VideoWindowLiveVideo } from './VideoWindowLiveVideo';


export function ManualButton() {
    const layout = [
        {i: 'a', x: 0, y: 0, w: 0.5, h: 1,static: true},
        {i: 'b', x: 0.7, y: 0, w: 0.5, h: 1,static: true},
        {i: 'c', x: 1.6, y: 0, w: 0.5, h: 1,static: true},
        {i: 'd', x: 2.7, y: 0, w: 0.5, h: 1,static: true},
        {i: 'e', x: 3.6, y: 0, w: 0.5, h: 1,static: true},
        {i: 'f', x: 4.35, y: 0, w: 0.5, h: 1,static: true},
        {i: 'g', x: 5.55, y: 0, w: 0.5, h: 1,static: true},
        {i: 'h', x: 0.3, y: 1, w: 5, h: 2,static: true},
        {i: 'i', x: 0.3, y: 3, w: 5, h: 5,static: true},
        {i: 'j', x: 5.5, y: 1, w: 2, h: 2,static: true}
      ];
      
    
      return (
        <ROS>
            <div className="ManualLayout">
                <GridLayout className="manuallayout" layout={layout} cols={12} rowHeight={30} width={1200}>
                    <div key="a" className="Reset">
                        <button>Reset</button>
                    </div>
                    <div key="b" className="GetState">
                        <button>GetState</button>
                    </div>
                    <div key="c" className="LockCamera">
                        <button>LockCamera</button>
                    </div>
                    <div key="d" className="InputBox">
                        <button>InputBox</button>
                    </div>
                    <div key="e" className="ParkIn">
                        <button>ParkIn</button>
                    </div>
                    <div key="f" className="CabMoveOutt">
                        <button>CabMoveOutt</button>
                    </div>
                    <div key="g" className="ReturnHome">
                        <button>ReturnHome</button>
                    </div>
                    <div key="h" className="ToggleConnectLayout">
                        <ToggleConnect />
                    </div>
                    <div key="i" className="ReactNippleLayout">
                        <ReactNippleExample
                        title="Manual Driving Controller"
                        width={400}
                        height={400}
                        options={{ 
                            mode: "static", 
                            color: "red",
                            position: { top: "50%", left: "50%" },
                            size: 200 
                        }}
                        />
                    </div>
                    <div key="j" className="VideoWindowLiveVideoLayout">
                        <VideoWindowLiveVideo/>
                    </div>
                </GridLayout>
            </div>
        </ROS>
      );

}
export default ManualButton


