import React from 'react';
import GridLayout from 'react-grid-layout';
//import { ROS } from './ROS';
import ReactNippleExample from "./ReactNippleExample";
import { ToggleConnect } from './ToggleConnect';
import { VideoWindowLiveVideo } from './VideoWindowLiveVideo';
import { useMediaQuery } from 'react-responsive';
import Button from 'react-bootstrap/Button';

export function ManualPage() {
    
    const Portrait = ({ children }) => {
        const isPortrait = useMediaQuery({ orientation: 'portrait' })
        return isPortrait ? children : null
    }

    const Landscape = ({ children }) => {
        const isLandscape = useMediaQuery({ orientation: 'landscape' })
        return isLandscape ? children : null
    }

    const layout = [
        {i: 'a3', x: 0, y: 7, w: 0.5, h: 1,static: true},
        {i: 'b3', x: 0.7, y: 8, w: 0.5, h: 1,static: true},
        {i: 'c3', x: 1.6, y: 8, w: 0.5, h: 1,static: true},
        {i: 'd3', x: 2.7, y: 8, w: 0.5, h: 1,static: true},
        {i: 'e3', x: 0, y: 9, w: 0.5, h: 1,static: true},
        {i: 'f3', x: 0.7, y: 9, w: 0.5, h: 1,static: true},
        {i: 'g3', x: 1.9, y: 9, w: 0.5, h: 1,static: true},
        {i: 'h3', x: 0, y: 10, w: 5, h: 2,static: true},
        {i: 'i3', x: 2.5, y: 0, w: 5, h: 5,static: true},
        {i: 'j3', x: 5.5, y: 0, w: 2, h: 2,static: true},
        {i: 'a4', x: 0, y: 0, w: 0.5, h: 1,static: true},
        {i: 'b4', x: 0.7, y: 0, w: 0.5, h: 1,static: true},
        {i: 'c4', x: 1.6, y: 0, w: 0.5, h: 1,static: true},
        {i: 'd4', x: 2.7, y: 0, w: 0.5, h: 1,static: true},
        {i: 'e4', x: 3.6, y: 0, w: 0.5, h: 1,static: true},
        {i: 'f4', x: 4.35, y: 0, w: 0.5, h: 1,static: true},
        {i: 'g4', x: 5.55, y: 0, w: 0.5, h: 1,static: true},
        {i: 'h4', x: 0.3, y: 8, w: 5, h: 2,static: true},
        {i: 'i4', x: 0.3, y: 1, w: 5, h: 5,static: true},
        {i: 'j4', x: 5.5, y: 1, w: 2, h: 2,static: true},
    ];
      
    
    return (
        
        <div>
            <Portrait>
                <GridLayout className="manuallayout" layout={layout} cols={10} rowHeight={20} width={1000}>
                   
                    <div key="a3" className="Reset">
                        <Button variant="danger" size="sm">Reset</Button>
                      
                    </div>
                    <div key="b3" className="GetState">
                        <button>GetState</button>
                    </div>
                    <div key="c3" className="LockCamera">
                        <button>LockCamera</button>
                    </div>
                    <div key="d3" className="InputBox">
                        <button>InputBox</button>
                    </div>
                    <div key="e3" className="ParkIn">
                        <button>ParkIn</button>
                    </div>
                    <div key="f3" className="CabMoveOutt">
                        <button>CabMoveOutt</button>
                    </div>
                    <div key="g3" className="ReturnHome">
                        <button>ReturnHome</button>
                    </div>
                    <div key="h3" className="ToggleConnectLayout">
                        <ToggleConnect />
                    </div>
                    <div key="i3" className="ReactNippleLayout">
                        <ReactNippleExample
                        title="Manual Controller"
                        width={150}
                        height={150}
                        options={{ 
                            mode: "static", 
                            color: "red",
                            position: { top: "50%", left: "50%" },
                            size: 100 
                        }}
                        />
                    </div>
                    {/* <div key="j3" className="VideoWindowLiveVideoLayout">
                        <VideoWindowLiveVideo
                            width={150}
                            height={150}
                        />
                    </div> */}
                </GridLayout>
            </Portrait>

            <Landscape>
                <GridLayout className="manuallayout" layout={layout} cols={12} rowHeight={20} width={1200}>
                    <div key="a4" className="Reset">
                        <Button variant="danger" size="lg">Reset</Button>
                    </div>
                    <div key="b4" className="GetState">
                        <button>GetState</button>
                    </div>
                    <div key="c4" className="LockCamera">
                        <button>LockCamera</button>
                    </div>
                    <div key="d4" className="InputBox">
                        <button>InputBox</button>
                    </div>
                    <div key="e4" className="ParkIn">
                        <button>ParkIn</button>
                    </div>
                    <div key="f4" className="CabMoveOutt">
                        <button>CabMoveOutt</button>
                    </div>
                    <div key="g4" className="ReturnHome">
                        <button>ReturnHome</button>
                    </div>
                    <div key="h4" className="ToggleConnectLayout">
                        <ToggleConnect />
                    </div>
                    <div key="i4" className="ReactNippleLayout">
                        <ReactNippleExample
                        title="Manual Controller"
                        width={150}
                        height={150}
                        options={{ 
                            mode: "static", 
                            color: "red",
                            position: { top: "50%", left: "50%" },
                            size: 100 
                        }}
                        />
                    </div>
                    <div key="j4" className="VideoWindowLiveVideoLayout">
                        <VideoWindowLiveVideo
                            width={150}
                            height={150}
                        />
                    </div>
                </GridLayout>
            </Landscape>
        </div>
        
    );

}

export default ManualPage


