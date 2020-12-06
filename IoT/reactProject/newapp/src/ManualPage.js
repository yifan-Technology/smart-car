import React from 'react';
import GridLayout from 'react-grid-layout';
// import { ROS } from './ROS';
import ReactNippleExample from "./ReactNippleExample";
// import { ToggleConnect } from './ToggleConnect';
// import { VideoWindowLiveVideo } from './VideoWindowLiveVideo';
import { useMediaQuery } from 'react-responsive';
import Button from 'react-bootstrap/Button';
import { useState, useEffect } from 'react';

// function getWindowDimensions() {
//     const { innerWidth: width, innerHeight: height } = window;
//     return {
//       width,
//       height
//     };
// }

// if mobil phone return sm; if Laptop return lg
function ButtonSize(){
    const { innerWidth: width} = window;
    if(width<512){
        return "sm"
    }
    else if(width>1024){
        return "lg"
    }
    else {
        return ""
    }
}

function useWindowSize() {
    // Initialize state with undefined width/height so server and client renders match
    // Learn more here: https://joshwcomeau.com/react/the-perils-of-rehydration/
    const [windowSize, setWindowSize] = useState({
      width: 2100,
      height: 100,
    });
  
    useEffect(() => {
      // Handler to call on window resize
      function handleResize() {

        // Set window width/height to state
        setWindowSize({
          width: window.innerWidth,
          height: window.innerHeight,
        });
      }
      
      // Add event listener
      window.addEventListener("resize", handleResize);
      
      // Call handler right away so state gets updated with initial window size
      handleResize();
      
      // Remove event listener on cleanup
      return () => window.removeEventListener("resize", handleResize);
    }, []); // Empty array ensures that effect is only run on mount
  
    return windowSize.width/2100;
  }

function ButtonLayout(){
    var i;
    const layout = [];
    for( i=0; i<8; i++) {
        layout[i] = {i:'a'+i, x: i*2, y:0, w:1, h:1, static: true }
    }
    return layout
}

export function ManualPage() {
    const layout = ButtonLayout();
    //layout[100] = {i:'a'+i, x: i*2, y:0, w:1, h:1, static: true }
    console.log(layout);
    const [btz] = ButtonSize();
    const scalingratio = useWindowSize();
    const NarrowScreen = ({ children }) => {
        const isPortrait = useMediaQuery({ orientation: 'portrait' })
        return isPortrait ? children : null
    }

    const WideScreen = ({ children }) => {
        const isLandscape = useMediaQuery({ orientation: 'landscape' })
        return isLandscape ? children : null
    }
    
    return (
        <body>
            <WideScreen >
                <GridLayout className="widescreen" layout={layout} cols={50} rowHeight={22} width={5000}>
                        <div key={layout[0].i} className="Reset">
                            <Button variant="danger" size={btz}>Reset</Button>
                        </div>
                        <div key={layout[1].i} className="GetState">
                            <Button variant="primary" size={btz}>GetState</Button>
                        </div>
                        <div key={layout[2].i} className="LockCamera">
                            <Button variant="primary" size={btz}>LockCamera</Button>
                        </div>
                        <div key="e3" className="ReactNippleLayout">
                        <ReactNippleExample
                            title="JOY"
                            width={400*scalingratio}
                            height={400*scalingratio}
                            options={{ 
                            mode: "static", 
                            color: "red",
                            position: { top: "50%", left: "50%" },
                            size: 200*scalingratio
                            }}
                        />
                        </div>     
                </GridLayout>
            </WideScreen>
            <NarrowScreen>
                <GridLayout className="narrowscreen" layout={layout} cols={50} rowHeight={22} width={2100*scalingratio}>
                    <div key={layout[0].i} className="Reset">
                        <Button variant="danger" size={btz}>Reset</Button>
                    </div>
                    <div key={layout[1].i} className="GetState">
                        <Button variant="primary" size={btz}>GetState</Button>
                    </div>
                    <div key={layout[2].i} className="LockCamera">
                        <Button variant="primary" size={btz}>LockCamera</Button>
                    </div>
                    <div key={layout[3].i} className="InputBox">
                        <Button variant="primary" size={btz}>InputBox</Button>
                    </div>
                    <div key="e3" className="ReactNippleLayout">
                        <ReactNippleExample
                            title="JOY"
                            width={400*scalingratio}
                            height={400*scalingratio}
                            options={{ 
                            mode: "static", 
                            color: "red",
                            position: { top: "50%", left: "50%" },
                            size: 200*scalingratio
                            }}
                        />
                    </div>
                </GridLayout>
            </NarrowScreen>
        </body>      
    );
}
                   
    


export default ManualPage


