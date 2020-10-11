import React from 'react';
import { useROS } from './ROS';

export function ToggleConnect() {
  
  const { isConnected, url, changeUrl, toggleConnection } = useROS();

  return (
    <div>
      <p>
        <b>Simple connect:  </b><button onClick={ toggleConnection }>Toggle Connect</button>  <br />
        <b>ROS url input:  </b><input name="urlInput" defaultValue={ url } onChange={event => changeUrl(event.target.value)} />  <br />
        <b>Status of ROS:</b> { isConnected ? "connected" : "not connected" }   <br />
      </p>
    </div>
  );
}

export default ToggleConnect;
