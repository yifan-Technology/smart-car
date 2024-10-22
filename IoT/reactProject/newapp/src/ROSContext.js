import React, { createContext, useState } from 'react';
import PropTypes from 'prop-types';

const rosObj = {
  ROS: null,
  url: "ws://192.168.8.214:9090",
  isConnected: false,
  autoconnect: false,
  topics: [],
  listeners: [],
};

const ROSContext = createContext([{}, () => {}]);

const ROSProvider = (props) => {
  const [ ros, setROS ] = useState(rosObj);
  return (
    <ROSContext.Provider value={[ros, setROS]}>
      {props.children}
    </ROSContext.Provider>
  );
};

ROSProvider.propTypes = {
  children: PropTypes.node.isRequired,
};

export { ROSContext, ROSProvider };
