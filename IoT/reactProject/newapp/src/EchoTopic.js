import React, { useState, useEffect } from 'react';
import { useROS } from './ROS';

var listener = null;

export function EchoTopic() {
  // eslint-disable-next-line
  const { createListener, topics, isConnected, listeners } = useROS(); 
  // eslint-disable-next-line
  const [ lastMsg, setLastMsg ] = useState(''); 
  const [ topic, setTopic ] = useState('/');
  const [ queue, setQueue ] = useState(0);
  const [ compression, setCompression ] = useState('none');

  useEffect(() => {
    handleTopic(topic);
  });

  const unsubscribe = () => {
    if (listener) {
      console.log("Unsubscribing");
      listener.unsubscribe();
    }
  };

  const handleTopic = (topicInput) => {
    if (topic !== topicInput) {
      setTopic(topicInput);
      unsubscribe();
      return;
    }

    unsubscribe();
    listener = null;
    console.log('before topic input');
    var i = null;
    console.log(topics);

    for (i in topics) {
      console.log('topic input begin');

      if (topics[i].path === topicInput) {
        listener = createListener( topics[i].path,
                                   topics[i].msgType,
                                   Number(queue),
                                   compression);
        break;
      }
      console.log('topic input end');
    }

    if (listener) {
      console.log("Subscribing to messages...");
      listener.subscribe(handleMsg);
    } else {
      console.log("Topic '" + topic + "' not found...make sure to input the full topic path - including the leading '/'");
    }
  };

  const handleQueue = (queueInput) => {
    setQueue(queueInput);
  };

  const handleCompression = (compInput) => {
    setCompression(compInput);
  };

  const handleMsg = (msg) => {
    console.log(msg);
  };

  return (
    <div>
      <b>Message Queue Length:  </b><input name="queueInput" defaultValue={ queue } onChange={event => handleQueue(event.target.value)} />  <br />
      <b>Compression:  </b><input name="compInput" defaultValue={ compression } onChange={event => handleCompression(event.target.value)} />  <br />
      <b>Topic to echo:  </b><input name="topicInput" defaultValue={ topic } onChange={event => handleTopic(event.target.value)} />  <br />
    </div>
  );
}

export default EchoTopic;
