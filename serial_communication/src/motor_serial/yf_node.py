import rclpy
import queue
import numpy as np
from std_msgs.msg import Float32MultiArray


class YF_Node():
    def __init__(self,nodeName,name,msgType,nodeType):
        self._node = rclpy.create_node(name)
        if nodeType == "sub":
            self.sub = self._node.create_subscription(
                msgType,
                nodeName,
                self.subscription,
                1)
            self.sub
        if nodeType == "pub":
            self.pub = self._node.create_publisher(
                msgType,
                nodeName,
                1)
            self.pub # prevent unused variable warning
        self._subMsg = None
        self._pubMsg = None

    def subscription(self, msg):
        self._subMsg = msg
    def publishMsg(self,income):
        self.pub.publish(income)  

    @property
    def node(self):
        return self._node 
    @property
    def subMsg(self):
        return self._subMsg
    @property
    def pubMsg(self):
        return self._pubMsg

class YF_SollSpeed(YF_Node):
    def __init__(self,nodeName, name,nodeType):
        super().__init__(nodeName,name, Float32MultiArray,nodeType)
        self._subMsg = [0.0,0.0,0.0,0.0]
        self._pubMsg = Float32MultiArray()
        self.write_queue = queue.Queue()

    def subscription(self, msg):
        self._subMsg = np.array(msg.data)
        self.write_queue.put(self._subMsg)

    def publishMsg(self,income):
        self._pubMsg.data = income
        self.pub.publish(self._pubMsg)  

class YF_RealSpeed(YF_Node):
    def __init__(self,nodeName, name,nodeType):
        super().__init__(nodeName,name, Float32MultiArray,nodeType)
        self._subMsg = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self._pubMsg = Float32MultiArray()

    def subscription(self, msg):
        self._subMsg = np.array(msg.data)
    def publishMsg(self,income):
        self._pubMsg.data = income
        self.pub.publish(self._pubMsg)  
  