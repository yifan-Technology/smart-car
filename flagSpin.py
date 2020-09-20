
import rclpy
import numpy as np
from std_msgs.msg import Int8

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

class YF_ObjectFlag(YF_Node):
    def __init__(self,nodeName, name,nodeType):
        super().__init__(nodeName,name, Int8,nodeType)
        self._subMsg = Int8()
        self._subMsg.data = 127
        self._pubMsg = Int8()
        self._pubMsg.data = 127

    def subscription(self, msg):
        self._subMsg = np.array(msg.data)
    def publishMsg(self,income):
        self._pubMsg.data = income
        self.pub.publish(self._pubMsg)     

def main():
    rclpy.init()
    flagSub = YF_ObjectFlag("/yf_camera/flag",'ObjectFlagSub',"sub")
    flagPub = YF_ObjectFlag("/yf_camera/flag",'ObjectFlagPub',"pub")
    rclpy.spin_once(flagSub.node,timeout_sec=0.001)
    rclpy.spin_once(flagPub.node,timeout_sec=0.001)  
    # flag节点初始化
    flagPub.publishMsg(101)    
    while 1:
        rclpy.spin_once(flagSub.node,timeout_sec=0.001)
        flag = flagSub.subMsg.data
        if type(flag) != int:
            flag = flag.tolist()
        print("flag:",flag)
        flagPub.publishMsg(flag)    
        rclpy.spin_once(flagPub.node,timeout_sec=0.001)  