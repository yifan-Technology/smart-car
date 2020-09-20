import rclpy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
# from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int8
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped

# from geometry_msgs.msg import Twist 
import ros2_numpy

class YF_Node():
    def __init__(self,nodeName,name,msgType):
        self._node = rclpy.create_node(name)
        self.sub = self._node.create_subscription(
            msgType,
            nodeName,
            self.subscription,
            1)
        self.sub

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

class YF_Image_PY(YF_Node):    
    def __init__(self,nodeName, name):
        super().__init__(nodeName, name, Image)
        self.bridge = CvBridge()
        self._pubMsg = Image()
    def subscription(self, msg): 
        self._subMsg = self.bridge.imgmsg_to_cv2(msg, "bgra8") 
          
    def publishMsg(self,income):
        self._pubMsg = self.bridge.cv2_to_imgmsg(income, "bgra8")
        self._pubMsg.header.stamp = self.node.get_clock().now().to_msg()
        self.pub.publish(self._pubMsg)  

class YF_Image(YF_Node):    
    def __init__(self,nodeName, name):
        super().__init__(nodeName, name, Image)
        self.bridge = CvBridge()
        self._pubMsg = Image()
    def subscription(self, msg): 
        self._subMsg = self.bridge.imgmsg_to_cv2(msg, "bgr8") 
          
    def publishMsg(self,income):
        self._pubMsg = self.bridge.cv2_to_imgmsg(income, "bgr8")
        self._pubMsg.header.stamp = self.node.get_clock().now().to_msg()
        self.pub.publish(self._pubMsg)  

class YF_CompressedImage(YF_Node):    
    def __init__(self,nodeName, name):
        super().__init__(nodeName,name, CompressedImage)
        self._pubMsg = CompressedImage()

    def subscription(self, msg):         
        np_arr = np.fromstring(np.array(msg.data), np.uint8)
        self._subMsg = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
          
    def publishMsg(self,income):
        self._pubMsg.header.stamp = self.node.get_clock().now().to_msg()
        self._pubMsg.format = "jpeg"
        self._pubMsg.data = np.array(cv2.imencode('.jpg', income)[1]).tostring()
        self.pub.publish(self._pubMsg)  

class YF_PointCloud(YF_Node):    
    def __init__(self,nodeName, name):
        super().__init__(nodeName,name, PointCloud2)
        self._poc_array = None
        self._poc_image = None
    
    def subscription(self, msg):
        msg = ros2_numpy.numpify(msg)
        self._poc_array = ros2_numpy.point_cloud2.get_xyz_points(msg, remove_nans=True)
        self._poc_image = ros2_numpy.point_cloud2.get_xyz_points(msg, remove_nans=False)
        
    def publishMsg(self,income):        
        self._pubMsg = ros2_numpy.point_cloud2.array_to_pointcloud2(income)
        self._pubMsg.header.stamp = self.node.get_clock().now().to_msg()
        self.pub.publish(self._pubMsg)  

    @property
    def poc_image(self):
        return self._poc_image 
    @property
    def poc_array(self):
        return self._poc_array

class YF_ObjectsArray(YF_Node):
    def __init__(self,nodeName, name):
        super().__init__(nodeName,name, MarkerArray)

class YF_Goal(YF_Node):
    def __init__(self,nodeName, name):
        super().__init__(nodeName, name, PoseStamped)
        self._pubMsg = PoseStamped()
    
    def publishMsg(self,income):
        self.pub.publish(income)  
        

class YF_CostMap(YF_Node):
    def __init__(self,nodeName, name):
        super().__init__(nodeName,name, Int8MultiArray)
        self._pubMsg = Int8MultiArray()
    
    def subscription(self, msg):
        self._subMsg = msg.data
    def publishMsg(self,income):
        self._pubMsg.data = income.tolist()        
        self.pub.publish(self._pubMsg)  

class YF_SollSpeed(YF_Node):
    def __init__(self,nodeName, name):
        super().__init__(nodeName,name, Float32MultiArray)
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
    def __init__(self,nodeName, name):
        super().__init__(nodeName,name, Float32MultiArray)
        self._subMsg = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self._pubMsg = Float32MultiArray()

    def subscription(self, msg):
        self._subMsg = np.array(msg.data)
    def publishMsg(self,income):
        self._pubMsg.data = income
        self.pub.publish(self._pubMsg)  
  
class YF_ObjectFlag(YF_Node):
    def __init__(self,nodeName, name):
        super().__init__(nodeName,name, Int8)
        self._subMsg = Int8()
        self._subMsg.data = 127
        self._pubMsg = Int8()
        self._pubMsg.data = 127

    def subscription(self, msg):
        self._subMsg = np.array(msg.data)
    def publishMsg(self,income):
        self._pubMsg.data = income
        self.pub.publish(self._pubMsg)     
