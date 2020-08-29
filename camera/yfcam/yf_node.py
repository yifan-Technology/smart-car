import rclpy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist 
from cv_bridge import CvBridge
import ros2_numpy

class SollSpeed():
    def __init__(self):
        self._nodeSoll = rclpy.create_node('SollSpeed')
        self.subSoll = self._nodeSoll.create_subscription(
            Twist,
            '/soll_speed',
            self.soll_callback,
            1)
        self.subSoll# prevent unused variable warning

        self._soll_speed = None

    def real_callback(self, msg):
        left_front = msg.linear.x
        right_front = msg.linear.y
        left_back = msg.angular.x
        right_back = msg.angular.y
        self._soll_speed = np.array([(left_front+left_back)/2,(right_front+right_back)/2])
    
    @property
    def nodePoc(self):
        return self._nodePoc    
    @property
    def soll_speed(self):
        return self._soll_speed

class RealSpeed():
    def __init__(self):
        self._nodeReal = rclpy.create_node('RealSpeed')
        self.pubReal = self._nodeReal.create_publisher(
            Twist,
            '/real_speed',
            1)
        self.pubReal  # prevent unused variable warning       

    def cost_callback(self,wheel_speed):
        lf,rf,lb,rb = wheel_speed
        msg = Twist()
        msg.linear.x = lf
        msg.linear.y = rf
        msg.angular.x = lb
        msg.angular.y = rb
        self.pubReal.publish(msg)
    
    @property
    def nodeReal(self):
        return self._nodeReal  
    

class PointCloud():
    def __init__(self):
        self._nodePoc = rclpy.create_node('PointCloud')
        self.subPoc = self._nodePoc.create_subscription(
            PointCloud2,
            '/zed2/zed_node/point_cloud/cloud_registered',
            self.poc_callback,
            1)
        self.subPoc# prevent unused variable warning

        self._poc_array = None
        self._poc_image = None

    def poc_callback(self, msg):
        msg = ros2_numpy.numpify(msg)
        self._poc_array = ros2_numpy.point_cloud2.get_xyz_points(msg, remove_nans=True)
        self._poc_image = ros2_numpy.point_cloud2.get_xyz_points(msg, remove_nans=False)

    
    @property
    def nodePoc(self):
        return self._nodePoc    
    @property
    def poc_image(self):
        return self._poc_image 
    @property
    def poc_array(self):
        return self._poc_array

class ObjectsArray():
    def __init__(self):
        self._nodeObj = rclpy.create_node('objects')
        self.subObj = self._nodeObj.create_subscription(
            MarkerArray,
            '/zed2/zed_node/obj_det/objects_yf',
            self.obj_callback,
            1)
        self.subObj# prevent unused variable warning

        self._obj_array = None

    def obj_callback(self, msg):
        self._obj_array = msg

    
    @property
    def nodeObj(self):
        return self._nodeObj    
    @property
    def obj_array(self):
        return self._obj_array
    
    
class LeftCam():
    def __init__(self):
        self._nodeImg = rclpy.create_node('image')
        self.subImg = self._nodeImg.create_subscription(
            Image,
            '/zed2/zed_node/left/image_rect_color',
            self.img_callback,
            1)
        self.subImg  # prevent unused variable warning

        self.bridge = CvBridge()
        self._image_data = None

    def img_callback(self, msg):
        self._image_data = self.bridge.imgmsg_to_cv2(msg, "bgr8") 
    
    @property
    def nodeImg(self):
        return self._nodeImg    
    @property
    def image_data(self):
        return self._image_data 

class GoalPub():
    def __init__(self):
        self._nodeGoal = rclpy.create_node('goal')
        self.pubGoal = self._nodeGoal.create_publisher(
            PoseStamped,
            '/move_base_simple/goal',
            1)
        self.pubGoal  # prevent unused variable warning

        self._goal = PoseStamped()

    def goal_callback(self):
        self.pubGoal.publish(self._goal)
    
    @property
    def nodeGoal(self):
        return self._nodeGoal   
    
    @property
    def goal(self):
        return self._goal 
    @goal.setter
    def goal(self, incoming):
        self._goal = incoming
        
class CostMap():
    def __init__(self):
        self._nodeCost = rclpy.create_node('costmap')
        self.pubCost = self._nodeCost.create_publisher(
            Int8MultiArray,
            '/yf_camera/costmap',
            1)
        self.pubCost  # prevent unused variable warning

        self._cost = Int8MultiArray()

    def cost_callback(self):
        self.pubCost.publish(self._cost)
    
    @property
    def nodeCost(self):
        return self._nodeCost  
    
    @property
    def cost(self):
        return self._cost 
    @cost.setter
    def cost(self, incoming):
        self._cost = incoming