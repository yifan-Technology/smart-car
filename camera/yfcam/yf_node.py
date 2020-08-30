import rclpy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist 
from cv_bridge import CvBridge
import ros2_numpy

class SUB_RealSpeed_Main():
    def __init__(self):
        self._nodeReal = rclpy.create_node('RealSpeedSUB')
        self.subReal = self._nodeReal.create_subscription(
            Float32MultiArray,
            '/real_speed',
            self.real_callback,
            1)
        self.subReal # prevent unused variable warning

        self._real_speed = None

    def real_callback(self, msg):
        left_front,left_front_angle,right_front,right_front_angle,left_back,left_back_angle,right_back,right_back_angle = msg.data
        #self._real_speed = np.array([(left_front+left_back)/2,(right_front+right_back)/2])
        self._real_speed = np.array([left_front,left_front_angle,right_front,right_front_angle,left_back,left_back_angle,right_back,right_back_angle])
    
    @property
    def nodeReal(self):
        return self._nodeReal 
    @property
    def real_speed(self):
        return self._real_speed

class PUB_RealSpeed_Serial():
    def __init__(self):
        self._nodeReal = rclpy.create_node('RealSpeedPUB')
        self.pubReal = self._nodeReal.create_publisher(
            Float32MultiArray,
            '/real_speed',
            1)
        self.pubReal  # prevent unused variable warning       

    def real_publish(self,real_speed):
        msg = Float32MultiArray()
#        print(type(np.array(real_speed).astype(np.float)))
        msg.data = real_speed
        self.pubReal.publish(msg)
    
    @property
    def nodeReal(self):
        return self._nodeReal  

    @property
    def real_speed(self):
        return self._real_speed


class SUB_SollSpeed_Serial():
    def __init__(self):
        self._nodeSoll = rclpy.create_node('SollSpeedSUB')
        self.subSoll = self._nodeSoll.create_subscription(
            Float32MultiArray,
            '/soll_speed',
            self.soll_callback,
            1)
        self.subSoll # prevent unused variable warning

        self._soll_speed = [0.0,0.0,0.0,0.0]

    def soll_callback(self, msg):
        print("msg.data: ",msg.data)
        left_front,right_front,left_back,right_back = msg.data
        self._soll_speed = np.array([left_front,right_front,left_back,right_back])
    
    @property
    def nodeSoll(self):
        return self._nodeSoll 
    @property
    def soll_speed(self):
        return self._soll_speed

class PUB_SollSpeed_Main():
    def __init__(self):
        self._nodeSoll = rclpy.create_node('SollSpeedPUB')
        self.pubSoll = self._nodeSoll.create_publisher(
            Float32MultiArray,
            '/soll_speed',
            1)
        self.pubSoll  # prevent unused variable warning       

    def soll_publish(self,wheel_speed):
#        lf,rf,lb,rb = wheel_speed
#        output = [lf,rf,lb,rb]
        msg = Float32MultiArray()
        msg.data = wheel_speed
        self.pubSoll.publish(msg)
    
    @property
    def nodeSoll(self):
        return self._nodeSoll  

    @property
    def soll_speed(self):
        return self._soll_speed

# class RealSpeed():
#     def __init__(self):
#         self._nodeReal = rclpy.create_node('RealSpeed')
#         self.subReal = self._nodeReal.create_subscription(
#             Twist,
#             '/real_speed',
#             self.real_callback,
#             1)
#         self.subReal # prevent unused variable warning

#         self._real_speed = None

#     def real_callback(self, msg):
#         left_front = msg.linear.x
#         right_front = msg.linear.y
#         left_back = msg.angular.x
#         right_back = msg.angular.y
#         self._real_speed = np.array([(left_front+left_back)/2,(right_front+right_back)/2])
    
#     @property
#     def nodeReal(self):
#         return self._nodeReal 
#     @property
#     def real_speed(self):
#         return self._real_speed

# class SollSpeed():
#     def __init__(self):
#         self._nodeSoll = rclpy.create_node('SollSpeed')
#         self.pubSoll = self._nodeSoll.create_publisher(
#             Twist,
#             '/Soll_speed',
#             1)
#         self.pubSoll  # prevent unused variable warning       

#     def Soll_callback(self,wheel_speed):
#         lf,rf,lb,rb = wheel_speed
#         msg = Twist()
#         msg.linear.x = lf
#         msg.linear.y = rf
#         msg.angular.x = lb
#         msg.angular.y = rb
#         self.pubSoll.publish(msg)
    
#     @property
#     def nodeSoll(self):
#         return self._nodeSoll  

#     @property
#     def soll_speed(self):
#         return self._soll_speed
    

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


class GoalSub():
    def __init__(self):
        self._nodeGoal = rclpy.create_node('goal')
        self.subGoal = self._nodeGoal.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            1)
        self.subGoal  # prevent unused variable warning

        self._goal = None

    def goal_callback(self,msg):
        self._goal = msg
    
    @property
    def nodeGoal(self):
        return self._nodeGoal     
    @property
    def goal(self):
        return self._goal 
        
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
