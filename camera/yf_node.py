import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import ros2_numpy


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