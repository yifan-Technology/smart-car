import rclpy
from rclpy.node import Node

import pyzed.sl as sl
import cv2
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import cv2

from matplotlib import pyplot as plt
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

class LeftCam():
    def __init__(self):
        self._nodeImg = rclpy.create_node('image')
        self.subImg = self._nodeImg.create_subscription(
            Image,
            '/zed2/zed_node/left/image_rect_color',
            self.img_callback,
            10)
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

class Object_Detection:
    """物体识别初始化

    cam = zed cam类
    objects = 物体
    obj_runtime_params = obj_runtime_params
    """
    def __init__(self, cam, objects, obj_runtime_params):
        self.cam = cam
        self.objects = objects
        self.obj_runtime_params = obj_runtime_params

    def get_color_id_gr(self, idx):
        # 五种颜色，用来给Object画框框
        id_colors = [(59, 232, 176),
                     (25,175,208),
                     (105,102,205),
                     (255,185,0),
                     (252,99,107)]
        color_idx = idx % 5
        arr = id_colors[color_idx]
        return arr

    def run(self):
        res = []
        self.cam.retrieve_objects(self.objects, self.obj_runtime_params)
        obj_array = self.objects.object_list
        for obj in obj_array:
            bbox = obj.bounding_box_2d
            top_left = (int(bbox[0, 0]), int(bbox[0, 1]))
            down_right = (int(bbox[2, 0]), int(bbox[2, 1]))
            res.append( (top_left, down_right, self.get_color_id_gr(int(obj.id)) ))
        if res == []:
            return False
        return res

    def go(self, img):
        key = ""
        while key != 113: # 113 就是键盘上的"q"， 也就是说，按q就停止
            start = time.time()
            res = self.run()
            
            for (top_left,down_right,color) in res:
                cv2.rectangle(image_data, top_left, down_right, 
                                        color, 3)
                cv2.circle(image_data, (int((top_left[0] + down_right[0]) / 2),
                                            int((top_left[1] + down_right[1]) / 2)), 
                                                2, (0,0,255), 3)                
        

def main(args=None):
    rclpy.init(args=args)

    camera = sl.Camera()
    c = sl.Objects()
    obj_runtime_params = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_params.detection_confidence_threshold = 40
    od = Object_Detection(camera, objects, obj_runtime_params)

    cam = LeftCam()
    od = GoogleObjectDetection(cam)
    od.go(cam.image_data)

    cam.nodeImg.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
