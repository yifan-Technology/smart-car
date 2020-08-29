#-*- coding: UTF-8 -*-
import sys
import argparse

import rclpy
import pyzed.sl as sl

from target.YF_Camera import YF_Camera
from target.YF_ROS import Ros

PRINT = False
if PRINT:
    print("Finished import things")

def manageParser(parser):
    """储存变量以便后期使用

    默认变量：
    --units = sl.UNIT.CENTIMETER
    --fps = 15
    
    必须变量：
    --mode = {object", "object+depth", "cam_only"}

    """
    parser.add_argument("--units", default=sl.UNIT.CENTIMETER,
                        help="the unit for depth measurement")
    parser.add_argument("--mode", required=False, default="object+depth", 
                        choices=["object", "object+depth", "cam_only"],
                        help="the mode for the car")
    parser.add_argument("--fps", default=30,
                        help="frames per second")
    return parser

def main(argsRos=None):
    # 一些必要步骤来把变量集中到一个名称里面
    parser = argparse.ArgumentParser(description="Args for the car")
    manageParser(parser)
    args = parser.parse_args()
    if PRINT:
        print(args)


    car = YF_Camera(args)
    rclpy.init(args=None)        

    rosYF = Ros()
    target = rosYF.nodeLoc
    pc = rosYF.nodePC
    
    car.done(rosYF)
    rclpy.spin(target)
    rclpy.spin(pc)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    target.destroy_node()
    pc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    
    main()
    
    sys.exit(0)