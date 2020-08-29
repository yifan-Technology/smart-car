import numpy as np
import cv2
import copy
import rclpy

import yf_humanDetect
import yf_pattenRecognition
import yf_costmap

import yaml

def init():    
    # 初始化模板匹配
    global TemMatch 
    # 初始化模式识别
    global FeaSpace 
    # 初始化损耗地图
    global CostMap 
    
    global showImg
    
    with open("/home/yf/yifan/config.yaml","r") as f:
        config=yaml.load(f)
        
    
    # 初始化模板匹配
    tSize = config["humanDetect"]["tSize"]
    TemMatch = yf_humanDetect.Template_Matching(tSize=tSize)
    # 初始化模式识别
    capasity = config["pR"]["capasity"]
    feaNum = config["pR"]["featureNum"]
    FeaSpace = yf_pattenRecognition.Feature_Space(capasity=capasity,featureNum=feaNum)
    # 初始化损耗地图
    mapSize = config["costMap"]["mapSize"]
    visSize = config["costMap"]["visSize"]
    h_self = config["costMap"]["h_self"]
    h_top = config["costMap"]["h_top"]
    feld = config["costMap"]["feld"]
    CostMap = yf_costmap.costMap(mapSize=mapSize,visSize=visSize,h_self=h_self,
                                 h_top=h_top,feld=feld)
    showImg = [config["showImg"]["ObMap"],
               config["showImg"]["CostMap"],
               config["showImg"]["LiveVideo"],
               config["showImg"]["PointCloud"]]

def runCam(cam,obj,poc,goal):   
        target = None
        # 读取 图像，物体，点云信息
        image_np = cam.image_data
        object_array = obj.obj_array  
#        point_cloud = poc.poc_image
        pcl = poc.poc_array
        
#         检测是否接收到所有文件
        if image_np is None:
            print("Waiting for new Image")
            return             
        if object_array is None:
            print("Waiting for new Object")
            return
        if pcl is None:
            print("Waiting for new Point Cloud")
            return   
    
        person = None
        cache_acc = 100.6859
        image_origin = copy.copy(image_np)
        for objects in object_array.markers:
            top_left = (int(objects.points[0].x), int(objects.points[0].y))
            down_right = (int(objects.points[2].x), int(objects.points[2].y))            
            cv2.rectangle(image_np, top_left, down_right, (255,0,255), 3)
            temAcc,aspRatio = TemMatch.new_template(image_origin,(top_left,down_right))
            
            distance = np.sqrt(np.sum(objects.pose.position.x**2 
                                      + objects.pose.position.y**2
                                      + objects.pose.position.z**2))
            
            acc = FeaSpace.predict([temAcc,objects.pose.position.x,objects.pose.position.y,objects.pose.position.z])
            
#            print(acc)
            if cache_acc > acc:
                person = [top_left,down_right,objects,[temAcc, distance]]
                

        if person is not None:
            goal.goal.header = person[2].header
            goal.goal.pose = person[2].pose
            
            FeaSpace.push(person[3])
            TemMatch.new_template(image_origin,(person[0],person[1]))
            TemMatch.set_tem()
            target = np.array([person[2].pose.position.y+2.5,person[2].pose.position.x])
            goal.goal_callback()
            rclpy.spin_once(goal.nodeGoal,timeout_sec=0.05)
            cv2.rectangle(image_np, person[0], person[1], (0,0,255), 3)
            cv2.circle(image_np, (int((person[0][0] + person[1][0]) / 2),
                                int((person[0][1] + person[1][1]) / 2)), 
                                2, (0,0,255), 3)
        
        
        CostMap.pointCloud2map(pcl)
        testmap = CostMap.visualMap()
        obMap = CostMap.obstaclemap()
        
        if showImg[0]:
            cv2.imshow("Send Map",obMap) 
        if showImg[1]:
            cv2.imshow("Cost Map",testmap)  
        if showImg[2]:
            cv2.imshow("Live Video",image_np) 
        if showImg[3]:
            point_cloud = poc.poc_image
            cv2.imshow("Point Cloud",point_cloud)
        
        return target,obMap,CostMap,testmap
        
