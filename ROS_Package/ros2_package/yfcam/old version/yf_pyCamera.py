# #-*- coding: UTF-8 -*-
# import pyzed.sl as sl
# import copy
# import cv2
# import numpy as np
# import rclpy
# import yf_node

# import yf_humanDetect
# import yf_pattenRecognition
# import yaml
# import time
# from yf_pyCostmap import costMap
# from yf_pyCostmap import get_obMap
# from yf_pyCostmap import PointCloud2_Img2Array

# import threading
# from queue import Queue

# class YF_Zed2:

#     def __init__(self):
#         # 初始化关于摄像头的参数
#         self.zed2 = sl.Camera()
#         self.init_params = sl.InitParameters()
#         self.init_params.camera_resolution = sl.RESOLUTION.VGA
#         self.init_params.camera_fps = 15 # default 15
#         self.init_params.coordinate_units = sl.UNIT.METER
#         self.init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
#         _ = self.zed2.open(self.init_params)
    
#         self.mat = sl.Mat()
#         self.objects = sl.Objects()
#         self.point_cloud = sl.Mat()
    
#         self.runtime_params = sl.RuntimeParameters()
#         self.runtime_params.sensing_mode = sl.SENSING_MODE.STANDARD    
#         self.obj_param = sl.ObjectDetectionParameters()
#         self.obj_param.enable_tracking = False
#         self.zed2.enable_object_detection(self.obj_param)
#         self.obj_runtime_params = sl.ObjectDetectionRuntimeParameters()
#         self.obj_runtime_params.detection_confidence_threshold = 40        
        
#     @property
#     def liveImage(self):
#         return self.mat.get_data()
#     @property
#     def pointCloud(self):
#         return self.point_cloud.get_data()
#     @property
#     def objectArray(self):
#         return self.objects.object_list
        
#     def getData(self):           
#         if self.zed2.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
#             # 获取一张左眼图片到args.mat里面去
#             self.zed2.retrieve_image(self.mat, sl.VIEW.LEFT)            
#             self.zed2.retrieve_measure(self.point_cloud, sl.MEASURE.XYZ)
#             self.zed2.retrieve_objects(self.objects, self.obj_runtime_params)
#         else: 
#             print("where is the camera")

# def init():    
#     # 全局化损耗地图
#     global CostMap 
#     # 全局化模板匹配
#     global TemMatch 
#     # 全局化模式识别
#     global FeaSpace 
#     # 全局化图像展示
#     global showImg
#     # 全局化节点名称
#     global nodeName
#     global FPS
#     global patient
#     patient = 0
#     # 读取yaml文件
#     with open("/home/yf/yifan/config.yaml","r") as f:
#         config=yaml.load(f)        
#     # 读取模板匹配参数
#     tSize = config["humanDetect"]["tSize"]
#     TemMatch = yf_humanDetect.Template_Matching(tSize=tSize)
#     # 读取模式识别参数
#     capasity = config["pR"]["capasity"]
#     feaNum = config["pR"]["featureNum"]
#     FeaSpace = yf_pattenRecognition.Feature_Space(capasity=capasity, featureNum=feaNum)
#     # 读取损耗地图参数
#     mapSize = config["costMap"]["mapSize"]
#     visSize = config["costMap"]["visSize"]
#     h_self = config["costMap"]["h_self"]
#     h_top = config["costMap"]["h_top"]
#     feld = config["costMap"]["feld"]
#     CostMap = costMap(mapSize=mapSize,visSize=visSize,h_self=h_self,
#                                  h_top=h_top,feld=feld)
#     # 读取展示图像参数
#     showImg = [config["showImg"]["ObMap"],
#                config["showImg"]["CostMap"],
#                config["showImg"]["LiveVideo"],
#                config["showImg"]["PointCloud"]]
#     # 读取节点名称参数
#     nodeName = config["RosTopic"]
#     FPS = config["FPS"]["FPS"]

# def runObjectDetection(liveImage,objectArray):    
#     image_np = liveImage
#     object_array = objectArray
    
#     idx = 0
#     # 遍历所有目标
#     for objects in object_array:
#         idx += 1
#         # 计算左上以及右下角坐标
#         bbox = objects.bounding_box_2d
#         top_left = (int(bbox[0, 0]), int(bbox[0, 1]))
#         down_right = (int(bbox[2, 0]), int(bbox[2, 1])) 
#         # 绘制所有的目标框（无中心标定点）           
#         cv2.rectangle(image_np, top_left, down_right, (255,0,255), 3)
#         cv2.putText(image_np, "{}".format(idx),
#                     org=(int((top_left[0]+down_right[0])/2),int((top_left[1]+down_right[1])/2)),
#                         fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=2,
#                         color=(255,0,255),thickness=4)
#         # TODO mark every
#     return image_np,object_array
        
# def runTargetSet(obj,videoCache,goal):    
#     # 计算左上以及右下角坐标
#     bbox = obj.bounding_box_2d
#     top_left = (int(bbox[0, 0]), int(bbox[0, 1]))
#     down_right = (int(bbox[2, 0]), int(bbox[2, 1])) 
        
        
#     goal.pubMsg.pose.position.x = obj.position[0]
#     goal.pubMsg.pose.position.y = obj.position[1]
#     goal.pubMsg.pose.position.z = obj.position[2]
#     # temAcc,aspo = TemMatch.new_template(videoCache,(top_left,down_right))
    
#     # feature = [temAcc,aspo,objects.position[0],obj.position[2] ]
#     # if 
#     # FeaSpace.push(feature)
#     # TemMatch.set_tem()
#     # 广播节点信息，spin
#     goal.publishMsg(goal.pubMsg)
#     rclpy.spin_once(goal.node,timeout_sec=0.01)

# def runTrack(liveImage,objectArray,goal,flag): 
#     global patient
#     # 声明 & 获取： 目标、图像、物体信息
#     target = None
#     image_np = liveImage
#     object_array = objectArray  
#     # 声明中间变量 Person
#     person = None
#     # 确定全局阈值
#     stand_acc = 0.6859
#     # 图像复制（避免画框后阻碍获取模板）
#     image_origin = copy.copy(image_np)

#     # 遍历所有目标
#     for objects in object_array:
#         # 计算左上以及右下角坐标
#         bbox = objects.bounding_box_2d
#         top_left = (int(bbox[0, 0]), int(bbox[0, 1]))
#         down_right = (int(bbox[2, 0]), int(bbox[2, 1])) 
#         # 绘制所有的目标框（无中心标定点）           
#         cv2.rectangle(image_np, top_left, down_right, (255,0,255), 3)
#         # TODO：尝试使用RGB & all 四个通道进行模板匹配，获得4D特征空间
#         # 模板匹配，获得置信度
#         temAcc, aspo = TemMatch.new_template(image_origin,(top_left,down_right))        
#         # 将置信度，目标的XYZ位置放入特征空间，计算acc #0: left right  1:up down 2: back front
#         feature = [temAcc,objects.position[0],objects.position[2]]
#         acc = FeaSpace.predict(feature)        
#         # print(acc)
#         # 阈值判定
#         if stand_acc > acc:
#             person = [top_left,down_right,objects,feature]
            
#     # 若找到匹配目标后
#     if person is not None:
#         patient = 0
#         # 对应键值设置
#         goal.pubMsg.pose.position.x = person[2].position[0]
#         goal.pubMsg.pose.position.y = person[2].position[1]
#         goal.pubMsg.pose.position.z = person[2].position[2]
#         # 将特征空间 & 模板匹配 更新
#         # TODO: if acc > (1-stand_acc):
#         FeaSpace.push(person[3])
#         TemMatch.new_template(image_origin,(person[0],person[1]))
#         TemMatch.set_tem()
#         # 计算目标位置
#         target = np.array([-1 * person[2].position[0]-2.5,person[2].position[2]])
        
#         # 广播节点信息，spin
#         goal.publishMsg(goal.pubMsg)
#         rclpy.spin_once(goal.node,timeout_sec=0.01)
#         # 绘制目标框（含中心标定点）
#         cv2.rectangle(image_np, person[0], person[1], (0,0,255), 3)
#         cv2.circle(image_np, (int((person[0][0] + person[1][0]) / 2),
#                             int((person[0][1] + person[1][1]) / 2)), 
#                             2, (0,0,255), 3)  
                            
#         flagPub.publishMsg(99)
#     else :
#         if patient >= 3:
#             flagPub.publishMsg(100)
#         patient += 1

                            
    
#     return target,image_np
 
# def queue_add(q,data):
#     if q.full():
#         q.get()
#     q.put_nowait(data)

# def cameraInfo(q_zed_input,q_LiveImage,q_ObjectArray,q_PointCloud):
#     zed = q_zed_input.get_nowait()
    
#     t = time.time()
#     while 1:
#         zed.getData()
        
#         liveImage = zed.liveImage
#         objectArray = zed.objectArray        
#         poc = zed.pointCloud

#         # 等待捕获所有信息
#         if liveImage is None:
#             print("Waiting for new Image")
#             time.sleep(0.2)
#             continue             
#         if objectArray is None:
#             print("Waiting for new Object")
#             time.sleep(0.2)
#             continue 
#         if poc is None:
#             print("Waiting for new PointCloud")
#             time.sleep(0.2)
#             continue

#         queue_add(q_LiveImage,liveImage)
#         queue_add(q_ObjectArray,objectArray)
#         queue_add(q_PointCloud,poc)
            
#         # print fps
#         framePerSecond = int(1/(time.time()-t))
#         # print("From Zed Fps: ", framePerSecond)        
#         t = time.time()         
    
# def objectTread(q_node_object,q_LiveImage,q_ObjectArray,q_target,q_pubMsg_Object):
#     [flagSub,flagPub],goal,video = q_node_object.get_nowait()
    
#     t = time.time()
#     liveVideo = None
#     objectCache = None
#     videoCache = None
#     flagMsg = None
#     videoMsg = None
#     frozenFrame = False
#     trackTarget = False
#     while 1:
#         if q_LiveImage.empty() or q_ObjectArray.empty():
#             # print("Wait img from queue")
#             time.sleep(1/10)
#             continue
#         liveImage = q_LiveImage.get_nowait()
#         objectArray = q_ObjectArray.get_nowait()
            
#         flagMsg = None
#         videoMsg = None
#         rclpy.spin_once(flagSub.node,timeout_sec=0.0001)
#         signal = flagSub.subMsg.data.tolist()   
        
#         print(signal)
#         if signal == 101:            
#             liveVideo,_ = runObjectDetection(liveImage, objectArray)
#         elif signal == 0 and not frozenFrame:   
#             videoCache = liveImage
#             liveVideo,objectCache = runObjectDetection(liveImage, objectArray)
#             if len(objectCache) == 0:
#                 flagMsg = 101 
#                 continue
                        
#             flagMsg = -1*len(objectCache)
#             frozenFrame = True
#             trackTarget = False
#         elif signal == 0 and frozenFrame:
#             continue
#         elif 100 >= signal > 0 and not trackTarget:
#             frozenFrame = False            
#             trackTarget = True
#             objIdx = flagSub.subMsg.tolist()-1
#             obj = objectCache[objIdx]
#             runTargetSet(obj,videoCache,goal)
#         elif 100 >= signal > 0 and trackTarget:
#             target,liveVideo = runTrack(liveImage, objectArray, goal,flagPub)
            
#             queue_add(q_target,target)
#         else:
#             print("Waiting User select target")
#             continue
#         # print fps
#         framePerSecond = int(1/(time.time()-t))
#         # print("LiveVideo Fps: ", framePerSecond)
#         t = time.time()         
        
#         # print live video        
#         if FPS:       
#             cv2.putText(liveVideo, "FPS:{}".format(framePerSecond),org=(5,25),
#                         fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=0.8,
#                         color=(0,255,0),thickness=2)
#         if showImg[2]:       
#             cv2.imshow("LiveVideo",liveVideo) 
#         # 中断守护
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             cv2.destroyAllWindows()
#             break
        
#         videoMsg = liveVideo
#         queue_add(q_pubMsg_Object,[flagMsg,videoMsg])
        
          

# def costmapTread(q_node_Costmap,q_PointCloud,q_target,q_pubMsg_CostMap):
#     cost,showMap,goal = q_node_Costmap.get_nowait()
#     t = time.time()
#     old_target = [0,0]
#     target = None
#     while 1:
#         t1 = time.time()
#         poc = q_PointCloud.get_nowait()
#         pcl = PointCloud2_Img2Array(poc) 
            
#         t2 = time.time()
#         costMsg = None
#         showMsg = None            
        
#         # 获得地图
#         if q_target.empty():
#             target = old_target
#         else:
#             target = q_target.get_nowait()
#             if target is None:
#                 target = old_target
#             else :
#                 old_target = target 
#         print(target)
#         t3 = time.time()
#         obMap,sendMap = get_obMap(pcl,poc,target,CostMap,showImg)
#         # 广播地图
#         t4 = time.time()
#         costMsg = obMap.flatten().astype(np.uint8)
#         showMsg = sendMap.astype(np.uint8)
#         queue_add(q_pubMsg_CostMap, [costMsg,showMsg]) 
#         t5 = time.time()
#         # print fps
#         framePerSecond = int(1/(time.time()-t))
#         # print("costMap Fps: ", framePerSecond)           
#         t = time.time()         
  

# def publishMsg(q_node,q_pubMsg_Object,q_pubMsg_CostMap):
#     [flagSub,flagPub],video,cost,showMap = q_node.get_nowait()
#     while 1:
#         if q_pubMsg_Object.empty() :
#             # print("Wait msg from object queue")
#             pass
#         else: 
#             flagMsg,videoMsg = q_pubMsg_Object.get_nowait()            
#             if flagMsg is not None:
#                 flagPub.publishMsg(flagMsg)
#             if videoMsg is not None:
#                 video.publishMsg(videoMsg)
                
#         if q_pubMsg_CostMap.empty():
#             # print("Wait msg from cost queue")
#             pass
#         else: 
#             costMsg,showMsg = q_pubMsg_CostMap.get_nowait()
#             if costMsg is not None:
#                 cost.publishMsg(costMsg)
#             if showMsg is not None:
#                 showMap.publishMsg(showMsg)

#         rclpy.spin_once(flagSub.node,timeout_sec=0.001)
#         rclpy.spin_once(flagPub.node,timeout_sec=0.001)        
#         rclpy.spin_once(video.node,timeout_sec=0.001)        
#         rclpy.spin_once(cost.node,timeout_sec=0.001)
#         rclpy.spin_once(showMap.node,timeout_sec=0.001)  
#         time.sleep(0.05)

# def main():
#     # 初始化 ros2 python - rclpy & 外置参数引入    
#     zed = YF_Zed2()   
#     init()
#     rclpy.init()
#     # 构建相关节点    
#     flagSub = yf_node.YF_ObjectFlag(nodeName['ObjectFlag'],'ObjectFlagSub',"sub")
#     flagPub = yf_node.YF_ObjectFlag(nodeName['ObjectFlag'],'ObjectFlagPub',"pub")
#     video = yf_node.YF_Image_PY(nodeName['Video'],'Video',"pub")
#     goal = yf_node.YF_Goal(nodeName['Goal'],'Goal',"pub")
#     cost = yf_node.YF_CostMap(nodeName["CostMap"],"CostMap","pub") 
#     showMap = yf_node.YF_Image_PY(nodeName["ShowMap"],"ShowMap","pub")  # YF_CompressedImage YF_Image_PY
#     # flag节点初始化
#     flagPub.publishMsg(101)    
#     # 广播节点的首次初始化
#     rclpy.spin_once(flagSub.node,timeout_sec=0.001)
#     rclpy.spin_once(flagPub.node,timeout_sec=0.001)   
#     rclpy.spin_once(goal.node,timeout_sec=0.001)   
#     rclpy.spin_once(video.node,timeout_sec=0.001)  
#     rclpy.spin_once(cost.node,timeout_sec=0.001)   
#     rclpy.spin_once(showMap.node,timeout_sec=0.001)
#     # 构建lifo队列
#     q_zed_input = Queue(1)
#     q_node_object = Queue(1)
#     q_node = Queue(1)
#     q_node_Costmap = Queue(1)
#     q_node_publish = Queue(1)
#     q_LiveImage = Queue(1)
#     q_ObjectArray = Queue(1)
#     q_PointCloud = Queue(1)
#     q_pubMsg_Object = Queue(1)
#     q_pubMsg_CostMap = Queue(1)
#     q_target = Queue(1)
#     # 放入数据    
#     # input
#     q_zed_input.put_nowait(zed)
#     q_node_object.put_nowait([[flagSub,flagPub],goal,video])
#     q_node_Costmap.put_nowait([cost,showMap,goal])
#     q_node_publish.put_nowait([[flagSub,flagPub],video,cost,showMap])

#     t_camera = threading.Thread(target=cameraInfo,args=(q_zed_input,q_LiveImage,q_ObjectArray,q_PointCloud,))
#     t_object = threading.Thread(target=objectTread,args=(q_node_object,q_LiveImage,q_ObjectArray,q_target,q_pubMsg_Object,))
#     t_costmap = threading.Thread(target=costmapTread,args=(q_node_Costmap,q_PointCloud,q_target,q_pubMsg_CostMap,))
#     t_publish = threading.Thread(target=publishMsg,args=(q_node_publish,q_pubMsg_Object,q_pubMsg_CostMap,))
    
#     t_camera.start()
#     time.sleep(3)
#     t_object.start()
#     t_costmap.start()
#     time.sleep(3)
#     t_publish.start()

#     t = time.time()
#     while 1:
#         if (t - time.time()) > 10:
#             t = time.time()
#             print("pyCamera still runing")
        
     

#     # 杀死所有订阅节点
#     video.node.destroy_node()    
#     cost.node.destroy_node()
#     showMap.node.destroy_node()
#     goal.node.destroy_node()
#     flag.node.destroy_node()
    
#     zed.zed2.close()
#     # 结束rclpy
#     rclpy.shutdown()

            
def init():    
    # 全局化损耗地图
    global CostMap 
    # 全局化模板匹配
    global TemMatch 
    # 全局化模式识别
    global FeaSpace 
    # 全局化图像展示
    global showImg
    # 全局化节点名称
    global nodeName
    global FPS
    global patient
    patient = 0
    # 读取yaml文件
    with open("/home/yf/yifan/config.yaml","r") as f:
        config=yaml.load(f)        
    # 读取展示图像参数
    showImg = [config["showImg"]["ObMap"],
               config["showImg"]["CostMap"],
               config["showImg"]["LiveVideo"],
               config["showImg"]["PointCloud"]]
    # 读取节点名称参数
    nodeName = config["RosTopic"]
    FPS = config["FPS"]["FPS"]

import numpy as np
import rclpy
import yf_node

import yaml
def main():
    init()
    rclpy.init()
    flagSub = yf_node.YF_ObjectFlag(nodeName['ObjectFlag'],'ObjectFlagSub',"sub")
    flagPub = yf_node.YF_ObjectFlag(nodeName['ObjectFlag'],'ObjectFlagPub',"pub")
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
