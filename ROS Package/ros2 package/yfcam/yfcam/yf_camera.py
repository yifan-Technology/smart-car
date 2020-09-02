import copy
import cv2
import numpy as np
import rclpy
#import yf_node
import node_test as yf_node
import yf_camera

import yf_humanDetect
import yf_pattenRecognition

import yaml
import time

def init():    
    # 全局化模板匹配
    global TemMatch 
    # 全局化模式识别
    global FeaSpace 
    # 全局化图像展示
    global showImg
    # 全局化节点名称
    global nodeName
    # 读取yaml文件
    with open("/home/yf/yifan/config.yaml","r") as f:
        config=yaml.load(f)        
    # 读取模板匹配参数
    tSize = config["humanDetect"]["tSize"]
    TemMatch = yf_humanDetect.Template_Matching(tSize=tSize)
    # 读取模式识别参数
    capasity = config["pR"]["capasity"]
    feaNum = config["pR"]["featureNum"]
    FeaSpace = yf_pattenRecognition.Feature_Space(capasity=capasity, featureNum=feaNum)
    # 读取展示图像参数
    showImg = [config["showImg"]["ObMap"],
               config["showImg"]["CostMap"],
               config["showImg"]["LiveVideo"],
               config["showImg"]["PointCloud"]]
    # 读取节点名称参数
    nodeName = config["RosTopic"]

def runCam(cam,obj,goal): 
    # 声明 & 获取： 目标、图像、物体信息
    target = None
    image_np = cam.subMsg
    object_array = obj.subMsg    
    # 等待捕获所有信息
    if image_np is None:
        print("Waiting for new Image")
        return             
    if object_array is None:
        print("Waiting for new Object")
        return
    # 声明中间变量 Person
    person = None
    # 确定全局阈值
    stand_acc = 0.6859
    # 图像复制（避免画框后阻碍获取模板）
    image_origin = copy.copy(image_np)

    # 遍历所有目标
    for objects in object_array.markers:
        # 计算左上以及右下角坐标
        top_left = (int(objects.points[0].x), int(objects.points[0].y))
        down_right = (int(objects.points[2].x), int(objects.points[2].y)) 
        # 绘制所有的目标框（无中心标定点）           
        cv2.rectangle(image_np, top_left, down_right, (255,0,255), 3)
        # TODO：尝试使用RGB & all 四个通道进行模板匹配，获得4D特征空间
        # 模板匹配，获得置信度
        temAcc,_ = TemMatch.new_template(image_origin,(top_left,down_right))        
        # 将置信度，目标的XYZ位置放入特征空间，计算acc
        feature = [temAcc,objects.pose.position.x,objects.pose.position.y,objects.pose.position.z]
        acc = FeaSpace.predict(feature)        
        # print(acc)
        # 阈值判定
        if stand_acc > acc:
            person = [top_left,down_right,objects,feature]
            
    # 若找到匹配目标后
    if person is not None:
        # 对应键值设置
        goal.pubMsg.header = person[2].header
        goal.pubMsg.pose = person[2].pose
        # 将特征空间 & 模板匹配 更新
        # TODO: if acc > (1-stand_acc):
        FeaSpace.push(person[3])
        TemMatch.new_template(image_origin,(person[0],person[1]))
        TemMatch.set_tem()
        # 计算目标位置：未使用！有利于debug
        target = np.array([person[2].pose.position.y+2.5,person[2].pose.position.x])
        # 广播节点信息，spin
        goal.publishMsg(goal.pubMsg)
        rclpy.spin_once(goal.node,timeout_sec=0.001)
        # 绘制目标框（含中心标定点）
        cv2.rectangle(image_np, person[0], person[1], (0,0,255), 3)
        cv2.circle(image_np, (int((person[0][0] + person[1][0]) / 2),
                            int((person[0][1] + person[1][1]) / 2)), 
                            2, (0,0,255), 3)  
    
    # print live video        
    if showImg[2]:       
        cv2.imshow("LiveVideo",image_np) 
    return target,image_np
    
def main(args=None):
    # 初始化 ros2 python - rclpy & 外置参数引入
    init()
    rclpy.init(args=args)
    # 构建相关节点
    cam = yf_node.YF_Image(nodeName['Image'],'Image')
    video = yf_node.YF_Image(nodeName['Video'],'Video')
    obj = yf_node.YF_ObjectsArray(nodeName['ObjectsArray'],'ObjectsArray')
    goal = yf_node.YF_Goal(nodeName['Goal'],'Goal')
    # 广播节点的首次初始化
    rclpy.spin_once(goal.node,timeout_sec=0.001)   
    rclpy.spin_once(video.node,timeout_sec=0.001)   
    # init a time point for fps
    t = time.time()

    while 1:  
        # 接受 图像，物体，点云信息
        rclpy.spin_once(cam.node,timeout_sec=0.001) 
        rclpy.spin_once(obj.node,timeout_sec=0.001)
        rclpy.spin_once(video.node,timeout_sec=0.001)  
        # 运行数据捕捉: become target only for debug
        res = runCam(cam,obj,goal)
        if res is None:
            continue
        else:
            target,liveVideo = res
        video.publishMsg(liveVideo.astype(np.uint8))
        # print fps
        print("fps: ", int(1/(time.time()-t)))        
        t = time.time() 
        # 中断守护
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
    # 杀死所有订阅节点
    cam.node.destroy_node()
    obj.node.destroy_node()
    # 结束rclpy
    rclpy.shutdown()
