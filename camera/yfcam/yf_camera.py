import copy
import cv2
import numpy as np
import rclpy
import yf_node
import yf_camera

import yf_humanDetect
import yf_pattenRecognition

import yaml


def init():    
    # 全局化模板匹配
    global TemMatch 
    # 全局化模式识别
    global FeaSpace 
    # 全局化图像展示
    global showImg
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

def runCam(cam,obj,goal): 
    # 声明 & 获取： 目标、图像、物体信息
    target = None
    image_np = cam.image_data
    object_array = obj.obj_array    
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
    cache_acc = 100.6859
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
        acc = FeaSpace.predict([temAcc,objects.pose.position.x,objects.pose.position.y,objects.pose.position.z])        
        # print(acc)
        # 阈值判定
        if cache_acc > acc:
            person = [top_left,down_right,objects,[temAcc, distance]]
            
    # 若找到匹配目标后
    if person is not None:
        # 对应键值设置
        goal.goal.header = person[2].header
        goal.goal.pose = person[2].pose
        # 将特征空间 & 模板匹配 更新
        FeaSpace.push(person[3])
        TemMatch.new_template(image_origin,(person[0],person[1]))
        TemMatch.set_tem()
        # 计算目标位置：未使用！有利于debug
        target = np.array([person[2].pose.position.y+2.5,person[2].pose.position.x])
        # 广播节点信息，spin
        goal.goal_callback()
        rclpy.spin_once(goal.nodeGoal,timeout_sec=0.05)
        # 绘制目标框（含中心标定点）
        cv2.rectangle(image_np, person[0], person[1], (0,0,255), 3)
        cv2.circle(image_np, (int((person[0][0] + person[1][0]) / 2),
                            int((person[0][1] + person[1][1]) / 2)), 
                            2, (0,0,255), 3)  
    return target
    
def main(args=None):
    # 初始化 ros2 python - rclpy & 外置参数引入
    init()
    rclpy.init(args=args)
    # 构建相关节点
    cam = yf_node.LeftCam()
    obj = yf_node.ObjectsArray()
    goal = yf_node.GoalPub()
    # 广播节点的首次初始化
    rclpy.spin_once(goal.nodeGoal,timeout_sec=0.05)   
    # 相机初始化
    yf_camera.init()

    while 1:  
        # 接受 图像，物体，点云信息
        rclpy.spin_once(cam.nodeImg,timeout_sec=0.05) 
        rclpy.spin_once(obj.nodeObj,timeout_sec=0.05)
        # 运行数据捕捉
        res = yf_camera.runCam(cam,obj,goal)
        # 中断守护
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
    # 杀死所有订阅节点
    cam.nodeImg.destroy_node()
    obj.nodeObj.destroy_node()
    # 结束rclpy
    rclpy.shutdown()

# if __name__ == '__main__':
#     main()
