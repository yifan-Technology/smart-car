import copy
import cv2
import numpy as np
import rclpy
import yf_node
import yf_humanDetect
import yf_pattenRecognition
import yf_costmap

def main(args=None):
    # 初始化 ros2 python - rclpy
    rclpy.init(args=args)
    # 构建相关节点
    cam = yf_node.LeftCam()
    obj = yf_node.ObjectsArray()
    goal = yf_node.GoalPub()
    poc = yf_node.PointCloud() 
    cost = yf_node.CostMap() 
    # 发送节点的首次初始化
    rclpy.spin_once(goal.nodeGoal,timeout_sec=0.05)
    rclpy.spin_once(cost.nodeCost,timeout_sec=0.05)
    # 初始化模板匹配
    TemMatch = yf_humanDetect.Template_Matching()
    # 初始化模式识别
    FeaSpace = yf_pattenRecognition.Feature_Space()
    # 初始化损耗地图
    CostMap = yf_costmap.costMap()

    while 1:  
        # 接受 图像，物体，点云信息
        rclpy.spin_once(cam.nodeImg,timeout_sec=0.05) 
        rclpy.spin_once(obj.nodeObj,timeout_sec=0.05)
        rclpy.spin_once(poc.nodePoc,timeout_sec=0.05)

        # 读取 图像，物体，点云信息
        image_np = cam.image_data
        object_array = obj.obj_array  
        point_cloud = poc.poc_image
        pcl = poc.poc_array
        
        # 检测是否接收到所有文件
        if image_np is None:
            print("Waiting for new Image")
            continue             
        if object_array is None:
            print("Waiting for new Object")
            continue
        if point_cloud is None:
            print("Waiting for new Point Cloud")
            continue

        # 跟踪特定的人
        person = None
        cache_acc = 100
        for objects in object_array.markers:
            top_left = (int(objects.points[0].x), int(objects.points[0].y))
            down_right = (int(objects.points[2].x), int(objects.points[2].y))            
            cv2.rectangle(image_np, person[0], person[1], (255,0,255), 3)
            temAcc,aspRatio = TemMatch.new_template(image_np,(top_left,down_right))
            acc = FeaSpace.predict([temAcc,aspRatio])
            if cache_acc > acc:
                person = [top_left,down_right,objects]

        if person is not None:
            goal.goal.header = person[2].header
            goal.goal.pose = person[2].pose
            goal.goal_callback()
            rclpy.spin_once(goal.nodeGoal,timeout_sec=0.05)
            cv2.rectangle(image_np, person[0], person[1], (0,0,255), 3)
            cv2.circle(image_np, (int((person[0][0] + person[1][0]) / 2),
                                int((person[0][1] + person[1][1]) / 2)), 
                                2, (0,0,255), 3)
        
        CostMap.pointCloud2map(pcl)
        testmap = CostMap.vismap
        obMap = CostMap.obstaclemap
        
#        cv2.imshow("Test Map",testmap)  
#        cv2.imshow("Live Video",image_np) 
        
#        cv2.imshow("Point Cloud",point_cloud)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

    cam.nodeImg.destroy_node()
    obj.nodeObj.destroy_node()
    poc.nodePoc.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
