import copy
import cv2
import numpy as np
import rclpy
import yf_node
import yf_humanDetect
import yf_pattenRecognition
import yf_costmap
import dwa_module
import matplotlib.pyplot as plt

def main(args=None):
    # 初始化 ros2 python - rclpy
    rclpy.init(args=args)
    # 构建相关节点
    cam = yf_node.LeftCam()
    obj = yf_node.ObjectsArray()
    goal = yf_node.GoalPub()
    poc = yf_node.PointCloud() 
    cost = yf_node.CostMap() 
    dwa = None
    # 发送节点的首次初始化
    rclpy.spin_once(goal.nodeGoal,timeout_sec=0.05)
    rclpy.spin_once(cost.nodeCost,timeout_sec=0.05)
    # 初始化模板匹配
    TemMatch = yf_humanDetect.Template_Matching()
    # 初始化模式识别
    FeaSpace = yf_pattenRecognition.Feature_Space()
    # 初始化损耗地图
    CostMap = yf_costmap.costMap()
    dwa = dwa_module.DWA_Controller()
    trajectory_ist = dwa.x
    idx = 0
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
        cache_acc = 0.6859
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
        
            
#        cv2.imshow("Send Map",obMap) 
#        cv2.imshow("Cost Map",testmap)  
#        cv2.imshow("Live Video",image_np) 
        
        
        dwa.x = np.array([2.5, 0.0, 0.5 * np.pi, 0.0, 0.0])
        u_soll, trajectory_soll, all_trajectory = dwa.dwa_control(dwa.x,target,obMap)
        wheel_speed = np.array([u_soll[0], u_soll[1], u_soll[0], u_soll[1]]) 
        
#        if idx % 15 == 0:
#            plt.ion()
#            plt.clf()
#            dwa.Goal_arrived = False
#            while dwa.Goal_arrived == False :
#                u_soll, trajectory_soll, all_trajectory = dwa.dwa_control(dwa.x,target,obMap)
#                wheel_speed = np.array([u_soll[0], u_soll[1], u_soll[0], u_soll[1]]) 
#                trajectory_ist = np.vstack((trajectory_ist, dwa.x))
#                obmap = dwa.obmap2coordinaten(obMap)
#                if dwa.show_animation:
#                    plt.cla()
#                    # for stopping simulation with the esc key.
#                    plt.gcf().canvas.mpl_connect(
#                        'key_release_event',
#                        lambda event: [exit(0) if event.key == 'escape' else None])
#        
#                    plt.plot(dwa.x[0], dwa.x[1], "xr")
#                    plt.plot(target[0], target[1], "^r")
#                    plt.plot(obmap[:,0], obmap[:,1], "sk")
#                    for i in range(len(all_trajectory)):
#                        plt.plot(all_trajectory[i][:, 0], all_trajectory[i][:, 1], "-c")
#                    plt.plot(trajectory_soll[:, 0], trajectory_soll[:, 1], "-g")
#                    dwa_module.plot_robot(dwa.x[0], dwa.x[1], dwa.x[2], dwa)
#                    dwa_module.plot_arrow(dwa.x[0], dwa.x[1], dwa.x[2])
#                    plt.plot(trajectory_ist[:, 0], trajectory_ist[:, 1], "-r")
#                    plt.axis("equal")
#                    plt.grid(True)
#                    plt.pause(0.0001)     
#                    print('wheel speed is:',wheel_speed)
#                    print("Position :", dwa.x[:2])
#            plt.pause(1)
#            
#            plt.clf()
#            plt.ioff()         
           
        
#        cv2.imshow("Point Cloud",point_cloud)
        idx += 1
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
    cam.nodeImg.destroy_node()
    obj.nodeObj.destroy_node()
    poc.nodePoc.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
