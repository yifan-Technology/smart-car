import numpy as np
import cv2
import copy
import rclpy
import yaml
import yf_costmap
import motorSerial 
import yf_node
import dwa_module
import matplotlib.pyplot as plt
import motorSerial 
import time 

def init():    
    # 全局化损耗地图
    global CostMap 
    # 全局化图像展示
    global showImg    
    # 读取yaml文件
    with open("/home/yf/yifan/config.yaml","r") as f:
        config=yaml.load(f)  
    # 读取损耗地图参数
    mapSize = config["costMap"]["mapSize"]
    visSize = config["costMap"]["visSize"]
    h_self = config["costMap"]["h_self"]
    h_top = config["costMap"]["h_top"]
    feld = config["costMap"]["feld"]
    CostMap = yf_costmap.costMap(mapSize=mapSize,visSize=visSize,h_self=h_self,
                                 h_top=h_top,feld=feld)
    # 读取图像展示参数
    showImg = [config["showImg"]["ObMap"],
               config["showImg"]["CostMap"],
               config["showImg"]["LiveVideo"],
               config["showImg"]["PointCloud"]]

def get_obMap(pcl,poc):  
    # 生成必要图像
    CostMap.pointCloud2map(pcl)
    obMap = CostMap.obstaclemap()
    # 展示图像
    if showImg[0]:
        cv2.imshow("Send Map",obMap) 
    if showImg[1]:        
        testmap = CostMap.visualMap()
        cv2.imshow("Cost Map",testmap)  
    if showImg[3]:
        point_cloud = poc.poc_image
        cv2.imshow("Point Cloud",point_cloud)
    return obMap

def get_target(ziel):
    # 计算坐标位置
    target = np.array([ziel.pose.position.y+2.5,ziel.pose.position.x])
    return target

def matplotlib_show(trajectory_soll,dwa,obMap,target,all_trajectory,wheel_speed):
    # 是否用Matplotlib 展示地图
    # ！！！！！！！！！ 不可与cv2图片展示共存
    trajectory_ist = np.vstack((trajectory_soll, dwa.x))
    count = 0
    # for i in range(10000):
    #     if obMap[]
    obmap = dwa.obmap2coordinaten(obMap, 5/100)
    if dwa.show_animation:
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        plt.plot(dwa.x[0], dwa.x[1], "xr")
        plt.plot(target[0], target[1], "^r")
        plt.plot(obmap[:,0], obmap[:,1], "sk")
        for i in range(len(all_trajectory)):
            plt.plot(all_trajectory[i][:, 0], all_trajectory[i][:, 1], "-c")
        plt.plot(trajectory_soll[:, 0], trajectory_soll[:, 1], "-g")
        dwa_module.plot_robot(dwa.x[0], dwa.x[1], dwa.x[2], dwa)
        dwa_module.plot_arrow(dwa.x[0], dwa.x[1], dwa.x[2])
        plt.plot(trajectory_ist[:, 0], trajectory_ist[:, 1], "-r")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0001)     
        print('wheel speed is:',wheel_speed)
        print("Position :", dwa.x[:2])
#            plt.pause(1)
        
        plt.clf()
        plt.ioff() 

def main():    
    # 初始化 ros2 python - rclpy & 外置参数引入
    init()
    rclpy.init()     
    # 构建相关节点
    poc = yf_node.PointCloud()     
    real = yf_node.SUB_RealSpeed_Main()
    soll = yf_node.PUB_SollSpeed_Main()  
    goal = yf_node.GoalSub()       
    # 广播节点的首次初始化
    rclpy.spin_once(soll.nodeSoll,timeout_sec=0.05)
    # 构建DWA控制类
    dwa = dwa_module.DWA_Controller()
    while True:        
        # 刷新订阅的节点      
        rclpy.spin_once(poc.nodePoc,timeout_sec=0.05)
        rclpy.spin_once(goal.nodeGoal,timeout_sec=0.05)
        rclpy.spin_once(real.nodeReal,timeout_sec=0.05)
        # 广播节点        
        rclpy.spin_once(soll.nodeSoll,timeout_sec=0.05)
        # 捕获数据        
        pcl = poc.poc_array       
        ziel = goal.goal
        # 等待捕获所有信息
        if pcl is None:
                print("Waiting for new Point Cloud")
                return  
        if ziel is None:
                print("Waiting for new target")
                return             
        if real.real_speed is None:
            print("Waiting for real_speed and publish soll value [0,0,0,0]")
            soll.soll_publish([0.0,0.0, 0.0, 0.0])
            continue
        # 计算地图以及目标位置
        obMap = get_obMap(pcl,poc)   
        target = get_target(ziel)
        # 捕获真实速度值
        real_wheel = real.real_speed
        
#        real_wheel = [200.,0,200.,0,200.,0,200.]
        print("Got real_wheel: ",real_wheel)
        left_front = real_wheel[0]
        right_front = real_wheel[2]
        left_back = real_wheel[4]
        right_back = real_wheel[6]
        
#        target = np.array([2.5,4.])
        u_ist = np.array([(left_front+left_back)/2,(right_front+right_back)/2])
        if target is not None:
            print("target detected")
            dwa.Goal_arrived = False
#            while dwa.Goal_arrived == False :
            u_soll, trajectory_soll, all_trajectory = dwa.dwa_control(u_ist, dwa.x,target,obMap,5/100)
            u_soll[0] = float(int(u_soll[0]))
            u_soll[1] = float(int(u_soll[1]))
            wheel_speed =[u_soll[1], u_soll[0], u_soll[1], u_soll[0]]
            # # 使用Matplotlib展示地图
            # matplotlib_show(trajectory_soll,dwa,obMap,target,all_trajectory,wheel_speed)
            if dwa.GOAL_ARRIVAED:
                wheel_speed = [0.0,0.0,0.0,0.0]
                dwa.GOAL_ARRIVAED = False 
            print("outprint: ", wheel_speed)
            soll.soll_publish(wheel_speed)
        else:
            print('target not detected')
            soll.soll_publish([0.0,0.0, 0.0, 0.0])
        
    # 杀死无用节点
    poc.nodePoc.destroy_node()
    soll.nodeSoll.destroy_node()
    goal.nodeGoal.destroy_node()
    real.nodeReal.destroy_node()
        
        