import numpy as np
import cv2
import copy
import rclpy
import yf_node
import node_test
import dwa_module
import matplotlib.pyplot as plt
import motorSerial 
import time 

def init():    
    # 全局化损耗地图
    global CostMap 
    # 全局化图像展示
    global showImg  
    # 全局化节点名称
    global nodeName  
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
    # 读取节点名称参数
    nodeName = config["RosTopic"]

def get_target(ziel):
    # 计算坐标位置
    target = np.array([ziel.pose.position.y+2.5,ziel.pose.position.x])
    return target

def matplotlib_show(trajectory_soll,dwa,obmap,target,all_trajectory):
    # 是否用Matplotlib 展示地图
    # ！！！！！！！！！ 不可与cv2图片展示共存
    trajectory_ist = np.vstack((trajectory_soll, dwa.x))

    # for i in range(10000):
    #     if obMap[]
    obmap = dwa.obmap2coordinaten(obmap, 5/100)
    if dwa.show_animation:
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

        plt.plot(dwa.x[0], dwa.x[1], "xr")
        plt.plot(obmap[:, 0], obmap[:, 1], "sk")
        for i in range(len(all_trajectory)):
            plt.plot(all_trajectory[i][:, 0], all_trajectory[i][:, 1], "-c")
#        plt.plot(trajectory_soll[:, 0], trajectory_soll[:, 1], "-g")
        dwa_module.plot_robot(dwa.x[0], dwa.x[1], dwa.x[2], dwa)
        dwa_module.plot_arrow(dwa.x[0], dwa.x[1], dwa.x[2])
        plt.plot(trajectory_ist[:, 0], trajectory_ist[:, 1], "-r")
        plt.plot(target[0], target[1], "^r")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0001)     
#        print('wheel speed is:',wheel_speed)
        print("Position :", dwa.x[:2])
#            plt.pause(1)
        plt.clf()
        plt.ioff() 

def main():    
    # 初始化 ros2 python - rclpy & 外置参数引入
    rclpy.init()     
    # 构建相关节点
    maps = node_test.YF_CostMap(nodeName["CostMap"])    
    real = yf_node.SUB_RealSpeed_Main()
    soll = yf_node.PUB_SollSpeed_Main()  
    ziel = node_test.YF_Goal(nodeName["Goal"])       
    # 广播节点的首次初始化
    rclpy.spin_once(soll.nodeSoll,timeout_sec=0.001)
    # 构建DWA控制类
    dwa = dwa_module.DWA_Controller()
    old_target = np.array([-1.,-1.])
    # init a time point for fps
    t = time.time()
    u_soll = np.array([0.,0.])
    while True:           
        # 中断守护
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
        # 刷新订阅的节点
        rclpy.spin_once(maps.node,timeout_sec=0.001)
        rclpy.spin_once(ziel.node,timeout_sec=0.001)
        rclpy.spin_once(real.nodeReal,timeout_sec=0.001)
        # 广播节点        
        rclpy.spin_once(soll.nodeSoll,timeout_sec=0.001)
        # 捕获数据           
        obMap = maps.subMsg        
        zielMsg = ziel.subMsg
        # print fps
        print("fps: ", int(1/(time.time()-t)))        
        t = time.time() 
        # 等待捕获所有信息
        
        if obMap is None:
            print("Waiting for new obMap")
            continue  
        if zielMsg is None:
            print("Waiting for new target")
            continue 
        else:
            target = get_target(zielMsg)
            # target = np.array([2.5,5.])
            if (old_target != target).any():
                dwa.RESET_STATE = True 
                
                # np.save('/home/yf/dev_ws/src/yfcam/yfcam/test_map.npy',obMap)
                # obMap = np.load('/home/yf/dev_ws/src/yfcam/yfcam/test_map.npy')
                old_target = target
                print('old target:',old_target)
                print('update obmap')
            else:
                dwa.RESET_STATE = False
                print('do not update obmap')
        
        if real.real_speed is None:
            print("Waiting for real_speed and publish soll value [0,0,0,0]")
            soll.soll_publish([0.0,0.0, 0.0, 0.0])
            continue
                  
        # 捕获真实速度值
        real_wheel = real.real_speed
        
        # real_wheel = [200.,0,200.,0,200.,0,200.]
        # print("Got real_wheel: ",real_wheel)
        left_front = real_wheel[0]
        right_front = real_wheel[2]
        left_back = real_wheel[4]
        right_back = real_wheel[6]
        
        # target = np.array([2.5,4.])
        u_ist = np.array([(left_front+left_back)/2,(right_front+right_back)/2])
        if target is not None:
            print("target detected")
            dwa.GOAL_ARRIVAED = False
            # while dwa.Goal_arrived == False :
            # t1 = time.time()
            u_soll, trajectory_soll, all_trajectory = dwa.dwa_control(u_ist, dwa.x,target,obMap,5/100)
            t2 = time.time()
            print("run time:", t2-t1)
            u_ist = u_soll
            print('u_ist:',u_ist)
            print('u_soll:',u_soll)
            # speed infomation transforamtion
            u_soll[0] = float(int(u_soll[0]))
            u_soll[1] = float(int(u_soll[1]))
            wheel_speed =[u_soll[0], u_soll[1], u_soll[0], u_soll[1]]
            # # 使用Matplotlib展示地图
            matplotlib_show(trajectory_soll,dwa,obMap,target,all_trajectory)
            if dwa.GOAL_ARRIVAED:
                wheel_speed = [0.0,0.0,0.0,0.0]
                dwa.GOAL_ARRIVAED = False 
            print("wheel speed: ", wheel_speed)
            soll.soll_publish(wheel_speed)        
    # 杀死无用节点
    maps.node.destroy_node()
    soll.nodeSoll.destroy_node()
    ziel.node.destroy_node()
    real.nodeReal.destroy_node()
        
        