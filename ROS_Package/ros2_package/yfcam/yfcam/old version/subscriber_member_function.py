import copy
import cv2
import numpy as np
import rclpy
import yf_node
import dwa_module
import yf_camera
import matplotlib.pyplot as plt
import motorSerial 
import time 

def main(args=None):
    # 初始化 ros2 python - rclpy
    rclpy.init(args=args)
    # 构建相关节点
    cam = yf_node.LeftCam()
    obj = yf_node.ObjectsArray()
    goal = yf_node.GoalPub()
    poc = yf_node.PointCloud() 
    real = yf_node.SUB_RealSpeed_Main()
    soll = yf_node.PUB_SollSpeed_Main()
#    cost = yf_node.CostMap() 
    dwa = None
    # 发送节点的首次初始化
    rclpy.spin_once(goal.nodeGoal,timeout_sec=0.05)
#    rclpy.spin_once(cost.nodeCost,timeout_sec=0.05)
    
    # init camera
    yf_camera.init()

#    Motor_serial = motorSerial.SerialThread("/dev/ttyUSB0")
    
    dwa = dwa_module.DWA_Controller()
    trajectory_ist = dwa.x
#    idx = 0
    init = True
#    signal = 25.0
    while 1:  
        # 接受 图像，物体，点云信息
        rclpy.spin_once(cam.nodeImg,timeout_sec=0.05) 
        rclpy.spin_once(obj.nodeObj,timeout_sec=0.05)
        rclpy.spin_once(poc.nodePoc,timeout_sec=0.05)
        rclpy.spin_once(soll.nodeSoll,timeout_sec=0.05)
        rclpy.spin_once(real.nodeReal,timeout_sec=0.05)

        # Run cam
        res = yf_camera.runCam(cam,obj,poc,goal)
        if res is not None:
            target,obMap,CostMap,TestMap = res            
            obMap = np.zeros((100,100)) 
            obMap[0,0]=1
            obMap[99,99] = 1
        else :
            target = np.array([2.5, 0.0]) 
            print(111111)
#            obMap = np.random.rand(10000)
#            obMap[obMap>0.5]=1
#            obMap[obMap<=0.5]=0
#            obMap = obMap.reshape(100,100)
#            obMap = np.random.shuffle(obMap)
            obMap = np.ones((100,100))

    
        if real.real_speed is None:
            print("Waiting for real_speed and publish soll value [0,0,0,0]")
            soll.soll_publish([0.0,0.0, 0.0, 0.0])
            continue
        
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
#            trajectory_ist = np.vstack((trajectory_soll, dwa.x))
#            count = 0
#            for i in range(10000):
#                if obMap[]
#            obmap = dwa.obmap2coordinaten(obMap, 5/100)
#            if dwa.show_animation:
#                plt.cla()
#                # for stopping simulation with the esc key.
#                plt.gcf().canvas.mpl_connect(
#                    'key_release_event',
#                    lambda event: [exit(0) if event.key == 'escape' else None])
#    
#                plt.plot(dwa.x[0], dwa.x[1], "xr")
#                plt.plot(target[0], target[1], "^r")
#                plt.plot(obmap[:,0], obmap[:,1], "sk")
#                for i in range(len(all_trajectory)):
#                    plt.plot(all_trajectory[i][:, 0], all_trajectory[i][:, 1], "-c")
#                plt.plot(trajectory_soll[:, 0], trajectory_soll[:, 1], "-g")
#                dwa_module.plot_robot(dwa.x[0], dwa.x[1], dwa.x[2], dwa)
#                dwa_module.plot_arrow(dwa.x[0], dwa.x[1], dwa.x[2])
#                plt.plot(trajectory_ist[:, 0], trajectory_ist[:, 1], "-r")
#                plt.axis("equal")
#                plt.grid(True)
#                plt.pause(0.0001)     
#                print('wheel speed is:',wheel_speed)
#                print("Position :", dwa.x[:2])
##            plt.pause(1)
               
#                plt.clf()
#                plt.ioff()         
            
            if dwa.GOAL_ARRIVAED:
                wheel_speed = [0.0,0.0,0.0,0.0]
                dwa.GOAL_ARRIVAED = False 
            print("outprint: ", wheel_speed)
            soll.soll_publish(wheel_speed)
        else:
            print('target not detected')
            soll.soll_publish([0.0,0.0, 0.0, 0.0])
        
#           
#               
        
#        idx += 1
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
    cam.nodeImg.destroy_node()
    obj.nodeObj.destroy_node()
    poc.nodePoc.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
