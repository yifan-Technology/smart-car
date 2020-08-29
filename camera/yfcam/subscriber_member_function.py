import copy
import cv2
import numpy as np
import rclpy
import yf_node
import dwa_module
import yf_camera
import matplotlib.pyplot as plt

def main(args=None):
    # 初始化 ros2 python - rclpy
    rclpy.init(args=args)
    # 构建相关节点
    cam = yf_node.LeftCam()
    obj = yf_node.ObjectsArray()
    goal = yf_node.GoalPub()
    poc = yf_node.PointCloud() 
    soll = yf_node.SollSpeed()
    real = yf_node.RealSpeed()
#    cost = yf_node.CostMap() 
    dwa = None
    # 发送节点的首次初始化
    rclpy.spin_once(goal.nodeGoal,timeout_sec=0.05)
#    rclpy.spin_once(cost.nodeCost,timeout_sec=0.05)
    
    # init camera
    yf_camera.init()
    
    dwa = dwa_module.DWA_Controller()
    trajectory_ist = dwa.x
    idx = 0
    while 1:  
        # 接受 图像，物体，点云信息
        rclpy.spin_once(cam.nodeImg,timeout_sec=0.05) 
        rclpy.spin_once(obj.nodeObj,timeout_sec=0.05)
        rclpy.spin_once(poc.nodePoc,timeout_sec=0.05)
        rclpy.spin_once(soll.nodeSoll,timeout_sec=0.05)
        rclpy.spin_once(soll.nodeReal,timeout_sec=0.05)

        # Run cam
        res = yf_camera.runCam(cam,obj,poc,goal)
        if res is not None:
            target,obMap,CostMap,TestMap = res
        else :
            continue

        ################################
        if soll.soll_speed is None:
            continue
        print(soll.soll_speed)
        ################################

#        # update Position init X
#        dwa.x[0] = 2.5
#        dwa.x[1] = 0.0
#        dwa.x[2] = 0.5 * np.pi
#        a,b,c,d  = np.loadtxt("/home/yf/yifan/real.txt")
#        u_ist = np.array([(a+b)/2,(c+d)/2])*(2*np.pi)/60
#            
#        # update u_ist
#        u_soll, trajectory_soll, all_trajectory = dwa.dwa_control(u_ist, dwa.x,target,obMap, 5/100)
#        wheel_speed = np.array([u_soll[0], u_soll[1], u_soll[0], u_soll[1]]) *60/(2*np.pi)
#        outPrint = np.around(wheel_speed,decimals=3)
#        real.real_callback(wheel_speed)
#        np.savetxt("/home/yf/yifan/soll.txt",outPrint)
            
        
#            if idx % 15 == 0:
#                plt.ion()
#                plt.clf()
#                dwa.Goal_arrived = False
#                while dwa.Goal_arrived == False :
#                    u_soll, trajectory_soll, all_trajectory = dwa.dwa_control(dwa.x,target,obMap)
#                    wheel_speed = np.array([u_soll[0], u_soll[1], u_soll[0], u_soll[1]]) 
#                    trajectory_ist = np.vstack((trajectory_ist, dwa.x))
#                    obmap = dwa.obmap2coordinaten(obMap)
#                    if dwa.show_animation:
#                        plt.cla()
#                        # for stopping simulation with the esc key.
#                        plt.gcf().canvas.mpl_connect(
#                            'key_release_event',
#                            lambda event: [exit(0) if event.key == 'escape' else None])
#            
#                        plt.plot(dwa.x[0], dwa.x[1], "xr")
#                        plt.plot(target[0], target[1], "^r")
#                        plt.plot(obmap[:,0], obmap[:,1], "sk")
#                        for i in range(len(all_trajectory)):
#                            plt.plot(all_trajectory[i][:, 0], all_trajectory[i][:, 1], "-c")
#                        plt.plot(trajectory_soll[:, 0], trajectory_soll[:, 1], "-g")
#                        dwa_module.plot_robot(dwa.x[0], dwa.x[1], dwa.x[2], dwa)
#                        dwa_module.plot_arrow(dwa.x[0], dwa.x[1], dwa.x[2])
#                        plt.plot(trajectory_ist[:, 0], trajectory_ist[:, 1], "-r")
#                        plt.axis("equal")
#                        plt.grid(True)
#                        plt.pause(0.0001)     
#                        print('wheel speed is:',wheel_speed)
#                        print("Position :", dwa.x[:2])
#                plt.pause(1)
#                
#                plt.clf()
#                plt.ioff()         
#               
        
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
