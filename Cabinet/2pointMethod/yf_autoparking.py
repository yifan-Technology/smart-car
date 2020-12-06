import numpy as np
import cv2
import copy
import rclpy
import yf_node
import threading
import time
import yaml
from AutoParking import Parking_Controller
from cameraPingPong import ANTI_LGBT

def init():
    # ???????
    global nodeName
    global mapSize
    # ??yaml??  /home/yf/yifan/config.yaml
    with open("/home/yf/yifan/config.yaml", "r") as f:
        config = yaml.load(f)

    mapSize = config["costMap"]["mapSize"]
    # ????????
    nodeName = config["RosTopic"]

def parker_init():
    path = '/home/yf/dev_ws/src/2pointMethod/dwa_config.yaml'
    # path = "/home/taungdrier/Desktop/dwa_config.yaml"
    with open(path, "r") as f:
        config = yaml.load(f)
        print("load successful")

    car_state = config['Car_State']
    flag_state = config['Flag_State']
    return car_state, flag_state

def get_target(ziel):
    # ??????
    target = np.array([ziel.pose.position.x + 0.06, ziel.pose.position.z])
    return target

def wheel_speed_caculator(v_l_rpm, v_r_rpm):
    minimal = np.min(np.abs([v_l_rpm, v_r_rpm]))

    if minimal == 0 and np.max(np.abs([v_l_rpm, v_r_rpm])) == 0:
        wheel_speed = [v_l_rpm, v_r_rpm, v_l_rpm, v_r_rpm]
        print("cal 00000")
        return wheel_speed

    if minimal < 300:
        shift = 300 - minimal
        v_l_rpm += np.sign(v_l_rpm) * shift
        v_r_rpm += np.sign(v_r_rpm) * shift

    wheel_speed = [v_l_rpm, v_r_rpm, v_l_rpm, v_r_rpm]
    return wheel_speed

def thread_job(cam):
    cam.run()

def main():
    # ??? ros2 python - rclpy & ??????
    rclpy.init()

    # ??????
    init()
    # maps = yf_node.YF_Image_PY(nodeName["CostMap"], "CostMap", "sub")
    # ziel = yf_node.YF_Goal(nodeName['Goal'], 'Goal', 'sub')
    real = yf_node.YF_RealSpeed(nodeName["RealSpeed"], "RealSpeed", "sub")
    soll = yf_node.YF_SollSpeed(nodeName["SollSpeed"], "SollSpeed", "pub")

    rclpy.spin_once(soll.node, timeout_sec=0.001)  # original 0.001
    rclpy.spin_once(real.node, timeout_sec=0.001)
    # rclpy.spin_once(maps.node, timeout_sec=0.001)  # original 0.001
    # rclpy.spin_once(ziel.node, timeout_sec=0.001)

    # ??DWA???
    car_state, flag_state = parker_init()
    goalGenerator = ANTI_LGBT()
    parkplanner = Parking_Controller(car_state, flag_state)

    # thread = threading.Thread(target=thread_job,args=(goalGenerator,))
    # thread.start()

    # # init a time point for fps
    # target = np.array([2., 1.3])
    # tartget1 = np.array([2., 4.3])
    trajectory_ist = parkplanner.state
    flagcount = 0
    u_soll = np.array([0.,0.])
    try:
        while True:
            rclpy.spin_once(soll.node, timeout_sec=0.001)  # original 0.001
            rclpy.spin_once(real.node, timeout_sec=0.001)
            # rclpy.spin_once(maps.node, timeout_sec=0.001)  # original 0.001
            # rclpy.spin_once(ziel.node, timeout_sec=0.001)

            # if real.subMsg is None:
            #     print("Waiting for realspeed message")
            #     continue
            # elif ziel.subMsg is None:
            #     print("Waiting for ziel message")
            #     continue
            # elif maps.subMsg is None:
            #     print("Waiting for map message")
            #     continue
            # elif np.sum(maps.subMsg) == 0:
            #     print("Map is None but given")
            #     continue
            # else:
            # get target
            # target = get_target(ziel.subMsg)
            # if np.linalg.norm(target - parkplanner.state[:2]) < 0.5 and parkplanner.ControlState == "stop":
            #     parkplanner.TEMPORARY_GOAL_ARRIVED = True
            #
            # if parkplanner.TEMPORARY_GOAL_ARRIVED:
            #     target = tartget1
            # try:
            goalGenerator.run()
            target = goalGenerator.get_pose()
            flag = target[-1]
            print("target:", goalGenerator.get_pose())
            # except:
            #     flag = np.nan

            if np.isnan(flag):
                parkplanner.MovingState = "stop"
                u_soll = np.array([0.,0.])
                print("nan flag")

            else:
                # flagcount = 0
                # get wheel speed
                target = np.array(target)
                real_wheel = real.subMsg
                left_front = real_wheel[0]
                right_front = real_wheel[2]
                left_back = real_wheel[4]
                right_back = real_wheel[6]
                motor_ist = np.array([(left_front + left_back) / 2, (right_front + right_back) / 2])
                u_soll = parkplanner.parking_control(motor_ist, target, flag)

            print("car control state:" + parkplanner.MovingState)
            print("speed:" , u_soll)
            # speed infomation transforamtion
            u_soll[0] = float(int(u_soll[0]))
            u_soll[1] = float(int(u_soll[1]))
            wheel_speed = [u_soll[0], u_soll[1], u_soll[0], u_soll[1]]
            trajectory_ist = np.vstack((trajectory_ist, parkplanner.state))

            if parkplanner.SHOW_ANIMATION:
                parkplanner.show_animation(target, trajectory_ist)
            soll.publishMsg(wheel_speed)

            
            key = cv2.waitKey(1) 
            if(key== ord('q')): 
                print("尝试退出")
                goalGenerator.zed.close()
                break        
            elif(key == ord('g')): 
                print("切换成进入状态")
                goalGenerator.state = "Phase: goin"            
                cv2.destroyAllWindows()
            elif(key == ord('a')): 
                print("切换成抵近状态")
                goalGenerator.state = "Phase: approach"
                cv2.destroyAllWindows()
         

    finally:
        wheel_speed = [0.0, 0.0, 0.0, 0.0]
        soll.publishMsg(wheel_speed)
        rclpy.spin_once(soll.node, timeout_sec=0.001)
        soll.node.destroy_node()
        real.node.destroy_node()

    plt.show()

main()