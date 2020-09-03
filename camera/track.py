# import rclpy
import numpy as np
import math
# import yaml
# import yf_node 
import time



# from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion, Twist  # message aufrufen
# from std_msgs.msg import Header
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float32MultiArray


def init():    
    # 全局化节点名称
    global nodeName
    # 读取yaml文件
    with open("/home/yf/yifan/config.yaml","r") as f:
        config=yaml.load(f)  
        
    # 读取节点名称参数
    nodeName = config["RosTopic"]

def controlalgorithm(goal_x, goal_y):

    robot_homePosition_x = 0+0.06
    robot_homePosition_y = 0
    r_d = 0.5     #safe distance
    v_max = 0.8
    w_max = 3.14

    r_speedCut = 1 # design parameter for distanz speed cut

    delta_x = goal_x - robot_homePosition_x
    delta_y = goal_y - robot_homePosition_y

    if delta_x == 0:
        phi = 3.1415926/2
    else:
        phi = math.atan(delta_y / delta_x)
    
    r = math.sqrt(goal_x**2 + goal_y**2)
    r_expect = r - r_d
    
    k_max = 0.5 * w_max - 0.5 * v_max / 0.5 #k_max for debug
    print("k max: ",k_max)

    k = 0.5        #  k > 0 and k < k_max

    l = 0.095      # Wheel spacing

    if r_expect < -r_speedCut:
        v = -1 * v_max * math.cos(phi)
    elif -r_speedCut < r_expect <= 0:
        v = v_max * (math.sqrt((r_expect + 0.5) / 0.5) - 1) * math.cos(phi)
    elif 0 < r_expect <= r_speedCut:
        v = v_max * (1 - math.sqrt((r_expect - 0.5) / 0.5)) * math.cos(phi)
    else:
        v = v_max * 1 * math.cos(phi)

    omega = -0.5 * k * phi + 0.5 * v * math.sin(phi) / r

    print("v: ", v)
    print("omega: ", omega)

    v_l = v - omega * l /2
    v_r = v + omega * l /2

    return v_l, v_r  # return deine Parametern


def wheel_speed_caculator(v_l,v_r):
    v_l_rpm = float(int(v_l / 0.1 * 19 * 60 / 2 * 3.14))
    v_r_rpm = float(int(v_r / 0.1 * 19 * 60 / 2 * 3.14))
    wheel_speed = [v_l_rpm,v_r_rpm,v_l_rpm,v_r_rpm]
    return wheel_speed

def main(args=None):  # main Funktion
    # rclpy.init(args=args)  # initialisieren
    # # init()
    # soll = yf_node.YF_SollSpeed(nodeName["SollSpeed"],"SollSpeed")    
    # goal = yf_node.YF_Goal(nodeName['Goal'],'Goal')


    # track_subscriber = trackSubscriber()  # der Name von dem zu subscribten Node
    # rclpy.spin(track_subscriber)
    
    t = time.time() 
    while 1:        
        # rclpy.spin_once(goal.node,timeout_sec=0.05) 
        # target = goal.subMsg
        target = [0.06,1]
        v_l, v_r = controlalgorithm(target[0], target[1])
        
        wheel_speed = wheel_speed_caculator(v_l,v_r)
        print("speed: ", wheel_speed)
        # soll.publishMsg(wheel_speed)        
        # rclpy.spin_once(goal.node,timeout_sec=0.05) 
        
        # print fps
        print("fps: ", int(1/(time.time()-t)))        
        t = time.time() 

        time.sleep(0.05)

    # goal.destroy_node()  # eben vom Oben
    # soll.destroy_node()  # eben vom Oben
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
