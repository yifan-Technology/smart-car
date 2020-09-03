import rclpy
import numpy as np
import math
import yaml
import yf_node 
import time



# from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion, Twist  # message aufrufen
# from std_msgs.msg import Header
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float32MultiArray


def init():    
    # 全局化节点名称
    global nodeName, target_x,target_y
    # 读取yaml文件
    with open("/home/yf/yifan/config.yaml","r") as f:
        config=yaml.load(f)  
    target_x = -1
    target_y = -1
        
    # 读取节点名称参数
    nodeName = config["RosTopic"]

def controlalgorithm(goal_x, goal_y):

    robot_homePosition_x = 0
    robot_homePosition_y = 0
    r_d = 0.8     #safe distance
    v_max = 0.5
    w_max = 3.14/12

    r_speedCut = 0.5 # design parameter for distanz speed cut


    delta_x = goal_x - robot_homePosition_x
    delta_y = goal_y - robot_homePosition_y

    if delta_x == 0:
        phi = 0.0
    else:
        phi = math.atan(delta_x / delta_y)
    
    
    r = math.sqrt(goal_x**2 + goal_y**2)
    r_expect = r - r_d
    
    k_max = np.abs(w_max) - 0.5 * v_max / r_d #k_max for debug
    print("k max: ",k_max,phi/3.14*180)

    k = 0.5 * k_max       #  k > 0 and k < k_max

    l = 0.45      # Wheel spacing

    if r_expect < -r_speedCut:
        s = -1 
    elif -r_speedCut < r_expect <= 0:
        s = ((r_expect + r_speedCut)/r_speedCut)**2 - 1
    elif 0 < r_expect <= r_speedCut:
        s = 1 - ((r_expect - r_speedCut)/r_speedCut)**2
    else:
        s = 1
        
    v = v_max * s * math.cos(phi) 

    omega = (-k * phi + v / (r_expect+r_d) + math.sin(phi)) / 2
    
    if r_expect < 0:
        v = 0
        omega = 0
        
    global target_x
    global target_y
    if goal_x == target_x and target_y == goal_y:        
        v = 0
        omega = 0
    
    target_x = goal_x
    target_y = goal_y

    print("v: ", v)
    print("omega: ", omega)

    v_l = v - omega * l /2
    v_r = v + omega * l /2

    return v_l, v_r  # return deine Parametern


def wheel_speed_caculator(v_l,v_r):
    v_l_rpm = float(int((v_l * 19 * 60) / (2 * 3.14 * 0.1)))
    v_r_rpm = float(int((v_r * 19 * 60) / (2 * 3.14 * 0.1)))
    wheel_speed = [v_l_rpm,v_r_rpm,v_l_rpm,v_r_rpm]
    return wheel_speed


def get_target(ziel):
    # 计算坐标位置
    target = np.array([ziel.pose.position.x,ziel.pose.position.z])
    return target


def main(args=None):  # main Funktion
    rclpy.init(args=args)  # initialisieren
    init()
    soll = yf_node.YF_SollSpeed(nodeName["SollSpeed"],"SollSpeed")    
    goal = yf_node.YF_Goal(nodeName['Goal'],'Goal')    
    flag = yf_node.YF_ObjectFlag(nodeName['ObjectFlag'],'ObjectFlag')
    flag.publishMsg(101)
    rclpy.spin_once(flag.node,timeout_sec=0.05) 
    
    # track_subscriber = trackSubscriber()  # der Name von dem zu subscribten Node
    # rclpy.spin(track_subscriber)
    
    t = time.time() 
    while 1:        
        rclpy.spin_once(flag.node,timeout_sec=0.01) 
        signal = flag.subMsg.data
        if signal == 101:
            print("Waiting for a Target")
            continue
        if signal.tolist() > 0:
            rclpy.spin_once(goal.node,timeout_sec=0.01) 
            ziel = goal.subMsg
            if ziel is None:
                print("Waiting for a Target")
                continue
            target = get_target(ziel)
            print(target)
            v_l, v_r = controlalgorithm(target[0], target[1])
            
            print("vl vr: ", v_l, v_r)
            
            wheel_speed = wheel_speed_caculator(v_l,v_r)
            print("speed: ", wheel_speed)
            soll.publishMsg(wheel_speed)        
            rclpy.spin_once(goal.node,timeout_sec=0.01) 
            
#            # print fps
#            print("fps: ", int(1/(time.time()-t)))        
#            t = time.time() 
        
            time.sleep(0.05)
        else:
            continue

    goal.destroy_node()  # eben vom Oben
    soll.destroy_node()  # eben vom Oben
    rclpy.shutdown()


if __name__ == '__main__':
    main()
