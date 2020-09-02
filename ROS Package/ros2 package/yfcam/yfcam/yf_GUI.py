import node_test as yf_node
import cv2
import rclpy
import yaml


def init():    
    # 全局化节点名称
    global nodeName  
    # 读取yaml文件
    with open("/home/yf/yifan/config.yaml","r") as f:
        config=yaml.load(f)  
    # 读取节点名称参数
    nodeName = config["RosTopic"]

def main(args=None):
    init()
    rclpy.init()     
    cam = yf_node.YF_Image(nodeName['Image'],'Image')     
    video = yf_node.YF_Image(nodeName['Video'],'Video')     
    showMap = yf_node.YF_Image(nodeName["ShowMap"],"ShowMap")
    while True:
        rclpy.spin_once(cam.node,timeout_sec=0.001) 
        rclpy.spin_once(video.node,timeout_sec=0.001)
        rclpy.spin_once(showMap.node,timeout_sec=0.001)
        if video.subMsg is None:
            print("Waiting for video")
            continue 
        if showMap.subMsg is None:
            print("Waiting for showMap")
            continue  
        cv2.imshow("LiveVideo",video.subMsg)
        cv2.imshow("CostMap",showMap.subMsg)
        
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
