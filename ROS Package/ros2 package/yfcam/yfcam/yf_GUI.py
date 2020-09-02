import node_test as yf_node
import cv2


def init():    
    # 全局化节点名称
    global nodeName  
    # 读取yaml文件
    with open("/home/yf/yifan/config.yaml","r") as f:
        config=yaml.load(f)  
    # 读取节点名称参数
    nodeName = config["RosTopic"]

def main(args=None):
    cam = yf_node.YF_Image(nodeName['Image'])     
    showMap = yf_node.YF_Image(nodeName["ShowMap"])
    while True:
        rclpy.spin_once(cam.node,timeout_sec=0.001) 
        rclpy.spin_once(showMap.node,timeout_sec=0.001)

        cv2.imshow("LiveVideo",cam.subMsg)
        cv2.imshow("CostMap",showMap.subMsg)
        
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
