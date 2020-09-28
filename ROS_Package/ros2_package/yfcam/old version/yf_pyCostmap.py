import cv2
import numpy as np
import yf_node
import time 
import rclpy
import yaml
   
class costMap:
    '''
    损耗地图 类
    默认值：
        mapSize（地图尺寸） --> 散点地图尺寸
        visSize（可视尺寸） --> 可视化地图尺寸
        h_self(自我高度) --> 摄像头高度
        h_top（上限高度） --> 障碍物上限高度（天花板问题）
        feld（场） --> 实际需求场        
    # 单位：cm
    '''
    def __init__(self, mapSize=100, visSize = 501, h_self=80, h_top=180, feld=500):
        # 私有化所有变量
        self.mapSize = mapSize
        self.visSize = visSize
        self.feld = feld
        self.h_top = h_top
        self.h_self = h_self
        # 声明变量
        self.gridmap = None

    def pointCloud2map(self,pcl):
        # 点云单位 m->cm
        pcl = np.floor(pcl * 100).astype(np.uint32)# z up x forward
        # 构造数组
        gridmap = np.zeros_like(pcl)
        # 赋予左右方向的值 + offset
        gridmap[:,0] = pcl[:,1] + int(self.feld/2)
        # 赋予前后方向的值
        gridmap[:,1] = pcl[:,0]
        # 赋予上下方向的值
        gridmap[:,2] = np.abs(pcl[:,2] + self.h_self)
        # 删除左右方向中，实际需求场外的点
        gridmap = np.delete(gridmap,np.where(gridmap[:,0]>self.feld),0)   
        gridmap = np.delete(gridmap,np.where(gridmap[:,0]<0),0)
        # 删除前后方向中，实际需求场外的点
        gridmap = np.delete(gridmap,np.where(gridmap[:,1]>self.feld),0)
        # 删除上下方向中，高于障碍物上限高度的点
        gridmap = np.delete(gridmap,np.where(gridmap[:,2]>self.h_top),0)
        # 私有化变量
        self.gridmap = gridmap
        
    def obstaclemap(self):    
        # 构建空地图，并赋值
        self.sendmap = np.zeros([self.visSize,self.visSize])
        gridmap = self.gridmap            
        # 提取高度高于25cm的点云的index
        large = np.where([gridmap[:,2]>=25])
        # 使用index完成赋值
        self.sendmap[gridmap[large,1],gridmap[large,0]] = 1
        # 地图缩放为需要的大小
        sendmap = cv2.resize(self.sendmap,(self.mapSize,self.mapSize))
        return sendmap

    def visualMap(self):
        # 构建空地图，并赋值
        self.vismap = np.zeros([self.visSize,self.visSize,3])
        gridmap = self.gridmap  
        testmap = self.vismap  
        # 初始化膨胀算法的核
        kernel = np.ones((5,5),np.uint8)
        # 提取高度高于以及低于25cm的点云的idx
        small = np.where([gridmap[:,2]<25])
        large = np.where([gridmap[:,2]>=25])
        # 使用idx完成绘图
        # 将小于25cm点设置为绿色，大于点设置为红色
        testmap[gridmap[small,1],gridmap[small,0],1] = 1
        testmap[gridmap[large,1],gridmap[large,0],2] = 1
        # 将整张地图设置为蓝色
        testmap[:,:,0] = 0
        # 完成红，绿两通道的膨胀
        testmap[:,:,1] = cv2.dilate(testmap[:,:,1],kernel,iterations=1)*255
        testmap[:,:,2] = cv2.dilate(testmap[:,:,2],kernel,iterations=1)*255
        # 当红，绿有颜色时，蓝色置为0
        testmap[:,:,0][testmap[:,:,1]==255] = 0
        testmap[:,:,0][testmap[:,:,2]==255] = 0

        return testmap

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
    CostMap = costMap(mapSize=mapSize,visSize=visSize,h_self=h_self,
                                 h_top=h_top,feld=feld)
    # 读取图像展示参数
    showImg = [config["showImg"]["ObMap"],
               config["showImg"]["CostMap"],
               config["showImg"]["LiveVideo"],
               config["showImg"]["PointCloud"]]
    # 读取节点名称参数
    nodeName = config["RosTopic"]

def get_obMap(pcl,poc,target,CostMap,showImg):  
    # 生成必要图像
    CostMap.pointCloud2map(pcl)
    obMap = CostMap.obstaclemap()
    
    maps = cv2.resize(obMap,(500,500))
    maps[maps>0] = 255
    sendMap = np.zeros([500,500,4])
    sendMap[:,:,0] = maps
    sendMap[:,:,1] = maps
    sendMap[:,:,2] = maps
    
    y = int(target[0] * 100)
    x = int(target[1] * 100)
    
    sendMap[(x-10):(x+10),(y-10):(y+10),0] = 0
    sendMap[(x-10):(x+10),(y-10):(y+10),1] = 0
    sendMap[(x-10):(x+10),(y-10):(y+10),2] = 255
    sendMap = sendMap[::-1,::-1]
    # 展示图像
    if showImg[0]:
        cv2.imshow("Send Map",sendMap) 
    if showImg[1]:        
        testmap = CostMap.visualMap()
        cv2.imshow("Cost Map",testmap)  
    if showImg[3]:
        point_cloud = poc
        cv2.imshow("Point Cloud",point_cloud)
    return obMap,sendMap
 
def get_target(ziel):
    # 计算坐标位置
    target = np.array([ziel.position.y+2.5,ziel.position.x])
    return target

def PointCloud2_Img2Array(cloud_array, remove_nans=True, dtype=np.float):
    '''Pulls out x, y, and z columns from the cloud recordarray, and returns
    a 3xN matrix.
    '''
    # remove crap points
    if remove_nans:
        mask = np.isfinite(cloud_array[:,:,0]) & np.isfinite(cloud_array[:,:,1]) & np.isfinite(cloud_array[:,:,2])
        cloud_array = cloud_array[mask]
    # pull out x, y, and z values
    points = np.zeros([np.sum(mask),3], dtype=dtype)
    points[:,0] = cloud_array[:,2] # 012 120 021 102 201 210
    points[:,1] = -1*cloud_array[:,0]
    points[:,2] = -1*cloud_array[:,1]
    return points


def main():    
    # 初始化 ros2 python - rclpy & 外置参数引入
    init()
#    rclpy.init()     
    # 构建相关节点
    poc = yf_node.YF_PointCloud(nodeName["PointCloud"],"PointCloud") 
    cost = yf_node.YF_CostMap(nodeName["CostMap"],"CostMap") 
    showMap = yf_node.YF_CompressedImage(nodeName["ShowMap"],"ShowMap")    
    goal = yf_node.YF_Goal(nodeName["Goal"],"GoalCostMap")       
    # 广播节点的首次初始化
    rclpy.spin_once(poc.node,timeout_sec=0.01)
    rclpy.spin_once(cost.node,timeout_sec=0.01)  
    rclpy.spin_once(showMap.node,timeout_sec=0.01)  
    rclpy.spin_once(goal.node,timeout_sec=0.01)   
    # init a time point for fps
    t = time.time()
    ziel = [0,0]
    while True:         
        # 中断守护
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break       
        # 刷新订阅的节点  
        rclpy.spin_once(cost.node,timeout_sec=0.001)   
        rclpy.spin_once(showMap.node,timeout_sec=0.001)      
        # print fps
        print("CostMap Fps: ", int(1/(time.time()-t)))        
        t = time.time()         
        # 等待捕获所有信息
        pcl = poc.poc_array
        ziel = goal.subMsg
        if pcl is not None:
            print("Waiting for new point cloud")
            continue
        if ziel is not None:
            print("Waiting for new target")
            continue
        # 获得地图
        target = get_target(ziel)
        obMap,sendMap = get_obMap(pcl,poc,target)
        obMap = obMap.flatten().astype(np.uint8)
        # 广播地图
        cost.publishMsg(obMap)
        showMap.publishMsg(sendMap.astype(np.uint8))
    # 杀死无用节点
    cost.node.destroy_node()
    showMap.node.destroy_node()
    poc.node.destroy_node()
    goal.node.destroy_node()
    # 结束rclpy
    rclpy.shutdown()
