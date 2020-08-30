import cv2
import numpy as np


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
        gridmap[:,0] = pcl[:,1] + self.feld/2
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
        testmap[:,:,0] = 255
        # 完成红，绿两通道的膨胀
        testmap[:,:,1] = cv2.dilate(testmap[:,:,1],kernel,iterations=1)*255
        testmap[:,:,2] = cv2.dilate(testmap[:,:,2],kernel,iterations=1)*255
        # 当红，绿有颜色时，蓝色置为0
        testmap[:,:,0][testmap[:,:,1]==255] = 0
        testmap[:,:,0][testmap[:,:,2]==255] = 0

        return testmap
