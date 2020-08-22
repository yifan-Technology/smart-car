import cv2
import numpy as np


class costMap:
    def __init__(self, mapSize=250, visSize = 501, h_self=80, h_top=180, feld=500):
        self.sendmap = np.zeros([mapSize,mapSize])
        self.vismap = np.zeros([visSize,visSize,3])
        self.feld = feld
        self.h_top = h_top
        self.h_self = h_self

        self.gridmap = None

    def pointCloud2map(self,pcl):
        pcl = np.floor(pcl * 100).astype(np.uint32)# z up x forward
        gridmap = np.zeros_like(pcl)
        gridmap[:,0] = pcl[:,1]+ np.floor(self.feld/2)
        gridmap[:,1] = pcl[:,0]
        gridmap[:,2] = np.abs(pcl[:,2] + 80)
        gridmap = np.delete(gridmap,np.where(gridmap[:,0]>self.feld),0)   
        gridmap = np.delete(gridmap,np.where(gridmap[:,0]<0),0)
        gridmap = np.delete(gridmap,np.where(gridmap[:,1]>self.feld),0)
        gridmap = np.delete(gridmap,np.where(gridmap[:,2]>self.h_top),0)
        self.gridmap = gridmap
        
    def obstaclemap(self):    
        gridmap = self.gridmap    
        small = np.where([gridmap[:,2]<25])
        large = np.where([gridmap[:,2]>=25])
        sendmap[gridmap[large,1],gridmap[large,0]] = 1
        sendmap = cv2.resize(sendmap,(250,250))
        return sendmap

    def visualMap(self):
                
        kernel = np.ones((5,5),np.uint8)
        gridmap = self.gridmap  
        testmap = self.vismap  
        small = np.where([gridmap[:,2]<25])
        large = np.where([gridmap[:,2]>=25])
        #        print(large,small)
        testmap[gridmap[small,1],gridmap[small,0],1] = 1
        testmap[gridmap[large,1],gridmap[large,0],2] = 1

        testmap[:,:,0] = 255
        testmap[:,:,1] = cv2.dilate(testmap[:,:,1],kernel,iterations=1)*255
        testmap[:,:,2] = cv2.dilate(testmap[:,:,2],kernel,iterations=1)*255

        testmap[:,:,0][testmap[:,:,1]==255] = 0
        testmap[:,:,0][testmap[:,:,2]==255] = 0

        return testmap