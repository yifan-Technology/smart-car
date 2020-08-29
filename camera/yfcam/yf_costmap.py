import cv2
import numpy as np


class costMap:
    def __init__(self, mapSize=100, visSize = 501, h_self=80, h_top=180, feld=500):
        self.mapSize = mapSize
        self.visSize = visSize
        self.feld = feld
        self.h_top = h_top
        self.h_self = h_self

        self.gridmap = None

    def pointCloud2map(self,pcl):
        pcl = np.floor(pcl * 100).astype(np.uint32)# z up x forward
        gridmap = np.zeros_like(pcl)
        gridmap[:,0] = pcl[:,1]+ 250
        gridmap[:,1] = pcl[:,0]
        gridmap[:,2] = np.abs(pcl[:,2] + 53)
        gridmap = np.delete(gridmap,np.where(gridmap[:,0]>self.feld),0)   
        gridmap = np.delete(gridmap,np.where(gridmap[:,0]<0),0)
        gridmap = np.delete(gridmap,np.where(gridmap[:,1]>self.feld),0)
        gridmap = np.delete(gridmap,np.where(gridmap[:,2]>self.h_top),0)
        self.gridmap = gridmap
        
    def obstaclemap(self):    
        
        self.sendmap = np.zeros([self.visSize,self.visSize])
        gridmap = self.gridmap            
        large = np.where([gridmap[:,2]>=25])
        self.sendmap[gridmap[large,1],gridmap[large,0]] = 1
        sendmap = cv2.resize(self.sendmap,(self.mapSize,self.mapSize))
        return sendmap

    def visualMap(self):
                
        self.vismap = np.zeros([self.visSize,self.visSize,3])
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
