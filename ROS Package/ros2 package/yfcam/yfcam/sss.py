
import cv2
import copy
import numpy as np


class Template_Matching:
    """多尺度多模板的模板匹配模块初始化数据

        tNum： 储存模板数量（default = 10）
        tSize： resize模板（default = 10）

        treshold: 模板匹配的阈值（0-1）（default = 0.6）
        """
    def __init__(self, tNum=1, tSize=150, threshold=0.60):
        self.tNum = tNum
        self.tSize = tSize

        self.threshold = threshold

        self.tem_buffer = np.zeros([tNum+1,tSize,int(tSize*1.5)])
        self.bufferIndex = 0

        self.origin_template = None
        self.method = cv2.TM_CCOEFF_NORMED
        
                  

    def ready_tem(self, img, objs):        
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        tem = img[ objs[0][1]:objs[1][1],objs[0][0]:objs[1][0]]

        tem = cv2.resize(tem, (self.tSize,self.tSize))
        return tem
    
    def set_tem(self, tem):
        self.tem_buffer[self.bufferIndex,:,:] = tem
#        cv2.imshow("template",tem)
    def get_tem(self):
        return (np.sum(self.tem_buffer[1:,:,:],axis=0)/self.tNum).astype(np.uint8)
        
    def new_template(self, init,img, objs):
        new_tem = self.ready_tem(img, objs)
        
        if self.bufferIndex >= self.tNum:
            self.bufferIndex = 1
            
        if init:            
            self.tem_buffer = np.zeros([self.tNum+1,self.tSize,self.tSize])
            for i in range(self.tNum+1):
                self.bufferIndex = i
                self.set_tem(new_tem) 
            return True,1
                
        old_tem = self.get_tem()
        
        res = cv2.matchTemplate(new_tem,old_tem,self.method)
        (_, maxVal, _, _) = cv2.minMaxLoc(res)

        if maxVal > self.threshold:
            self.set_tem(new_tem)
            self.bufferIndex += 1
            return True,maxVal
        
        return False,0
    
def main(args=None):
    rclpy.init(args=args)
    
    cam = LeftCam()
    obj = ObjectsArray()
    goal = GoalPub()
    poc = PointCloud() 
    cost = CostMap() 

    rclpy.spin_once(goal.nodeGoal,timeout_sec=0.05)
    rclpy.spin_once(cost.nodeCost,timeout_sec=0.05)
    i_get_tem = True
    TemMatch = Template_Matching()


    cache = None
    while 1:  
        rclpy.spin_once(cam.nodeImg,timeout_sec=0.05) 
        rclpy.spin_once(obj.nodeObj,timeout_sec=0.05)
        rclpy.spin_once(poc.nodePoc,timeout_sec=0.05)
        # Read frame from camera
        image_np = cam.image_data
        object_array = obj.obj_array  
        point_cloud = poc.poc_image
        pcl = poc.poc_array
        
        if image_np is None:
            print("Waiting for new Image")
            continue             
        if object_array is None:
            print("Waiting for new Object")
            continue
        if point_cloud is None:
            print("Waiting for new Point Cloud")
            continue
        acc_l = []
        cache = None
        for objects in object_array.markers:
            top_left = (int(objects.points[0].x), int(objects.points[0].y))
            down_right = (int(objects.points[2].x), int(objects.points[2].y))
            res,acc = TemMatch.new_template(i_get_tem,image_np,(top_left,down_right))
            cv2.rectangle(image_np, top_left, down_right, (255,0,255), 3)
            i_get_tem = not res
            if res:
                acc_l.append(acc)
                cache = objects
           
        pcl = np.floor(pcl * 100).astype(np.uint32)# z up x forward
        gridmap = np.zeros_like(pcl)
        gridmap[:,0] = pcl[:,1]+ 250
        gridmap[:,1] = pcl[:,0]
        gridmap[:,2] = np.abs(pcl[:,2] + 80)
        gridmap = np.delete(gridmap,np.where(gridmap[:,0]>500),0)   
        gridmap = np.delete(gridmap,np.where(gridmap[:,0]<0),0)
        gridmap = np.delete(gridmap,np.where(gridmap[:,1]>500),0)
        gridmap = np.delete(gridmap,np.where(gridmap[:,2]>180),0)
        
        if cache is not None:
            goal.goal.header = cache.header
            goal.goal.pose = cache.pose
            goal.goal_callback()
            rclpy.spin_once(goal.nodeGoal,timeout_sec=0.05)
            cv2.rectangle(image_np, top_left, down_right, (0,0,255), 3)
            cv2.circle(image_np, (int((top_left[0] + down_right[0]) / 2),
                                    int((top_left[1] + down_right[1]) / 2)), 
                                    2, (0,0,255), 3)
        else:
#            print("tm fail ~~~~~~~~~~~~~~~~~")
            pass
        
        kernel = np.ones((5,5),np.uint8)
        testmap = np.zeros([501,501,3])
        sendmap = np.zeros([501,501])
        
        small = np.where([gridmap[:,2]<25])
        large = np.where([gridmap[:,2]>=25])
#        print(large,small)
        testmap[gridmap[small,1],gridmap[small,0],1] = 1
        testmap[gridmap[large,1],gridmap[large,0],2] = 1
        
        sendmap[gridmap[large,1],gridmap[large,0]] = 1
        sendmap = cv2.resize(sendmap,(250,250))
        cv2.imshow("Test Map",sendmap)
        
        testmap[:,:,0] = 255
        testmap[:,:,1] = cv2.dilate(testmap[:,:,1],kernel,iterations=1)*255
        testmap[:,:,2] = cv2.dilate(testmap[:,:,2],kernel,iterations=1)*255
        
        testmap[:,:,0][testmap[:,:,1]==255] = 0
        testmap[:,:,0][testmap[:,:,2]==255] = 0
        
        costmap = np.zeros([501,501])
        costmap[testmap[:,:,0]==255] = 0 # dont know
        costmap[testmap[:,:,1]==255] = 0 # moveable
        costmap[testmap[:,:,2]==255] = 1 # can not go
#        costmap[testmap[:,:,1]==255 and testmap[:,:,0]==2] = 200 # try to avoid
        
        testmap = cv2.resize(testmap,(250,250))
        cost.data = costmap
        
        rclpy.spin_once(cost.nodeCost,timeout_sec=0.05)
        
#        cv2.imshow("Test Map",testmap)  
#        cv2.imshow("Live Video",image_np) 
        
#        cv2.imshow("Point Cloud",point_cloud)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

    cam.nodeImg.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
