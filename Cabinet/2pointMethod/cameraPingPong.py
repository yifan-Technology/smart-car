import pyzed.sl as sl
import cv2
import numpy as np

DEBUG = False
ERROR = "Camera Error: "

class ANTI_LGBT:
    """[YF camera ]
    """    
    def __init__(self,fps = 30,delta = 10):
        """[YF enter cabinet environment awareness]

        Args:
            fps (int, optional): [set the fps of zed2 camera]. Defaults to 30.
            delta (int, optional): [set delta for threshold]. Defaults to 10. not used
        """
        
        # Create a Camera object
        self.zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.VGA  # Use vga 378 * 672 video mode
        init_params.camera_fps = fps                       # Set fps at 30
        init_params.coordinate_units = sl.UNIT.METER       # Set unit meter

        # If already Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)

        # Inside sm
        self.state = "Phase: approach"                     # Phase: approach/Phase: goin

        # Init all needed matrix
        self.image = sl.Mat()
        self.image_both = sl.Mat()
        self.point_cloud = sl.Mat()

        # Delta member by trying to give a window size for the threshold
        # Because tranlate from BGR color space to HSV color space
        #  , delta was been abandoned
        self.delta = delta

        # Must have this part when runing the zed camere
        self.runtime_parameters = sl.RuntimeParameters()

        # Init give out information: Post, and error info lostTarget
        self.loseTarget = [np.nan, np.nan,np.nan,np.nan,np.nan,np.nan, np.nan]
        self.pose = [np.nan, np.nan,np.nan,np.nan,np.nan,np.nan, np.nan]

    def __del__( self ):  
        """[Destructor]
        """       
        # close all windows
        cv2.destroyAllWindows()
        # Close the camera
        self.zed.close() 

    def _findPoint(self, img, low, up, r = 8):
        """generate the mask, by using low up thresholds, after that do a open operation


        Args:
            img ([cv2:mat]): Target image for now using hsv image
            low ([np.array]): lower limit of the threshold
            up ([np.array]): upper limit of the threshold
            r (int, optional): [description]. Defaults to 8.

        Returns:
            [cv2:mat]: a hsv image after threshold segmentation and open operation
        """        
        # Threshold segmentation
        lower = np.array(low,np.uint8)
        upper = np.array(up,np.uint8)
        mask = cv2.inRange(img, lower, upper)
        mask[mask > 0] = 1
        res = cv2.bitwise_and(img,img, mask= mask)
        # Open operation

        # hzs: We might can try to use different kernel to do a unclassical Openoperation
        # when eroded use a big kernel, the use a small kernel to dilated. The benefit should 
        # be that can avoid target point is at the edge of the disc, then the point cloud can 
        # not obtain a meanfull value

        kernel1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(r, r))
        kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(r, r))
        eroded = cv2.erode(mask,kernel1)
        dilated = cv2.dilate(eroded,kernel2)
        return dilated

    def _findCenter(self, mask):
        """[find the gravity center of the mask]

        !: did not use replace by findLeftCenter && findRightCenter

        Args:
            mask ([cv2.mat]): a mask bw image

        Returns:
            [tuple]: a point which is the gravity of the mask
        """        
        idxList = np.argwhere(mask!=0)
        num,_ = idxList.shape
        sumIdx = np.sum(idxList,axis = 0)
        if num == 0:
            return -255,-255
        return  int(sumIdx[1]/num), int(sumIdx[0]/num)

    def _findLeftCenter(self, mask):
        """[find the rightest point of the mask]

        Args:
            mask ([cv2.mat]): a mask bw image

        Returns:
            [tuple]: a point which is the rightest point of the mask
        """        
        idxList = np.argwhere(mask!=0)
        num,_ = idxList.shape
        sumIdx = np.sum(idxList,axis = 0)
        if num == 0:
            return -255,-255
        return  int(np.max(idxList[:,1])), int(sumIdx[0]/num)

    def _findRightCenter(self, mask):
        """[find the leftest point of the mask]

        Args:
            mask ([cv2.mat]): a mask bw image

        Returns:
            [tuple]: a point which is the leftest point of the mask
        """        
        idxList = np.argwhere(mask!=0)
        num,_ = idxList.shape
        sumIdx = np.sum(idxList,axis = 0)
        if num == 0:
            return -255,-255
        return  int(np.min(idxList[:,1])), int(sumIdx[0]/num)

    def _recursive(self,point_cloud,point,delta):
        """[a recursive function]

        Args:
            point_cloud ([cv2:mat]): Point cloud
            point ([tuple]): a point (seed)
            delta ([int]): search step
        Returns:
            [tuple]: the mean value of the search area
        """        
        smallPointCloud = point_cloud[point[0]-delta:point[0]+delta, point[1]-delta:point[1]+delta]
        temp = smallPointCloud[~np.isnan(smallPointCloud)].reshape([-1,4])
        return np.mean(temp,axis=0)

    def _search_Point(self,point_cloud,input_p):
        """[search point cloud from select point]

        Args:
            point_cloud ([cv2.mat]): target point cloud
            input_p ([tuple]): start point

        Returns:
            [list]: a position in point cloud
        """        
        point = (input_p[1],input_p[0])
        num = 0    
        delta = 1
        if np.isnan(point_cloud[point][0]):        
            while delta <= 20:
                res = self._recursive(point_cloud,point,delta)
                # print (res, delta)
                if not np.isnan(res[0]):
                    return res
                else :
                    delta += 1
        else:
            return point_cloud[point]
        return [-1,-1,-1,-1]

    def _get_Pose(self, point_cloud,r,l):
        """[From image space to world space]

        Args:
            point_cloud ([cv2.mat]): Point cloud
            r ([tuple]): right point(image space)
            l ([tuple]): left  point(image space)

        Returns:
            [tuple]: (center position,right position,left position)
        """        
        rightPoint = self._search_Point(point_cloud,r)
        left_Point = self._search_Point(point_cloud,l)

        if rightPoint[0] == -255 or np.isneginf(rightPoint[0]) or np.isnan(rightPoint[0]):
            if DEBUG:
                print(ERROR+"bad right point cloud")
            return [[-255,-255,-255,-255]]
        if left_Point[0] == -255 or np.isneginf(left_Point[0]) or np.isnan(left_Point[0]):
            if DEBUG:
                print(ERROR+"bad left point cloud")
            return [[-255,-255,-255,-255]]

        dis = np.sqrt((rightPoint[0]-left_Point[0])**2+(rightPoint[1]
                    -left_Point[1])**2+(rightPoint[2]-left_Point[2])**2)
        
        if np.abs(dis-0.65)>1.55 :       
            if DEBUG: 
                print(ERROR+"bad width")
            return [[-255,-255,-255,-255]]
        else:
            return (rightPoint+left_Point)/2,rightPoint,left_Point

    def get_pose(self):
        """[getter for resule]

        Returns:
            [list]: [result list] 1 * 7
        """        
        res = self.pose  
        # self.pose = self.loseTarget
        return res  

    def _connectedComponent(self, mask):
        """
        """
        
        num_labels, _, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if num_labels == 3:
            point1 = centroids[1]
            point2 = centroids[2]

            if point1[0] < point2[0]:
                leftPoint = (int(point1[0]),int(point1[1]))
                rightPoint = (int(point2[0]),int(point2[1]))
                return leftPoint,rightPoint
            else:                
                leftPoint = (int(point2[0]),int(point2[1]))
                rightPoint = (int(point1[0]),int(point1[1]))
                return leftPoint,rightPoint
        else :
            return [-255,-255],[-255,-255]

    def _approach(self):
        """1st mode: approach enter position
        """        
        # A new image is available if grab() returns SUCCESS
        self.zed.retrieve_image(self.image, sl.VIEW.LEFT)            
        self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZ)
        # zed to numpy
        img = self.image.get_data()   
        poc = self.point_cloud.get_data()
        
        delta = 0 # self.delta by using bgr color space should use delta

        img= np.delete(img, -1, axis=2)  
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        """ if use BGR color space we can use this threshold
        # BGR
        mask_right  = self._findPoint(img, [ 20 - delta, 20 - delta,  90 - delta],
                                          [ 35 + delta, 35 + delta, 110 + delta])

        mask_left = self._findPoint(img, [ 120 - delta, 45 - delta, 20 - delta],
                                        [ 150 + delta, 55 + delta, 30 + delta])
        """
        # HSV
        mask_right = self._findPoint(hsv,  [ 5 - delta, 200 - delta,  150 - delta],
                                          [ 10 + delta, 255 + delta, 255 + delta])

        mask_left = self._findPoint(hsv, [ 100 - delta, 150 - delta, 100 - delta],
                                        [ 124 + delta, 255 + delta, 255 + delta])
        
        if DEBUG:            
            cv2.imshow("Point Cloud", poc)
            cv2.imshow("Mask merge" , (mask_left + mask_right)*255 ) 
            cv2.imshow("Mask on the right", mask_right * 255 ) 
            cv2.imshow("Mask on the left", mask_left  * 255 ) 

        # find the r&l position
        right_Center = self._findCenter(mask_right)
        left__Center = self._findCenter(mask_left)
        
        # check if there is error happend
        if right_Center[0] == -255 and left__Center[0] != -255:
            print(ERROR+"only find left Sign!!!")                
            cv2.circle(img, left__Center, 15, (255, 0,  0), -1)
            self.pose = [np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,2]
        elif left__Center[0] == -255 and right_Center[0] != -255:
            print(ERROR+"only find right Sign!!!")                
            cv2.circle(img, right_Center, 15, (0, 165,255), -1) 
            self.pose = [np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,1]
        elif right_Center[0] == -255 and left__Center[0] == -255:   
            
            if DEBUG:         
                print(ERROR+"can not find center")                    
            self.pose = self.loseTarget
        elif right_Center[0] < left__Center[0]: 
            if DEBUG:   
                print(ERROR+"wrong center fined")                    
            self.pose = self.loseTarget
        else :
            cv2.circle(img, left__Center, 5, (255, 0,  0), -1) 
            cv2.circle(img, right_Center, 5, (0, 165,  255), -1)

            c_0 = int((right_Center[0]+left__Center[0])/2)
            c_1 = int((right_Center[1]+left__Center[1])/2)
            cv2.circle(img, (c_0,c_1), 5, (0, 0,  255), -1)

            position = self._get_Pose(poc,right_Center,left__Center) 
            if position[0][0] == -255 and position[0][1] == -255 and position[0][2] == -255 and position[0][3] == -255:                    
                if DEBUG:
                    print(ERROR+"bad position")
                self.pose = self.loseTarget
            else:
                result = (  position[2][0] - 0.06, position[2][2], 
                            position[0][0] - 0.06, position[0][2],
                            position[1][0] - 0.06, position[1][2], 
                            0)
                self.pose = result
                
        cv2.imshow("origin", img)

    def _goin(self):
        """2nd mode: straight going in
        """        
        
        self.zed.retrieve_image(self.image_both, sl.VIEW.LEFT)  
        
        img = self.image_both.get_data()
        img= np.delete(img, -1, axis=2)   
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        delta = 0 # self.delta
        mask_in = self._findPoint(hsv, [ 47 - delta, 35 - delta, 75 - delta],
                                        [ 67 + delta, 127 + delta, 127 + delta])
        right = [-255,-255]
        left  = [-255,-255]
        # right = self._findLeftCenter( mask_in)
        # left  = self._findRightCenter(mask_in)
        left,right = self._connectedComponent( mask_in)
        print(left,right)
        if (right[0]!=-255 or left[0]!= -255):
            cv2.circle(img, left, 5, (255, 0,  0), -1) 
            cv2.circle(img, right, 5, (0, 165,  255), -1)            

            c_0 = int((right[0]+left[0])/2)#336
            c_1 = int((right[1]+left[1])/2)
            cv2.circle(img, (c_0,c_1), 5, (0, 0,  255), -1)
            if np.abs(left[0] - right[0]) < 20:
                result = ( np.nan, np.nan, np.nan, np.nan,np.nan, np.nan, np.nan)
            else:
                result = (  left[0] - 336, left[1], 
                            c_0 - 336, c_1,
                            right[0] - 336, right[1], 3)   
        else:
            result = self.loseTarget                 
        self.pose = result
        if DEBUG:
            cv2.imshow("mask", mask_in *255)  
        cv2.imshow("mask", mask_in *255)    
        
        cv2.imshow("origin", img)      

    def run(self):
        # while 1:
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            if self.state == "Phase: approach":
                self._approach()
            elif self.state == "Phase: goin": 
                self._goin()   
            
            if(cv2.waitKey(1) == ord('q')):         
                # Close the camera
                self.zed.close()
            
              
if __name__ == "__main__":
    testClass = ANTI_LGBT()
    DEBUG = True
    while 1:

        testClass.run()
        key = cv2.waitKey(1) 
        if(key== ord('q')): 
            print("尝试退出")
            break        
        elif(key == ord('g')): 
            print("切换成进入状态")
            testClass.state = "Phase: goin"            
            cv2.destroyAllWindows()
        elif(key == ord('a')): 
            print("切换成抵近状态")
            testClass.state = "Phase: approach"
            cv2.destroyAllWindows()
         
    
    