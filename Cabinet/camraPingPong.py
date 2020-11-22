#import pyzed.sl as sl
import pyzed.sl as sl
import cv2
import numpy as np

ERROR = "error: "
class ANTI_LGBT:
    def __init__(self,fps = 30,delta = 5):
        
        # Create a Camera object
        self.zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.VGA  # Use HD1080 video mode
        init_params.camera_fps = 30  # Set fps at 30
        init_params.coordinate_units = sl.UNIT.METER

        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)

        self.image = sl.Mat()
        self.point_cloud = sl.Mat()
        self.runtime_parameters = sl.RuntimeParameters()
        self.delta = delta

        self.pose = [0, 0, 0]

    def __del__( self ):        
        # out.release()
        cv2.destroyAllWindows()
        # Close the camera
        self.zed.close()

    def get_pose(self):
        return self.pose

    def findPoint(self, img, low, up):
        lower = np.array(low)
        upper = np.array(up)
        mask = cv2.inRange(img, lower, upper)
        res = cv2.bitwise_and(img,img, mask= mask)
        return mask

    def findCenter(self, mask):
        idxList = np.argwhere(mask!=0)
        num,_ = idxList.shape
        sumIdx = np.sum(idxList,axis = 0)
        if num == 0:
            return -1,-1
        return  int(sumIdx[1]/num), int(sumIdx[0]/num)

    def rukusive(self,point_cloud,point,delta):    
        smallPointCloud = point_cloud[point[0]-delta:point[0]+delta, point[1]-delta:point[1]+delta]
        temp = smallPointCloud[~np.isnan(smallPointCloud)].reshape([-1,4])
        return np.mean(temp,axis=0)

    def search_Point(self,point_cloud,input_p):    
        point = (input_p[1],input_p[0])
        num = 0    
        delta = 1
        if np.isnan(point_cloud[point][0]):        
            while delta <= 20:
                res = self.rukusive(point_cloud,point,delta)
                # print (res, delta)
                if not np.isnan(res[0]):
                    return res
                else :
                    delta += 1
        else:
            return point_cloud[point]
        return [-1,-1,-1,-1]

    def get_Pose(self, point_cloud,g,o):
        greenPoint = self.search_Point(point_cloud,g)
        orangePoint = self.search_Point(point_cloud,o)

        if greenPoint[0] == -1 :
            print(ERROR+"bad left point cloud")
            return [-1,-1,-1,-1]
        if orangePoint[0] == -1:
            print(ERROR+"bad right point cloud")
            return [-1,-1,-1,-1]

        dis = np.linalg.norm(greenPoint-orangePoint)
        if np.abs(dis-0.7)>0.1:        
            print(ERROR+"bad width")
            return [-1,-1,-1,-1]
        else:
            return (greenPoint+orangePoint)/2
    
    def run(self):
        while 1:
            if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                # A new image is available if grab() returns SUCCESS
                self.zed.retrieve_image(self.image, sl.VIEW.LEFT)            
                self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZ)
                delta = self.delta
                # 取出图片
                img = self.image.get_data()   
                poc = self.point_cloud.get_data()
                # cv2.imshow("origi2n", poc)
                img= np.delete(img, -1, axis=2)  

                mask_green  = self.findPoint(img, [ 90 - delta, 145 - delta,  45 - delta],
                                            [ 100 + delta, 155 + delta, 55 + delta])

                mask_orange = self.findPoint(img, [ 40 - delta,  100 - delta, 145 - delta],
                                            [ 55 + delta, 115 + delta, 160 + delta])
                
                # cv2.imshow("test orange", mask_orange)
                # cv2.imshow("test green" , mask_green )

                green_Center = self.findCenter(mask_green)
                orangeCenter = self.findCenter(mask_orange)
                
                if green_Center[0] == -1 or orangeCenter[0] == -1:            
                    print(ERROR+"can not find center")
                    continue
                elif green_Center[0] > orangeCenter[0]:    
                    print(ERROR+"wrong center finded")
                    continue
                else :
                    cv2.circle(img, green_Center, 15, (0, 255,  0), -1) 
                    cv2.circle(img, orangeCenter, 15, (0, 165,255), -1)

                    position = self.get_Pose(poc,green_Center,orangeCenter) 
                    if position[0] == -1 and position[1] == -1 and position[2] == -1 and position[3] == -1:                    
                        print(ERROR+"bad position")
                        continue
                    result = (position[2],position[0],0)
                    self.pose = result
                
                # cv2.imshow("origin", img)
                # cv2.imshow("point Cloud", poc)
                if(cv2.waitKey(1) == ord('q')): 
                    break

                
if __name__ == "__main__":
    # main()
    testClass = ANTI_LGBT()
    testClass.run()