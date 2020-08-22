
import cv2

import numpy as np

class Template_Matching:
    def __init__(self, tSize=200, method=cv2.TM_CCOEFF_NORMED):
        self.method = method
        self.tSize = tSize
        self.tem = None     
        self.new_tem = None  

    def ready_tem(self, img, objs):        
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        tem = img[ objs[0][1]:objs[1][1],objs[0][0]:objs[1][0]]
        tem = cv2.resize(tem, (self.tSize,self.tSize))
        return tem
    
    def set_tem(self):
        self.tem = self.new_tem
#        cv2.imshow("template",tem)

    def get_tem(self):
        return (np.sum(self.tem_buffer[1:,:,:],axis=0)/self.tNum).astype(np.uint8)
        
    def new_template(self, img, objs):
        self.new_tem = self.ready_tem(img, objs)
                    
        if self.tem is None:
            self.tem = self.new_tem
            m,n = self.tem.shape
            return 1,m/n

        res = cv2.matchTemplate(self.new_tem,self.tem,self.method)
        (_, maxVal, _, _) = cv2.minMaxLoc(res)

        m,n = self.tem.shape
        return maxVal,m/n