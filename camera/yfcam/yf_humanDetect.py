
import cv2

import numpy as np

class Template_Matching:
    def __init__(self, tSize=200, method=cv2.TM_CCOEFF_NORMED):
        self.method = method
        self.tSize = tSize
        self.tem = None     
        self.new_tem = None  
        self.asp = None

    def ready_tem(self, img, objs):        
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        tem = img[ objs[0][1]:objs[1][1],objs[0][0]:objs[1][0]]
        self.asp = (objs[0][1]-objs[1][1])/(objs[0][0]-objs[1][0])
        tem = cv2.resize(tem, (self.tSize,int(self.tSize*1.5)))
        return tem
    
    def set_tem(self):
        self.tem = self.new_tem
        

        
    def new_template(self, img, objs):
        self.new_tem = self.ready_tem(img, objs)
                    
        if self.tem is None:
            self.tem = self.new_tem
            return 1,self.asp

        res = cv2.matchTemplate(self.new_tem,self.tem,self.method)
        (_, maxVal, _, _) = cv2.minMaxLoc(res)
        cv2.imshow("template",self.tem)
        return maxVal,self.asp
