import cv2
import numpy as np

class Template_Matching:
    '''
    模板匹配 类
    默认值：
        tSize（模板尺寸）--> 决定模板的宽度
        method（模板匹配方法） --> CCOEFF_NORMED
    '''
    def __init__(self, tSize=200, method=cv2.TM_CCOEFF_NORMED):
        # 私有化所有变量
        self.method = method
        self.tSize = tSize
        # 声明变量
        self.tem = None     
        self.new_tem = None  
        self.asp = None

    def ready_tem(self, img, objs): 
        # bgr2gray       
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # 切分模板
        tem = img[ objs[0][1]:objs[1][1],objs[0][0]:objs[1][0]]
        # 计算长宽比
        self.asp = (objs[0][1]-objs[1][1])/(objs[0][0]-objs[1][0])
        # 保存模板为合适的尺寸
        tem = cv2.resize(tem, (self.tSize,int(self.tSize*1.5)))
        return tem
    
    def set_tem(self):
        # 将新的模板保存为标准模板
        self.tem = self.new_tem

    def new_template(self, img, objs):
        # 获得新的模板
        self.new_tem = self.ready_tem(img, objs)
        # 将第一个obj作为目标
        if self.tem is None:
            print("error: did not set template")
            return 0,0
        # 模板匹配函数
        res = cv2.matchTemplate(self.new_tem,self.tem,self.method)
        # 获得最高的置信度
        (_, maxVal, _, _) = cv2.minMaxLoc(res)
        # # 展示匹配的模板
        # cv2.imshow("template",self.tem)
        return maxVal,self.asp
