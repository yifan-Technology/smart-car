import numpy as np
class Feature_Space:
    '''
    特征空间 类

    默认值：
        capasity(容量) => 追溯多少帧之前的目标
        featureNum(特征数量) => 特征空间维度
    '''
    def __init__(self, capasity = 15, featureNum = 4):
        # 私有化输入值
        self.featureNum = featureNum
        self.capasity = capasity
        # 构建特征空间矩阵
        self.fs = np.ones([capasity,featureNum])
        # 填补空特征矩阵-->index
        self.idx = 0

    def push(self, inputs):
        # 构建对应格式该目标的特征
        data = np.zeros([1,self.featureNum])
        for i in range(len(inputs)):
            data[:,i] = inputs[i]
        # 删除最久远的特征点
        self.fs = np.delete(self.fs,0,axis=0)
        # 将新的特征点加入特征空间
        self.fs = np.vstack((self.fs,data))

    def get_mean(self):
        # 计算方差与均值
        var = np.abs(np.max(self.fs,axis=0) - np.min(self.fs,axis=0))
        mean = np.mean(self.fs, axis=0)
        return var,mean

    def predict(self, data):    
        # 填补空特征矩阵    
        if self.idx <self.capasity:              
            self.idx += 1            
            return 0
        # 获得方差均值
        var,mean = self.get_mean()
#        print("var,mean:",self.get_mean())
#        print("inpu: ",data)
#        print("in fs:",(data-mean))
#        print("acc",np.sum((np.abs(data-mean))**2))
        return np.sum((np.abs(data-mean))**2)
        

        
