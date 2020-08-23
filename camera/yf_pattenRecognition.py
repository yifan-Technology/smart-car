import numpy as np
class Feature_Space:
    def __init__(self, capasity = 15, featureNum = 4):
        self.featureNum = featureNum
        self.capasity = capasity
        self.fs = np.ones([capasity,featureNum])
        self.idx = 0

    def push(self, inputs):
        data = np.zeros([1,self.featureNum])
        for i in range(len(inputs)):
            data[:,i] = inputs[i]
        self.fs = np.delete(self.fs,0,axis=0)

#        print(self.fs.shape,data.shape)
        self.fs = np.vstack((self.fs,data))

    def get_mean(self):
        var = np.abs(np.max(self.fs,axis=0) - np.min(self.fs,axis=0))
        mean = np.mean(self.fs, axis=0)
        return var,mean

    def predict(self, data):        
        if self.idx <self.capasity:              
            self.idx += 1            
            return 0
        var,mean = self.get_mean()
#        print("var,mean:",self.get_mean())
#        print("inpu: ",data)
#        print("in fs:",(data-mean))
#        print("acc",np.sum((np.abs(data-mean))**2))
        
        
        return np.sum((np.abs(data-mean))**2)
        

        
