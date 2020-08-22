import numpy as np
class Feature_Space:
    def __init__(self, capasity = 15, featureNum = 2):
        self.fs = np.ones([capasity,2])
        self.init = True

    def push(self, data):

        self.fs = np.delete(self.fs,0,axis=0)

        self.fs = np.hstack(self.fs,data)

    def get_mean(self):
        var = np.var(self.fs,axis=1)
        mean = np.mean(self.fs, axis=1)
        return mean/var

    def predict(self, data):        
        if self.init:
            for i in range(len(data)):
                self.fs[:,i] = data[i]
            return 0

        return np.linalg.norm(data-self.get_mean())

        

        
