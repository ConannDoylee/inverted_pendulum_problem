import numpy as np
from google.protobuf import text_format
from matplotlib import pyplot as plt
import os

from conf_proto import params_conf_pb2

class PID(object):

    def __init__(self,path_root,T=0.01):
        self.root = path_root
        self.conf_file = self.root + "/conf/controller_conf.pb.txt"
        self.load_conf()
        self.T = T
        return
    
    def load_conf(self):
        self.controller_conf = params_conf_pb2.PIDConf()
        f = open(self.conf_file,'rb')
        text_format.Parse(f.read(),self.controller_conf)
        print("controller_conf: ",self.controller_conf)
        f.close()

        self.integ_error = np.zeros(1)
        self.error_pre = np.zeros(1)
        return

    def compute_d_error(self,error):
        d_error = (error - self.error_pre) / self.T
        return d_error

    def update(self,x,xd):
        KP = self.controller_conf.kp
        KI = self.controller_conf.ki
        KD = self.controller_conf.kd
        error = xd - x
        d_error = self.compute_d_error(error)
        u = -(KP * error + KI * self.integ_error + KD * d_error)

        self.integ_error += error * self.T
        self.error_pre = error
        return u

    def test(self):
        for i in np.arange(12):
            u = self.update(1,2)
            print(u,self.integ_error)
        return


def main(root):
    pid = PID(root,0.01)
    pid.test()
    plt.show()

# if __name__ == '__main__':
#     main(".")