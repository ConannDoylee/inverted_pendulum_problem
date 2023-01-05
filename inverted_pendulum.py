import numpy as np
from google.protobuf import text_format
from conf_proto import params_conf_pb2
import copy
from matplotlib import pyplot as plt
import animator

# Inverted Pendulem 
# 1. Parameters
# cart: 
# (1) mass(kg): M
# (2) length(m): 2l
# pole:
# (1) mass(kg): m
# 2. States
# cart translation(m): y
# pole rotation(rad): theta, clockwise is positive
# 3. Inputs
# cart translation force(N): u
# 4. Else
# resistance coefficient: K

class InvertedPendulem(object):

    def __init__(self,path_root):
        self.root = path_root
        self.conf_file = self.root + "/conf/model.pb.txt"
        self.load_conf()
        return

    def load_conf(self):
        self.model_conf = params_conf_pb2.PendulumConf()
        f = open(self.conf_file,'rb')
        text_format.Parse(f.read(),self.model_conf)
        print("model_conf: ",self.model_conf)
        
        # sim params
        self.N = self.model_conf.N
        self.T = self.model_conf.T
        # initial state
        self.X = np.zeros(4)
        self.X[0] = self.model_conf.x1_init
        self.X[1] = self.model_conf.x2_init
        self.X[2] = self.model_conf.y1_init
        self.X[3] = self.model_conf.y2_init
        self.y1_max = self.model_conf.y1_max
        self.y1_min = self.model_conf.y1_min

        return

    def update(self,u):
        self.odeRK4(u)
        return
    
    def f(self,X_k,u):
        F = np.zeros(4)
        x1 = X_k[0]
        x2 = X_k[1]
        y1 = X_k[2]
        y2 = X_k[3]
        # model params
        m = self.model_conf.m
        M = self.model_conf.M
        l = self.model_conf.l
        a = 1 / (m + M)
        g = 9.81
        K = self.model_conf.K

        # y1 saturation situation
        if u < 0 and y1 <= self.y1_min or u > 0 and y1 >= self.y1_max:
            F[0] = x2
            F[1] = (g*np.sin(x1)-l*x1) / (4/3*l)
            F[2] = 0
            F[3] = 0
            return F


        F[0] = x2
        F[1] = (g*np.sin(x1)-l*x1-a*m*l*x2*x2*np.sin(2*x1)/2-a*np.cos(x1)*u) / (4/3*l-a*m*l*np.cos(x1)*np.cos(x1))
        F[2] = y2
        F[3] = a*(u+m*l*np.sin(x1)*x2*x2-m*l*np.cos(x1)*F[1]-K*y2)

        return F
    
    def odeRK4(self,u):
        dt = self.T / self.N
        for i in np.arange(self.N):
            K1 = self.f(self.X,u)
            K2 = self.f(self.X+K1*dt/2,u)
            K3 = self.f(self.X+K2*dt/2,u)
            K4 = self.f(self.X+K3*dt,u)
            self.X += dt/6.0*(K1+2.0*K2+2.0*K3+K4)
            # y1 saturation
            if self.X[2] <= self.y1_min:
                self.X[2] = self.y1_min
                self.X[3] = 0
            if self.X[2] >= self.y1_max:
                self.X[2] = self.y1_max
                self.X[3] = 0
        return

    def state(self):
        return copy.copy(self.X)

    def test(self):
        x1_list = []
        x2_list = []
        y1_list = []
        y2_list = []
        # u_list = np.random.normal(0,1,1000)
        u_list = [0]*1000
        for u in u_list:
            self.update(u)
            X = self.state()
            x1_list.append(X[0])
            x2_list.append(X[1])
            y1_list.append(X[2])
            y2_list.append(X[3])

        plt.figure('Inverted Pendulum Test')
        plt.subplot(3,1,1)
        plt.plot(x1_list,label='x1')
        plt.plot(x2_list,label='x2')
        plt.grid()
        plt.legend()

        plt.subplot(3,1,2)
        plt.plot(y1_list,label='y1')
        plt.plot(y2_list,label='y2')
        plt.grid()
        plt.legend()

        plt.subplot(3,1,3)
        plt.plot(u_list)
        plt.grid()
        plt.legend()


        animator_obj = animator.Animator(self.root)

        locations = []
        for y1,x1 in zip(y1_list,x1_list):
            temp = [y1,x1]
            locations.append(temp)
        animator_obj.set_locations(locations)
        animator_obj.animate()
        animator_obj.save_gif()

        return

def main(root):
    model = InvertedPendulem(root)
    model.test()
    # plt.show()

if __name__ == '__main__':
    main(".")