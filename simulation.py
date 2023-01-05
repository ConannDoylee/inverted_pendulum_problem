import inverted_pendulum
import animator
from matplotlib import pyplot as plt
import numpy as np
import pid_controller
from google.protobuf import text_format
from conf_proto import params_conf_pb2


class Simulation(object):
    
    def __init__(self,path_root):
        self.root = path_root
        self.inverted_pendulum = inverted_pendulum.InvertedPendulem(path_root)
        self.pid_controller = pid_controller.PID(path_root)
        self.animator = animator.Animator(path_root)

        self.conf_file = self.root + "/conf/simulation_conf.pb.txt"
        self.load_conf()
        self.init_list()
        return

    def load_conf(self):
        self.simu_conf = params_conf_pb2.SimulationConf()
        f = open(self.conf_file,'rb')
        text_format.Parse(f.read(),self.simu_conf)
        print("simu_conf: ",self.simu_conf)
        self.T = self.simu_conf.T
        self.cycle = self.simu_conf.cycle

        return
        
    def init_list(self):
        self.t_list = []
        self.x1_list = []
        self.x2_list = []
        self.y1_list = []
        self.y2_list = []
        self.u_list = []
        return

    def simulate(self):
        for i in np.arange(self.cycle):
            t = i * self.T
            # ref && state
            x1_d = 0
            X = self.inverted_pendulum.state()
            x1 = X[0]
            x2 = X[1]
            y1 = X[2]
            y2 = X[3]
            # controller
            u = self.pid_controller.update(x1,x1_d)
            # model update
            self.inverted_pendulum.update(u)

            # record
            self.t_list.append(t)
            self.x1_list.append(x1)
            self.x2_list.append(x2)
            self.y1_list.append(y1)
            self.y2_list.append(y2)
            self.u_list.append(u)
        return

    def plot_date(self):
        plt.figure()
        plt.subplot(2,3,1)
        plt.plot(self.t_list,self.x1_list,label='x1')
        plt.grid()
        plt.legend()

        plt.subplot(2,3,2)
        plt.plot(self.t_list,self.x2_list,label='x2')
        plt.grid()
        plt.legend()

        plt.subplot(2,3,3)
        plt.plot(self.t_list,self.y1_list,label='y1')
        plt.grid()
        plt.legend()

        plt.subplot(2,3,4)
        plt.plot(self.t_list,self.y2_list,label='y2')
        plt.grid()
        plt.legend()

        plt.subplot(2,3,5)
        plt.plot(self.t_list,self.u_list,label='u')
        plt.grid()
        plt.legend()

        return

    def plot_diagram(self,name="demo",save_gif=False):
        locations = []
        for y1,x1 in zip(self.y1_list,self.x1_list):
            temp = [y1,x1]
            locations.append(temp)
        self.animator.set_locations(locations)
        self.animator.animate()
        if save_gif:
            self.animator.save_gif(name)
        return
    
    def test(self):

        return

def main(path_root):
    simu = Simulation(path_root)
    simu.simulate()
    simu.plot_date()
    simu.plot_diagram("result",True)
    # plt.show()

if __name__ == '__main__':
    main('.')
