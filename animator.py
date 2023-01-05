import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
from conf_proto import params_conf_pb2
from google.protobuf import text_format

class Animator(object):

    def __init__(self,path_root):
        self.root = path_root
        self.conf_file = self.root + "/conf/animator_conf.pb.txt"
        self.load_conf()
        return

    def load_conf(self):
        self.simu_conf = params_conf_pb2.AnimatorConf()
        f = open(self.conf_file,'rb')
        text_format.Parse(f.read(),self.simu_conf)
        print("animator_conf: ",self.simu_conf)
        self.workbench_length = self.simu_conf.workbench_length
        self.cart_length = self.simu_conf.cart_length
        self.pole_length = self.simu_conf.pole_length

        self.x_bound = self.workbench_length/2 + self.cart_length
        self.y_bound = 1.5*self.pole_length
        return

    def set_locations(self,locations):
        self.locations = locations
        return

    def init(self):
        self.ax.set_ylim(-self.y_bound, self.y_bound)
        self.ax.set_xlim(-self.x_bound, self.x_bound)
        return

    def data_gen(self):
        for location in self.locations:
            yield location

    def run(self,location):
        y = location[0]
        theta = location[1]
        worbench_bound = self.workbench_length/2 + self.cart_length/2.0
        self.lines[0].set_data([-worbench_bound,-worbench_bound,worbench_bound,worbench_bound],[0.2,0,0,0.2])
        self.lines[1].set_data([y-self.cart_length/2.0,y+self.cart_length/2.0],[0,0])
        self.lines[2].set_data([y,y+self.pole_length*np.sin(theta)],[0,self.pole_length*np.cos(theta)])

        self.ax.set_ylim(-self.y_bound, self.y_bound)
        self.ax.set_xlim(-self.x_bound, self.x_bound)
        self.ax.figure.canvas.draw()
        return self.lines

    def animate(self):
        self.fig, self.ax = plt.subplots()
        # self.ax.axis("equal")
        self.lines=[]
        self.lines.extend(self.ax.plot([], [], 'k',lw=2))
        self.lines.extend(self.ax.plot([], [],lw=4))
        self.lines.extend(self.ax.plot([], [], 'g',lw=2))

        self.ani = animation.FuncAnimation(self.fig, self.run, self.data_gen, blit=False, interval=10,repeat=False,init_func=self.init,save_count=500)
        return

    def save_gif(self,name='demo'):
        self.ani.save(name+".gif",writer='pillow')
        return

    def test(self):
        locations = [[0,-0.3],[0,0],[0,0.3]]
        self.set_locations(locations)
        self.animate()
        return

def main():
    animator = Animator()
    animator.test()
    # plt.show()

if __name__ == '__main__':
    main()
