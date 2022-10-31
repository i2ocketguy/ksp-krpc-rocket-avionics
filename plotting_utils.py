import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

class data_stream_plot():

    def __init__(self, xlim=None, ylim=None):
        self.fig, self.ax = plt.subplots(1,1)
        #self.ax.set_aspect('equal')
        if xlim is not None:
            self.ax.set_xlim(xlim[0], xlim[1])
        if ylim is not None:
            self.ax.set_ylim(ylim[0], ylim[1])
        self.xdata = []
        self.ydata = []

    def update_data_stream(self, x, y):
        self.xdata.append(x)
        self.ydata.append(y)

    def plot(self):
        self.ax.plot(self.xdata,self.ydata)
    
