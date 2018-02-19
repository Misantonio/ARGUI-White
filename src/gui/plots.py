#!/usr/bin/env python

import sys
import time
from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt import QtGui
import pyqtgraph as pg
import numpy as np
import matplotlib.pyplot as plt


class Plotter(object):
    def __init__(self):
        pass

    def plot_pos_graph(self, data, dataD, t):
        x, xd = np.array(data[0]), np.array(dataD[0])
        y, yd = np.array(data[1]), np.array(dataD[1])
        z, zd = np.array(data[2]), np.array(dataD[2])

        fig1, (ax0, ax1, ax2) = plt.subplots(nrows=3, figsize=(12, 10))
        ax0.plot(t, x-xd)
        ax0.set_title('Error de Posicion en X')
        ax0.set_ylabel('Error (m)')
        ax0.grid()

        ax1.plot(t, y-yd)
        ax1.set_title('Error de Posicion en Y')
        ax1.set_ylabel('Error (m)')
        ax1.grid()

        ax2.plot(t, z-zd)
        ax2.set_title('Error de Posicion en Z')
        ax2.set_xlabel('Tiempo (s)')
        ax2.set_ylabel('Error (m)')
        ax2.grid()

        plt.show()

    def plot_angle_graph(self, data, dataD, t):
        roll, rolld = np.array(data[0]), np.array(dataD[0])
        pitch, pitchd = np.array(data[1]), np.array(dataD[1])
        yaw, yawd = np.array(data[2]), np.array(dataD[2])

        fig1, (ax0, ax1, ax2) = plt.subplots(nrows=3, figsize=(12, 10))
        ax0.plot(t, roll-rolld)
        ax0.set_title('Error de Orientacion en Roll')
        ax0.set_ylabel('Error (rad)')
        ax0.grid()

        ax1.plot(t, pitch-pitchd)
        ax1.set_title('Error de Orientacion en Pitch')
        ax1.set_ylabel('Error (rad)')
        ax1.grid()

        ax2.plot(t, yaw-yawd)
        ax2.set_title('Error de Orientacion en Yaw')
        ax2.set_xlabel('Tiempo (s)')
        ax2.set_ylabel('Error (rad)')
        ax2.grid()

        plt.show()

    def plot_3d_graph(self, data, dataD):
        x, xd = data[0], dataD[0]
        y, yd = data[1], dataD[1]
        z, zd = data[2], dataD[2]

        from mpl_toolkits.mplot3d import Axes3D
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ##### TRAYECTORIA ######
        ax.plot(x, y, z, label="Drone")
        # ax.plot(self.xd_pos_array, self.yd_pos_array, self.zd_pos_array,
        #         label="Trayectoria Deseada")
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()


class GraphWindow(pg.GraphicsWindow):
    def __init__(self, rows, cols, titles):
        super(GraphWindow, self).__init__()

        # Code to draw the GUI
        self.rows = rows
        self.cols = cols
        self.titles = titles
        self.plots = [] # List of PlotItems
        self.curves = [] # List of PlotDataItems in each PlotItem
        self.data = [[]for i in range(self.rows*self.cols)]
        self.dataD = [[] for i in range(self.rows * self.cols)]

        for i in range(self.rows):
            for j in range(self.cols):
                p = self.addPlot()
                self.plots.append(p)
                if titles != [] or len(titles) == 1:
                    p.setLabel('left', self.titles[i+j+1])
                p.setDownsampling(mode='peak')
                p.setClipToView(True)
                pp = p.plot(pen=(155, 255, 50))
                pp1 = p.plot(pen=(55, 130, 255))
                self.curves.append((pp, pp1))
            self.nextRow()

        # Timer to redraw plots
        self.timer = QtCore.QTimer(self)
        self.connect(self.timer, QtCore.SIGNAL('timeout()'),self.update_plots)
        self.timer.start(100)

    def set_titles(self, titles):
        for i, p in enumerate(self.plots):
            p.setLabel('left', titles[i])

    def set_data(self, data, dataD):
        self.data = data
        self.dataD = dataD

    def update_plots(self):
        for i in range(self.rows*self.cols):
            for j,curve in enumerate(self.curves):
                curve[0].setData(self.data[j])
                curve[1].setData(self.dataD[j])


if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    gui = GraphWindow(2,2,[])
    gui.show()
    app.exec_()

