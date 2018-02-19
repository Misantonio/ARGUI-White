#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import importlib
from multiprocessing import Process
import rospy
from pyqtgraph.Qt import QtGui, QtCore
from modules.drones import *
from modules.xml_parser import XMLParser
from modules.utils import *
from gui.plots import GraphWindow
from gui.controlsUi import Ui_Form
from gui.plots import Plotter


class GUI(QtGui.QTabWidget):
    def __init__(self):
        super(GUI, self).__init__()
        # Resize width and height
        self.resize(1000, 520)

        # Widgets List
        self.tabs_list = [] # List of QtGui.Qwigdet
        self.states_list = [] # List of Graphwindows for pos and angle
        self.signals_list = [] # List of GraphWindows for control signals

        # Files
        self.control_file = ''
        self.launch_file = ''

        # Drones
        self.drones = DroneList()
        self.__num_drones = 0
        self.start_signal = False

        # Matplots
        self.plotter = Plotter()

        self.control_period = 5./1000.
        self.data_period = 100./1000.

        start_roscore()
        time.sleep(1)
        rospy.init_node('GUI', anonymous=True)
        self.controlTimer = rospy.Timer(
            rospy.Duration(self.control_period), self.control_law)
        self.dataTimer = rospy.Timer(rospy.Duration(self.data_period),
                                     self.set_plot_data)

        # First Command Tab
        self.tabControl = QtGui.QWidget()
        self.form = Ui_Form()
        self.addTab(self.tabControl, 'Controls')
        self.controlsUI()
        self.setupUi()

        self.bind_signals()
        self.init_plot_data()

    def init_plot_data(self):
        self.data = [[] for _ in range(4)]
        self.dataD = [[] for _ in range(4)]
        self.data_matplot = [[] for _ in range(4)]
        self.dataD_matplot = 0

        self.index = 0
        self.t = 0
        self.t_list = []

    def bind_signals(self):
        # Buttons
        self.form.btn_launch.clicked.connect(self.get_launch_file)
        self.form.btn_control.clicked.connect(self.get_control_file)
        self.form.btnStart.clicked.connect(self.start)
        self.form.btnStop.clicked.connect(self.stop)
        self.form.btnReset.clicked.connect(self.reset)

    def setupUi(self):
        """ Method that initialize additional items of the Qt Creator UI """
        self.form.radioButton_sim.setChecked(True)
        self.form.btnStop.setEnabled(False)
        self.form.label_console.setWordWrap(True)

    def set_drone_tabs(self):
        """ Method that sets the number of tabs for graphing each drone's
        information """
        for i in range(self.__num_drones):
            tab = QtGui.QTabWidget()
            self.tabs_list.append(tab)
            gridLayout = QtGui.QGridLayout()
            tab.setLayout(gridLayout)
            w = GraphWindow(2,2,[])
            gridLayout.addWidget(w, 0, 0)
            self.states_list.append(w)
            self.addTab(tab, 'AR.Drone {}'.format(i+1))

    def set_signals_tabs(self):
        """ Method that sets the number of tabs for graphing each drone's
        information """
        for i in range(self.__num_drones):
            tab = QtGui.QTabWidget()
            self.tabs_list.append(tab)
            gridLayout = QtGui.QGridLayout()
            tab.setLayout(gridLayout)
            w = GraphWindow(2,2,[])
            gridLayout.addWidget(w, 0, 0)
            self.states_list.append(w)
            self.addTab(tab, 'Signals {}'.format(i+1))

    def set_plot_data(self, _):
        """ Method that sets data arrays for matplotlib """
        if self.start_signal:
            self.trajectory.append_vals(self.t)
            self.dataD_matplot = self.trajectory.get_plot_data()
            # get each drone data
            vals = ["x", "y", "z", "yaw_angle"]
            for n in range(self.__num_drones):
                for i,val in enumerate(vals):
                    drone = getattr(self.drones, self.drones_names[n])
                    attr = getattr(drone, val)
                    self.data_matplot[i].append(attr)
                    self.data[i] = self.data_matplot[i][self.index:]
                    self.dataD[i] = self.dataD_matplot[i][self.index:]
                    if len(self.data_matplot[i]) >= 50 and i == 0:
                        self.index+=1

                graphWindow = self.states_list[n]
                graphWindow.set_data(self.data, self.dataD)
            self.t_list.append(self.t)

    def remove_drone_tabs(self):
        self.tabs_list = []
        for i in reversed(range(1, self.__num_drones+1)):
            self.removeTab(i)
            time.sleep(0.1)

    def controlsUI(self):
        """ Method that sets the QtCreator UI to the control tab layout """
        layout = QtGui.QGridLayout()
        layout.addWidget(self.form)
        self.tabControl.setLayout(layout)

    def get_launch_file(self):
        """ Method that opens a QFileDialog to load the launch file """
        dialog = QtGui.QFileDialog(directory=QtCore.QString('./examples'))
        dialog.setFileMode(QtGui.QFileDialog.AnyFile)
        dialog.setFilter('Launch Files (*.launch)')

        if dialog.exec_():
            filenames = dialog.selectedFiles()
            self.form.lineEdit_launch.setText(filenames[0])
            if not self.launch_file == filenames[0]:
                self.remove_drone_tabs()
                self.launch_file = filenames[0]
                self.check_launch_file()
                self.set_drone_tabs()
                self.set_signals_tabs()

    @timing_val
    def control_law(self, _):
        if self.start_signal:
            self.control(self)
            self.t+=self.control_period
            self.trajectory.get_vals(self.t)

    def get_control_file(self):
        """ Method that opens a QFileDialog to load the control file """
        dialog = QtGui.QFileDialog(directory=QtCore.QString('./'))
        dialog.setFileMode(QtGui.QFileDialog.AnyFile)
        dialog.setFilter('Python Files (*.py)')

        if dialog.exec_():
            filenames = dialog.selectedFiles()
            self.form.lineEdit_control.setText(filenames[0])
            if not self.launch_file == filenames[0]:
                self.control_file = filenames[0]
                self.check_control_file()

    def check_launch_file(self):
        """ Method that loads launch file and sets self.num_drones and
        drones initial position """
        launch_file = self.form.lineEdit_launch.text()
        if is_empty(launch_file):
            text = 'IOError: Launch file is not specified'
            self.write_to_console(text, 'red')
            raise IOError('Launch file is not specified')
        try:
            parser = XMLParser(launch_file)
        except IOError as e:
            text = 'IOError: Launch file not found {}'.format(file)
            self.write_to_console(text, 'red')
            raise IOError('Launch file not found: {}'.format(file))
        self.__num_drones = parser.get_num_drones()
        self.drones_names = parser.drones_names()

    def check_control_file(self):
        """ Method that loads python control file """
        control_file = self.form.lineEdit_control.text()
        if is_empty(control_file):
            text = 'IOError: Control file is not specified'
            self.write_to_console(text, 'red')
            raise IOError('Control file is not specified')

    def load_trajectory(self):
        # Load Trajectory
        try:
            mx = str(self.form.lineEdit_mx.text())
            my = str(self.form.lineEdit_my.text())
            mz = str(self.form.lineEdit_mz.text())
            myaw = str(self.form.lineEdit_myaw.text())
            self.veces = int(self.form.lineEdit_veces.text())
            self.trajectory = RefTrajectory(mx, my, mz, myaw)
        except Exception as e:
            self.write_to_console(str(e), 'red')

    def import_control(self):
        """ Method that imports the user ControlLaw from .py file """
        module = str(self.control_file)
        module = module.split('/')[-1][:-3]

        control_module = importlib.import_module('control_files.'+module)
        attributes = dir(control_module)
        index = attributes.index('control')
        self.control = getattr(control_module, attributes[index])

    def set_drones(self):
        for i in range(1, self.__num_drones+1):
            if self.form.radioButton_sim.isChecked():
                self.drones.add_drone(self.drones_names[i-1],
                                      DroneSimulation(self.drones_names[i-1],
                                                      i))
            else:
                self.drones.add_drone(self.drones_names[i-1],
                                      DroneReal(self.drones_names[i-1], i))

    def call_matplots_process(self):
        """ Method that spawns process for each matplot"""
        process_pos = Process(target=self.plotter.plot_pos_graph,
                              args=(self.data_matplot, self.dataD_matplot,
                                    self.t_list))
        process_pos.start()

        # process_angle = Process(target=self.plotter.plot_angle_graph,
        #                       args=(self.data_matplot, self.dataD_matplot,
        #                             self.t_list))
        # process_angle.start()

        process_3d = Process(target=self.plotter.plot_3d_graph,
                              args=(self.data_matplot, self.dataD_matplot))
        process_3d.start()


    def start(self):
        """ Method that loads all the controls tab info and starts the test """
        self.form.btnStart.setEnabled(False)
        self.form.btnStop.setEnabled(True)
        self.init_plot_data()
        self.check_launch_file()
        self.set_drones()
        self.check_control_file()
        self.load_trajectory()
        self.import_control()
        time.sleep(0.3)
        self.drones.takeoff()
        self.start_signal = True

    def stop(self):
        """ Method that stops the test """
        self.drones.land()
        self.start_signal = False

        if self.form.checkBox_show.isChecked():
            self.call_matplots_process()

        self.form.btnStart.setEnabled(True)
        self.form.btnStop.setEnabled(False)

    def reset(self):
        """ Method that clears all values of the control tab """
        self.remove_drone_tabs()
        self.form.lineEdit_mx.setText('')
        self.form.lineEdit_my.setText('')
        self.form.lineEdit_mz.setText('')
        self.form.lineEdit_myaw.setText('')
        self.form.lineEdit_veces.setText('')
        self.form.lineEdit_launch.setText('')
        self.form.lineEdit_control.setText('')
        self.form.checkBox_save.setChecked(False)
        self.form.checkBox_show.setChecked(False)

    def write_to_console(self, text, color):
        """ Method that writes text to the console Qlabel """
        self.form.label_console.setText(text)
        self.form.label_console.setStyleSheet('color:{}'.format(color))

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    g = GUI()
    g.show()
    status = app.instance().exec_()
    kill_roscore()
    rospy.signal_shutdown('Bye')
    time.sleep(0.5)
    sys.exit(status)