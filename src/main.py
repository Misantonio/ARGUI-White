#!/usr/bin/env python

import sys
import rospy
from pyqtgraph.Qt import QtGui
from modules.utils import kill_roscore
from gui.MainWindow import GUI


if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    g = GUI()
    g.show()
    status = app.instance().exec_()
    kill_roscore()
    rospy.signal_shutdown('Bye')
    sys.exit(status)