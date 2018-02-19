#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    DRONE CLASSES

    Author: Misael Antonio Alarcón


"""

import logging
import math
import time
import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped
import numpy

from drone_controller import BasicDroneController
from utils import *

CONTROL_PERIOD = 10 #ms

class DroneList(object):
    def __init__(self):
        self.__n = 1
        self.names = []

    def add_drone(self, name, object):
        self.names.append(name)
        setattr(self, name, object) # self.ardrone = Drone()
        self.__n+=1

    def takeoff(self):
        for i in range(self.__n-1):
            drone = getattr(self, self.names[i])
            drone.send_takeoff()

    def land(self):
        for i in range(self.__n-1):
            drone = getattr(self, self.names[i])
            drone.set_command(0,0,0,0)
            time.sleep(0.1)
            drone.send_land()


class Drone(BasicDroneController):
    def __init__(self, name):
        super(Drone, self).__init__(name)
        self.x = 0
        self.y = 0
        self.z = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.vyaw = 0
        self.ax = 0
        self.ay = 0
        self.az = 0
        self.roll_angle = 0
        self.pitch_angle = 0
        self.yaw_angle = 0
        # Control signals
        self.roll_control = 0
        self.pitch_control = 0
        self.z_control = 0
        self.yaw_control = 0


class DroneReal(Drone):

    def __init__(self, name, num):
        super(DroneReal, self).__init__(name)
        self.num = num

    def recv_optidata(self,data):
        self.x = data.data[0]
        self.y = data.data[1]
        self.z = data.data[2]
        self.yaw_angle = data.data[3]
        self.roll_angle = data.data[4]
        self.pitch_angle = data.data[5]

    def recv_navdata(self,navdata):
        self.status = navdata.state
        self.vx = navdata.vx / 1000.0
        self.vy = navdata.vy / 1000.0
        self.ax = navdata.ax / 1000.0
        self.ay = navdata.ay / 1000.0
        self.az = navdata.az / 1000.0


class DroneSimulation(Drone):

    def __init__(self, name, num):
        super(DroneSimulation, self).__init__(name)
        self.num = num
        self.name = name
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.recv_optidata)

    def recv_optidata(self, data):
        num = self.num
        self.x = data.pose[-num].position.x
        self.y = data.pose[-num].position.y
        self.z = data.pose[-num].position.z

    def recv_navdata(self,navdata):
        self.status = navdata.state
        self.vx = navdata.vx / 1000.0
        self.vy = navdata.vy / 1000.0
        self.roll_angle = degtorad(navdata.rotX)
        self.pitch_angle = degtorad(navdata.rotY)
        self.yaw_angle = degtorad(navdata.rotZ)

#rospy.init_node('Hola')
# lider = LeaderReal()
# Drone().num_drones
# print lider.num_drones