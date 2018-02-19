#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    This document was modified from the original version to line up with the
    PEP-8 Python Coding Style and other purposes

    Original Version:
    A basic drone controller class for the tutorial "Up and flying with the
    AR.Drone and ROS | Getting Started"
    https://github.com/mikehamer/ardrone_tutorials_getting_started
"""

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata


from drone_status import DroneStatus

COMMAND_PERIOD = 10 #ms

class BasicDroneController(object):
	def __init__(self, name):
		self.status = -1

		rospy.Subscriber('/'+name+'/navdata',Navdata, self.recv_navdata)

		self.pubLand = rospy.Publisher('/'+name+'/land',Empty,
									   queue_size=1)
		self.pubTakeoff = rospy.Publisher('/'+name+'/takeoff',Empty,
										  queue_size=1)
		self.pubReset = rospy.Publisher('/'+name+'/reset',Empty,
										queue_size=1)
		
		self.pubCommand = rospy.Publisher('/'+name+'/cmd_vel',Twist,
										  queue_size=1)

		self.command = Twist()
		self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),
										self.send_command)

		rospy.on_shutdown(self.send_land)

	def recv_navdata(self,navdata):
		self.status = navdata.state

	def send_takeoff(self):
		self.pubTakeoff.publish(Empty())

	def send_land(self):
		self.pubLand.publish(Empty())

	def send_emergency(self):
		self.pubReset.publish(Empty())

	def set_command(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
		self.command.linear.x  = pitch
		self.command.linear.y  = roll
		self.command.linear.z  = z_velocity
		self.command.angular.z = yaw_velocity

	def send_command(self,event):
		if self.status == DroneStatus.Flying \
				or self.status == DroneStatus.GotoHover \
				or self.status == DroneStatus.Hovering:
			self.pubCommand.publish(self.command)
