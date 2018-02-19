# -*- coding: utf-8 -*-

kpx = 2
kpy = 2
kpz = 3
kdx = 2
kdy = 2
kdz = 3
kpa = 2

def control(self):
    x = self.drones.ardrone_1.x
    y = self.drones.ardrone_1.y
    z = self.drones.ardrone_1.z
    yaw_angle = self.drones.ardrone_1.yaw_angle
    vx = self.drones.ardrone_1.vx
    vy = self.drones.ardrone_1.vy
    vz = self.drones.ardrone_1.vz
    mx = self.trajectory.mx
    my = self.trajectory.my
    mz = self.trajectory.mz
    myaw = self.trajectory.myaw
    mxp = self.trajectory.mxp
    myp = self.trajectory.myp
    mzp = self.trajectory.mzp
    mxpp = self.trajectory.mxpp
    mypp = self.trajectory.mypp
    mzpp = self.trajectory.mzpp

    roll_control = self.drones.ardrone_1.roll_control
    pitch_control = self.drones.ardrone_1.pitch_control

    yaw_control = -kpa*(yaw_angle - myaw)
    z_control = -kpz*(z-mz)+mzp

    self.drones.ardrone_1.z_control = z_control
    self.drones.ardrone_1.yaw_control = yaw_control

    self.drones.ardrone_1.set_command(0,0,yaw_control,z_control)