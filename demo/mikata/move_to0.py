#!/usr/bin/python
#Move to initial target position with trajectory control.

from _path import *
from ay_py.misc.dxl_mikata import *

#Setup the device
mikata= TMikata()
mikata.Setup()
mikata.EnableTorque()

#More effort to follow trajectory.
#mikata.SetPWM({jname:e for e,jname in zip([80,80,50,50,50],mikata.JointNames())})

pose= [0, 0, 0, 0, 0]
mikata.FollowTrajectory(mikata.JointNames(),[pose],[3.0],blocking=True)

#mikata.DisableTorque()
mikata.Quit()
