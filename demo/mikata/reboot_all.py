#!/usr/bin/python
#Reboot Dynamixel.

from _path import *
from ay_py.misc.dxl_mikata import *

#Setup the device
mikata= TMikata()
mikata.Setup()

print 'Rebooting Dynamixel...'
mikata.Reboot()

#mikata.DisableTorque()
mikata.Quit()
