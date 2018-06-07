#!/usr/bin/python
#Enable servo motors.

from _path import *
from ay_py.misc.dxl_mikata import *

#Setup the device
mikata= TMikata()
mikata.Setup()
mikata.EnableTorque()
mikata.Quit()
