#!/usr/bin/python
#Record key points by moving the arm manually.
#Press a key to print the current pose.  Press q to exit.

from _path import *
from ay_py.misc.dxl_mikata import *
from ay_py.core import TKBHit
import time

#Setup the device
mikata= TMikata()
mikata.Setup()
mikata.EnableTorque()

#Relax mode:
mikata.SetPWM({jname:0 for jname in mikata.JointNames()})

mikata.StartStateObs()

try:
  kbhit= TKBHit()
  while True:
    c= kbhit.KBHit()
    if c=='q':  break
    elif c is not None:
      #print mikata.State()
      print mikata.State()['position']
    #mikata.MoveTo(mikata.Position(as_dict=True),wait=False)
    time.sleep(0.0025)
except KeyboardInterrupt:
  pass

mikata.StopStateObs()

#mikata.DisableTorque()
mikata.Quit()
