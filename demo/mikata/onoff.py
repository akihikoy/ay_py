#!/usr/bin/python
#Switching to enable and disable robot by pressing a key.  Press q to exit.
from _path import *
from ay_py.misc.dxl_mikata import *
from ay_py.core import TKBHit
import time

#Setup the device
mikata= TMikata()
mikata.Setup()
#mikata.EnableTorque()
#mikata.SetPWM({jname:0 for jname in mikata.JointNames()})

state= 'disabled'

try:
  kbhit= TKBHit()
  while True:
    c= kbhit.KBHit()
    if c=='q':  break
    elif c is not None:
      state= {'enabled':'disabled', 'disabled':'enabled'}[state]
      if state=='enabled':
        mikata.EnableTorque()
      else:
        mikata.DisableTorque()
    time.sleep(0.0025)
except KeyboardInterrupt:
  pass

#mikata.DisableTorque()
mikata.Quit()

