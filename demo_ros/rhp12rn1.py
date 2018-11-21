#!/usr/bin/python
#\file    rhp12rn1.py
#\brief   Test of TRHP12RNGripper.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Nov.20, 2018
from _path import *
from ay_py.core import *
from ay_py.ros import *
from ay_py.ros.rbt_rhp12rn import *

if __name__=='__main__':
  DXLG_DEV= os.environ['DXLG_DEV'] if 'DXLG_DEV' in os.environ else '/dev/ttyUSB0'
  dev= sys.argv[1] if len(sys.argv)>1 else DXLG_DEV
  rospy.init_node('rhp12rn1')
  robot= TRobotRHP12RNGripper(dev=dev)
  print 'Initializing...'
  robot.Init()
  print 'Done.'

  robot.OpenGripper(blocking=True)
  print 'GripperPos=',robot.GripperPos()
  robot.CloseGripper(blocking=True)
  print 'GripperPos=',robot.GripperPos()

  robot.Cleanup()
