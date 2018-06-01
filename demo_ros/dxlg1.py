#!/usr/bin/python
#\file    dxlg1.py
#\brief   Test of TDxlGripper.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Nov.07, 2017
from _path import *
from ay_py.core import *
from ay_py.ros import *
from ay_py.ros.rbt_dxlg import *

if __name__=='__main__':
  DXLG_DEV= os.environ['DXLG_DEV'] if 'DXLG_DEV' in os.environ else '/dev/ttyUSB0'
  dev= sys.argv[1] if len(sys.argv)>1 else DXLG_DEV
  rospy.init_node('dxlg1')
  robot= TRobotDxlGripper(dev=dev)
  print 'Initializing...'
  robot.Init()
  print 'Done.'

  robot.OpenGripper(blocking=True)
  print 'GripperPos=',robot.GripperPos()
  robot.CloseGripper(blocking=True)
  print 'GripperPos=',robot.GripperPos()

  robot.Cleanup()
