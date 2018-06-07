#!/usr/bin/python
#\file    mikata_off.py
#\brief   Reboot all Dynamixel of Mikata Arm.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.07, 2018
from _path import *
from ay_py.core import *
from ay_py.ros import *
from ay_py.ros.rbt_mikata import *

if __name__=='__main__':
  DXLG_DEV= os.environ['DXLG_DEV'] if 'DXLG_DEV' in os.environ else '/dev/ttyUSB0'
  dev= sys.argv[1] if len(sys.argv)>1 else DXLG_DEV
  rospy.init_node('mikata_off')
  robot= TRobotMikata(dev=dev)
  print 'Initializing...'
  robot.Init()
  print 'Done.'

  CPrint(1, 'Rebooting all Dynamixel...')
  robot.mikata.Reboot()

  robot.Cleanup()
