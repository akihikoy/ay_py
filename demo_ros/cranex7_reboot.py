#!/usr/bin/python
#\file    cranex7_off.py
#\brief   Reboot all Dynamixel of Crane-X7 Arm.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Nov.01, 2018
from _path import *
from ay_py.core import *
from ay_py.ros import *
from ay_py.ros.rbt_cranex7 import *

if __name__=='__main__':
  rospy.init_node('mikata_off')
  robot= TRobotCraneX7()
  print 'Initializing...'
  robot.Init()
  print 'Done.'

  CPrint(1, 'Rebooting all Dynamixel...')
  robot.mikata.Reboot()

  robot.Cleanup()
