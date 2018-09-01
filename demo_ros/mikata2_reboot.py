#!/usr/bin/python
#\file    mikata2_off.py
#\brief   Reboot all Dynamixel of Mikata Arm (using TRobotMikata2).
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.07, 2018
from _path import *
from ay_py.core import *
from ay_py.ros import *
from ay_py.ros.rbt_mikata2 import *

if __name__=='__main__':
  rospy.init_node('mikata_off')
  robot= TRobotMikata2()
  print 'Initializing...'
  robot.Init()
  print 'Done.'

  CPrint(1, 'Rebooting all Dynamixel...')
  robot.mikata.Reboot()

  robot.Cleanup()
