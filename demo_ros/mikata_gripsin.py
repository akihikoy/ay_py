#!/usr/bin/python
#\file    mikata_off.py
#\brief   Test Mikata arm.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.01, 2018
from _path import *
from ay_py.core import *
from ay_py.ros import *
from ay_py.ros.rbt_mikata import *

if __name__=='__main__':
  DXLG_DEV= os.environ['DXLG_DEV'] if 'DXLG_DEV' in os.environ else '/dev/ttyUSB0'
  dev= sys.argv[1] if len(sys.argv)>1 else DXLG_DEV
  rospy.init_node('mikata_test')
  robot= TRobotMikata(dev=dev,is_sim=True)
  print 'Initializing...'
  robot.Init()
  rospy.sleep(0.2)
  print 'Done.'

  #q= [qj for qj in robot.Q()]
  q= robot.GripperPos()
  #q= 0.0
  t= 0.0
  dt= 0.1
  while not rospy.is_shutdown():
    #q[-1]= math.sin(5.0*t)
    q= 1.0*math.sin(2.0*t)
    #print q
    #robot.MoveToQ(q, dt)
    robot.MoveGripper(q, dt)
    t+= dt
    rospy.sleep(dt)

  robot.Cleanup()
