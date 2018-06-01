#!/usr/bin/python
#\file    mikata_off.py
#\brief   Turn off the Mikata arm.
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
  rospy.init_node('mikata_off')
  robot= TRobotMikata(dev=dev)
  print 'Initializing...'
  robot.Init()
  print 'Done.'

  CPrint(1, 'Robot is relaxing...')
  for effort in range(5,-1,-1):  
    robot.mikata.MoveTo({jname:q for (jname,q) in zip(robot.JointNames(),robot.Q(arm=0))}, blocking=False)
    robot.mikata.SetPWM({jname:effort for jname in robot.JointNames()})
    robot.EndEff().Move(robot.EndEff().Position(), max_effort=effort)
    rospy.sleep(0.5)

  for i in range(3,0,-1):  
    CPrint(1, 'Torque is disabled after {} sec...'.format(i))
    rospy.sleep(1.0)
  robot.mikata.DisableTorque()

  robot.Cleanup()
