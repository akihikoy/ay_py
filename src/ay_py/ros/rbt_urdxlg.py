#! /usr/bin/env python
#Robot controller for Universal Robots UR* with Dynamixel Gripper.
from const import *

import roslib
import rospy

from rbt_ur import *
from rbt_dxlg import TDxlGripper
#from rbt_rq import TRobotiq

'''Robot control class for single Universal Robots UR* with Dynamixel Gripper.'''
class TRobotURDxlG(TRobotUR):
  def __init__(self, name='UR', is_sim=False, dev='/dev/ttyUSB0'):
    super(TRobotURDxlG,self).__init__(name=name,is_sim=is_sim)
    self.dev= dev

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    ra(super(TRobotURDxlG,self).Init())

    self.dxl_gripper= TDxlGripper(dev=self.dev)
    self.grippers= [self.dxl_gripper]

    print 'Initializing and activating DxlGripper gripper...'
    ra(self.dxl_gripper.Init())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('URDxlG',):  return True
    return super(TRobotURDxlG,self).Is(q)
