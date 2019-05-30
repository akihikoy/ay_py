#! /usr/bin/env python
#Robot controller for Universal Robots UR* with RH-P12-RN Gripper (Thormang3 gripper).
from const import *

import roslib
import rospy

from rbt_ur import *

'''Robot control class for single Universal Robots UR* with RH-P12-RN Gripper (Thormang3 gripper).'''
class TRobotURThG(TRobotUR):
  def __init__(self, name='UR', ur_series='CB', robot_ip=None, is_sim=False, dev='/dev/ttyUSB0'):
    super(TRobotURThG,self).__init__(name=name,ur_series=ur_series,robot_ip=robot_ip,is_sim=is_sim)
    self.dev= dev

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    ra(super(TRobotURThG,self).Init())

    if not self.is_sim:
      mod= __import__('rbt_rhp12rn',globals(),None,('TRHP12RNGripper',))
      self.th_gripper= mod.TRHP12RNGripper(dev=self.dev)
    else:
      self.th_gripper= TSimGripper2F1(('RHP12RNGripper','ThGripper'),pos_range=[0.0,0.109])
    self.grippers= [self.th_gripper]

    print 'Initializing and activating RHP12RNGripper gripper...'
    ra(self.th_gripper.Init())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('URThG',):  return True
    return super(TRobotURThG,self).Is(q)

