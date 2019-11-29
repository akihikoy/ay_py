#! /usr/bin/env python
#Robot controller for Kinova Gen3 with RH-P12-RN Gripper (Thormang3 gripper).
from const import *

import roslib
import rospy

from rbt_gen3 import TRobotGen3

'''Robot control class for single Kinova Gen3 with RH-P12-RN Gripper (Thormang3 gripper).'''
class TRobotGen3ThG(TRobotGen3):
  def __init__(self, name='Gen3', gen3ns='gen3a', is_sim=False, dev='/dev/ttyUSB0'):
    super(TRobotGen3ThG,self).__init__(name=name,gen3ns=gen3ns,is_sim=is_sim)
    self.dev= dev

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    ra(super(TRobotGen3ThG,self).Init())

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
    if q in ('Gen3ThG',):  return True
    return super(TRobotGen3ThG,self).Is(q)

