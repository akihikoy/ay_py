#! /usr/bin/env python
#Robot controller for Kinova Gen3 with DxlO3 Gripper.
from const import *

import roslib
import rospy

from rbt_gen3 import TRobotGen3
from rbt_dxlo3 import TDxlO3Gripper

'''Robot control class for single Kinova Gen3 with DxlO3 Gripper.'''
class TRobotGen3DxlO3(TRobotGen3):
  def __init__(self, name='Gen3DxlO3', gen3ns='gen3a', is_sim=False, gripper_node='gripper_driver'):
    super(TRobotGen3DxlO3,self).__init__(name=name,gen3ns=gen3ns,is_sim=is_sim)
    self.gripper_node= gripper_node

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    ra(super(TRobotGen3DxlO3,self).Init())

    self.dxlo3_gripper= TDxlO3Gripper(node_name=self.gripper_node)
    #self.dxlo3_gripper= TSimGripper2F1(('DxlO3',),pos_range=[0.0,0.118])
    self.grippers= [self.dxlo3_gripper]

    print 'Initializing and activating DxlO3 gripper...'
    ra(self.dxlo3_gripper.Init())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('Gen3DxlO3',):  return True
    return super(TRobotGen3DxlO3,self).Is(q)

