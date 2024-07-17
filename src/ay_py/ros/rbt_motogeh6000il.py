#! /usr/bin/env python
#Robot controller for Motoman robots with GEH6000IL Gripper.
from const import *

import roslib
import rospy

from rbt_moto import *
from rbt_geh6000il import TGEH6000ILGripper

'''Robot control class for single Motoman robots with GEH6000IL Gripper.'''
class TRobotMotomanGEH6000IL(TRobotMotoman):
  def __init__(self, name='Motoman', is_sim=False, gripper_node='gripper_driver'):
    super(TRobotMotomanGEH6000IL,self).__init__(name=name,is_sim=is_sim)
    self.gripper_node= gripper_node
    self.geh_gripper= None
    self.cat_name1= 'MotomanGEH6000IL'
    self.gripper_type_name= self.Name.replace(self.type_name,'')
    self.cat_name2= self.Name.replace(self.gripper_type_name,'GEH6000IL')

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    ra(super(TRobotMotomanGEH6000IL,self).Init())

    self.geh_gripper= TGEH6000ILGripper(node_name=self.gripper_node)
    self.grippers= [self.geh_gripper]

    print 'Initializing and activating GEH6000ILGripper...'
    ra(self.geh_gripper.Init())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in (self.cat_name1,self.cat_name2):  return True
    #if self.geh_gripper is not None:
      #if q.startswith('Motoman'+self.geh_gripper.gripper_type):  return True
    #else:
      #print 'WARNING: TRobotMotomanGEH6000IL.geh_gripper is not initialized.'
    return super(TRobotMotomanGEH6000IL,self).Is(q)

