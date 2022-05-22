#! /usr/bin/env python
#Robot controller for Universal Robots UR* with DxlpY1 gripper.
from const import *

import roslib
import rospy

from rbt_ur import *

'''Robot control class for single Universal Robots UR* with DxlpY1 gripper.'''
class TRobotURDxlpY1(TRobotUR):
  def __init__(self, name='UR', ur_series='CB', robot_ip=None, is_sim=False, gripper_node='gripper_driver'):
    super(TRobotURDxlpY1,self).__init__(name=name,ur_series=ur_series,robot_ip=robot_ip,is_sim=is_sim)
    self.gripper_node= gripper_node

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    ra(super(TRobotURDxlpY1,self).Init())

    if not self.is_sim:
      #The gripper module is imported here to avoid importing it in simulation mode.
      mod= __import__('rbt_dxlpy1',globals(),None,('TDxlpY1Gripper',))
      self.dxlpy1_gripper= mod.TDxlpY1Gripper(node_name=self.gripper_node)
    else:
      self.dxlpy1_gripper= TSimGripper2F1(pos_range=[0.0,0.133])
    self.grippers= [self.dxlpy1_gripper]

    print 'Initializing and activating DxlGripper gripper...'
    ra(self.dxlpy1_gripper.Init())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('URDxlpY1',):  return True
    return super(TRobotURDxlpY1,self).Is(q)
