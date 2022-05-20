#! /usr/bin/env python
#\brief   Robot controller for DxlpO2 gripper.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jan.28, 2020
#\version 0.2
#\date    Feb.28, 2020
#         Completely modified the implementation: now we use the gripper driver ROS node.
from const import *

from rbt_dxlg import TDxlGripper, TRobotDxlGripper


'''DxlpO2 gripper utility class'''
class TDxlpO2Gripper(TDxlGripper):
  def __init__(self, node_name='gripper_driver', finger_type=None):
    super(TDxlpO2Gripper,self).__init__(node_name=node_name, gripper_type='DxlpO2Gripper', finger_type=finger_type)

  def Cleanup(self):
    super(TDxlpO2Gripper,self).Cleanup()

  '''Answer to a query q by {True,False}. e.g. Is('Robotiq').'''
  def Is(self, q):
    if q=='DxlpO2Gripper':  return True
    return super(TDxlpO2Gripper,self).Is(q)

  '''Get a fingertip height offset in meter.
    The fingertip trajectory of some grippers has a rounded shape.
    This function gives the offset from the highest (longest) point (= closed fingertip position),
    and the offset is always negative.
      pos: Gripper position to get the offset. '''
  def FingertipOffset(self, pos):
    #WARNING: NotImplemented
    return 0.0


'''Robot control class for DxlpO2Gripper.
  This is defined as a subclass of TMultiArmRobot,
  but actually it does not have a body (only DxlpO2Gripper gripper).
  This virtual body is designed for a compatibility of programs.'''
class TRobotDxlpO2Gripper(TRobotDxlGripper):
  def __init__(self, name='DxlpO2Gripper', gripper_node='gripper_driver', finger_type=None):
    super(TRobotDxlpO2Gripper,self).__init__(name=name,gripper_node=gripper_node)
    self.finger_type= finger_type

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    dxl_gripper= TDxlpO2Gripper(node_name=self.gripper_node, finger_type=self.finger_type)
    return self.internal_init(dxl_gripper)

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('DxlpO2','DxlpO2Gripper'):  return True
    return super(TRobotDxlpO2Gripper,self).Is(q)

