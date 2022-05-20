#! /usr/bin/env python
#\brief   Robot controller for SAKE EZGripper Gen2.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jul.13, 2019
#\version 0.2
#\date    Feb.28, 2020
#         Completely modified the implementation: now we use the gripper driver ROS node.
from const import *

from rbt_dxlg import TDxlGripper, TRobotDxlGripper


'''SAKE EZGripper Gen2 utility class'''
class TEZGripper(TDxlGripper):
  def __init__(self, dev='/dev/ttyUSB0'):
    super(TEZGripper,self).__init__(node_name=node_name, gripper_type='EZGripper')

  def Cleanup(self):
    super(TEZGripper,self).Cleanup()

  '''Answer to a query q by {True,False}. e.g. Is('Robotiq').'''
  def Is(self, q):
    if q=='EZGripper':  return True
    return super(TEZGripper,self).Is(q)

  '''Get a fingertip height offset in meter.
    The fingertip trajectory of some grippers has a rounded shape.
    This function gives the offset from the highest (longest) point (= closed fingertip position),
    and the offset is always negative.
      pos: Gripper position to get the offset. '''
  def FingertipOffset(self, pos):
    #WARNING: NotImplemented
    return 0.0


'''Robot control class for EZGripper.
  This is defined as a subclass of TMultiArmRobot,
  but actually it does not have a body (only EZGripper gripper).
  This virtual body is designed for a compatibility of programs.'''
class TRobotEZGripper(TRobotDxlGripper):
  def __init__(self, name='EZGripper', gripper_node='gripper_driver'):
    super(TRobotEZGripper,self).__init__(name=name,gripper_node=gripper_node)

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    dxl_gripper= TEZGripper(node_name=self.gripper_node)
    return self.internal_init(dxl_gripper)

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q=='EZGripper':  return True
    return super(TRobotEZGripper,self).Is(q)

