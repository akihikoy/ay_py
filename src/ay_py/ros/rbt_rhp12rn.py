#! /usr/bin/env python
#\brief   Robot controller for RH-P12-RN Gripper (Thormang3 gripper).
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Nov.21, 2018
#\version 0.2
#\date    Feb.28, 2020
#         Completely modified the implementation: now we use the gripper driver ROS node.
from const import *

from rbt_dxlg import TDxlGripper, TRobotDxlGripper


'''RH-P12-RN Gripper (Thormang3 gripper) utility class'''
class TRHP12RNGripper(TDxlGripper):
  def __init__(self, node_name='gripper_driver'):
    super(TRHP12RNGripper,self).__init__(node_name=node_name, gripper_type='RHP12RNGripper')

  def Cleanup(self):
    super(TRHP12RNGripper,self).Cleanup()

  '''Answer to a query q by {True,False}. e.g. Is('Robotiq').'''
  def Is(self, q):
    if q in ('RHP12RNGripper','ThGripper'):  return True
    return super(TRHP12RNGripper,self).Is(q)

  '''Get a fingertip height offset in meter.
    The fingertip trajectory of some grippers has a rounded shape.
    This function gives the offset from the highest (longest) point (= closed fingertip position),
    and the offset is always negative.
      pos: Gripper position to get the offset. '''
  def FingertipOffset(self, pos):
    #WARNING: This is accurate only at the open (width=109mm) and close (width=0mm) positions.
    #0.067... = (145.-(145.**2-(109./2.)**2-117.**2)/(2.*145.-2.*117.))*1.e-3
    return (0.06704017857142858**2-0.25*pos*pos)**0.5 - 0.06704017857142858


'''Robot control class for RHP12RNGripper.
  This is defined as a subclass of TMultiArmRobot,
  but actually it does not have a body (only RHP12RNGripper gripper).
  This virtual body is designed for a compatibility of programs.'''
class TRobotRHP12RNGripper(TRobotDxlGripper):
  def __init__(self, name='RHP12RNGripper', gripper_node='gripper_driver'):
    super(TRobotRHP12RNGripper,self).__init__(name=name,gripper_node=gripper_node)

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    dxl_gripper= TRHP12RNGripper(node_name=self.gripper_node)
    return self.internal_init(dxl_gripper)

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('RHP12RNGripper','ThGripper'):  return True
    return super(TRobotRHP12RNGripper,self).Is(q)

