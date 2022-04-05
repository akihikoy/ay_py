#! /usr/bin/env python
#\brief   Robot controller for DxlO3 gripper.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Feb.10, 2020
#\version 0.2
#\date    Feb.28, 2020
#         Completely modified the implementation: now we use the gripper driver ROS node.
from const import *

from robot import TGripper2FN,TMultiArmRobot
from rbt_dxlg import TDxlGripper


'''DxlO3 gripper utility class'''
class TDxlO3Gripper(TDxlGripper,TGripper2FN):
  def __init__(self, node_name='gripper_driver'):
    TDxlGripper.__init__(self, node_name=node_name, gripper_type='DxlO3Gripper')
    TGripper2FN.__iadd__(self, dof=2)

    #Gripper position-angles conversions (for 2F1 emulation mode).
    self.g2f1_ang_open= 0.6136  #cmd=+400
    self.g2f1_ang_close= 0.1657  #cmd=+108; With FV+
    self.g2f1_range= [0.0,0.118]  #cmd=[108,400]; With FV+
    self.g2f1_ang2pos= lambda ang: max(self.g2f1_range[0], self.g2f1_range[0] + (ang-self.g2f1_ang_close)*(self.g2f1_range[1]-self.g2f1_range[0])/(self.g2f1_ang_open-self.g2f1_ang_close))
    self.g2f1_pos2ang= lambda pos: self.g2f1_ang_close + (pos-self.g2f1_range[0])*(self.g2f1_ang_open-self.g2f1_ang_close)/(self.g2f1_range[1]-self.g2f1_range[0])

  def Cleanup(self):
    TDxlGripper.Cleanup(self)

  '''Answer to a query q by {True,False}. e.g. Is('Robotiq').'''
  def Is(self, q):
    if q in ('DxlO3','DxlO3Gripper'):  return True
    if TDxlGripper.Is(self,q):  return True
    return TGripper2FN.Is(self,q)

  '''Range of gripper position as an emulation of 2F1 gripper.'''
  def PosRange2F1(self):
    return self.g2f1_range

  '''Get fingertip height offsets in meter.
    The fingertip trajectory of some grippers has a rounded shape.
    This function gives the offset from the highest (longest) point,
    and the offsets are always negative.
      pos: Gripper positions to get the offsets. '''
  def FingertipOffset(self, pos):
    #WARNING: NotImplemented
    return [0.0,0.0]

  '''Get fingertip height offsets in meter as an emulation of 2F1 gripper.'''
  def FingertipOffset2F1(self, pos):
    #WARNING: NotImplemented
    return 0.0

  '''Get current positions as an emulation of 2F1 gripper.'''
  def Position2F1(self):
    pos= self.Position()
    if pos is None:  return pos
    return self.g2f1_ang2pos(0.5*(pos[0]+pos[1])) if len(pos)==2 else None

  '''Control a gripper as an emulation of 2F1 gripper.
    pos: target position.
    max_effort: maximum effort to control.
    speed: speed of the movement.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Move2F1(self, pos, max_effort=50.0, speed=50.0, blocking=False):
    pos= self.g2f1_pos2ang(pos)
    self.Move([pos,pos], max_effort, speed, blocking)


'''Robot control class for DxlO3Gripper.
  This is defined as a subclass of TMultiArmRobot,
  but actually it does not have a body (only DxlO3Gripper gripper).
  This virtual body is designed for a compatibility of programs.'''
class TRobotDxlO3Gripper(TMultiArmRobot):
  def __init__(self, name='DxlO3Gripper', gripper_node='gripper_driver'):
    super(TRobotDxlO3Gripper,self).__init__(name=name)
    self.currarm= 0
    self.gripper_node= gripper_node

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    self.dxlo3_gripper= TDxlO3Gripper(node_name=self.gripper_node)
    self.grippers= [self.dxlo3_gripper]

    print 'Initializing and activating DxlO3Gripper gripper...'
    ra(self.dxlo3_gripper.Init())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    self.dxlo3_gripper.Cleanup()
    super(TRobotDxlO3Gripper,self).Cleanup()

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('DxlO3','DxlO3Gripper'):  return True
    return super(TRobotDxlO3Gripper,self).Is(q)

  @property
  def NumArms(self):
    return 1

  @property
  def BaseFrame(self):
    return 'world'

  '''Return range of gripper.
    arm: arm id, or None (==currarm). '''
  def GripperRange(self, arm=None):
    arm= 0
    return self.grippers[arm].PosRange2F1()

  '''Return range of gripper.
    arm: arm id, or None (==currarm). '''
  def GripperRange2(self, arm=None):
    arm= 0
    return self.grippers[arm].PosRange()

  '''End effector of an arm.'''
  def EndEff(self, arm):
    arm= 0
    return self.grippers[arm]

  #Dummy FK.
  def FK(self, q=None, x_ext=None, arm=None, with_st=False):
    x_res= [0,0,0, 0,0,0,1]
    return (x_res, True) if with_st else x_res

  '''Open a gripper.
    arm: arm id, or None (==currarm).
    blocking: False: move background, True: wait until motion ends.  '''
  def OpenGripper(self, arm=None, blocking=False):
    arm= 0
    gripper= self.grippers[arm]
    with self.gripper_locker:
      gripper.Open(blocking=blocking)

  '''Close a gripper.
    arm: arm id, or None (==currarm).
    blocking: False: move background, True: wait until motion ends.  '''
  def CloseGripper(self, arm=None, blocking=False):
    arm= 0
    gripper= self.grippers[arm]
    with self.gripper_locker:
      gripper.Close(blocking=blocking)

  '''High level interface to control a gripper.
    arm: arm id, or None (==currarm).
    pos: target position in meter.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum).
    blocking: False: move background, True: wait until motion ends.  '''
  def MoveGripper(self, pos, max_effort=50.0, speed=50.0, arm=None, blocking=False):
    arm= 0
    gripper= self.grippers[arm]
    with self.gripper_locker:
      gripper.Move2F1(pos, max_effort, speed, blocking=blocking)

  '''Low level interface to control a gripper.
    arm: arm id, or None (==currarm).
    pos: target positions.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum).
    blocking: False: move background, True: wait until motion ends.  '''
  def MoveGripper2(self, pos, max_effort=50.0, speed=50.0, arm=None, blocking=False):
    arm= 0
    gripper= self.grippers[arm]
    with self.gripper_locker:
      gripper.Move(pos, max_effort, speed, blocking=blocking)

  '''Get a gripper position in meter.
    arm: arm id, or None (==currarm). '''
  def GripperPos(self, arm=None):
    arm= 0
    gripper= self.grippers[arm]
    with self.sensor_locker:
      pos= gripper.Position2F1()
    return pos

  '''Get gripper positions.
    arm: arm id, or None (==currarm). '''
  def GripperPos2(self, arm=None):
    arm= 0
    gripper= self.grippers[arm]
    with self.sensor_locker:
      pos= gripper.Position()
    return pos

  '''Get a fingertip height offset in meter.
    The fingertip trajectory of some grippers has a rounded shape.
    This function gives the offset from the highest (longest) point (= closed fingertip position),
    and the offset is always negative.
    NOTE: In the previous versions (before 2019-12-10), this offset was from the opened fingertip position.
      pos: Gripper position to get the offset. None: Current position.
      arm: arm id, or None (==currarm).'''
  def FingertipOffset(self, pos=None, arm=None):
    arm= 0
    if pos is None:  pos= self.GripperPos(arm)
    return self.grippers[arm].FingertipOffset2F1(pos)

  '''Get a fingertip height offset in meter.
    The fingertip trajectory of some grippers has a rounded shape.
    This function gives the offset from the highest (longest) point (= closed fingertip position),
    and the offset is always negative.
    NOTE: In the previous versions (before 2019-12-10), this offset was from the opened fingertip position.
      pos: Gripper position to get the offset. None: Current position.
      arm: arm id, or None (==currarm).'''
  def FingertipOffset2(self, pos=None, arm=None):
    arm= 0
    if pos is None:  pos= self.GripperPos2(arm)
    return self.grippers[arm].FingertipOffset(pos)


