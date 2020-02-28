#! /usr/bin/env python
#\brief   Robot controller for DxlO3 gripper.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Feb.10, 2020
from const import *

from robot import TGripper2F2,TMultiArmRobot
from ..misc.dxl_dxlo3 import TDxlO3


'''DxlO3 gripper utility class'''
class TDxlO3Gripper(TGripper2F2):
  def __init__(self, dev='/dev/ttyUSB0'):
    super(TDxlO3Gripper,self).__init__()

    self.dxlo3= TDxlO3(dev=dev)

    #Gripper position-angles conversions (for 2F1 emulation mode).
    self.g2f1_ang_open= 0.6136  #cmd=+400
    self.g2f1_ang_close= 0.1657  #cmd=+108; With FV+
    self.g2f1_range= [0.0,0.118]  #cmd=[108,400]; With FV+
    self.g2f1_ang2pos= lambda ang: max(self.g2f1_range[0], self.g2f1_range[0] + (ang-self.g2f1_ang_close)*(self.g2f1_range[1]-self.g2f1_range[0])/(self.g2f1_ang_open-self.g2f1_ang_close))
    self.g2f1_pos2ang= lambda pos: self.g2f1_ang_close + (pos-self.g2f1_range[0])*(self.g2f1_ang_open-self.g2f1_ang_close)/(self.g2f1_range[1]-self.g2f1_range[0])


  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= self.dxlo3.Init()

    if self._is_initialized:
      self.dxlo3.StartStateObs()
      self.dxlo3.StartMoveTh()

    return self._is_initialized

  def Cleanup(self):
    if self._is_initialized:
      self.dxlo3.StopMoveTh()
      self.dxlo3.StopStateObs()
      self.dxlo3.Cleanup()
      self._is_initialized= False
    super(TDxlO3Gripper,self).Cleanup()

  '''Answer to a query q by {True,False}. e.g. Is('Robotiq').'''
  def Is(self, q):
    if q in ('DxlO3','DxlO3Gripper'):  return True
    return super(TDxlO3Gripper,self).Is(q)

  '''Range of gripper positions.'''
  def PosRange(self):
    return self.dxlo3.PosRange()

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

  '''Get current positions.'''
  def Position(self):
    return self.dxlo3.State()['position']

  '''Get current positions as an emulation of 2F1 gripper.'''
  def Position2F1(self):
    pos= self.Position()
    if pos is None:  return pos
    return self.g2f1_ang2pos(0.5*(pos[0]+pos[1])) if len(pos)==2 else None

  '''Activate gripper (torque is enabled).
    Return success or not.'''
  def Activate(self):
    return self.dxlo3.Activate()

  '''Deactivate gripper (torque is disabled).
    Return success or not.'''
  def Deactivate(self):
    return self.dxlo3.Deactivate()

  '''Open a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Open(self, blocking=False):
    self.dxlo3.Open(blocking=blocking)

  '''Close a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Close(self, blocking=False):
    self.dxlo3.Close(blocking=blocking)

  '''Control a gripper.
    pos: target positions.
    max_effort: maximum effort to control.
    speed: speed of the movement.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Move(self, pos, max_effort=50.0, speed=50.0, blocking=False):
    self.dxlo3.MoveTh(pos, max_effort, speed, blocking)

  '''Control a gripper as an emulation of 2F1 gripper.
    pos: target position.
    max_effort: maximum effort to control.
    speed: speed of the movement.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Move2F1(self, pos, max_effort=50.0, speed=50.0, blocking=False):
    pos= self.g2f1_pos2ang(pos)
    self.dxlo3.MoveTh([pos,pos], max_effort, speed, blocking)

  '''Stop the gripper motion. '''
  def Stop(self):
    self.dxlo3.StopMoveTh()


'''Robot control class for DxlO3Gripper.
  This is defined as a subclass of TMultiArmRobot,
  but actually it does not have a body (only DxlO3Gripper gripper).
  This virtual body is designed for a compatibility of programs.'''
class TRobotDxlO3Gripper(TMultiArmRobot):
  def __init__(self, name='DxlO3Gripper', dev='/dev/ttyUSB0'):
    super(TRobotDxlO3Gripper,self).__init__(name=name)
    self.currarm= 0
    self.dev= dev

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    self.dxlo3_gripper= TDxlO3Gripper(dev=self.dev)
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


