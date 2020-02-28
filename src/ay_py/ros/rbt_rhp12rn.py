#! /usr/bin/env python
#\brief   Robot controller for RH-P12-RN Gripper (Thormang3 gripper).
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Nov.21, 2018
from const import *

from robot import TGripper2F1,TMultiArmRobot
from ..misc.dxl_rhp12rn import TRHP12RN


'''RH-P12-RN Gripper (Thormang3 gripper) utility class'''
class TRHP12RNGripper(TGripper2F1):
  def __init__(self, dev='/dev/ttyUSB0'):
    super(TRHP12RNGripper,self).__init__()

    self.thg= TRHP12RN(dev=dev)

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= self.thg.Init()

    if self._is_initialized:
      self.thg.StartStateObs()
      self.thg.StartMoveTh()

    return self._is_initialized

  def Cleanup(self):
    if self._is_initialized:
      self.thg.StopMoveTh()
      self.thg.StopStateObs()
      self.thg.Cleanup()
      self._is_initialized= False
    super(TRHP12RNGripper,self).Cleanup()

  '''Answer to a query q by {True,False}. e.g. Is('Robotiq').'''
  def Is(self, q):
    if q in ('RHP12RNGripper','ThGripper'):  return True
    return super(TRHP12RNGripper,self).Is(q)

  '''Get current position.'''
  def Position(self):
    return self.thg.State()['position']

  '''Get a fingertip height offset in meter.
    The fingertip trajectory of some grippers has a rounded shape.
    This function gives the offset from the highest (longest) point (= closed fingertip position),
    and the offset is always negative.
      pos: Gripper position to get the offset. '''
  def FingertipOffset(self, pos):
    #WARNING: This is accurate only at the open (width=109mm) and close (width=0mm) positions.
    #0.067... = (145.-(145.**2-(109./2.)**2-117.**2)/(2.*145.-2.*117.))*1.e-3
    return (0.06704017857142858**2-0.25*pos*pos)**0.5 - 0.06704017857142858

  def PosRange(self):
    return self.thg.PosRange()
  def Activate(self):
    return self.thg.Activate()
  def Deactivate(self):
    return self.thg.Deactivate()
  def Open(self, blocking=False):
    self.Move(pos=self.thg.thg_range[1], blocking=blocking)
  def Close(self, blocking=False):
    self.Move(pos=self.thg.thg_range[0], blocking=blocking)
  def Move(self, pos, max_effort=50.0, speed=50.0, blocking=False):
    self.thg.MoveTh(pos, max_effort, speed, blocking)
  def Stop(self):
    self.thg.StopMoveTh()


'''Robot control class for RHP12RNGripper.
  This is defined as a subclass of TMultiArmRobot,
  but actually it does not have a body (only RHP12RNGripper gripper).
  This virtual body is designed for a compatibility of programs.'''
class TRobotRHP12RNGripper(TMultiArmRobot):
  def __init__(self, name='RHP12RNGripper', dev='/dev/ttyUSB0'):
    super(TRobotRHP12RNGripper,self).__init__(name=name)
    self.currarm= 0
    self.dev= dev

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    self.th_gripper= TRHP12RNGripper(dev=self.dev)
    self.grippers= [self.th_gripper]

    print 'Initializing and activating RHP12RNGripper gripper...'
    ra(self.th_gripper.Init())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    self.th_gripper.Cleanup()
    super(TRobotRHP12RNGripper,self).Cleanup()

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('RHP12RNGripper','ThGripper'):  return True
    return super(TRobotRHP12RNGripper,self).Is(q)

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
      gripper.Move(pos, max_effort, speed, blocking=blocking)

  '''Get a gripper position in meter.
    arm: arm id, or None (==currarm). '''
  def GripperPos(self, arm=None):
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
    return self.grippers[arm].FingertipOffset(pos)


