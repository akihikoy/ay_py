#! /usr/bin/env python
#Robot controller for a 3D-printed Gripper with Dynamixel.
from const import *
#if ROS_ROBOT not in ('ANY','DxlGripper','Mikata'):
  #raise ImportError('Stop importing: ROS_ROBOT is not DxlGripper')
#if ROS_DISTRO not in ('groovy','hydro','indigo'):  return

from robot import TGripper2F1,TMultiArmRobot
from ..misc.dxl_gripper import TDynamixelGripper


'''3D-printed Dynamixel Gripper utility class'''
class TDxlGripper(TGripper2F1):
  def __init__(self, dev='/dev/ttyUSB0'):
    super(TDxlGripper,self).__init__()

    self.dxlg= TDynamixelGripper(dev=dev)

    self.dxlg.CmdMax= 2200  #Gripper closed (with FingerVision).
    self.dxlg.CmdMin= 1200  #Gripper opened widely.
    self.dxlg.CmdOpen= 1900  #Gripper opened moderately.

    self.PosRange     = self.dxlg.PosRange
    self.Activate     = self.dxlg.Activate
    self.Deactivate   = self.dxlg.Deactivate
    self.Open         = self.dxlg.Open
    self.Close        = self.dxlg.Close
    self.Move         = self.dxlg.MoveTh
    self.Stop         = self.dxlg.StopMoveTh
    self.StartHolding = self.dxlg.StartHolding
    self.StopHolding  = self.dxlg.StopHolding

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= self.dxlg.Init()

    if self._is_initialized:
      self.dxlg.StartStateObs()
      self.dxlg.StartMoveTh()

    return self._is_initialized

  def Cleanup(self):
    self.dxlg.StopMoveTh()
    self.dxlg.StopStateObs()
    self.dxlg.Cleanup()
    super(TDxlGripper,self).Cleanup()

  '''Answer to a query q by {True,False}. e.g. Is('Robotiq').'''
  def Is(self, q):
    if q in ('DxlGripper',):  return True
    return super(TDxlGripper,self).Is(q)

  '''Get current position.'''
  def Position(self):
    return self.dxlg.State()['position']


'''Robot control class for DxlGripper.
  This is defined as a subclass of TMultiArmRobot,
  but actually it does not have a body (only DxlGripper gripper).
  This virtual body is designed for a compatibility of programs.'''
class TRobotDxlGripper(TMultiArmRobot):
  def __init__(self, name='DxlGripper', dev='/dev/ttyUSB0'):
    super(TRobotDxlGripper,self).__init__(name=name)
    self.currarm= 0
    self.dev= dev

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    self.dxl_gripper= TDxlGripper(dev=self.dev)
    self.grippers= [self.dxl_gripper, self.dxl_gripper]

    print 'Initializing and activating DxlGripper gripper...'
    ra(self.dxl_gripper.Init())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    self.dxl_gripper.Cleanup()
    super(TRobotDxlGripper,self).Cleanup()

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('DxlGripper',):  return True
    return super(TRobotDxlGripper,self).Is(q)

  @property
  def BaseFrame(self):
    return 'world'

  '''Return range of gripper.
    arm: arm id, or None (==currarm). '''
  def GripperRange(self, arm=None):
    if arm is None:  arm= self.Arm
    return self.grippers[arm].PosRange()

  '''End effector of an arm.'''
  def EndEff(self, arm):
    if arm is None:  arm= self.Arm
    return self.grippers[arm]

  #Dummy FK.
  def FK(self, q=None, x_ext=None, arm=None, with_st=False):
    x_res= [0,0,0, 0,0,0,1]
    return (x_res, True) if with_st else x_res

  '''Open a gripper.
    arm: arm id, or None (==currarm).
    blocking: False: move background, True: wait until motion ends.  '''
  def OpenGripper(self, arm=None, blocking=False):
    if arm is None:  arm= self.Arm
    gripper= self.grippers[arm]
    with self.control_locker:
      gripper.Open(blocking=blocking)

  '''Close a gripper.
    arm: arm id, or None (==currarm).
    blocking: False: move background, True: wait until motion ends.  '''
  def CloseGripper(self, arm=None, blocking=False):
    if arm is None:  arm= self.Arm
    gripper= self.grippers[arm]
    with self.control_locker:
      gripper.Close(blocking=blocking)

  '''High level interface to control a gripper.
    arm: arm id, or None (==currarm).
    pos: target position in meter.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum).
    blocking: False: move background, True: wait until motion ends.  '''
  def MoveGripper(self, pos, max_effort=50.0, speed=50.0, arm=None, blocking=False):
    if arm is None:  arm= self.Arm

    gripper= self.grippers[arm]
    with self.control_locker:
      gripper.Move(pos, max_effort, speed, blocking=blocking)

  '''Get a gripper position in meter.
    arm: arm id, or None (==currarm). '''
  def GripperPos(self, arm=None):
    if arm is None:  arm= self.Arm

    gripper= self.grippers[arm]
    with self.sensor_locker:
      pos= gripper.Position()
    return pos

  '''Get fingertip offset in meter.
    The fingertip trajectory of gripper has a round shape.
    This function gives the offset from the opening posture.
      pos: Gripper position to get the offset. None: Current position.
      arm: arm id, or None (==currarm).'''
  def FingertipOffset(self, pos=None, arm=None):
    return 0.0

