#! /usr/bin/env python
#Robot controller for a 3D-printed Gripper with Dynamixel.
from const import *
#if ROS_ROBOT not in ('ANY','DxlGripper','Mikata'):
  #raise ImportError('Stop importing: ROS_ROBOT is not DxlGripper')
#if ROS_DISTRO not in ('groovy','hydro','indigo'):  return

from robot import TGripper2F1,TMultiArmRobot
from ..misc.dxl_util import TDynamixel1
import threading


'''3D-printed Dynamixel Gripper utility class'''
class TDxlGripper(TGripper2F1):
  def __init__(self, dev='/dev/ttyUSB0'):
    super(TDxlGripper,self).__init__()
    self.dxl= TDynamixel1('XM430-W350',dev=dev)

    #Thread locker:
    self.port_locker= threading.RLock()

    #self.CmdMax= 2382  #Gripper closed.
    self.CmdMax= 2200  #Gripper closed (with FingerVision).
    self.CmdMin= 1200  #Gripper opened widely.
    self.CmdOpen= 1900  #Gripper opened moderately.

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    with self.port_locker:
      ra(self.dxl.Setup())

    ra(self.Activate())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    self.Deactivate()
    with self.port_locker:
      self.dxl.Quit()

    #Check the thread lockers status:
    print 'Count of port_locker:',self.port_locker._RLock__count

    super(TDxlGripper,self).Cleanup()

  '''Answer to a query q by {True,False}. e.g. Is('Robotiq').'''
  def Is(self, q):
    if q in ('DxlGripper',):  return True
    return super(TDxlGripper,self).Is(q)

  '''Get current position.'''
  def Position(self):
    with self.port_locker:
      pos= self.dxl.Position()
    if pos is None:  print 'DxlGripper: Failed to read position'
    return pos

  '''Activate gripper (torque is enabled).
    Return success or not.'''
  def Activate(self):
    with self.port_locker:
      res= self.dxl.EnableTorque()
    return res

  '''Deactivate gripper (torque is disabled).
    Return success or not.'''
  def Deactivate(self):
    with self.port_locker:
      res= self.dxl.DisableTorque()
    return res

  '''Open a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Open(self, blocking=False):
    self.Move(pos=self.CmdOpen, blocking=blocking)

  '''Close a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Close(self, blocking=False):
    self.Move(pos=self.CmdMax, blocking=blocking)

  '''Control a gripper.
    pos: target position; CmdMin (open widely), CmdOpen (open moderately), CmdMax (close).
    max_effort: maximum effort to control; NOT_IMPLEMENTED.
    speed: speed of the movement; NOT_IMPLEMENTED.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Move(self, pos, max_effort=None, speed=None, blocking=False):
    pos= max(self.CmdMin,min(self.CmdMax,int(pos)))
    with self.port_locker:
      self.dxl.MoveTo(pos, wait=True if blocking else False)

  '''Stop the gripper motion. '''
  def Stop(self):
    with self.port_locker:
      self.dxl.MoveTo(self.dxl.Position(), wait=False)



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

    #Gripper command-position conversions.
    self.dxlg_range= [0.0,0.095]
    self.dxlg_cmd2pos= lambda cmd: self.dxlg_range[1] + (cmd-self.dxl_gripper.CmdOpen)*(self.dxlg_range[0]-self.dxlg_range[1])/(self.dxl_gripper.CmdMax-self.dxl_gripper.CmdOpen)
    self.dxlg_pos2cmd= lambda pos: self.dxl_gripper.CmdOpen + (pos-self.dxlg_range[1])*(self.dxl_gripper.CmdMax-self.dxl_gripper.CmdOpen)/(self.dxlg_range[0]-self.dxlg_range[1])

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
    return self.dxlg_range

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
    self.MoveGripper(pos=0.1, arm=arm, blocking=blocking)

  '''Close a gripper.
    arm: arm id, or None (==currarm).
    blocking: False: move background, True: wait until motion ends.  '''
  def CloseGripper(self, arm=None, blocking=False):
    self.MoveGripper(pos=0.0, arm=arm, blocking=blocking)

  '''High level interface to control a gripper.
    arm: arm id, or None (==currarm).
    pos: target position in meter.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum).
    blocking: False: move background, True: wait until motion ends.  '''
  def MoveGripper(self, pos, max_effort=50.0, speed=50.0, arm=None, blocking=False):
    if arm is None:  arm= self.Arm

    gripper= self.grippers[arm]
    cmd= self.dxlg_pos2cmd(pos)
    with self.control_locker:
      gripper.Move(cmd, blocking=blocking)

  '''Get a gripper position in meter.
    arm: arm id, or None (==currarm). '''
  def GripperPos(self, arm=None):
    if arm is None:  arm= self.Arm

    gripper= self.grippers[arm]
    with self.sensor_locker:
      pos= self.dxlg_cmd2pos(gripper.Position())
    return pos

  '''Get fingertip offset in meter.
    The fingertip trajectory of gripper has a round shape.
    This function gives the offset from the opening posture.
      pos: Gripper position to get the offset. None: Current position.
      arm: arm id, or None (==currarm).'''
  def FingertipOffset(self, pos=None, arm=None):
    return 0.0

