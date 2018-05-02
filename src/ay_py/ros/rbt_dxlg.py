#! /usr/bin/env python
#Robot controller for a 3D-printed Gripper with Dynamixel.
from const import *
#if ROS_ROBOT not in ('ANY','DxlGripper','Mikata'):
  #raise ImportError('Stop importing: ROS_ROBOT is not DxlGripper')
#if ROS_DISTRO not in ('groovy','hydro','indigo'):  return

from robot import TGripper2F1,TMultiArmRobot
from ..misc.dxl_util import TDynamixel1
from ..misc.dxl_holding import TDxlHolding
import threading


'''3D-printed Dynamixel Gripper utility class'''
class TDxlGripper(TGripper2F1):
  def __init__(self, dev='/dev/ttyUSB0'):
    super(TDxlGripper,self).__init__()
    self.dev= dev

    #Thread locker:
    self.port_locker= threading.RLock()

    #Holding controller:
    self.holding= None
    self.holding_max_pwm_rate= 0.9

    #self.CmdMax= 2382  #Gripper closed.
    self.CmdMax= 2200  #Gripper closed (with FingerVision).
    self.CmdMin= 1200  #Gripper opened widely.
    self.CmdOpen= 1900  #Gripper opened moderately.

    #Gripper command-position conversions.
    self.dxlg_range= [0.0,0.095]
    self.dxlg_cmd2pos= lambda cmd: self.dxlg_range[1] + (cmd-self.CmdOpen)*(self.dxlg_range[0]-self.dxlg_range[1])/(self.CmdMax-self.CmdOpen)
    self.dxlg_pos2cmd= lambda pos: self.CmdOpen + (pos-self.dxlg_range[1])*(self.CmdMax-self.CmdOpen)/(self.dxlg_range[0]-self.dxlg_range[1])

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    self.dxl= TDynamixel1('XM430-W350',dev=self.dev)

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

  '''Range of gripper position.'''
  def PosRange(self):
    return self.dxlg_range

  '''Get current position.'''
  def Position(self):
    with self.port_locker:
      pos= self.dxl.Position()
    if pos is None:
      print 'DxlGripper: Failed to read position'
      return None
    pos= self.dxlg_cmd2pos(pos)
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
    self.Move(pos=self.dxlg_range[1], blocking=blocking)

  '''Close a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Close(self, blocking=False):
    self.Move(pos=self.dxlg_range[0], blocking=blocking)

  '''Control a gripper.
    pos: target position in meter.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum); NOT_IMPLEMENTED.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Move(self, pos, max_effort=50.0, speed=50.0, blocking=False):
    cmd= max(self.CmdMin,min(self.CmdMax,int(self.dxlg_pos2cmd(pos))))
    max_pwm= self.dxl.InvConvPWM(max_effort)
    if self.holding is None:
      with self.port_locker:
        self.dxl.SetPWM(max_pwm)
        self.dxl.MoveTo(cmd, blocking=True if blocking else False)
    else:
      with self.port_locker:
        self.dxl.SetPWM(max_pwm)
      self.holding.SetTarget(cmd, self.holding_max_pwm_rate*max_pwm)

  '''Stop the gripper motion. '''
  def Stop(self):
    self.Move(self.Position(), blocking=False)

  ##Low level position control.
  #def low_move(self, cmd, blocking):
    #self.dxl.MoveTo(cmd, blocking=blocking)
  ##Low level pwm control.
  #def low_set_pwm(self, pwm):
    #with self.port_locker:
      #self.dxl.SetPWM(pwm)
  ##Low level observer.
  #def low_observe(self):
    #with self.port_locker:
      #pos,vel,pwm= self.dxl.Position(), self.dxl.Velocity(), self.dxl.PWM()
    #return pos,vel,pwm

  #Start holding controller with control rate (Hz).
  def StartHolding(self, rate=30):
    self.StopHolding()

    def holding_observer():
      with self.port_locker:
        pos,vel,pwm= self.dxl.Position(), self.dxl.Velocity(), self.dxl.PWM()
      return pos,vel,pwm
    def holding_controller(target_position):
      with self.port_locker:
        self.dxl.MoveTo(target_position, blocking=False)

    with self.port_locker:
      goal_pos= self.dxl.Read('GOAL_POSITION')
      max_pwm= self.dxl.Read('GOAL_PWM')

    self.holding= TDxlHolding(rate)
    self.holding.observer= holding_observer
    self.holding.controller= holding_controller
    self.holding.SetTarget(goal_pos, self.holding_max_pwm_rate*max_pwm)
    self.holding.Start()

  def StopHolding(self):
    if self.holding is not None:
      self.holding.Stop()
    self.holding= None


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

