#!/usr/bin/python
#\file    dxl_gripper.py
#\brief   Control module of Dynamixel Gripper (3D-printed Gripper with Dynamixel).
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Oct.29, 2017
from dxl_util import TDynamixel1
from ..misc.dxl_holding import TDxlHolding
import threading

'''3D-printed Dynamixel Gripper utility class'''
class TDynamixelGripper(object):
  def __init__(self, dev='/dev/ttyUSB0'):
    self.dxl_type= 'XM430-W350'
    self.dev= dev

    #Thread locker:
    self.port_locker= threading.RLock()

    #Holding controller:
    self.holding= None
    self.holding_max_pwm_rate= 0.9

    self.CmdMax= 2382  #Gripper closed.
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

    self.dxl= TDynamixel1(self.dxl_type,dev=self.dev)

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
