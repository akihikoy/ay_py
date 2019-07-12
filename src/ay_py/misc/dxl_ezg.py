#!/usr/bin/python
#\file    dxl_ezg.py
#\brief   Control module of SAKE EZGripper Gen2.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jul.12, 2019
#         NOTE: Dynamixel MX-64AR of EZGripper is originally protocol 1.0; update the firmware.
from dxl_util import TDynamixel1
from ..misc.dxl_holding import TDxlHolding
from ..core.util import TRate
import time
import threading
import copy

'''SAKE EZGripper Gen2 utility class'''
class TEZG(object):
  def __init__(self, dev='/dev/ttyUSB0'):
    self.dxl_type= 'MX-64AR'
    self.dev= dev
    self.baudrate= 2e6
    self.op_mode= 'CURRPOS'
    self.dxl= TDynamixel1(self.dxl_type,dev=self.dev)

    #Thread locker:
    self.port_locker= threading.RLock()
    self.state_locker= threading.RLock()
    self.moveth_locker= threading.RLock()
    self.state= {'stamp':0.0, 'position':None, 'velocity':None, 'effort':None}
    self.moveth_cmd= {'pos':None,'max_effort':None}
    self.hz_state_obs= 40  #State observation rate (Hz).
    self.hz_moveth_ctrl= 60  #MoveTh control rate (Hz).
    self.threads= {  #ThreadName:[IsActive,ThreadObject]
      'StateObserver':[False,None],
      'MoveThController':[False,None],}

    self.CmdMax= 2104  #Gripper opened widely.
    self.CmdMin= 50  #Gripper closed strongly.
    self.CmdOpen= 809  #Gripper opened.
    self.CmdClose= 233  #Gripper closed.

    #Gripper command-position conversions.
    self.ezg_range= [0.0,0.150]
    self.ezg_cmd2pos= lambda cmd: max(self.ezg_range[0], self.ezg_range[0] + (cmd-self.CmdClose)*(self.ezg_range[1]-self.ezg_range[0])/(self.CmdOpen-self.CmdClose))
    self.ezg_pos2cmd= lambda pos: self.CmdClose + (pos-self.ezg_range[0])*(self.CmdOpen-self.CmdClose)/(self.ezg_range[1]-self.ezg_range[0])
    self.ezg_cmd2vel= lambda cmd: (self.ezg_range[1]-self.ezg_range[0])/(self.dxl.ConvPos(self.CmdOpen)-self.dxl.ConvPos(self.CmdClose))*self.dxl.ConvVel(cmd)
    self.ezg_vel2cmd= lambda vel: self.dxl.InvConvVel(vel*(self.dxl.ConvPos(self.CmdOpen)-self.dxl.ConvPos(self.CmdClose))/(self.ezg_range[1]-self.ezg_range[0]))

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    #self.dxl= TDynamixel1(self.dxl_type,dev=self.dev)
    self.dxl.Baudrate= self.baudrate
    self.OpMode= self.op_mode

    with self.port_locker:
      ra(self.dxl.Setup())

    ra(self.Activate())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    if self._is_initialized:
      self.Deactivate()
      with self.port_locker:
        self.dxl.Quit()
      self._is_initialized= False

    #Check the thread lockers status:
    print 'Count of port_locker:',self.port_locker._RLock__count

  '''Range of gripper position.'''
  def PosRange(self):
    return self.ezg_range

  '''Get current position.'''
  def Position(self):
    with self.port_locker:
      pos= self.dxl.Position()
    if pos is None:
      print 'EZGripper: Failed to read position'
      return None
    pos= self.ezg_cmd2pos(pos)
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
    self.Move(pos=self.ezg_range[1], blocking=blocking)

  '''Close a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Close(self, blocking=False):
    self.Move(pos=self.ezg_range[0], blocking=blocking)

  '''Control a gripper.
    pos: target position in meter.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum); NOT_IMPLEMENTED.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Move(self, pos, max_effort=50.0, speed=50.0, blocking=False):
    cmd= max(self.CmdMin,min(self.CmdMax,int(self.ezg_pos2cmd(pos))))
    trg_curr= self.dxl.CurrentLimit*max_effort*0.01
    with self.port_locker:
      self.dxl.MoveToC(cmd, trg_curr, blocking=True if blocking else False)

  '''Stop the gripper motion. '''
  def Stop(self):
    with self.port_locker:
      self.dxl.MoveTo(self.dxl.Position(), blocking=False)


  #Get current state saved in memory (no port access when running this function).
  #Run StartStateObs before using this.
  def State(self):
    with self.state_locker:
      state= copy.deepcopy(self.state)
    return state

  #Start state observation.
  #  callback: Callback function at the end of each observation cycle.
  def StartStateObs(self, callback=None):
    self.StopStateObs()
    th_func= lambda:self.StateObserver(callback)
    self._state_observer_callback= callback  #For future use.
    self.threads['StateObserver']= [True, threading.Thread(name='StateObserver', target=th_func)]
    self.threads['StateObserver'][1].start()

  #Stop state observation.
  def StopStateObs(self):
    if self.threads['StateObserver'][0]:
      self.threads['StateObserver'][0]= False
      self.threads['StateObserver'][1].join()
    self.threads['StateObserver']= [False,None]

  #Set rate (Hz) of state observation.
  #Works anytime.
  def SetStateObsRate(self, rate):
    if self.hz_state_obs!=rate:
      self.hz_state_obs= rate
      if self.threads['StateObserver'][0]:
        self.StartStateObs(self._state_observer_callback)

  '''Thread version of Move: Control a gripper.
    pos: target position in meter.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum); NOT_IMPLEMENTED.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def MoveTh(self, pos, max_effort=50.0, speed=50.0, blocking=False):
    cmd= max(self.CmdMin,min(self.CmdMax,int(self.ezg_pos2cmd(pos))))
    with self.moveth_locker:
      self.moveth_cmd= {'pos':pos,'max_effort':max_effort}
    rate= TRate(self.hz_moveth_ctrl)
    while blocking:
      p= self.State()['position']
      if p is None:  return
      if not (abs(cmd - self.ezg_pos2cmd(p)) > self.dxl.GoalThreshold):  break
      #print abs(cmd - self.ezg_pos2cmd(p)) - self.dxl.GoalThreshold
      rate.sleep()


  #Start MoveTh controller.
  def StartMoveTh(self):
    self.StopMoveTh()
    with self.port_locker:
      goal_pos= self.ezg_cmd2pos(self.dxl.Read('GOAL_POSITION'))
      max_effort= self.dxl.Read('GOAL_CURRENT')/self.dxl.CurrentLimit*100.0
    with self.moveth_locker:
      self.moveth_cmd= {'pos':goal_pos,'max_effort':max_effort}
    th_func= lambda:self.MoveThController()
    self.threads['MoveThController']= [True, threading.Thread(name='MoveThController', target=th_func)]
    self.threads['MoveThController'][1].start()

  #Stop MoveTh.
  def StopMoveTh(self):
    if self.threads['MoveThController'][0]:
      self.threads['MoveThController'][0]= False
      self.threads['MoveThController'][1].join()
    self.threads['MoveThController']= [False,None]

  #Set rate (Hz) of MoveTh controller.
  def SetMoveThCtrlRate(self, rate):
    if self.hz_moveth_ctrl!=rate:
      self.hz_moveth_ctrl= rate

  #State observer thread.
  #NOTE: Don't call this function directly.  Use self.StartStateObs and self.State
  def StateObserver(self, callback):
    rate= TRate(self.hz_state_obs)
    while self.threads['StateObserver'][0]:
      with self.port_locker:
        state= {
          'stamp':time.time(),
          'position':self.ezg_cmd2pos(self.dxl.Position()),
          'velocity':self.ezg_cmd2vel(self.dxl.Velocity()),
          'effort':self.dxl.Current()/self.dxl.CurrentLimit*100.0,  #FIXME: PWM vs. Current
          }
        #print state['position']
      with self.state_locker:
        self.state= state
      if callback is not None:
        callback(state)
      rate.sleep()
    self.threads['StateObserver'][0]= False

  #MoveTh controller thread.
  #NOTE: Don't call this function directly.  Use self.StartMoveTh
  def MoveThController(self):
    rate= TRate(self.hz_moveth_ctrl)
    while self.threads['MoveThController'][0]:
      with self.moveth_locker:
        moveth_cmd= copy.deepcopy(self.moveth_cmd)
      pos,max_effort= moveth_cmd['pos'],moveth_cmd['max_effort']
      cmd= max(self.CmdMin,min(self.CmdMax,int(self.ezg_pos2cmd(pos))))
      trg_curr= self.dxl.CurrentLimit*max_effort*0.01

      with self.port_locker:
        self.dxl.MoveToC(cmd, trg_curr, blocking=False)

      #print 'dxl_gripper:MoveThController:rate.remaining:',rate.remaining()
      rate.sleep()
    self.threads['MoveThController'][0]= False

