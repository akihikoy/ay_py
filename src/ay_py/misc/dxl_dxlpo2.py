#!/usr/bin/python
#\file    dxl_dxlpo2.py
#\brief   Control module of DxlpO2 gripper.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jan.28, 2020
from dxl_util import TDynamixel1
from ..core.util import TRate
import time
import threading
import copy

'''DxlpO2 gripper utility class'''
class TDxlpO2(object):
  def __init__(self, dev='/dev/ttyUSB0'):
    self.dxl_type= 'PH54-200-S500'
    self.dev= dev
    self.baudrate= 2e6
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

    self.CmdMax= 0  #Gripper opened widely.
    self.CmdMin= -250962  #Gripper closed.
    self.CmdOpen= -150000  #Gripper opened.
    self.CmdClose= -250962  #Gripper closed.

    #Gripper command-position conversions.
    self.gripper_range= [0.0,0.300]  #FIXME: 0.3 is inaccurate.
    self.gripper_cmd2pos= lambda cmd: max(self.gripper_range[0], self.gripper_range[0] + (cmd-self.CmdClose)*(self.gripper_range[1]-self.gripper_range[0])/(self.CmdOpen-self.CmdClose))
    self.gripper_pos2cmd= lambda pos: self.CmdClose + (pos-self.gripper_range[0])*(self.CmdOpen-self.CmdClose)/(self.gripper_range[1]-self.gripper_range[0])
    #WARNING: The following functions are not considered well (coped from other gripper).
    self.gripper_cmd2vel= lambda cmd: (self.gripper_range[1]-self.gripper_range[0])/(self.dxl.ConvPos(self.CmdOpen)-self.dxl.ConvPos(self.CmdClose))*self.dxl.ConvVel(cmd)
    self.gripper_vel2cmd= lambda vel: self.dxl.InvConvVel(vel*(self.dxl.ConvPos(self.CmdOpen)-self.dxl.ConvPos(self.CmdClose))/(self.gripper_range[1]-self.gripper_range[0]))

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    #self.dxl= TDynamixel1(self.dxl_type,dev=self.dev)
    self.dxl.Baudrate= self.baudrate

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
    return self.gripper_range

  '''Get current position.'''
  def Position(self):
    with self.port_locker:
      pos= self.dxl.Position()
    if pos is None:
      print 'DxlpO2: Failed to read position'
      return None
    pos= self.gripper_cmd2pos(pos)
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
    if not self.threads['MoveThController'][0]:
      self.Move(pos=self.gripper_range[1], blocking=blocking)
    else:
      self.MoveTh(pos=self.gripper_range[1], blocking=blocking)

  '''Close a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Close(self, blocking=False):
    if not self.threads['MoveThController'][0]:
      self.Move(pos=self.gripper_range[0], blocking=blocking)
    else:
      self.MoveTh(pos=self.gripper_range[0], blocking=blocking)

  '''Control a gripper.
    pos: target position in meter.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum); NOT_IMPLEMENTED.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Move(self, pos, max_effort=50.0, speed=50.0, blocking=False):
    cmd= max(self.CmdMin,min(self.CmdMax,int(self.gripper_pos2cmd(pos))))
    trg_curr= self.dxl.CurrentLimit*max_effort*0.01
    with self.port_locker:
      self.dxl.MoveToC(cmd, trg_curr, blocking=True if blocking else False)

  '''Stop the gripper motion. '''
  def Stop(self):
    if not self.threads['MoveThController'][0]:
      self.Move(self.Position(), blocking=False)
    else:
      self.MoveTh(pos=self.State()['position'], blocking=blocking)


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
    cmd= max(self.CmdMin,min(self.CmdMax,int(self.gripper_pos2cmd(pos))))
    with self.moveth_locker:
      self.moveth_cmd= {'pos':pos,'max_effort':max_effort}
    rate= TRate(self.hz_moveth_ctrl)
    p_log= []
    while blocking:
      p= self.State()['position']
      if p is None:  return
      p_log.append(self.gripper_pos2cmd(p))
      if len(p_log)>10:  p_log.pop(0)
      if not (abs(cmd - p_log[-1]) > self.dxl.GoalThreshold):  break
      #Detecting stuck:
      if len(p_log)>=10 and not (abs(p_log[0] - p_log[-1]) > self.dxl.GoalThreshold):
        CPrint(4,'TDxlpO2: Control gets stuck in MoveTh. Abort.')
        break
      #print abs(cmd - p_log[-1]) - self.dxl.GoalThreshold
      rate.sleep()


  #Start MoveTh controller.
  def StartMoveTh(self):
    self.StopMoveTh()
    with self.port_locker:
      goal_pos= self.gripper_cmd2pos(self.dxl.Read('GOAL_POSITION'))
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
        p,v,c= self.dxl.Position(),self.dxl.Velocity(),self.dxl.Current()
      state= {
        'stamp':time.time(),
        'position':self.gripper_cmd2pos(p) if p is not None else None,
        'velocity':self.gripper_cmd2vel(v) if v is not None else None,
        'effort':(c/self.dxl.CurrentLimit*100.0) if c is not None else None,
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
      cmd= max(self.CmdMin,min(self.CmdMax,int(self.gripper_pos2cmd(pos))))
      trg_curr= self.dxl.CurrentLimit*max_effort*0.01

      with self.port_locker:
        self.dxl.MoveToC(cmd, trg_curr, blocking=False)

      #print 'dxl_gripper:MoveThController:rate.remaining:',rate.remaining()
      rate.sleep()
    self.threads['MoveThController'][0]= False

