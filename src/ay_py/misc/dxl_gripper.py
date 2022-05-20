#!/usr/bin/python
#\file    dxl_gripper.py
#\brief   Control module of Dynamixel Gripper (3D-printed Gripper with Dynamixel).
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Oct.29, 2017
#\version 0.2
#\date    May.21, 2022
#         Refactored the code.
from dxl_util import TDynamixel1
from ..misc.dxl_holding import TDxlHolding
from ..core.util import TRate, CPrint
import time
import threading
import copy


'''Dynamixel Gripper base class'''
class TDxlGripperBase(object):
  def __init__(self, dxl_type, dev='/dev/ttyUSB0'):
    self.dxl_type= dxl_type
    self.dev= dev
    self.baudrate= 2e6
    self.op_mode= None  #Using default
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

    self.CmdMax= None
    self.CmdMin= None
    self.CmdOpen= None
    self.CmdClose= None
    #Gripper range in meter:
    self.GrpClose= None
    self.GrpOpen= None
    self.GrpMin= None
    self.GrpMax= None

    #Gripper command-position conversions.
    self.gripper_cmd2pos= lambda cmd: min(self.GrpMax,max(self.GrpMin, self.GrpClose + (cmd-self.CmdClose)*(self.GrpOpen-self.GrpClose)/(self.CmdOpen-self.CmdClose) ))
    self.gripper_pos2cmd= lambda pos: min(self.CmdMax,max(self.CmdMin, self.CmdClose + (pos-self.GrpClose)*(self.CmdOpen-self.CmdClose)/(self.GrpOpen-self.GrpClose) ))
    self.gripper_cmd2vel= lambda cmd: (self.GrpOpen-self.GrpClose)/(self.dxl.ConvPos(self.CmdOpen)-self.dxl.ConvPos(self.CmdClose))*self.dxl.ConvVel(cmd)
    self.gripper_vel2cmd= lambda vel: self.dxl.InvConvVel(vel*(self.dxl.ConvPos(self.CmdOpen)-self.dxl.ConvPos(self.CmdClose))/(self.GrpOpen-self.GrpClose))

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    #self.dxl= TDynamixel1(self.dxl_type,dev=self.dev)
    self.dxl.Baudrate= self.baudrate
    if self.op_mode is not None:  self.dxl.OpMode= self.op_mode

    with self.port_locker:
      ra(self.dxl.Setup())

    ra(self.Activate())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    if self._is_initialized:
      self.StopMoveTh()
      self.StopStateObs()
      self.Deactivate()
      with self.port_locker:
        self.dxl.Quit()
      self._is_initialized= False

    #Check the thread lockers status:
    print 'Count of port_locker:',self.port_locker._RLock__count

  '''Range of gripper position.'''
  def PosRange(self):
    return [self.GrpMin, self.GrpMax]

  '''Get current position.'''
  def Position(self):
    with self.port_locker:
      pos= self.dxl.Position()
    if pos is None:
      print 'DxlG: Failed to read position'
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
      self.Move(pos=self.GrpOpen, blocking=blocking)
    else:
      self.MoveTh(pos=self.GrpOpen, blocking=blocking)

  '''Close a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Close(self, blocking=False):
    if not self.threads['MoveThController'][0]:
      self.Move(pos=self.GrpClose, blocking=blocking)
    else:
      self.MoveTh(pos=self.GrpClose, blocking=blocking)

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
  def Stop(self, blocking=False):
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
        CPrint(4,'DxlG: Control gets stuck in MoveTh. Abort.')
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


'''3D-printed Dynamixel Gripper utility class'''
class TDynamixelGripper(TDxlGripperBase):
  def __init__(self, dev='/dev/ttyUSB0'):
    super(TDynamixelGripper,self).__init__(dxl_type='XM430-W350',dev=dev)

    #Holding controller:
    self.holding= None
    self.holding_max_pwm_rate= 0.9

    self.CmdMax= 2382  #Gripper closed.
    self.CmdMin= 1200  #Gripper opened widely.
    self.CmdOpen= 1900  #Gripper opened moderately.
    self.CmdClose= 2200  #Gripper closed (with FingerVision).
    #Gripper range in meter:
    self.GrpClose= 0.0
    self.GrpOpen= 0.095
    self.GrpMin= 0.0
    self.GrpMax= 0.095

  '''Control a gripper.
    pos: target position in meter.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum); NOT_IMPLEMENTED.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Move(self, pos, max_effort=50.0, speed=50.0, blocking=False):
    cmd= max(self.CmdMin,min(self.CmdMax,int(self.gripper_pos2cmd(pos))))
    max_pwm= self.dxl.InvConvPWM(max_effort)
    if self.holding is None:
      with self.port_locker:
        self.dxl.SetPWM(max_pwm)
        self.dxl.MoveTo(cmd, blocking=True if blocking else False)
    else:
      with self.port_locker:
        self.dxl.SetPWM(max_pwm)
      self.holding.SetTarget(cmd, self.holding_max_pwm_rate*max_pwm)

  '''Thread version of Move: Control a gripper.
    pos: target position in meter.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum); NOT_IMPLEMENTED.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def MoveTh(self, pos, max_effort=50.0, speed=50.0, blocking=False):
    cmd= max(self.CmdMin,min(self.CmdMax,int(self.gripper_pos2cmd(pos))))
    max_pwm= self.dxl.InvConvPWM(max_effort)
    if self.holding is None:
      with self.moveth_locker:
        self.moveth_cmd= {'pos':pos,'max_effort':max_effort}
      rate= TRate(self.hz_moveth_ctrl)
      while blocking:
        p= self.State()['position']
        if p is None:  return
        if not (abs(cmd - self.gripper_pos2cmd(p)) > self.dxl.GoalThreshold):  break
        #print abs(cmd - self.gripper_pos2cmd(p)) - self.dxl.GoalThreshold
        rate.sleep()
    else:
      with self.moveth_locker:
        self.moveth_cmd= {'pos':pos,'max_effort':max_effort}
      self.holding.SetTarget(cmd, self.holding_max_pwm_rate*max_pwm)

  #Start MoveTh controller.
  def StartMoveTh(self):
    self.StopMoveTh()
    with self.port_locker:
      goal_pos= self.gripper_cmd2pos(self.dxl.Read('GOAL_POSITION'))
      max_effort= self.dxl.ConvPWM(self.dxl.Read('GOAL_PWM'))
    with self.moveth_locker:
      self.moveth_cmd= {'pos':goal_pos,'max_effort':max_effort}
    th_func= lambda:self.MoveThController()
    self.threads['MoveThController']= [True, threading.Thread(name='MoveThController', target=th_func)]
    self.threads['MoveThController'][1].start()

  #MoveTh controller thread.
  #NOTE: Don't call this function directly.  Use self.StartMoveTh
  def MoveThController(self):
    rate= TRate(self.hz_moveth_ctrl)
    while self.threads['MoveThController'][0]:
      with self.moveth_locker:
        moveth_cmd= copy.deepcopy(self.moveth_cmd)
      pos,max_effort= moveth_cmd['pos'],moveth_cmd['max_effort']
      cmd= max(self.CmdMin,min(self.CmdMax,int(self.gripper_pos2cmd(pos))))
      max_pwm= self.dxl.InvConvPWM(max_effort)

      #print 'entry..'
      if self.holding is None:
        #print 'a..'
        with self.port_locker:
          self.dxl.SetPWM(max_pwm)
          self.dxl.MoveTo(cmd, blocking=False)
        #print 'b..'
      else:
        with self.port_locker:
          self.dxl.SetPWM(max_pwm)
        #self.holding.SetTarget(cmd, self.holding_max_pwm_rate*max_pwm)

      #print 'dxl_gripper:MoveThController:rate.remaining:',rate.remaining()
      rate.sleep()
    self.threads['MoveThController'][0]= False


  #Start holding controller with control rate (Hz).
  #Use state observer (StartStateObs).
  def StartHolding(self, rate=30):
    self.StopHolding()

    def holding_observer():
      pos= self.gripper_pos2cmd(self.State()['position'])
      vel= self.gripper_vel2cmd(self.State()['velocity'])
      pwm= self.dxl.InvConvPWM(self.State()['effort'])
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
