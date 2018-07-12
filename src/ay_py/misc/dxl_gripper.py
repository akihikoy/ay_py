#!/usr/bin/python
#\file    dxl_gripper.py
#\brief   Control module of Dynamixel Gripper (3D-printed Gripper with Dynamixel).
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Oct.29, 2017
from dxl_util import TDynamixel1
from ..misc.dxl_holding import TDxlHolding
from ..core.util import TRate
import time
import threading
import copy

'''3D-printed Dynamixel Gripper utility class'''
class TDynamixelGripper(object):
  def __init__(self, dev='/dev/ttyUSB0'):
    self.dxl_type= 'XM430-W350'
    self.dev= dev

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
    self.dxlg_cmd2vel= lambda cmd: (self.dxlg_range[0]-self.dxlg_range[1])/(TDynamixel1.ConvPos(self.CmdMax)-TDynamixel1.ConvPos(self.CmdOpen))*TDynamixel1.ConvVel(cmd)
    self.dxlg_vel2cmd= lambda vel: TDynamixel1.InvConvVel(vel*(TDynamixel1.ConvPos(self.CmdMax)-TDynamixel1.ConvPos(self.CmdOpen))/(self.dxlg_range[0]-self.dxlg_range[1]))

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
    #self.StopMoveTh()
    #th_func= lambda:self.MoveThController(pos, max_effort, speed)
    #self.threads['MoveThController']= [True, threading.Thread(name='MoveThController', target=th_func)]
    #self.threads['MoveThController'][1].start()
    #if blocking:
      #self.threads['MoveThController'][1].join()
      #self.StopMoveTh()

    cmd= max(self.CmdMin,min(self.CmdMax,int(self.dxlg_pos2cmd(pos))))
    max_pwm= self.dxl.InvConvPWM(max_effort)
    if self.holding is None:
      with self.moveth_locker:
        self.moveth_cmd= {'pos':pos,'max_effort':max_effort}
      rate= TRate(self.hz_moveth_ctrl)
      while blocking:
        p= self.State()['position']
        if p is None:  return
        if not (abs(cmd - self.dxlg_pos2cmd(p)) > self.dxl.GoalThreshold):  break
        #print abs(cmd - self.dxlg_pos2cmd(p)) - self.dxl.GoalThreshold
        rate.sleep()
    else:
      with self.moveth_locker:
        self.moveth_cmd= {'pos':pos,'max_effort':max_effort}
      self.holding.SetTarget(cmd, self.holding_max_pwm_rate*max_pwm)


  #Start MoveTh controller.
  def StartMoveTh(self):
    self.StopMoveTh()
    with self.port_locker:
      goal_pos= self.dxlg_cmd2pos(self.dxl.Read('GOAL_POSITION'))
      max_effort= self.dxl.ConvPWM(self.dxl.Read('GOAL_PWM'))
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
          'position':self.dxlg_cmd2pos(self.dxl.Position()),
          'velocity':self.dxlg_cmd2vel(self.dxl.Velocity()),
          'effort':self.dxl.ConvPWM(self.dxl.PWM()),  #FIXME: PWM vs. Current
          }
        #print state['position']
      with self.state_locker:
        self.state= state
      if callback is not None:
        callback(state)
      rate.sleep()
    self.threads['StateObserver'][0]= False

  ##MoveTh controller thread.
  ##NOTE: Don't call this function directly.  Use self.MoveTh
  #def MoveThController(self, pos, max_effort, speed):
    #cmd= max(self.CmdMin,min(self.CmdMax,int(self.dxlg_pos2cmd(pos))))
    #max_pwm= self.dxl.InvConvPWM(max_effort)

    ##print 'entry..'
    #if self.holding is None:
      ##print 'a..'
      #with self.port_locker:
        #self.dxl.SetPWM(max_pwm)
        #self.dxl.MoveTo(cmd, blocking=False)
      ##print 'b..'
      #rate= TRate(self.hz_moveth_ctrl)
      #while self.threads['MoveThController'][0]:
        #p= self.State()['position']
        #if p is None:  return
        #if not (abs(cmd - self.dxlg_pos2cmd(p)) > self.dxl.GoalThreshold):  break
        ##print abs(cmd - self.dxlg_pos2cmd(p)) - self.dxl.GoalThreshold
        #rate.sleep()
      #self.threads['MoveThController'][0]= False
    #else:
      #with self.port_locker:
        #self.dxl.SetPWM(max_pwm)
      #self.holding.SetTarget(cmd, self.holding_max_pwm_rate*max_pwm)

  #MoveTh controller thread.
  #NOTE: Don't call this function directly.  Use self.StartMoveTh
  def MoveThController(self):
    rate= TRate(self.hz_moveth_ctrl)
    while self.threads['MoveThController'][0]:
      with self.moveth_locker:
        moveth_cmd= copy.deepcopy(self.moveth_cmd)
      pos,max_effort= moveth_cmd['pos'],moveth_cmd['max_effort']
      cmd= max(self.CmdMin,min(self.CmdMax,int(self.dxlg_pos2cmd(pos))))
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


  #OBSOLETE: Old implementation holding controller.
  ##Start holding controller with control rate (Hz).
  #def StartHolding(self, rate=30):
    #self.StopHolding()

    #def holding_observer():
      #with self.port_locker:
        #pos,vel,pwm= self.dxl.Position(), self.dxl.Velocity(), self.dxl.PWM()
      #return pos,vel,pwm
    #def holding_controller(target_position):
      #with self.port_locker:
        #self.dxl.MoveTo(target_position, blocking=False)

    #with self.port_locker:
      #goal_pos= self.dxl.Read('GOAL_POSITION')
      #max_pwm= self.dxl.Read('GOAL_PWM')

    #self.holding= TDxlHolding(rate)
    #self.holding.observer= holding_observer
    #self.holding.controller= holding_controller
    #self.holding.SetTarget(goal_pos, self.holding_max_pwm_rate*max_pwm)
    #self.holding.Start()

  #Start holding controller with control rate (Hz).
  #Use state observer (StartStateObs).
  def StartHolding(self, rate=30):
    self.StopHolding()

    def holding_observer():
      pos= self.dxlg_pos2cmd(self.State()['position'])
      vel= self.dxlg_vel2cmd(self.State()['velocity'])
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
