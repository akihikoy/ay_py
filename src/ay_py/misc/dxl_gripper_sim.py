#!/usr/bin/python
#\file    dxl_gripper_sim.py
#\brief   Dynamixel gripper simulation.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Aug.18, 2023
import __future__
import threading
import numpy as np
import time
import copy
from ..core.util import TRate, CPrint

'''Multiple DoF simple gripper simulation model.'''
class TGripperSim(object):
  def __init__(self, dof=1):
    self.dof= dof
    self.is_initialized= False
    self.curr_pos= [0.0 for n in range(dof)]
    self.curr_vel= [0.0 for n in range(dof)]
    self.target_pos= self.curr_pos
    self.pos_max= [100.0]*dof
    self.pos_min= [-100.0]*dof
    self.is_active= False
    self.sim_rate= 100.0
    self.max_vel= 0.5
    self.state_locker= threading.RLock()
    self.th= None

  def Init(self):
    self.EnableTorque()
    self.is_initialized= True
    return True

  def Quit(self):
    self.is_active= False
    if self.th is not None:
      self.th.join()
      self.th= None
    self.is_initialized= False

  def EnableTorque(self):
    if self.is_active:  return True
    self.is_active= True
    self.th= threading.Thread(name='loop', target=self.Loop)
    self.th.start()
    return True

  def DisableTorque(self):
    self.is_active= False
    if self.th is not None:
      self.th.join()
      self.th= None
    self.curr_vel= [0.0 for n in range(self.dof)]
    return True

  def Position(self):
    with self.state_locker:
      return self.curr_pos

  def Velocity(self):
    with self.state_locker:
      return self.curr_vel

  def TargetPos(self):
    with self.state_locker:
      return self.target_pos

  def MoveTo(self, target_pos, blocking=False):
    with self.state_locker:
      assert(len(self.target_pos)==self.dof)
      self.target_pos= target_pos
    if blocking:
      self.WaitToReach()

  def WaitToReach(self, rate=None):
    rate_adjuster= TRate(rate if rate is not None else self.sim_rate/5.0)
    while self.is_active:
      with self.state_locker:
        diff= [abs(target_pos-curr_pos) for curr_pos,target_pos in zip(self.curr_pos,self.target_pos)]
      if max(diff)<1.0e-8:  break
      rate_adjuster.sleep()

  def Loop(self):
    rate= self.sim_rate
    step= self.max_vel/rate
    rate_adjuster= TRate(rate)
    #get updated curr_pos, curr_vel per dof.
    def update_pos(curr_pos, target_pos):
      if abs(curr_pos-target_pos)<=1.0e-8:   return target_pos
      elif abs(curr_pos-target_pos)<=step:   return target_pos
      elif curr_pos<target_pos:              return curr_pos+step
      elif curr_pos>target_pos:              return curr_pos-step
    while self.is_active:
      with self.state_locker:
        curr_pos= np.clip([update_pos(curr_pos,target_pos)
                           for curr_pos,target_pos in zip(self.curr_pos,self.target_pos)],
                          self.pos_min, self.pos_max)
        curr_vel= (curr_pos-self.curr_pos)*rate
        self.curr_pos= curr_pos.tolist()
        self.curr_vel= curr_vel.tolist()
      rate_adjuster.sleep()


'''
Dynamixel Gripper simulation model.
  dxlg_ref: Reference to a gripper driver object
      such as TDynamixelGripper(ay_py.misc.dxl_gripper), TRHP12RN(ay_py.misc.dxl_rhp12rn).
      dxlg_ref does not have to run dxlg_ref.Init()
'''
class TDxlGripperSim(object):
  def __init__(self, dxlg_ref):
    self.dxlg_ref= dxlg_ref

    #Thread locker:
    self.sim_locker= threading.RLock()
    self.state_locker= threading.RLock()
    self.moveth_locker= threading.RLock()
    self.state= {'stamp':0.0, 'position':None, 'velocity':None, 'effort':None}
    self.moveth_cmd= {'pos':None,'max_effort':None}
    self.hz_state_obs= self.dxlg_ref.hz_state_obs  #State observation rate (Hz).
    self.hz_moveth_ctrl= self.dxlg_ref.hz_moveth_ctrl  #MoveTh control rate (Hz).
    self.threads= {  #ThreadName:[IsActive,ThreadObject]
      'StateObserver':[False,None],
      'MoveThController':[False,None],}

    for key in ('CmdMax','CmdMin','CmdOpen','CmdClose',
                'GrpClose','GrpOpen','GrpMin','GrpMax',
                'pos_open','pos_close',
                'gripper_cmd2pos','gripper_pos2cmd','gripper_cmd2vel','gripper_vel2cmd',
                'PosRange'):
      setattr(self,key, getattr(self.dxlg_ref,key,None))

    #Simulation model:
    tolist= lambda x: x if isinstance(x,list) else [x]
    self.sim_gripper= TGripperSim(dof=len(tolist(self.PosRange()[0])))
    self.sim_gripper.pos_min= tolist(self.PosRange()[0])
    self.sim_gripper.pos_max= tolist(self.PosRange()[1])
    self.sim_gripper.curr_pos= (0.5*(np.array(self.sim_gripper.pos_min)+self.sim_gripper.pos_max)).tolist()
    self.sim_gripper.target_pos= self.sim_gripper.curr_pos
    self.sim_gripper.sim_rate= max(100.0, self.hz_state_obs, self.hz_moveth_ctrl)

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= self.sim_gripper.Init()
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    if self._is_initialized:
      self.StopMoveTh()
      self.StopStateObs()
      self.Deactivate()
      self.sim_gripper.Quit()
      self._is_initialized= False

  '''Get current position.'''
  def Position(self):
    with self.sim_locker:
      pos= self.sim_gripper.Position()
    if pos is None:
      print('DxlG: Failed to read position')
      return None
    return pos[0] if len(pos)==1 else pos

  '''Activate gripper (torque is enabled).
    Return success or not.'''
  def Activate(self):
    with self.sim_locker:
      res= self.sim_gripper.EnableTorque()
    return res

  '''Deactivate gripper (torque is disabled).
    Return success or not.'''
  def Deactivate(self):
    with self.sim_locker:
      res= self.sim_gripper.DisableTorque()
    return res

  '''Open a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Open(self, blocking=False):
    pos= self.GrpOpen if self.GrpOpen is not None else self.pos_open
    if not self.threads['MoveThController'][0]:
      self.Move(pos=pos, blocking=blocking)
    else:
      self.MoveTh(pos=pos, blocking=blocking)

  '''Close a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Close(self, blocking=False):
    pos= self.GrpClose if self.GrpClose is not None else self.pos_close
    if not self.threads['MoveThController'][0]:
      self.Move(pos=pos, blocking=blocking)
    else:
      self.MoveTh(pos=pos, blocking=blocking)

  '''Control a gripper.
    pos: target position in meter.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum); NOT_IMPLEMENTED.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Move(self, pos, max_effort=50.0, speed=50.0, blocking=False):
    with self.sim_locker:
      self.sim_gripper.MoveTo(pos, blocking=blocking)

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
    with self.moveth_locker:
      self.moveth_cmd= {'pos':pos,'max_effort':max_effort}
    if blocking:
      self.sim_gripper.WaitToReach()

  #Start MoveTh controller.
  def StartMoveTh(self):
    self.StopMoveTh()
    with self.sim_locker:
      goal_pos= self.sim_gripper.TargetPos()
      max_effort= 100.0
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
      with self.sim_locker:
        p,v,c= self.sim_gripper.Position(),self.sim_gripper.Velocity(),np.abs(self.sim_gripper.Velocity()).tolist()
      state= {
        'stamp':time.time(),
        'position':p[0] if len(p)==1 else p,
        'velocity':v[0] if len(v)==1 else v,
        'effort':c[0] if len(c)==1 else c,
        }
      #print(state['position'])
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
      with self.sim_locker:
        self.sim_gripper.MoveTo(pos if isinstance(pos,list) else [pos], blocking=False)
      #print('#DEBUG:{}'.format(pos))
      #print('--SIM:Trg={} Pos={}, Vel={}'.format(self.sim_gripper.TargetPos(), self.sim_gripper.Position(), self.sim_gripper.Velocity()))
      rate.sleep()
    self.threads['MoveThController'][0]= False


'''
Test this module.
ay_py/src$ python
>>> from ay_py.misc.dxl_gripper_sim import *
>>> TEST_dxl_gripper_sim()
'''
def TEST_dxl_gripper_sim():
  #Test of TGripperSim
  print('Test of TGripperSim')
  gripper1= TGripperSim()
  gripper1.pos_min, gripper1.pos_max= [0.0], [0.15]
  gripper1.Init()
  t_start= time.time()
  rate_adjuster= TRate(20)
  try:
    while True:
      t_curr= time.time()-t_start
      if t_curr<1.0:          gripper1.MoveTo([0.1])
      elif 1.0<=t_curr<2.0:   gripper1.MoveTo([0.05])
      print('Trg={} Pos={}, Vel={}'.format(gripper1.TargetPos(), gripper1.Position(), gripper1.Velocity()))
      rate_adjuster.sleep()
  except KeyboardInterrupt:
    print('Interrupted')
  finally:
    gripper1.Quit()

  #Test of TGripperSim(2)
  print('Test of TGripperSim')
  gripper2= TGripperSim(dof=2)
  gripper2.pos_min, gripper2.pos_max= [0.0]*2, [0.15]*2
  gripper2.Init()
  t_start= time.time()
  rate_adjuster= TRate(20)
  try:
    while True:
      t_curr= time.time()-t_start
      if t_curr<1.0:          gripper2.MoveTo([0.05,0.2])
      elif 1.0<=t_curr<2.0:   gripper2.MoveTo([0.1,0.0])
      print('Trg={} Pos={}, Vel={}'.format(gripper2.TargetPos(), gripper2.Position(), gripper2.Velocity()))
      rate_adjuster.sleep()
  except KeyboardInterrupt:
    print('Interrupted')
  finally:
    gripper2.Quit()

  #Test of TDxlGripperSim
  print('Test of TGripperSim as an RHP12RNAGripper')
  mod= __import__('ay_py.misc.dxl_rhp12rn',globals(),None,('TRHP12RN',))
  dxlg_ref= mod.TRHP12RN(dev=None,type='(A)')
  def JointStatesCallback(state):
    print state
  gripper3= TDxlGripperSim(dxlg_ref)
  gripper3.Init()
  gripper3.StartStateObs(JointStatesCallback)
  gripper3.StartMoveTh()
  t_start= time.time()
  rate_adjuster= TRate(20)
  try:
    while True:
      t_curr= time.time()-t_start
      if t_curr<1.0:          gripper3.Open(blocking=False)
      elif 1.0<=t_curr<2.0:   gripper3.Close(blocking=True)
      elif 2.0<=t_curr<3.0:   gripper3.MoveTh(0.05)
      print('{:.4f}: Trg={:.4f}, Pos={:.4f}, Vel={:.4f}'.format(t_curr, gripper3.moveth_cmd['pos'], gripper3.State()['position'], gripper3.State()['velocity']))
      rate_adjuster.sleep()
  except KeyboardInterrupt:
    print('Interrupted')
  finally:
    gripper3.StopMoveTh()
    gripper3.StopStateObs()
    gripper3.Cleanup()

