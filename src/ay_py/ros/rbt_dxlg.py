#! /usr/bin/env python
#\brief   Robot controller for a 3D-printed Gripper with Dynamixel.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Nov.9, 2017
#\version 0.2
#\date    Feb.28, 2020
#         Completely modified the implementation: now we use the gripper driver ROS node.
from const import *

import threading

from robot import TGripper2F1,TMultiArmRobot
import sensor_msgs.msg
import ay_util_msgs.srv


'''3D-printed Dynamixel Gripper utility class'''
class TDxlGripper(TGripper2F1):
  def __init__(self, node_name='gripper_driver', gripper_type='DxlGripper', finger_type=None):
    super(TDxlGripper,self).__init__()

    #FIXME:gripper_type and finger_type can be obtained from a topic from node_name,
    #  which will reduce the configuration complexity without loss of generality.

    self.gripper_type= gripper_type
    self.node_name= node_name
    #We use an instance of low-level gripper controller without activating it (i.e. no device communication)
    #in order to get the parameters.
    if self.gripper_type=='DxlGripper':
      mod= __import__('ay_py.misc.dxl_gripper',globals(),None,('TDynamixelGripper',))
      self.gripper= mod.TDynamixelGripper(dev=None)
      self.joint_names= ['joint0']
    elif self.gripper_type=='RHP12RNGripper':
      mod= __import__('ay_py.misc.dxl_rhp12rn',globals(),None,('TRHP12RN',))
      self.gripper= mod.TRHP12RN(dev=None)
      self.joint_names= ['joint0']
    elif self.gripper_type=='EZGripper':
      mod= __import__('ay_py.misc.dxl_ezg',globals(),None,('TEZG',))
      self.gripper= mod.TEZG(dev=None)
      self.joint_names= ['joint0']
    elif self.gripper_type=='DxlpO2Gripper':
      mod= __import__('ay_py.misc.dxl_dxlpo2',globals(),None,('TDxlpO2',))
      self.gripper= mod.TDxlpO2(dev=None, finger_type=finger_type)
      self.joint_names= ['joint0']
    elif self.gripper_type=='DxlO3Gripper':
      mod= __import__('ay_py.misc.dxl_dxlo3',globals(),None,('TDxlO3',))
      self.gripper= mod.TDxlO3(dev=None)
      self.joint_names= ['joint0','joint1']
    elif self.gripper_type=='DxlpY1Gripper':
      mod= __import__('ay_py.misc.dxl_dxlpy1',globals(),None,('TDxlpY1',))
      self.gripper= mod.TDxlpY1(dev=None, finger_type=finger_type)
      self.joint_names= ['joint0']
    else:
      raise Exception('Invalid gripper type: {gripper_type}'.format(gripper_type=gripper_type))

    self.sensor_locker= threading.RLock()
    self.port_locker= threading.RLock()


  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    ra(self.AddSrvP('move', '/{0}/move'.format(self.node_name), ay_util_msgs.srv.DxlGMove, persistent=False, time_out=3.0))
    ra(self.AddSrvP('dxl_io', '/{0}/dxl_io'.format(self.node_name), ay_util_msgs.srv.DxlIO, persistent=False, time_out=3.0))

    ra(self.AddSub('joint_states', '/{0}/joint_states'.format(self.node_name), sensor_msgs.msg.JointState, self.JointStatesCallback))

    return self._is_initialized

  def Cleanup(self):
    super(TDxlGripper,self).Cleanup()

  '''Answer to a query q by {True,False}. e.g. Is('Robotiq').'''
  def Is(self, q):
    if q==self.gripper_type:  return True
    return super(TDxlGripper,self).Is(q)

  def JointStatesCallback(self, msg):
    with self.sensor_locker:
      self.x_curr= msg
      self.q_curr= self.x_curr.position
      self.dq_curr= self.x_curr.velocity
      self.effort_curr= self.x_curr.effort

  '''Get current position.'''
  def Position(self):
    with self.sensor_locker:
      q= self.q_curr
    return q[0] if len(self.joint_names)==1 else list(q)

  '''Get a fingertip height offset in meter.
    The fingertip trajectory of some grippers has a rounded shape.
    This function gives the offset from the highest (longest) point (= closed fingertip position),
    and the offset is always negative.
      pos: Gripper position to get the offset. '''
  def FingertipOffset(self, pos):
    #WARNING: NotImplemented
    return 0.0

  def PosRange(self):
    return self.gripper.PosRange()

  def Activate(self):
    req= ay_util_msgs.srv.DxlIORequest()
    req.joint_names= []
    req.command= 'EnableTorque'
    with self.port_locker:
      res= self.srvp.dxl_io(req)

  def Deactivate(self):
    req= ay_util_msgs.srv.DxlIORequest()
    req.joint_names= []
    req.command= 'DisableTorque'
    with self.port_locker:
      res= self.srvp.dxl_io(req)

  def Open(self, blocking=False):
    req= ay_util_msgs.srv.DxlGMoveRequest()
    req.command= 'Open'
    req.blocking= blocking
    with self.port_locker:
      res= self.srvp.move(req)

  def Close(self, blocking=False):
    req= ay_util_msgs.srv.DxlGMoveRequest()
    req.command= 'Close'
    req.blocking= blocking
    with self.port_locker:
      res= self.srvp.move(req)

  def Move(self, pos, max_effort=50.0, speed=50.0, blocking=False):
    req= ay_util_msgs.srv.DxlGMoveRequest()
    req.command= 'Move'
    req.pos= pos if isinstance(pos,list) else [pos]
    req.max_effort= max_effort if isinstance(max_effort,list) else [max_effort]
    req.speed= speed if isinstance(speed,list) else [speed]
    req.blocking= blocking
    with self.port_locker:
      res= self.srvp.move(req)

  def Stop(self, blocking=False):
    req= ay_util_msgs.srv.DxlGMoveRequest()
    req.command= 'Stop'
    req.blocking= blocking
    with self.port_locker:
      res= self.srvp.move(req)

  def StartHolding(self):
    req= ay_util_msgs.srv.DxlGMoveRequest()
    req.command= 'StartHolding'
    with self.port_locker:
      res= self.srvp.move(req)

  def StopHolding(self):
    req= ay_util_msgs.srv.DxlGMoveRequest()
    req.command= 'StopHolding'
    with self.port_locker:
      res= self.srvp.move(req)

  '''Dynamixel control through /gripper_driver/dxl_io'''

  #Read from Dynamixel address.
  #Return: {joint_name:value}
  def DxlRead(self, address, joint_names=[]):
    req= ay_util_msgs.srv.DxlIORequest()
    req.joint_names= joint_names
    req.command= 'Read'
    req.data_s= address
    with self.port_locker:
      res= self.srvp.dxl_io(req)
    if len(joint_names)==0:  joint_names= self.joint_names
    return {joit_name:value for joit_name,value in zip(joint_names, res.res_ia)}

  #Read to Dynamixel.  data: {joint_name:value} or [value0,value1,...]
  def DxlWrite(self, address, data):
    req= ay_util_msgs.srv.DxlIORequest()
    req.command= 'Write'
    req.data_s= address
    if isinstance(data,dict):
      req.joint_names= data.keys()
      req.data_ia= data.values()
    elif isinstance(data,list):
      req.joint_names= []
      req.data_ia= data
    else:
      req.joint_names= []
      req.data_ia= [data]
    with self.port_locker:
      res= self.srvp.dxl_io(req)


'''Robot control class for DxlGripper.
  This is defined as a subclass of TMultiArmRobot,
  but actually it does not have a body (only DxlGripper gripper).
  This virtual body is designed for a compatibility of programs.'''
class TRobotDxlGripper(TMultiArmRobot):
  def __init__(self, name='DxlGripper', gripper_node='gripper_driver'):
    super(TRobotDxlGripper,self).__init__(name=name)
    self.currarm= 0
    self.gripper_node= gripper_node

  def internal_init(self, dxl_gripper):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    self.dxl_gripper= dxl_gripper
    self.grippers= [self.dxl_gripper]

    print 'Initializing and activating {} gripper...'.format(self.Name)
    ra(self.dxl_gripper.Init())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    dxl_gripper= TDxlGripperdxl_gripper(node_name=self.gripper_node)
    return self.internal_init(dxl_gripper)

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    for gripper in self.grippers:  gripper.Cleanup()
    self._is_initialized= False
    super(TRobotDxlGripper,self).Cleanup()

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('DxlGripper',):  return True
    return super(TRobotDxlGripper,self).Is(q)

  @property
  def NumArms(self):
    return 1

  @property
  def BaseFrame(self):
    return 'world'

  '''Return range of gripper.
    arm: arm id, or None (==currarm). '''
  def GripperRange(self, arm=None):
    if arm is None:  arm= self.Arm
    gripper= self.grippers[arm]
    if gripper.Is('Gripper2F1'):  return gripper.PosRange()
    elif gripper.Is('Gripper2F2'):  return gripper.PosRange2F1()

  '''Return range of gripper.
    arm: arm id, or None (==currarm). '''
  def GripperRange2(self, arm=None):
    if arm is None:  arm= self.Arm
    return self.grippers[arm].PosRange()

  '''End effector of an arm.'''
  def EndEff(self, arm=None):
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
    with self.gripper_locker:
      gripper.Open(blocking=blocking)

  '''Close a gripper.
    arm: arm id, or None (==currarm).
    blocking: False: move background, True: wait until motion ends.  '''
  def CloseGripper(self, arm=None, blocking=False):
    if arm is None:  arm= self.Arm
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
    if arm is None:  arm= self.Arm

    gripper= self.grippers[arm]
    if gripper.Is('Gripper2F1'):
      with self.gripper_locker:
        gripper.Move(pos, max_effort, speed, blocking=blocking)
    elif gripper.Is('Gripper2F2'):
      with self.gripper_locker:
        gripper.Move2F1(pos, max_effort, speed, blocking=blocking)

  '''Low level interface to control a gripper.
    arm: arm id, or None (==currarm).
    pos: target positions.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum).
    blocking: False: move background, True: wait until motion ends.  '''
  def MoveGripper2(self, pos, max_effort=50.0, speed=50.0, arm=None, blocking=False):
    if arm is None:  arm= self.Arm
    gripper= self.grippers[arm]
    with self.gripper_locker:
      gripper.Move(pos, max_effort, speed, blocking=blocking)

  '''Get a gripper position in meter.
    arm: arm id, or None (==currarm). '''
  def GripperPos(self, arm=None):
    if arm is None:  arm= self.Arm

    gripper= self.grippers[arm]
    if gripper.Is('Gripper2F1'):
      with self.sensor_locker:
        pos= gripper.Position()
    elif gripper.Is('Gripper2F2'):
      with self.sensor_locker:
        pos= gripper.Position2F1()
    return pos

  '''Get gripper positions.
    arm: arm id, or None (==currarm). '''
  def GripperPos2(self, arm=None):
    if arm is None:  arm= self.Arm
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
    if arm is None:  arm= self.Arm
    if pos is None:  pos= self.GripperPos(arm)
    gripper= self.grippers[arm]
    if gripper.Is('Gripper2F1'):  return gripper.FingertipOffset(pos)
    elif gripper.Is('Gripper2F2'):  return gripper.FingertipOffset2F1(pos)

  '''Get a fingertip height offset in meter.
    The fingertip trajectory of some grippers has a rounded shape.
    This function gives the offset from the highest (longest) point (= closed fingertip position),
    and the offset is always negative.
    NOTE: In the previous versions (before 2019-12-10), this offset was from the opened fingertip position.
      pos: Gripper position to get the offset. None: Current position.
      arm: arm id, or None (==currarm).'''
  def FingertipOffset2(self, pos=None, arm=None):
    if arm is None:  arm= self.Arm
    if pos is None:  pos= self.GripperPos2(arm)
    return self.grippers[arm].FingertipOffset(pos)

