#!/usr/bin/python
#\file    rbt_geh6000il.py
#\brief   Gripper control interface for GEH6000IL.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Nov.19, 2023
from const import *

import roslib
import rospy
import threading

from robot import TGripper2F1,TMultiArmRobot
import sensor_msgs.msg
import ay_util_msgs.srv


'''Gripper control interface for GEH6000IL ROS driver'''
class TGEH6000ILGripper(TGripper2F1):
  def __init__(self, node_name='gripper_driver'):
    super(TGEH6000ILGripper,self).__init__()

    self.node_name= node_name
    self.gripper_type= None
    self.pos_range= None
    self.joint_names= None

    self.sensor_locker= threading.RLock()
    self.port_locker= threading.RLock()

    self.x_curr= None
    self.q_curr= None
    self.dq_curr= None
    self.effort_curr= None

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    for key in ('gripper_type','pos_range','joint_names',):
      value= rospy.get_param('/{}/{}'.format(self.node_name,key),None)
      setattr(self, key, value)
      ra(value is not None)

    ra(self.AddSrvP('move', '/{}/move'.format(self.node_name), ay_util_msgs.srv.DxlGMove, persistent=False, time_out=3.0))

    ra(self.AddSub('joint_states', '/{}/joint_states'.format(self.node_name), sensor_msgs.msg.JointState, self.JointStatesCallback))

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    super(TGEH6000ILGripper,self).Cleanup()

  '''Answer to a query q by {True,False}. e.g. Is('Robotiq').'''
  def Is(self, q):
    if q in ('GEH6000IL',self.gripper_type):  return True
    return super(TGEH6000ILGripper,self).Is(q)

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
    return None if q is None else q[0] if len(self.joint_names)==1 else list(q)

  '''Get a fingertip height offset in meter.
    The fingertip trajectory of some grippers has a rounded shape.
    This function gives the offset from the highest (longest) point (= closed fingertip position),
    and the offset is always negative.
      pos: Gripper position to get the offset. '''
  def FingertipOffset(self, pos):
    return 0.0

  def PosRange(self):
    return self.pos_range

  def Activate(self):
    req= ay_util_msgs.srv.DxlGMoveRequest()
    req.command= 'Activate'
    with self.port_locker:
      res= self.srvp.move(req)

  def Deactivate(self):
    req= ay_util_msgs.srv.DxlGMoveRequest()
    req.command= 'Deactivate'
    with self.port_locker:
      res= self.srvp.move(req)

  def Homing(self):
    req= ay_util_msgs.srv.DxlGMoveRequest()
    req.command= 'Homing'
    with self.port_locker:
      res= self.srvp.move(req)

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


