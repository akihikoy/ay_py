#! /usr/bin/env python
#Robot controller for Universal Robots UR* with DxlpO2 gripper.
from const import *

import roslib
import rospy

from rbt_ur import *

'''Robot control class for single Universal Robots UR* with DxlpO2 gripper.'''
class TRobotURDxlpO2(TRobotUR):
  def __init__(self, name='UR', ur_series='CB', robot_ip=None, is_sim=False, gripper_node='gripper_driver', finger_type=None):
    super(TRobotURDxlpO2,self).__init__(name=name,ur_series=ur_series,robot_ip=robot_ip,is_sim=is_sim)
    self.gripper_node= gripper_node
    self.finger_type= finger_type

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    ra(super(TRobotURDxlpO2,self).Init())

    if not self.is_sim:
      #The gripper module is imported here to avoid importing it in simulation mode.
      mod= __import__('rbt_dxlpo2',globals(),None,('TDxlpO2Gripper',))
      self.dxlpo2_gripper= mod.TDxlpO2Gripper(node_name=self.gripper_node, finger_type=self.finger_type)
    else:
      if self.finger_type=='Straight1':
        self.gripper_range= [0.0,0.300]  #FIXME: 0.3 is inaccurate.
      elif self.finger_type=='SRound1':  #Small, round finger (yellow)
        self.gripper_range= [0.0,0.1950]
      elif self.finger_type=='Fork1':
        self.gripper_range= [0.0,0.200]
      elif self.finger_type=='???':
        self.gripper_range= [0.0,0.2200]
      else:
        raise Exception('TDxlpO2: Unknown finger_type: {finger_type}'.format(finger_type=self.finger_type))
      self.dxlpo2_gripper= TSimGripper2F1(pos_range=self.gripper_range)
    self.grippers= [self.dxlpo2_gripper]

    print 'Initializing and activating DxlGripper gripper...'
    ra(self.dxlpo2_gripper.Init())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('URDxlpO2',):  return True
    return super(TRobotURDxlpO2,self).Is(q)
