#! /usr/bin/env python
#Robot controller for Universal Robots UR* with Dynamixel Gripper.
from const import *

import roslib
import rospy

from rbt_ur import *
from rbt_dxlg import TDxlGripper

'''Robot control class for single Universal Robots UR* with Dynamixel Gripper.'''
class TRobotURDxlG(TRobotUR):
  def __init__(self, name='UR', ur_series='CB', robot_ip=None, is_sim=False, gripper_node='gripper_driver'):
    super(TRobotURDxlG,self).__init__(name=name,ur_series=ur_series,robot_ip=robot_ip,is_sim=is_sim)
    self.gripper_node= gripper_node

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    ra(super(TRobotURDxlG,self).Init())

    self.dxl_gripper= TDxlGripper(node_name=self.gripper_node)
    #self.dxl_gripper= TSimGripper2F1(pos_range=[0.0,0.095])
    self.grippers= [self.dxl_gripper]

    print 'Initializing and activating DxlGripper gripper...'
    ra(self.dxl_gripper.Init())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('URDxlG',):  return True
    return super(TRobotURDxlG,self).Is(q)
