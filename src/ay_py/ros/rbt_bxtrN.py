#! /usr/bin/env python
#Robot controller for BaxterN.
from const import *
if ROS_ROBOT not in ('ANY','BaxterN'):
  raise ImportError('Stop importing: ROS_ROBOT is not BaxterN')
#if ROS_DISTRO not in ('groovy','hydro','indigo'):  return

from rbt_bxtr import *

'''Robot control class for Baxter.'''
class TRobotBaxterN(TRobotBaxter):
  def __init__(self, name='BaxterN'):
    super(TRobotBaxterN,self).__init__(name=name)

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    self.default_face= 'config/baymax.jpg'
    self.ChangeFace(self.default_face)

    self.limbs= [None,None]
    self.limbs[RIGHT]= baxter_interface.Limb(LRTostr(RIGHT))
    self.limbs[LEFT]=  baxter_interface.Limb(LRTostr(LEFT))

    self.kin= [None,None]
    self.kin[RIGHT]= baxter_kinematics(LRTostr(RIGHT))
    self.kin[LEFT]=  baxter_kinematics(LRTostr(LEFT))  #tip_link=_gripper(default),_wrist,_hand

    self.head= baxter_interface.Head()

    ra(self.AddActC('r_traj', '/robot/limb/right/follow_joint_trajectory',
                    control_msgs.msg.FollowJointTrajectoryAction, time_out=3.0))
    ra(self.AddActC('l_traj', '/robot/limb/left/follow_joint_trajectory',
                    control_msgs.msg.FollowJointTrajectoryAction, time_out=3.0))

    self.robotiq= [None,None]
    self.robotiq[RIGHT]= TRobotiqN('gripper_right')
    self.robotiq[LEFT]= TRobotiqN('gripper_left')
    self.grippers= [self.robotiq[RIGHT], self.robotiq[LEFT]]

    print 'Enabling the robot...'
    baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()

    print 'Initializing Robotiq grippers...'
    ra(self.robotiq[RIGHT].Init())
    ra(self.robotiq[LEFT].Init())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('Baxter','Baxter_SIM','BaxterN'):  return True
    return super(TRobotBaxterN,self).Is(q)

