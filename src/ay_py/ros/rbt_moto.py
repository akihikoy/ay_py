#! /usr/bin/env python
#Robot controller for Motoman.
from const import *
if ROS_ROBOT not in ('ANY','Motoman','Motoman_SIM'):
  raise ImportError('Stop importing: ROS_ROBOT is not Motoman')
#if ROS_DISTRO not in ('groovy','hydro','indigo'):  return

import roslib
import rospy
import sensor_msgs.msg
import trajectory_msgs.msg
import copy

from robot import *
from rbt_rq import TRobotiq
from kdl_kin import *

'''Robot control class for single Motoman SIA10F with a Robotiq gripper.'''
class TRobotMotoman(TMultiArmRobot):
  def __init__(self, name='Motoman'):
    super(TRobotMotoman,self).__init__(name=name)
    self.is_sim= (ROS_ROBOT=='Motoman_SIM')

    #Gripper command-position conversions.
    #rqg: Robotiq gripper.
    self.rqg_cmd2pos= lambda cmd: -0.00041*cmd+0.09249   #effective cmd in [12,230] ([0,255])
    self.rqg_pos2cmd= lambda pos: -(pos-0.09249)/0.00041 #pos in [0.0,0.0855] meter
    self.rqg_range= [0.0,0.0855]

    self.joint_names= [[]]
    self.joint_names[0]= rospy.get_param('controller_joint_names')
    #self.joint_names[0]= ['joint_'+jkey for jkey in ('s','l','e','u','r','b','t')]

    #Motoman all link names:
    #obtained from ay_py/demo_ros/kdl1.py (URDF link names)
    self.links= {}
    self.links['base']= ['base_link']
    self.links['r_arm']= ['link_s', 'link_l', 'link_e', 'link_u', 'link_r', 'link_b', 'link_t']
    self.links['robot']= self.links['base'] + self.links['r_arm']

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    self.kin= [None]
    self.kin[0]= TKinematics(base_link='base_link',end_link='link_t')

    ra(self.AddPub('joint_path_command', '/joint_path_command', trajectory_msgs.msg.JointTrajectory))

    #if self.is_sim:
      #ra(self.AddPub('joint_states', '/joint_states', sensor_msgs.msg.JointState))

    ra(self.AddSub('joint_states', '/joint_states', sensor_msgs.msg.JointState, self.JointStatesCallback))

    self.robotiq= TRobotiq()  #Robotiq controller
    self.grippers= [self.robotiq]

    print 'Enabling the robot...'

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    super(TRobotMotoman,self).Cleanup()

  '''Configure a state validity checker.'''
  def ConfigureSVC(self, c):
    c.JointNames= copy.deepcopy(self.joint_names)
    c.Links= copy.deepcopy(self.links)
    c.PaddingLinks= []
    c.PaddingValues= [0.002]*len(c.PaddingLinks)
    c.DefaultBaseFrame= 'base_link'
    c.HandLinkToGrasp[0]= 'link_t'
    c.IgnoredLinksInGrasp[0]= []

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('Motoman','Motoman_SIM'):  return True
    return super(TRobotMotoman,self).Is(q)

  @property
  def NumArms(self):
    return 1

  @property
  def BaseFrame(self):
    return 'base_link'

  '''End link of an arm.'''
  def EndLink(self, arm):
    return 'link_t'

  '''Names of joints of an arm.'''
  def JointNames(self, arm):
    return self.joint_names[arm]

  '''Return limits (lower, upper) of joint angles.
    arm: arm id, or None (==currarm). '''
  def JointLimits(self, arm=None):
    arm= 0
    return self.kin[arm].joint_limits_lower, self.kin[arm].joint_limits_upper

  '''Return limits of joint angular velocity.
    arm: arm id, or None (==currarm). '''
  def JointVelLimits(self, arm=None):
    #['s','l','e','u','r','b','t']
    #FIXME: Should be adjusted for Motoman
    return [0.5, 0.5, 0.8, 0.8, 0.8, 0.8, 0.8]

  '''Return range of gripper.
    arm: arm id, or None (==currarm). '''
  def GripperRange(self, arm=None):
    arm= 0
    gripper= self.grippers[arm]
    if gripper.Is('Robotiq'):  return self.rqg_range

  '''End effector of an arm.'''
  def EndEff(self, arm):
    arm= 0
    return self.grippers[arm]

  def JointStatesCallback(self, msg):
    with self.sensor_locker:
      self.x_curr= msg
      self.q_curr= self.x_curr.position
      self.dq_curr= self.x_curr.velocity

  '''Return joint angles of an arm.
    arm: arm id, or None (==currarm). '''
  def Q(self, arm=None):
    with self.sensor_locker:
      q= self.q_curr
    return q

  '''Return joint velocities of an arm.
    arm: arm id, or None (==currarm). '''
  def DQ(self, arm=None):
    with self.sensor_locker:
      dq= self.dq_curr
    return dq

  '''Compute a forward kinematics of an arm.
  Return self.EndLink(arm) pose on self.BaseFrame.
    return: x, res;  x: pose (None if failure), res: FK status.
    arm: arm id, or None (==currarm).
    q: list of joint angles, or None (==self.Q(arm)).
    x_ext: a local pose on self.EndLink(arm) frame.
      If not None, the returned pose is x_ext on self.BaseFrame.
    with_st: whether return FK status. '''
  def FK(self, q=None, x_ext=None, arm=None, with_st=False):
    arm= 0
    if q is None:  q= self.Q(arm)

    angles= {joint:q[j] for j,joint in enumerate(self.joint_names[arm])}  #Deserialize
    with self.sensor_locker:
      x= self.kin[arm].forward_position_kinematics(joint_values=angles)

    x_res= x if x_ext is None else Transform(x,x_ext)
    return (x_res, True) if with_st else x_res

  '''Compute a Jacobian matrix of an arm.
  Return J of self.EndLink(arm).
    return: J, res;  J: Jacobian (None if failure), res: status.
    arm: arm id, or None (==currarm).
    q: list of joint angles, or None (==self.Q(arm)).
    x_ext: a local pose (i.e. offset) on self.EndLink(arm) frame.
      If not None, we do not consider an offset.
    with_st: whether return the solver status. '''
  def J(self, q=None, x_ext=None, arm=None, with_st=False):
    arm= 0
    if q is None:  q= self.Q(arm)

    if x_ext is not None:
      #Since KDL does not provide Jacobian computation with an offset x_ext,
      #and converting J with x_ext is not simple, we raise an Exception.
      #TODO: Implement our own FK to solve this issue.
      raise Exception('TRobotMotoman.J: Jacobian with x_ext is not implemented yet.')

    angles= {joint:q[j] for j,joint in enumerate(self.joint_names[arm])}  #Deserialize
    with self.sensor_locker:
      J_res= self.kin[arm].jacobian(joint_values=angles)
    return (J_res, True) if with_st else J_res

  '''Compute an inverse kinematics of an arm.
  Return joint angles for a target self.EndLink(arm) pose on self.BaseFrame.
    return: q, res;  q: joint angles (None if failure), res: IK status.
    arm: arm id, or None (==currarm).
    x_trg: target pose.
    x_ext: a local pose on self.EndLink(arm) frame.
      If not None, the returned q satisfies self.FK(q,x_ext,arm)==x_trg.
    start_angles: initial joint angles for IK solver, or None (==self.Q(arm)).
    with_st: whether return IK status. '''
  def IK(self, x_trg, x_ext=None, start_angles=None, arm=None, with_st=False):
    arm= 0
    if start_angles is None:  start_angles= self.Q(arm)

    x_trg[3:]/= la.norm(x_trg[3:])  #Normalize the orientation:
    xw_trg= x_trg if x_ext is None else TransformRightInv(x_trg,x_ext)

    with self.sensor_locker:
      q= self.kin[arm].inverse_kinematics(xw_trg[:3], xw_trg[3:], seed=start_angles, maxiter=1000, eps=1.0e-6)

    if q is not None:  return (q, True) if with_st else q
    else:  return (None, False) if with_st else None


  '''Follow a joint angle trajectory.
    arm: arm id, or None (==currarm).
    q_traj: joint angle trajectory [q0,...,qD]*N.
    t_traj: corresponding times in seconds from start [t1,t2,...,tN].
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN. '''
  def FollowQTraj(self, q_traj, t_traj, arm=None, blocking=False):
    assert(len(q_traj)==len(t_traj))
    arm= 0

    #Insert current position to beginning.
    if t_traj[0]>1.0e-2:
      t_traj.insert(0,0.0)
      q_traj.insert(0,self.Q(arm=arm))

    dq_traj= QTrajToDQTraj(q_traj, t_traj)
    traj= ToROSTrajectory(self.JointNames(arm), q_traj, t_traj, dq_traj)

    with self.control_locker:
      self.pub.joint_path_command.publish(traj)

      if blocking != False:
        rospy.sleep(t_traj[-1])  #Just sleep.


  '''Open a gripper.
    arm: arm id, or None (==currarm).
    blocking: False: move background, True: wait until motion ends.  '''
  def OpenGripper(self, arm=None, blocking=False):
    self.MoveGripper(pos=0.1, arm=arm, blocking=blocking)

  '''Close a gripper.
    arm: arm id, or None (==currarm).
    blocking: False: move background, True: wait until motion ends.  '''
  def CloseGripper(self, arm=None, blocking=False):
    self.MoveGripper(pos=0.0, arm=arm, blocking=blocking)

  '''High level interface to control a gripper.
    arm: arm id, or None (==currarm).
    pos: target position in meter.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum).
    blocking: False: move background, True: wait until motion ends.  '''
  def MoveGripper(self, pos, max_effort=50.0, speed=50.0, arm=None, blocking=False):
    if self.is_sim:  return  #WARNING:We do nothing if the robot is on simulator.
    arm= 0

    gripper= self.grippers[arm]
    if gripper.Is('Robotiq'):
      clip= lambda c: max(0.0,min(255.0,c))
      cmd= clip(self.rqg_pos2cmd(pos))
      max_effort= clip(max_effort*(255.0/100.0))
      speed= clip(speed*(255.0/100.0))
      with self.control_locker:
        gripper.Move(cmd, max_effort, speed, blocking=blocking)

  '''Get a gripper position in meter.
    arm: arm id, or None (==currarm). '''
  def GripperPos(self, arm=None):
    if self.is_sim:  return 0.0  #WARNING:We do nothing if the robot is on simulator.
    arm= 0

    gripper= self.grippers[arm]
    if gripper.Is('Robotiq'):
      with self.sensor_locker:
        pos= self.rqg_cmd2pos(gripper.Position())
      return pos

  '''Get fingertip offset in meter.
    The fingertip trajectory of Robotiq gripper has a round shape.
    This function gives the offset from the opening posture.
      pos: Gripper position to get the offset. None: Current position.
      arm: arm id, or None (==currarm).'''
  def FingertipOffset(self, pos=None, arm=None):
    arm= 0
    gripper= self.grippers[arm]
    if gripper.Is('Robotiq'):
      if pos is None:  pos= self.GripperPos(arm)
      return -0.701*pos**3 - 2.229*pos**2 + 0.03*pos + 0.128 - 0.113

