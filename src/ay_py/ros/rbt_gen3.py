#! /usr/bin/env python
#Robot controller for Kinova Gen3.
from const import *

import roslib
import rospy
import actionlib
import control_msgs.msg
import sensor_msgs.msg
import copy

from robot import *
from kdl_kin import *


'''Interpolate a trajectory consisting of q_traj and t_traj with an interval dt.
cf. https://github.com/Kinovarobotics/matlab_kortex/blob/master/simplified_api/documentation/precomputed_joint_trajectories.md#hard-limits-and-conditions-to-respect'''
def ROSInterpolateTraj(q_traj, t_traj, dt=1.0e-3):
  if t_traj[0]>1.0e-6:
    raise Exception('t_traj[0] must be zero and q_traj[0] must be the current joint angles.')
  assert(len(q_traj)==len(t_traj))

  #Modeling the trajectory with spline.
  DoF= len(q_traj[0])
  splines= [TCubicHermiteSpline() for d in range(DoF)]
  for d in range(DoF):
    data_d= [[t,q[d]] for q,t in zip(q_traj,t_traj)]
    splines[d].Initialize(data_d, tan_method=splines[d].CARDINAL, c=0.0, m=0.0)

  traj_points= []
  t= 0.0
  while t<t_traj[-1]:
    q,v,a= [],[],[]
    for spline in splines:
      q_d,v_d,a_d= spline.Evaluate(t,with_dd=True)
      q.append(q_d)
      v.append(v_d)
      a.append(a_d)
    point= trajectory_msgs.msg.JointTrajectoryPoint()
    point.positions= q
    point.velocities= v
    point.accelerations= a
    point.time_from_start= rospy.Duration(t)
    traj_points.append(point)
    t+= dt
  #JTP= trajectory_msgs.msg.JointTrajectoryPoint
  #traj_points= np.array([JTP(*(np.array([[spline.Evaluate(t,with_dd=True)] for spline in splines]).T.tolist()+[[],rospy.Duration(t)]))
                         #for t in np.arange(0.0,t_traj[-1],dt)])
  return traj_points


'''Robot control class for single Kinova Gen3 with an empty gripper.
   gen3ns: namespace of the robot.'''
class TRobotGen3(TMultiArmRobot):
  def __init__(self, name='Gen3', gen3ns='gen3a', is_sim=False):
    super(TRobotGen3,self).__init__(name=name)
    self.is_sim= is_sim
    self.ns= gen3ns

    self.joint_names= [[]]
    #self.joint_names[0]= rospy.get_param('controller_joint_names')
    self.joint_names[0]= ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']

    #UR all link names:
    #obtained from ay_py/demo_ros/kdl1.py (URDF link names)
    self.links= {}
    self.links['base']= ['base_link']
    self.links['r_arm']= ['shoulder_link', 'half_arm_1_link', 'half_arm_2_link', 'forearm_link', 'spherical_wrist_1_link', 'spherical_wrist_2_link', 'bracelet_link', 'end_effector_link']
    self.links['robot']= self.links['base'] + self.links['r_arm']

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    self.kin= [None]
    self.kin[0]= TKinematics(base_link='base_link',end_link='end_effector_link',description='/'+self.ns+'/robot_description')

    ra(self.AddActC('traj', '/'+self.ns+'/gen3_joint_trajectory_controller/follow_joint_trajectory',
                    control_msgs.msg.FollowJointTrajectoryAction, time_out=3.0))

    ra(self.AddSub('joint_states', '/'+self.ns+'/joint_states', sensor_msgs.msg.JointState, self.JointStatesCallback))

    self.grippers= [TFakeGripper()]

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    super(TRobotGen3,self).Cleanup()

  '''Configure a state validity checker.'''
  def ConfigureSVC(self, c):
    c.JointNames= copy.deepcopy(self.joint_names)
    c.Links= copy.deepcopy(self.links)
    c.PaddingLinks= []
    c.PaddingValues= [0.002]*len(c.PaddingLinks)
    c.DefaultBaseFrame= 'base_link'
    c.HandLinkToGrasp[0]= 'end_effector_link'
    c.IgnoredLinksInGrasp[0]= []

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('Gen3','Gen3_SIM'):  return True
    return super(TRobotGen3,self).Is(q)

  @property
  def NumArms(self):
    return 1

  @property
  def BaseFrame(self):
    return 'base_link'

  '''Get the namespace of the robot.'''
  def NS(self, arm=None):
    return self.ns

  '''End link of an arm.'''
  def EndLink(self, arm=None):
    return 'end_effector_link'

  '''Names of joints of an arm.'''
  def JointNames(self, arm=None):
    arm= 0
    return self.joint_names[arm]

  def DoF(self, arm=None):
    return 7

  '''Return limits (lower, upper) of joint angles.
    arm: arm id, or None (==currarm). '''
  def JointLimits(self, arm=None):
    arm= 0
    return self.kin[arm].joint_limits_lower, self.kin[arm].joint_limits_upper

  '''Return limits of joint angular velocity.
    arm: arm id, or None (==currarm). '''
  def JointVelLimits(self, arm=None):
    #FIXME: Should be adjusted for Gen3
    return [0.5, 0.5, 0.5, 0.8, 0.8, 0.8, 0.8, 0.8]

  '''Return range of gripper.
    arm: arm id, or None (==currarm). '''
  def GripperRange(self, arm=None):
    arm= 0
    return self.grippers[arm].PosRange()

  '''End effector of an arm.'''
  def EndEff(self, arm=None):
    arm= 0
    return self.grippers[arm]

  def JointStatesCallback(self, msg):
    with self.sensor_locker:
      self.x_curr= msg
      self.q_curr= self.x_curr.position
      self.dq_curr= self.x_curr.velocity

  '''Return joint angles of an arm (list of floats).
    arm: arm id, or None (==currarm). '''
  def Q(self, arm=None):
    with self.sensor_locker:
      q= self.q_curr
    return list(q)

  '''Return joint velocities of an arm (list of floats).
    arm: arm id, or None (==currarm). '''
  def DQ(self, arm=None):
    with self.sensor_locker:
      dq= self.dq_curr
    return list(dq)

  '''Compute a forward kinematics of an arm.
  Return self.EndLink(arm) pose on self.BaseFrame.
    return: x, res;  x: pose (list of floats; None if failure), res: FK status.
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

    x_res= list(x if x_ext is None else Transform(x,x_ext))
    return (x_res, True) if with_st else x_res

  '''Compute a Jacobian matrix of an arm.
  Return J of self.EndLink(arm).
    return: J, res;  J: Jacobian (numpy.matrix; None if failure), res: status.
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
      raise Exception('TRobotGen3.J: Jacobian with x_ext is not implemented yet.')

    angles= {joint:q[j] for j,joint in enumerate(self.joint_names[arm])}  #Deserialize
    with self.sensor_locker:
      J_res= self.kin[arm].jacobian(joint_values=angles)
    return (J_res, True) if with_st else J_res

  '''Compute an inverse kinematics of an arm.
  Return joint angles for a target self.EndLink(arm) pose on self.BaseFrame.
    return: q, res;  q: joint angles (list of floats; None if failure), res: IK status.
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
      res,q= self.kin[arm].inverse_kinematics(xw_trg[:3], xw_trg[3:], seed=start_angles, maxiter=1000, eps=1.0e-6, with_st=True)
    if q is not None:  q= list(q)

    if res:  return (q, True) if with_st else q
    else:  return (q, False) if with_st else None


  '''Follow a joint angle trajectory.
    arm: arm id, or None (==currarm).
    q_traj: joint angle trajectory [q0,...,qD]*N.
    t_traj: corresponding times in seconds from start [t1,t2,...,tN].
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN. '''
  def FollowQTraj(self, q_traj, t_traj, arm=None, blocking=False):
    assert(len(q_traj)==len(t_traj))
    arm= 0

    #self.StopMotion(arm=arm)  #Ensure to cancel the ongoing goal.

    #Insert current position to beginning.
    if t_traj[0]>1.0e-4:
      t_traj.insert(0,0.0)
      q_traj.insert(0,self.Q(arm=arm))

    goal= control_msgs.msg.FollowJointTrajectoryGoal()
    for jn in self.joint_names[arm]:
      goal_tol= control_msgs.msg.JointTolerance()
      goal_tol.name= jn
      goal_tol.position= 0.01
      goal.goal_tolerance.append(goal_tol)
    goal.trajectory.joint_names= self.joint_names[arm]
    goal.trajectory.points= ROSInterpolateTraj(q_traj, t_traj)
    goal.trajectory.header.stamp= rospy.Time.now()

    with self.control_locker:
      self.actc.traj.send_goal(goal)
      BlockAction(self.actc.traj, blocking=blocking, duration=t_traj[-1])

  '''Stop motion such as FollowQTraj.
    arm: arm id, or None (==currarm). '''
  def StopMotion(self, arm=None):
    arm= 0

    with self.control_locker:
      self.actc.traj.cancel_goal()
      BlockAction(self.actc.traj, blocking=True, duration=10.0)  #duration does not matter.


  '''Open a gripper.
    arm: arm id, or None (==currarm).
    blocking: False: move background, True: wait until motion ends.  '''
  def OpenGripper(self, arm=None, blocking=False):
    if arm is None:  arm= self.Arm
    gripper= self.grippers[arm]
    with self.control_locker:
      gripper.Open(blocking=blocking)

  '''Close a gripper.
    arm: arm id, or None (==currarm).
    blocking: False: move background, True: wait until motion ends.  '''
  def CloseGripper(self, arm=None, blocking=False):
    if arm is None:  arm= self.Arm
    gripper= self.grippers[arm]
    with self.control_locker:
      gripper.Close(blocking=blocking)

  '''High level interface to control a gripper.
    arm: arm id, or None (==currarm).
    pos: target position in meter.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum).
    blocking: False: move background, True: wait until motion ends.  '''
  def MoveGripper(self, pos, max_effort=50.0, speed=50.0, arm=None, blocking=False):
    arm= 0

    gripper= self.grippers[arm]
    with self.control_locker:
      gripper.Move(pos, max_effort, speed, blocking=blocking)

  '''Get a gripper position in meter.
    arm: arm id, or None (==currarm). '''
  def GripperPos(self, arm=None):
    arm= 0

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
    arm= 0
    if pos is None:  pos= self.GripperPos(arm)
    return self.grippers[arm].FingertipOffset(pos)

