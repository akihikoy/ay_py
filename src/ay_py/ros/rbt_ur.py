#! /usr/bin/env python
#Robot controller for Universal Robots UR*.
from const import *

import roslib
import rospy
import actionlib
import std_msgs.msg
import controller_manager_msgs.srv
import control_msgs.msg
import sensor_msgs.msg
import trajectory_msgs.msg
import copy

from robot import *
from kdl_kin import *

'''Robot control class for single Universal Robots UR* with an empty gripper.'''
class TRobotUR(TMultiArmRobot):
  def __init__(self, name='UR', ur_series='CB', robot_ip=None, is_sim=False):
    super(TRobotUR,self).__init__(name=name)
    self.is_sim= is_sim
    self.ur_series= ur_series
    self.robot_ip= robot_ip  #If robot_ip is None, Init() fills it from the /ur_hardware_interface/robot_ip parameter.

    self.joint_names= [[]]
    #self.joint_names[0]= rospy.get_param('controller_joint_names')
    self.joint_names[0]= ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    #Tolerance of motion (FollowQTraj).  Increase this value when the payload is large.
    self.MotionTol= 0.05

    self.GoalTimeTol= 0.1  #Goal time tolerance for FollowQTraj.
    self.PathTol= []  #List of control_msgs.msg.JointTolerance for FollowQTraj.
    self.GoalTol= []  #List of control_msgs.msg.JointTolerance for FollowQTraj.
    #Examples to set PathTol:
    #  robot.PathTol= [control_msgs.msg.JointTolerance(jname,1.0,8.0,0.1) for jname in robot.JointNames(arm)]
    #  robot.PathTol= GetSimpleJointTol(robot.JointNames(arm), 1.0,8.0,0.1)

    #UR all link names:
    #obtained from ay_py/demo_ros/kdl1.py (URDF link names)
    self.links= {}
    self.links['base']= ['base_link']
    self.links['r_arm']= ['shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']
    self.links['robot']= self.links['base'] + self.links['r_arm']

    #Thread lockers for robot_mode, safety_mode, robot_program_running:
    self.robot_mode_locker= threading.RLock()
    self.safety_mode_locker= threading.RLock()
    self.robot_program_running_locker= threading.RLock()

    self.q_curr= None
    self.dq_curr= None

    self.io_states= None
    self.io_states_callback= None  #User defined callback f(ur_msgs.msg.IOStates).
    self.io_states_locker= threading.RLock()

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    if self.robot_ip is None and not self.is_sim:
      self.robot_ip= rospy.get_param('/ur_hardware_interface/robot_ip')

    #Check the validity of joint positions:
    try:
      x_curr= rospy.wait_for_message('/joint_states', sensor_msgs.msg.JointState, 3.0)
      """
      if not all([-1.5*math.pi<q and q<1.5*math.pi for q in x_curr.position]):
        CPrint(4,'''Warning: some joint angles exceed the expected range.
  Joint angles = {q}
  Running without notifying is very dangerous.
  Manually moving joints to proper range (-1.5*math.pi, 1.5*math.pi)
  is highly recommended.
  Hint: Use the Freedrive mode with observing the /joint_states topic by:
    $ rostopic echo '/joint_states/position[5]'

  Will you continue to set up the robot? (Recommended: N)'''.format(q=x_curr.position))
        if not AskYesNo():  return False
      """
    except (rospy.ROSException, rospy.ROSInterruptException):
      CPrint(4,'Cannot get data from /joint_states')
      return False

    self.kin= [None]
    self.kin[0]= TKinematics(base_link='base_link',end_link='tool0')

    if not self.is_sim:
      ra(self.AddPub('joint_vel', '/joint_group_vel_controller/command', std_msgs.msg.Float64MultiArray, queue_size=10))
      ra(self.AddPub('urscript','/ur_hardware_interface/script_command',std_msgs.msg.String, queue_size=10))
      ra(self.AddSrvP('sw_ctrl', '/controller_manager/switch_controller', controller_manager_msgs.srv.SwitchController, time_out=3.0))

    if not self.is_sim:
      ra(self.AddActC('traj', '/scaled_pos_joint_traj_controller/follow_joint_trajectory',
                      control_msgs.msg.FollowJointTrajectoryAction, time_out=3.0))
    else:
      ra(self.AddActC('traj', '/follow_joint_trajectory',
                      control_msgs.msg.FollowJointTrajectoryAction, time_out=3.0))

    ra(self.AddSub('joint_states', '/joint_states', sensor_msgs.msg.JointState, self.JointStatesCallback))

    if not self.is_sim:
      ur_dashboard_msgs= __import__('ur_dashboard_msgs',globals(),None,('msg',))
      ra(self.AddSub('robot_mode', '/ur_hardware_interface/robot_mode', ur_dashboard_msgs.msg.RobotMode, self.RobotModeCallback))
      ra(self.AddSub('safety_mode', '/ur_hardware_interface/safety_mode', ur_dashboard_msgs.msg.SafetyMode, self.SafetyModeCallback))
      ra(self.AddSub('robot_program_running', '/ur_hardware_interface/robot_program_running', std_msgs.msg.Bool, self.RobotProgramRunningCallback))
      #roslib.load_manifest('ur_msgs')

    #2023-04-10 modified to subscribe io_states even in simulator for simulated io_states topics.
    ur_msgs= __import__('ur_msgs',globals(),None,('msg',))
    ra(self.AddSub('io_states', '/ur_hardware_interface/io_states', ur_msgs.msg.IOStates, self.IOStatesCallback))

    self.grippers= [TFakeGripper()]

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    for gripper in self.grippers:  gripper.Cleanup()
    super(TRobotUR,self).Cleanup()

  '''Configure a state validity checker.'''
  def ConfigureSVC(self, c):
    c.JointNames= copy.deepcopy(self.joint_names)
    c.Links= copy.deepcopy(self.links)
    c.PaddingLinks= []
    c.PaddingValues= [0.002]*len(c.PaddingLinks)
    c.DefaultBaseFrame= 'base_link'
    c.HandLinkToGrasp[0]= 'tool0'
    c.IgnoredLinksInGrasp[0]= []

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('UR','UR_SIM'):  return True
    if q=='E':  return (self.ur_series=='E')
    if q=='CB':  return (self.ur_series=='CB')
    return super(TRobotUR,self).Is(q)

  #Check if the robot is normal state (i.e. running properly without stopping).
  def IsNormal(self):
    if self.is_sim:  return True
    with self.robot_mode_locker, self.safety_mode_locker, self.robot_program_running_locker:
      return all((self.robot_mode.mode==self.robot_mode.RUNNING, self.safety_mode.mode==self.safety_mode.NORMAL, self.robot_program_running))

  def PrintStatus(self):
    print 'robot_mode: {0} ({1})'.format(self.robot_mode.mode, 'running' if self.robot_mode.mode==self.robot_mode.RUNNING else 'not running')
    print 'safety_mode: {0} ({1})'.format(self.safety_mode.mode, 'normal' if self.safety_mode.mode==self.safety_mode.NORMAL else 'not normal')
    print 'robot_program_running: {0}'.format(self.robot_program_running)

  def RobotIP(self):
    return self.robot_ip

  @property
  def NumArms(self):
    return 1

  @property
  def BaseFrame(self):
    return 'base_link'

  '''End link of an arm.'''
  def EndLink(self, arm=None):
    return 'tool0'

  '''Names of joints of an arm.'''
  def JointNames(self, arm=None):
    if arm is None:  arm= self.Arm
    return self.joint_names[arm]

  def DoF(self, arm=None):
    return 6

  '''Return limits (lower, upper) of joint angles.
    arm: arm id, or None (==currarm). '''
  def JointLimits(self, arm=None):
    if arm is None:  arm= self.Arm
    return self.kin[arm].joint_limits_lower, self.kin[arm].joint_limits_upper

  '''Return limits of joint angular velocity.
    arm: arm id, or None (==currarm). '''
  def JointVelLimits(self, arm=None):
    #max_velocity (UR3,5,10) = 10 rad/s (according to ur_modern_driver/launch/*.launch)
    #FIXME: Should be adjusted for UR
    return [0.5, 0.5, 0.8, 0.8, 0.8, 0.8, 0.8]

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

  def JointStatesCallback(self, msg):
    arm= 0
    with self.sensor_locker:
      self.x_curr= msg
      q_map= {name:position for name,position in zip(self.x_curr.name,self.x_curr.position)}
      dq_map= {name:velocity for name,velocity in zip(self.x_curr.name,self.x_curr.velocity)}
      self.q_curr= [q_map[name] for name in self.joint_names[arm]]
      self.dq_curr= [dq_map[name] for name in self.joint_names[arm]]

  def RobotModeCallback(self, msg):
    with self.robot_mode_locker:
      self.robot_mode= msg
  def SafetyModeCallback(self, msg):
    with self.safety_mode_locker:
      self.safety_mode= msg
  def RobotProgramRunningCallback(self, msg):
    with self.robot_program_running_locker:
      self.robot_program_running= msg.data

  def IOStates(self):
    with self.io_states_locker:
      io_states= self.io_states
    return io_states
  def IOStatesCallback(self, msg):
    with self.io_states_locker:
      self.io_states= msg
    if self.io_states_callback:
      self.io_states_callback(self.io_states)

  '''Return joint angles of an arm (list of floats).
    arm: arm id, or None (==currarm). '''
  def Q(self, arm=None):
    with self.sensor_locker:
      q= self.q_curr
    return list(q) if q is not None else None

  '''Return joint velocities of an arm (list of floats).
    arm: arm id, or None (==currarm). '''
  def DQ(self, arm=None):
    with self.sensor_locker:
      dq= self.dq_curr
    return list(dq) if dq is not None else None

  '''Compute a forward kinematics of an arm.
  Return self.EndLink(arm) pose on self.BaseFrame.
    return: x, res;  x: pose (list of floats; None if failure), res: FK status.
    arm: arm id, or None (==currarm).
    q: list of joint angles, or None (==self.Q(arm)).
    x_ext: a local pose on self.EndLink(arm) frame.
      If not None, the returned pose is x_ext on self.BaseFrame.
    with_st: whether return FK status. '''
  def FK(self, q=None, x_ext=None, arm=None, with_st=False):
    if arm is None:  arm= self.Arm
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
    if arm is None:  arm= self.Arm
    if q is None:  q= self.Q(arm)

    if x_ext is not None:
      #Since KDL does not provide Jacobian computation with an offset x_ext,
      #and converting J with x_ext is not simple, we raise an Exception.
      #TODO: Implement our own FK to solve this issue.
      raise Exception('TRobotUR.J: Jacobian with x_ext is not implemented yet.')

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
    if arm is None:  arm= self.Arm
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
    if arm is None:  arm= self.Arm

    if not self.IsNormal():
      self.PrintStatus()
      raise Exception('Cannot execute FollowQTraj as the robot is not normal state.')

    self.StopMotion(arm=arm)  #Ensure to cancel the ongoing goal.

    #Insert current position to beginning.
    if t_traj[0]>1.0e-4:
      t_traj.insert(0,0.0)
      q_traj.insert(0,self.Q(arm=arm))

    dq_traj= QTrajToDQTraj(q_traj, t_traj)

    #copy q_traj, t_traj to goal
    goal= control_msgs.msg.FollowJointTrajectoryGoal()
    if self.GoalTimeTol is not None:  goal.goal_time_tolerance= rospy.Time(self.GoalTimeTol)
    if self.PathTol is not None:  goal.path_tolerance= self.PathTol
    if self.GoalTol is not None:  goal.goal_tolerance= self.GoalTol
    goal.trajectory.joint_names= self.joint_names[arm]
    goal.trajectory= ToROSTrajectory(self.JointNames(arm), q_traj, t_traj, dq_traj)

    with self.control_locker:
      self.actc.traj.send_goal(goal)
      BlockAction(self.actc.traj, blocking=blocking, duration=t_traj[-1])
      if blocking!=False:
        q_finished= self.Q(arm=arm)
        q_err= np.array(q_traj[-1])-q_finished
        if self.MotionTol is not None and np.max(np.abs(q_err)) > self.MotionTol:
          CPrint(4,'TRobotUR.FollowQTraj: Unacceptable error after movement')
          CPrint(4,'  Info:MotionTol:',self.MotionTol)
          CPrint(4,'  Info:q_traj[-1]:',q_traj[-1])
          CPrint(4,'  Info:q_finished:',q_finished)
          CPrint(4,'  Info:q_err:',q_err.tolist())
          CPrint(4,'  Info:q_traj:',q_traj)
          CPrint(4,'  Info:t_traj:',t_traj)
          CPrint(4,'  Info:dq_traj:',dq_traj)
          CPrint(4,'Action client result:',self.actc.traj.get_result()),'(cf. control_msgs/FollowJointTrajectoryActionResult)'
          raise ROSError('ctrl','TRobotUR.FollowQTraj: Unacceptable error after movement')

  '''Stop motion such as FollowQTraj.
    arm: arm id, or None (==currarm). '''
  def StopMotion(self, arm=None):
    if arm is None:  arm= self.Arm

    with self.control_locker:
      self.actc.traj.cancel_goal()
      try:
        BlockAction(self.actc.traj, blocking=True, duration=10.0)  #duration does not matter.
      except ROSError as e:
        #There will be an error when there is no goal. Ignoring.
        pass


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
