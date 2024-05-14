#! /usr/bin/env python
#Robot controller for Motoman.
from const import *

import roslib
import rospy
import actionlib
import control_msgs.msg
import sensor_msgs.msg
import trajectory_msgs.msg
import industrial_msgs.msg
import copy

from robot import *
from kdl_kin import *

'''Robot control class for a single Motoman robot.'''
class TRobotMotoman(TMultiArmRobot):
  def __init__(self, name='Motoman', is_sim=False):
    super(TRobotMotoman,self).__init__(name=name)
    self.is_sim= is_sim

    self.joint_names= [[]]
    self.joint_names[0]= rospy.get_param('controller_joint_names')

    #Tolerance of motion (FollowQTraj).  Increase this value when the payload is large.
    self.MotionTol= 0.05

    #Motoman trajectory controller specific configurations:
    #Number of retries the goal trajectory sending with updating the first goal poing.
    self.NumTrajCtrlRetry= 10
    #Duration of (interval between) each trajectory send_goal retry.
    self.DtTrajCtrlRetry= 0.005
    #Duration (interval) to check the trajectory action client state.
    self.DtTrajActCStateMonitor= 0.001
    #Duration to sleep while trajectory ACTIVE before trajectory ABORTED.
    #  NOTE: If trajectory abort error happens with the first point mismatch error (3011), increase this value.
    self.DtTrajActiveBeforeAbort= 0.5

    #Motoman all link names:
    #obtained from ay_py/demo_ros/kdl1.py (URDF link names)
    self.links= {}
    if self.Name.startswith('MotomanSIA10F'):
      self.type_name= 'MotomanSIA10F'
      self.links['base']= ['base_link']
      self.links['r_arm']= ['link_1_s', 'link_2_l', 'link_3_e', 'link_4_u', 'link_5_r', 'link_6_b', 'link_7_t']
    elif self.Name.startswith('MotomanMotoMINI'):
      self.type_name= 'MotomanMotoMINI'
      self.links['base']= ['base_link']
      self.links['r_arm']= ['link_1_s', 'link_2_l', 'link_3_u', 'link_4_r', 'link_5_b', 'link_6_t']
    elif self.Name.startswith('MotomanGP7'):
      self.type_name= 'MotomanGP7'
      self.links['base']= ['base_link']
      self.links['r_arm']= ['link_1_s', 'link_2_l', 'link_3_u', 'link_4_r', 'link_5_b', 'link_6_t']
    elif self.Name.startswith('MotomanGP8'):
      self.type_name= 'MotomanGP8'
      self.links['base']= ['base_link']
      self.links['r_arm']= ['link_1_s', 'link_2_l', 'link_3_u', 'link_4_r', 'link_5_b', 'link_6_t']
    elif self.Name.startswith('MotomanHC10SDTP'):
      self.type_name= 'MotomanHC10SDTP'
      self.links['base']= ['base_link']
      self.links['r_arm']= ['link_1_s', 'link_2_l', 'link_3_u', 'link_4_r', 'link_5_b', 'link_6_t']
    elif self.Name.startswith('MotomanSG650'):
      self.type_name= 'MotomanSG650'
      self.links['base']= ['base_link']
      self.links['r_arm']= ['link_1_s', 'link_1_s_cable', 'link_2_l', 'link_3_u', 'link_4_r']
    else:
      CPrint(4,'Invalid Motoman name: {}'.format(self.Name))
    self.links['robot']= self.links['base'] + self.links['r_arm']

    #Thread lockers for robot_status:
    self.robot_status_locker= threading.RLock()

    self.q_curr= None
    self.dq_curr= None
    self.robot_status= None

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    self.kin= [None]
    self.kin[0]= TKinematics(base_link=self.links['base'][0],end_link='tool0')

    ra(self.AddActC('traj', '/joint_trajectory_action',
                    control_msgs.msg.FollowJointTrajectoryAction, time_out=3.0))

    ra(self.AddSub('joint_states', '/joint_states', sensor_msgs.msg.JointState, self.JointStatesCallback))

    ra(self.AddSub('robot_status', '/robot_status', industrial_msgs.msg.RobotStatus, self.RobotStatusCallback))

    #self.robotiq= TRobotiq()  #Robotiq controller
    ##self.robotiq= TSimGripper2F1(('Robotiq',),pos_range=[0.0,0.0855])
    #self.grippers= [self.robotiq]
    self.grippers= [TFakeGripper()]

    #print 'Enabling the robot...'

    #print 'Initializing and activating Robotiq gripper...'
    #ra(self.robotiq.Init())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    for gripper in self.grippers:  gripper.Cleanup()
    super(TRobotMotoman,self).Cleanup()

  '''Configure a state validity checker.'''
  def ConfigureSVC(self, c):
    c.JointNames= copy.deepcopy(self.joint_names)
    c.Links= copy.deepcopy(self.links)
    c.PaddingLinks= []
    c.PaddingValues= [0.002]*len(c.PaddingLinks)
    c.DefaultBaseFrame= self.kin[0]._base_link
    c.HandLinkToGrasp[0]= self.kin[0]._tip_link
    c.IgnoredLinksInGrasp[0]= []

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('Motoman','Motoman_SIM',self.type_name,self.Name):  return True
    return super(TRobotMotoman,self).Is(q)

  #Check if the robot is normal state (i.e. running properly without stopping).
  def IsNormal(self):
    if self.is_sim:  return True
    with self.robot_status_locker:
      return all((self.robot_status.mode.val==industrial_msgs.msg.RobotMode.AUTO,
                  self.robot_status.e_stopped.val==industrial_msgs.msg.TriState.FALSE,
                  self.robot_status.drives_powered.val==industrial_msgs.msg.TriState.TRUE,
                  self.robot_status.motion_possible.val==industrial_msgs.msg.TriState.TRUE,
                  self.robot_status.in_error.val==industrial_msgs.msg.TriState.FALSE,))

  def PrintStatus(self):
    print 'robot_status: {}'.format(self.robot_status)

  @property
  def NumArms(self):
    return 1

  @property
  def BaseFrame(self):
    return self.kin[0]._base_link

  '''End link of an arm.'''
  def EndLink(self, arm=None):
    return self.kin[0]._tip_link

  '''Names of joints of an arm.'''
  def JointNames(self, arm=None):
    if arm is None:  arm= self.Arm
    return self.joint_names[arm]

  def DoF(self, arm=None):
    return len(self.JointNames(arm=arm))

  '''Return limits (lower, upper) of joint angles.
    arm: arm id, or None (==currarm). '''
  def JointLimits(self, arm=None):
    if arm is None:  arm= self.Arm
    return self.kin[arm].joint_limits_lower, self.kin[arm].joint_limits_upper

  '''Return limits of joint angular velocity.
    arm: arm id, or None (==currarm).
    NOTE: This limit is used only when it is used explicitly.'''
  def JointVelLimits(self, arm=None):
    if self.Name.startswith('MotomanSIA10F'):
      #['s','l','e','u','r','b','t']
      return [0.5, 0.5, 0.8, 0.8, 0.8, 0.8, 0.8]
    elif self.Name.startswith('MotomanMotoMINI'):
      return [0.5, 0.5, 0.8, 0.8, 0.8, 0.8]
    elif self.Name.startswith('MotomanGP7'):
      return [0.5, 0.5, 0.8, 0.8, 0.8, 0.8]
    elif self.Name.startswith('MotomanGP8'):
      return [0.5, 0.5, 0.8, 0.8, 0.8, 0.8]
    elif self.Name.startswith('MotomanHC10SDTP'):
      return [0.5, 0.5, 0.8, 0.8, 0.8, 0.8]
    elif self.Name.startswith('MotomanSG650'):
      return [1.0, 1.0, 1.0, 1.0]

  '''Return range of gripper.
    arm: arm id, or None (==currarm). '''
  def GripperRange(self, arm=None):
    if arm is None:  arm= self.Arm
    return self.grippers[arm].PosRange()

  '''End effector of an arm.'''
  def EndEff(self, arm=None):
    if arm is None:  arm= self.Arm
    return self.grippers[arm]

  def JointStatesCallback(self, msg):
    with self.sensor_locker:
      self.x_curr= msg
      self.q_curr= self.x_curr.position
      self.dq_curr= self.x_curr.velocity

  def RobotStatusCallback(self, msg):
    with self.robot_status_locker:
      self.robot_status= msg

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
      raise Exception('TRobotMotoman.J: Jacobian with x_ext is not implemented yet.')

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

    for i_retry in range(self.NumTrajCtrlRetry+1):
      #copy q_traj, t_traj to goal
      goal= control_msgs.msg.FollowJointTrajectoryGoal()
      goal.goal_time_tolerance= rospy.Time(0.1)
      goal.trajectory.joint_names= self.joint_names[arm]
      goal.trajectory= ToROSTrajectory(self.JointNames(arm), q_traj, t_traj, dq_traj)

      with self.control_locker:
        self.actc.traj.send_goal(goal)

      #Wait for the change of self.actc.traj state:
      t_wait_state_start= rospy.Time.now()
      while self.actc.traj.get_state()==actionlib_msgs.msg.GoalStatus.PENDING:
        #print 'DEBUG: Trial {}: action_client_state: {}, {}'.format(i_retry, self.actc.traj.get_state(), ACTC_STATE_TO_STR[self.actc.traj.get_state()])
        rospy.sleep(self.DtTrajActCStateMonitor)
        if (rospy.Time.now()-t_wait_state_start).to_sec()>1.0:
          print 'Timeout. action_client_state is PENDING for a while.'
          break
      #Wait during the self.actc.traj state==ACTIVE:
      #  As this check takes time (self.DtTrajActiveBeforeAbort), it is done only when blocking!=False
      if blocking!=False:
        t_wait_state_start= rospy.Time.now()
        while self.actc.traj.get_state()==actionlib_msgs.msg.GoalStatus.ACTIVE:
          #print 'DEBUG: Trial {}: action_client_state: {}, {}'.format(i_retry, self.actc.traj.get_state(), ACTC_STATE_TO_STR[self.actc.traj.get_state()])
          if not self.IsNormal():
            self.PrintStatus()
            raise Exception('FollowQTraj: Stopped as the robot is not in normal state (1).')
          rospy.sleep(self.DtTrajActCStateMonitor)
          if (rospy.Time.now()-t_wait_state_start).to_sec()>self.DtTrajActiveBeforeAbort:
            #print 'DEBUG: Stop ACTIVE waiting.'
            break
        #print 'DEBUG: Waited during ACTIVE for {}s'.format((rospy.Time.now()-t_wait_state_start).to_sec())
      #print 'DEBUG: Trial {}: action_client_state: {}, {}'.format(i_retry, self.actc.traj.get_state(), ACTC_STATE_TO_STR[self.actc.traj.get_state()])
      #In case the trajectory aborted, it can be guessed as the first position mismatch.
      #Retrying after updating the first point by the current position.
      if self.actc.traj.get_state()==actionlib_msgs.msg.GoalStatus.ABORTED:
        if not self.IsNormal():
          self.PrintStatus()
          raise Exception('FollowQTraj: Stopped as the robot is not in normal state (2).')
        print '{}: Trajectory aborted. Retrying by updating the first point to the current joint angles ({}).'.format(self.Name, i_retry)
        rospy.sleep(self.DtTrajCtrlRetry)
        q_traj[0]= self.Q(arm=arm)
      else:
        break

    with self.control_locker:
      BlockAction(self.actc.traj, blocking=blocking, duration=t_traj[-1])
      if blocking!=False:
        q_finished= self.Q(arm=arm)
        q_err= np.array(q_traj[-1])-q_finished
        if np.max(np.abs(q_err)) > self.MotionTol:
          CPrint(4,'TRobotMotoman.FollowQTraj: Unacceptable error after movement')
          CPrint(4,'  Info:q_traj[-1]:',q_traj[-1])
          CPrint(4,'  Info:q_finished:',q_finished)
          CPrint(4,'  Info:q_err:',q_err.tolist())
          CPrint(4,'  Info:q_traj:',q_traj)
          CPrint(4,'  Info:t_traj:',t_traj)
          CPrint(4,'  Info:dq_traj:',dq_traj)
          CPrint(4,'Action client result:',self.actc.traj.get_result()),'(cf. control_msgs/FollowJointTrajectoryActionResult)'
          raise ROSError('ctrl','TRobotMotoman.FollowQTraj: Unacceptable error after movement')

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
