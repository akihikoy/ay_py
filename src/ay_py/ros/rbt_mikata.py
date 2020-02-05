#! /usr/bin/env python
#Robot controller for Mikata Arm.
from const import *
#if ROS_ROBOT not in ('ANY','Mikata','Mikata_SIM'):
  #raise ImportError('Stop importing: ROS_ROBOT is not Mikata')
#if ROS_DISTRO not in ('groovy','hydro','indigo'):  return

import roslib
import rospy
import sensor_msgs.msg
import trajectory_msgs.msg
import copy
import copy
import threading

#TODO:FIXME:For Mikata simulator, avoid loading DynamixelSDK.
from robot import *
from rbt_dxlg import TDxlGripper
from ..misc.dxl_mikata import TMikata
from ..misc.dxl_holding import TDxlHolding
from kdl_kin import *


'''Mikata Gripper utility class (the same as TDxlGripper)'''
class TMikataGripper(TDxlGripper):
  def __init__(self):
    super(TMikataGripper,self).__init__(dev=None)

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self, mikata):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    #Using external Dynamixel object (maintained by the Mikata controller).
    self.mikata= mikata  #Reference to Mikata Arm.

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    pass

  '''Get current position.'''
  def Position(self):
    pos= self.mikata.State()['position'][-1]
    pos= self.dxlg.dxlg_cmd2pos(self.mikata.invconv_pos['gripper_joint_5'](pos))
    return pos

  '''Activate gripper (torque is enabled).
    Return success or not.'''
  def Activate(self):
    self.mikata.EnableTorque(['gripper_joint_5'])
    return True

  '''Deactivate gripper (torque is disabled).
    Return success or not.'''
  def Deactivate(self):
    self.mikata.DisableTorque(['gripper_joint_5'])
    return True

  '''Control a gripper.
    pos: target position in meter.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum); NOT_IMPLEMENTED.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Move(self, pos, max_effort=50.0, speed=50.0, blocking=False):
    cmd= max(self.dxlg.CmdMin,min(self.dxlg.CmdMax,int(self.dxlg.dxlg_pos2cmd(pos))))
    if self.dxlg.holding is None:
      with self.mikata.port_locker:
        self.mikata.SetPWM({'gripper_joint_5':max_effort})
        self.mikata.MoveTo({'gripper_joint_5':self.mikata.conv_pos['gripper_joint_5'](cmd)}, blocking=(blocking==True))
    else:
      with self.mikata.port_locker:
        self.mikata.SetPWM({'gripper_joint_5':max_effort})
      max_pwm= self.mikata.invconv_pwm['gripper_joint_5'](max_effort)
      self.dxlg.holding.SetTarget(cmd, self.dxlg.holding_max_pwm_rate*max_pwm)

  #Start holding controller with control rate (Hz).
  def StartHolding(self, rate=30):
    self.StopHolding()

    def holding_observer():
      pos= self.mikata.invconv_pos['gripper_joint_5'](self.mikata.State()['position'][-1])
      vel= self.mikata.invconv_vel['gripper_joint_5'](self.mikata.State()['velocity'][-1])
      pwm= self.mikata.invconv_pwm['gripper_joint_5'](self.mikata.State()['effort'][-1])
      return pos,vel,pwm
    def holding_controller(target_position):
      with self.mikata.port_locker:
        self.mikata.MoveTo({'gripper_joint_5':self.mikata.conv_pos['gripper_joint_5'](target_position)}, blocking=False)

    with self.mikata.port_locker:
      goal_pos= self.mikata.dxl['gripper_joint_5'].Read('GOAL_POSITION')
      max_pwm= self.mikata.dxl['gripper_joint_5'].Read('GOAL_PWM')

    self.dxlg.holding= TDxlHolding(rate)
    self.dxlg.holding.observer= holding_observer
    self.dxlg.holding.controller= holding_controller
    self.dxlg.holding.SetTarget(goal_pos, self.dxlg.holding_max_pwm_rate*max_pwm)
    self.dxlg.holding.Start()

  def StopHolding(self):
    if self.dxlg.holding is not None:
      self.dxlg.holding.Stop()
    self.dxlg.holding= None


'''Interface of dummy Mikata robot.'''
class TDummyMikata(TROSUtil):
  def __init__(self):
    super(TDummyMikata,self).__init__()
    self.joint_names= rospy.get_param('controller_joint_names')

    self.control_locker= threading.RLock()
    self.sensor_locker= threading.RLock()

    self.gripper_pos= None

  def __del__(self):
    self.Cleanup()
    print '%s: bye.'%self.Name

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    ra(self.AddPub('joint_path_command', '/joint_path_command', trajectory_msgs.msg.JointTrajectory))
    ra(self.AddSub('joint_states', '/joint_states', sensor_msgs.msg.JointState, self.JointStatesCallback))

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Setup(self):
    return self.Init()

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    super(TDummyMikata,self).Cleanup()

  def Quit(self):
    self.Cleanup()

  def JointNames(self):
    return self.joint_names

  def EnableTorque(self,joint_names=None):
    pass

  def DisableTorque(self,joint_names=None):
    pass

  def JointStatesCallback(self, msg):
    with self.sensor_locker:
      self.state={
        'position': msg.position,
        'velocity': msg.velocity,
        'effort': msg.effort,
        }

  #Get current state
  def State(self):
    with self.sensor_locker:
      state= copy.deepcopy(self.state)
    return state

  def StartStateObs(self, callback=None):
    pass
  def StopStateObs(self):
    pass
  def SetStateObsRate(self, rate):
    pass

  #Follow a trajectory.
  #  joint_names: Names of joints to be controlled. (IGNORED)
  #  (q_traj,t_traj): Sequence of (joint positions) and (time from start).
  #  current: Currents of joints (available when the operation mode=CURRPOS). (IGNORED)
  def FollowTrajectory(self, joint_names, q_traj, t_traj, current=None, blocking=False):
    curr_pos= self.State()['position']
    if self.gripper_pos is None:  self.gripper_pos= curr_pos[-1]

    if len(q_traj[0])==4:
      q_traj= [list(q)+[self.gripper_pos] for q in q_traj]
    elif len(q_traj[0])==1:
      self.gripper_pos= q_traj[-1][0]
      q_traj= [list(curr_pos[:-1])+list(q) for q in q_traj]

    #Insert current position to beginning.
    if t_traj[0]>1.0e-3:
      t_traj.insert(0,0.0)
      q_traj.insert(0,curr_pos)

    traj= ToROSTrajectory(self.JointNames(), q_traj, t_traj)

    with self.control_locker:
      self.pub.joint_path_command.publish(traj)
      if blocking != False:
        rospy.sleep(t_traj[-1])  #Just sleep.

  def StopTrajectory(self):
    pass
  def SetTrajectoryCtrlRate(self, rate):
    pass

'''Gripper utility class of TDummyMikata'''
class TDummyMikataGripper(TDxlGripper):
  def __init__(self):
    super(TDummyMikataGripper,self).__init__(dev=None)
    self.joint_name= 'gripper_joint_5'

    conv_pos= lambda value: (value-2048.0)*0.0015339808
    self.dxlg.CmdMax= conv_pos(2200)  #Gripper closed (with FingerVision).
    self.dxlg.CmdMin= conv_pos(1200)  #Gripper opened widely.
    self.dxlg.CmdOpen= conv_pos(1900)  #Gripper opened moderately.

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self, mikata):
    self.mikata= mikata  #Reference to Mikata Arm.
    return True

  def Cleanup(self):
    pass

  '''Get current position.'''
  def Position(self):
    pos= self.mikata.State()['position'][-1]
    pos= self.dxlg.dxlg_cmd2pos(pos)
    return pos

  '''Activate gripper (torque is enabled).
    Return success or not.'''
  def Activate(self):
    pass

  '''Deactivate gripper (torque is disabled).
    Return success or not.'''
  def Deactivate(self):
    pass

  '''Control a gripper.
    pos: target position in meter.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum); NOT_IMPLEMENTED.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Move(self, pos, max_effort=50.0, speed=50.0, blocking=False):
    cmd= max(self.dxlg.CmdMin,min(self.dxlg.CmdMax,(self.dxlg.dxlg_pos2cmd(pos))))
    self.mikata.FollowTrajectory([self.joint_name], [[cmd]], [0.2*110.0/(speed+10.0)], blocking=blocking)

  def StartHolding(self, rate=30):
    pass

  def StopHolding(self):
    pass



'''Robot control class for single Mikata Arm.'''
class TRobotMikata(TMultiArmRobot):
  def __init__(self, name='Mikata', dev='/dev/ttyUSB0', is_sim=False):
    super(TRobotMikata,self).__init__(name=name)
    self.is_sim= is_sim
    self.dev= dev

    if not self.is_sim:
      self.mikata= TMikata(dev=self.dev)
    else:
      self.mikata= TDummyMikata()

    self.joint_names= [[]]
    #self.joint_names[0]= rospy.get_param('controller_joint_names')
    self.joint_names[0]= self.mikata.JointNames()[:4]  #Gripper joint is not used.

    #Mikata all link names:
    #obtained from ay_py/demo_ros/kdl1.py (URDF link names)
    self.links= {}
    self.links['base']= ['base_link', 'plate_link']
    self.links['r_arm']= ['link_2', 'link_3', 'link_4', 'link_5']
    self.links['r_gripper']= ['left_gripper', 'right_gripper']
    self.links['robot']= self.links['base'] + self.links['r_arm'] + self.links['r_gripper']

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    self.kin= [None]
    self.kin[0]= TKinematics(base_link='base_link',end_link='link_5')

    #if self.is_sim:
      #ra(self.AddPub('joint_path_command', '/joint_path_command', trajectory_msgs.msg.JointTrajectory))
      #ra(self.AddSub('joint_states', '/joint_states', sensor_msgs.msg.JointState, self.JointStatesCallback))

    ra(self.AddPub('joint_states', '/joint_states', sensor_msgs.msg.JointState))

    #ra(self.AddSub('joint_path_command', '/joint_path_command', trajectory_msgs.msg.JointTrajectory, self.JointPathCommandCallback))

    if not self.is_sim:
      self.mikata_gripper= TMikataGripper()
    else:
      self.mikata_gripper= TDummyMikataGripper()
    self.grippers= [self.mikata_gripper]

    print 'Initializing and activating Mikata arm...'
    ra(self.mikata.Setup())
    ra(self.mikata_gripper.Init(self.mikata))

    if False in res:  return False

    self.js= None
    if not self.is_sim:
      self.mikata.EnableTorque()
      self.mikata.StartStateObs(self.JointStatesCallback)

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    self.mikata.StopStateObs()
    self.mikata.Quit()
    super(TRobotMikata,self).Cleanup()

  '''Configure a state validity checker.'''
  def ConfigureSVC(self, c):
    c.JointNames= copy.deepcopy(self.joint_names)
    c.Links= copy.deepcopy(self.links)
    c.PaddingLinks= []
    c.PaddingValues= [0.002]*len(c.PaddingLinks)
    c.DefaultBaseFrame= 'base_link'
    c.HandLinkToGrasp[0]= 'link_5'
    c.IgnoredLinksInGrasp[0]= []

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('Mikata','Mikata_SIM'):  return True
    return super(TRobotMikata,self).Is(q)

  @property
  def NumArms(self):
    return 1

  @property
  def BaseFrame(self):
    return 'base_link'

  '''End link of an arm.'''
  def EndLink(self, arm=None):
    return 'link_5'

  '''Names of joints of an arm.'''
  def JointNames(self, arm=None):
    arm= 0
    return self.joint_names[arm]

  def DoF(self, arm=None):
    return 4

  '''Return limits (lower, upper) of joint angles.
    arm: arm id, or None (==currarm). '''
  def JointLimits(self, arm=None):
    arm= 0
    return self.kin[arm].joint_limits_lower, self.kin[arm].joint_limits_upper

  '''Return limits of joint angular velocity.
    arm: arm id, or None (==currarm). '''
  def JointVelLimits(self, arm=None):
    #FIXME: Should be adjusted for Motoman
    return [0.5, 0.5, 0.5, 0.5]

  '''Return range of gripper.
    arm: arm id, or None (==currarm). '''
  def GripperRange(self, arm=None):
    if arm is None:  arm= self.Arm
    return self.grippers[arm].PosRange()

  '''End effector of an arm.'''
  def EndEff(self, arm=None):
    arm= 0
    return self.grippers[arm]

  def JointStatesCallback(self, state):
    if self.js is None:
      self.js= sensor_msgs.msg.JointState()
      self.js.name= state['name']
      self.js.header.seq= 0
    self.js.header.seq= self.js.header.seq+1
    self.js.header.stamp= rospy.Time.now()
    self.js.position= state['position']
    self.js.velocity= state['velocity']
    self.js.effort= state['effort']
    self.pub.joint_states.publish(self.js)

  '''Return joint angles of an arm (list of floats).
    arm: arm id, or None (==currarm). '''
  def Q(self, arm=None):
    with self.sensor_locker:
      q= self.mikata.State()['position'][:4]
    return list(q)

  '''Return joint velocities of an arm (list of floats).
    arm: arm id, or None (==currarm). '''
  def DQ(self, arm=None):
    with self.sensor_locker:
      dq= self.mikata.State()['velocity'][:4]
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
      raise Exception('TRobotMikata.J: Jacobian with x_ext is not implemented yet.')

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
    w_x:  weights on pose error (used for weighted pseudo inverse of Jacobian).
    start_angles: initial joint angles for IK solver, or None (==self.Q(arm)).
    with_st: whether return IK status. '''
  def IK(self, x_trg, x_ext=None, start_angles=None, arm=None, with_st=False, w_x=[1.0,1.0,1.0, 0.01,0.01,0.01]):
    arm= 0
    if start_angles is None:  start_angles= self.Q(arm)
    start_angles= [a+0.01*(random.random()-0.5) for a in start_angles]

    x_trg[3:]/= la.norm(x_trg[3:])  #Normalize the orientation:
    xw_trg= x_trg if x_ext is None else TransformRightInv(x_trg,x_ext)

    with self.sensor_locker:
      if w_x is not None:
        res,q= self.kin[arm].inverse_kinematics(
          xw_trg[:3], xw_trg[3:], seed=start_angles, w_x=np.diag(w_x).tolist(),
          maxiter=1000, eps=1.0e-4, with_st=True)
      else:
        res,q= self.kin[arm].inverse_kinematics(
          xw_trg[:3], xw_trg[3:], seed=start_angles,
          maxiter=1000, eps=1.0e-4, with_st=True)
      if not res and q is not None:
        p_err= la.norm(np.array(xw_trg[:3])-self.FK(q, arm=arm)[:3])
        if p_err<1.0e-3:  res= True
        else: print 'IK error:',p_err,xw_trg[:3]-self.FK(q, arm=arm)[:3]
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

    with self.control_locker:
      self.mikata.FollowTrajectory(self.joint_names[arm], q_traj, t_traj, blocking=(blocking==True))

      if blocking == 'time':
        rospy.sleep(t_traj[-1])  #Just sleep.


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
    with self.gripper_locker:
      gripper.Move(pos, max_effort, speed, blocking=blocking)

  '''Get a gripper position in meter.
    arm: arm id, or None (==currarm). '''
  def GripperPos(self, arm=None):
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
    return self.grippers[arm].FingertipOffset(pos)

