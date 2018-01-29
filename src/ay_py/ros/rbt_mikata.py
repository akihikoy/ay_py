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

from robot import *
from rbt_dxlg import TDxlGripper
from ..misc.dxl_mikata import TMikata
from kdl_kin import *

'''Robot control class for single Mikata Arm.'''
class TRobotMikata(TMultiArmRobot):
  def __init__(self, name='Mikata', dev='/dev/ttyUSB0'):
    super(TRobotMikata,self).__init__(name=name)
    #self.is_sim= (ROS_ROBOT=='Mikata_SIM')
    self.dev= dev

    self.mikata= TMikata(dev=self.dev)

    self.joint_names= [[]]
    #self.joint_names[0]= rospy.get_param('controller_joint_names')
    self.joint_names[0]= self.mikata.JointNames()[:4]  #Gripper joint is not used.

    #Mikata all link names:
    #obtained from ay_py/demo_ros/kdl1.py (URDF link names)
    self.links= {}
    #FIXME:Should be fixed.
    #self.links['base']= ['base_link']
    #self.links['r_arm']= ['link_s', 'link_l', 'link_e', 'link_u', 'link_r', 'link_b', 'link_t']
    #self.links['robot']= self.links['base'] + self.links['r_arm']

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

    #self.dxl_gripper= TDxlGripper(dev=self.mikata.dev)
    #self.dxl_gripper.dxl.Baudrate= self.mikata.baudrate
    #self.dxl_gripper.dxl.Id= self.mikata.dxl_ids[-1]
    #self.grippers= [self.dxl_gripper]

    print 'Initializing and activating Mikata arm...'
    ra(self.mikata.Setup())
    #ra(self.dxl_gripper.Init())

    if False in res:  return False

    self.mikata.EnableTorque()
    self.mikata.StartStateObs(self.JointStatesCallback)
    self.js= None

    #Gripper command-position conversions.
    self.dxlg_range= [0.0,0.095]
    self.dxlg_cmd2pos= lambda cmd: self.dxlg_range[1] + (cmd-1900)*(self.dxlg_range[0]-self.dxlg_range[1])/(2200-1900)
    self.dxlg_pos2cmd= lambda pos: 1900 + (pos-self.dxlg_range[1])*(2200-1900)/(self.dxlg_range[0]-self.dxlg_range[1])

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
  def EndLink(self, arm):
    return 'link_5'

  '''Names of joints of an arm.'''
  def JointNames(self, arm):
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
    return self.dxlg_range

  '''End effector of an arm.'''
  def EndEff(self, arm=None):
    arm= 0
    #return self.grippers[arm]
    return self

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

  '''Return joint angles of an arm.
    arm: arm id, or None (==currarm). '''
  def Q(self, arm=None):
    with self.sensor_locker:
      q= self.mikata.State()['position'][:4]
    return q

  '''Return joint velocities of an arm.
    arm: arm id, or None (==currarm). '''
  def DQ(self, arm=None):
    with self.sensor_locker:
      dq= self.mikata.State()['velocity'][:4]
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
      raise Exception('TRobotMikata.J: Jacobian with x_ext is not implemented yet.')

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
      #FIXME: We should ignore xw_trg[3:] since Mikata has only 4 dof.
      #q= self.kin[arm].inverse_kinematics(xw_trg[:3], xw_trg[3:], seed=start_angles, maxiter=1000, eps=1.0e-6)
      q= self.kin[arm].inverse_kinematics(xw_trg[:3], seed=start_angles, maxiter=2000, eps=1.0e-3)

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

    with self.control_locker:
      self.mikata.FollowTrajectory(self.joint_names[arm], q_traj, t_traj, wait=(blocking==True))

      if blocking == 'time':
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
    if arm is None:  arm= self.Arm

    #gripper= self.grippers[arm]
    cmd= self.dxlg_pos2cmd(pos)
    with self.control_locker:
      #gripper.Move(cmd, blocking=blocking)
      self.mikata.SetPWM({'gripper_joint_5':max_effort})
      self.mikata.MoveTo({'gripper_joint_5':self.mikata.conv_pos['gripper_joint_5'](cmd)}, wait=(blocking==True))

  '''Get a gripper position in meter.
    arm: arm id, or None (==currarm). '''
  def GripperPos(self, arm=None):
    if arm is None:  arm= self.Arm

    #gripper= self.grippers[arm]
    with self.sensor_locker:
      #pos= self.dxlg_cmd2pos(gripper.Position())
      pos= self.mikata.State()['position'][-1]
      pos= self.dxlg_cmd2pos(self.mikata.invconv_pos['gripper_joint_5'](pos))
    return pos

  '''Get fingertip offset in meter.
    The fingertip trajectory of Robotiq gripper has a round shape.
    This function gives the offset from the opening posture.
      pos: Gripper position to get the offset. None: Current position.
      arm: arm id, or None (==currarm).'''
  def FingertipOffset(self, pos=None, arm=None):
    return 0.0

