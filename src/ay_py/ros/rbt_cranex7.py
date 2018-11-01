#! /usr/bin/env python
#Robot controller for Crane-X7 Arm (using ay_util/mikata_driver).

from rbt_mikata2 import *

'''CraneX7 Gripper utility class'''
class TCraneX7Gripper(TMikataGripper2):
  def __init__(self):
    super(TCraneX7Gripper,self).__init__()
    self.joint_name= 'crane_x7_gripper_finger_a_joint'

#'''Gripper utility class of TDummyMikata'''
#class TDummyCraneX7Gripper(TDummyMikataGripper):
  #def __init__(self):
    #super(TDummyCraneX7Gripper,self).__init__()
    #self.joint_name= 'crane_x7_gripper_finger_a_joint'

'''Robot control class for single CraneX7 Arm whose controller is the ay_util/mikata_driver node.'''
class TRobotCraneX7(TRobotMikata2):
  #FIXME: is_sim=True is not implemented yet.
  def __init__(self, name='CraneX7', is_sim=False):
    super(TRobotCraneX7,self).__init__(name=name)

    self.is_sim= is_sim

    self.joint_names= [[]]
    #self.joint_names[0]= rospy.get_param('controller_joint_names')
    self.joint_names[0]= ['crane_x7_shoulder_fixed_part_pan_joint',
        'crane_x7_shoulder_revolute_part_tilt_joint',
        'crane_x7_upper_arm_revolute_part_twist_joint',
        'crane_x7_upper_arm_revolute_part_rotate_joint',
        'crane_x7_lower_arm_fixed_part_joint',
        'crane_x7_lower_arm_revolute_part_joint',
        'crane_x7_wrist_joint']  # 'crane_x7_gripper_finger_a_joint'

    #CraneX7 all link names:
    #obtained from ay_py/demo_ros/kdl1.py base_link crane_x7_gripper_base_link (URDF link names)
    self.links= {}
    self.links['base']= ['base_link', 'crane_x7_mounting_plate_link',]
    self.links['r_arm']= ['crane_x7_shoulder_fixed_part_link', 'crane_x7_shoulder_revolute_part_link',
                          'crane_x7_upper_arm_fixed_part_link', 'crane_x7_upper_arm_fixed_part_joint_cover_l_link',
                          'crane_x7_upper_arm_fixed_part_joint_cover_r_link', 'crane_x7_upper_arm_revolute_part_link',
                          'crane_x7_lower_arm_fixed_part_link', 'crane_x7_lower_arm_fixed_part_joint_cover_l_link',
                          'crane_x7_lower_arm_fixed_part_joint_cover_r_link', 'crane_x7_lower_arm_revolute_part_link',
                          'crane_x7_lower_arm_revolute_part_logo_text_l_link', 'crane_x7_lower_arm_revolute_part_logo_symbol_l_link',
                          'crane_x7_lower_arm_revolute_part_logo_text_r_link', 'crane_x7_lower_arm_revolute_part_logo_symbol_r_link',
                          'crane_x7_wrist_link', 'crane_x7_gripper_base_link']
    self.links['r_gripper']= ['crane_x7_gripper_finger_a_link', 'crane_x7_gripper_finger_b_link']
    self.links['robot']= self.links['base'] + self.links['r_arm'] + self.links['r_gripper']

    self.port_locker= threading.RLock()

    #Dummy Mikata object (compatible with TRobotMikata).
    self.mikata= TContainer()
    #DxlRead
    #DxlWrite
    self.mikata.port_locker= self.port_locker
    self.mikata.EnableTorque= self.EnableTorque
    self.mikata.DisableTorque= self.DisableTorque
    self.mikata.Reboot= self.Reboot
    self.mikata.MoveTo= self.MoveTo
    self.mikata.SetCurrent= self.SetCurrent
    self.mikata.SetVelocity= self.SetVelocity
    self.mikata.SetPWM= self.SetPWM

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    self.kin= [None]
    self.kin[0]= TKinematics(base_link='base_link',end_link='crane_x7_gripper_base_link')

    ra(self.AddSrvP('robot_io', '/cranex7_driver/robot_io',
                    ay_util_msgs.srv.MikataArmIO, persistent=False, time_out=3.0))

    ra(self.AddActC('traj', '/follow_joint_trajectory',
                    control_msgs.msg.FollowJointTrajectoryAction, time_out=3.0))

    ra(self.AddSub('joint_states', '/joint_states', sensor_msgs.msg.JointState, self.JointStatesCallback))

    if not self.is_sim:
      self.mikata_gripper= TCraneX7Gripper()
    else:
      self.mikata_gripper= TDummyCraneX7Gripper()
    self.grippers= [self.mikata_gripper]

    self.mikata.EnableTorque()
    ra(self.mikata_gripper.Init(self))

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    super(TRobotCraneX7,self).Cleanup()

  '''Configure a state validity checker.'''
  def ConfigureSVC(self, c):
    c.JointNames= copy.deepcopy(self.joint_names)
    c.Links= copy.deepcopy(self.links)
    c.PaddingLinks= []
    c.PaddingValues= [0.002]*len(c.PaddingLinks)
    c.DefaultBaseFrame= 'base_link'
    c.HandLinkToGrasp[0]= 'crane_x7_gripper_base_link'
    c.IgnoredLinksInGrasp[0]= []

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('CraneX7','CraneX7_SIM'):  return True
    return super(TRobotCraneX7,self).Is(q)

  @property
  def BaseFrame(self):
    return 'base_link'

  '''End link of an arm.'''
  def EndLink(self, arm=None):
    return 'crane_x7_gripper_base_link'

  def DoF(self, arm=None):
    return 7

  '''Return limits of joint angular velocity.
    arm: arm id, or None (==currarm). '''
  def JointVelLimits(self, arm=None):
    #FIXME: Should be adjusted for CraneX7
    return [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]

  def JointStatesCallback(self, msg):
    with self.sensor_locker:
      self.x_curr= msg
      self.q_curr= self.x_curr.position[:7]
      self.dq_curr= self.x_curr.velocity[:7]

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

