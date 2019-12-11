#! /usr/bin/env python
#Common robot interface for PR2 and Baxter.
from base import *
from const import *
from ..core.geom import *
from ..core.traj import *


'''Common robot control class for multi-arm robots such as PR2 and Baxter.
  Pose: Cartesian pose (position x,y,z and orientation in quaternion qx,qy,qz,qw).
  End-effector: wrist or something.
  Robot frame: base link, torso link, or something.
'''
class TMultiArmRobot(TROSUtil):
  def __init__(self, name):
    super(TMultiArmRobot,self).__init__()

    self._name= name
    self.currarm= 0
    self.is_sim= False  #Whether the robot is simulated.

    #Thread locker for self.currarm:
    self.currarm_locker= threading.RLock()

    #NOTE: the sub classes should use the following lockers during controlling/sensing.
    #Thread locker for control:
    self.control_locker= threading.RLock()
    #Thread locker for sensor:
    self.sensor_locker= threading.RLock()

  def __del__(self):
    self.Cleanup()
    print '%s: bye.'%self.Name

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    #NOTE: assign True into self._is_initialized after init.

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency

    #Check the thread lockers status:
    print 'Count of currarm_locker:',self.currarm_locker._RLock__count

    #Check the thread lockers status:
    print 'Count of control_locker:',self.control_locker._RLock__count
    print 'Count of sensor_locker:',self.sensor_locker._RLock__count

    super(TMultiArmRobot,self).Cleanup()

  '''Configure a state validity checker.'''
  def ConfigureSVC(self, c):
    pass

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('sim','SIM'):  return self.is_sim
    if q == self._name:  return True
    return False

  def IsInitialized(self):
    return self._is_initialized

  @property
  def Name(self):
    return self._name

  @property
  def NumArms(self):
    return 0

  @property
  def Arm(self):
    with self.currarm_locker:
      arm= self.currarm
    return arm

  @Arm.setter
  def Arm(self, arm):
    with self.currarm_locker:
      self.currarm= arm

  def ArmStr(self, arm=None):
    if arm is None:  arm= self.Arm
    return IDToStr(arm)

  def ArmStrS(self, arm=None):
    if arm is None:  arm= self.Arm
    return IDToStr(arm)

  def ArmStrs(self, arm=None):
    if arm is None:  arm= self.Arm
    return IDTostr(arm)

  @property
  def BaseFrame(self):
    pass

  '''End link of an arm.'''
  def EndLink(self, arm=None):
    pass

  '''Names of joints of an arm.'''
  def JointNames(self, arm=None):
    pass

  def DoF(self, arm=None):
    return 7

  '''Return limits (lower, upper) of joint angles.
    arm: arm id, or None (==currarm). '''
  def JointLimits(self, arm=None):
    pass

  '''Return limits of joint angular velocity.
    arm: arm id, or None (==currarm). '''
  def JointVelLimits(self, arm=None):
    pass

  '''End effector of an arm.'''
  def EndEff(self, arm=None):
    pass

  '''Return joint angles of an arm (list of floats).
    arm: arm id, or None (==currarm). '''
  def Q(self, arm=None):
    pass

  '''Return joint velocities of an arm (list of floats).
    arm: arm id, or None (==currarm). '''
  def DQ(self, arm=None):
    pass

  '''Compute a forward kinematics of an arm.
  Return self.EndLink(arm) pose on self.BaseFrame.
    return: x, res;  x: pose (list of floats; None if failure), res: FK status.
    arm: arm id, or None (==currarm).
    q: list of joint angles, or None (==self.Q(arm)).
    x_ext: a local pose on self.EndLink(arm) frame.
      If not None, the returned pose is x_ext on self.BaseFrame.
    with_st: whether return FK status. '''
  def FK(self, q=None, x_ext=None, arm=None, with_st=False):
    pass

  '''Compute a Jacobian matrix of an arm.
  Return J of self.EndLink(arm).
    return: J, res;  J: Jacobian (numpy.matrix; None if failure), res: status.
    arm: arm id, or None (==currarm).
    q: list of joint angles, or None (==self.Q(arm)).
    x_ext: a local pose (i.e. offset) on self.EndLink(arm) frame.
      If not None, we do not consider an offset.
    with_st: whether return the solver status. '''
  def J(self, q=None, x_ext=None, arm=None, with_st=False):
    pass

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
    pass

  '''Transform a Cartesian trajectory to joint angle trajectory.
    x_traj: pose sequence [x1, x2, ...].
    start_angles: joint angles used for initial pose of first IK.  '''
  def XTrajToQTraj(self, x_traj, x_ext=None, start_angles=None, arm=None):
    if arm is None:  arm= self.Arm
    if start_angles is None:  start_angles= self.Q(arm)
    q_traj= XTrajToQTraj(lambda x,q_start: self.IK(x, x_ext=x_ext, start_angles=q_start, arm=arm),
                         x_traj, start_angles=start_angles)
    if q_traj is None:
      raise ROSError('ik','XTrajToQTraj: IK failed')
    return q_traj

  '''Follow a joint angle trajectory.
    arm: arm id, or None (==currarm).
    q_traj: joint angle trajectory [q0,...,qD]*N.
    t_traj: corresponding times in seconds from start [t1,t2,...,tN].
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN. '''
  def FollowQTraj(self, q_traj, t_traj, arm=None, blocking=False):
    raise NotImplementedError('FollowQTraj is not implemented for:',self.Name)

  '''Follow a self.EndLink(arm)-pose trajectory.
    arm: arm id, or None (==currarm).
    x_traj: self.EndLink(arm)-pose trajectory [x,y,z,qx,qy,qz,qw]*N.
    t_traj: corresponding times in seconds from start [t1,t2,...,tN].
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.
    x_ext: a local pose on the self.EndLink(arm) frame.
      If not None, the final joint angles q satisfies self.FK(q,x_ext,arm)==x_trg. '''
  def FollowXTraj(self, x_traj, t_traj, x_ext=None, arm=None, blocking=False):
    if arm is None:  arm= self.Arm
    q_traj= self.XTrajToQTraj(x_traj, x_ext=x_ext, arm=arm)
    self.FollowQTraj(q_traj, t_traj, arm=arm, blocking=blocking)

  '''Control an arm to the target joint angles.
    arm: arm id, or None (==currarm).
    q_trg: target joint angles.
    dt: duration time in seconds.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN. '''
  def MoveToQ(self, q_trg, dt=4.0, arm=None, blocking=False):
    self.FollowQTraj(q_traj=[q_trg], t_traj=[dt], arm=arm, blocking=blocking)

  '''Control an arm to the target self.EndLink(arm) pose.
    arm: arm id, or None (==currarm).
    x_trg: target pose.
    dt: duration time in seconds.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.
    x_ext: a local pose on the self.EndLink(arm) frame.
      If not None, the final joint angles q satisfies self.FK(q,x_ext,arm)==x_trg. '''
  def MoveToX(self, x_trg, dt=4.0, x_ext=None, arm=None, blocking=False):
    self.FollowXTraj(x_traj=[x_trg], t_traj=[dt], x_ext=x_ext, arm=arm, blocking=blocking)

  '''Control an arm to the target self.EndLink(arm) pose with a linearly interpolated trajectory.
    arm: arm id, or None (==currarm).
    x_trg: target pose.
    dt: duration time in seconds (this is modified when limit_vel=True and acc_phase>1).
    inum: number of interpolation points.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.
    x_ext: a local pose on the self.EndLink(arm) frame.
      If not None, the final joint angles q satisfies self.FK(q,x_ext,arm)==x_trg.
    limit_vel: If True, joint angular velocities are limited to JointVelLimits.
    acc_phase: Number of points in the acceleration and deceleration phases (>=1). '''
  def MoveToXI(self, x_trg, dt=4.0, x_ext=None, inum=30, arm=None, blocking=False, limit_vel=True, acc_phase=9):
    if arm is None:  arm= self.Arm

    x_curr= self.FK(q=None, x_ext=x_ext, arm=arm)
    x_traj= XInterpolation(x_curr,x_trg,inum)
    t_traj= TimeTraj(dt,inum)

    #self.FollowXTraj(x_traj, t_traj, x_ext=x_ext, arm=arm, blocking=blocking)
    q_curr= self.Q(arm)
    q_traj= self.XTrajToQTraj(x_traj, x_ext=x_ext, start_angles=q_curr, arm=arm)
    if limit_vel:
      LimitQTrajVel(q_start=q_curr, q_traj=q_traj, t_traj=t_traj, qvel_limits=self.JointVelLimits(arm), acc_phase=acc_phase)
    self.FollowQTraj(q_traj, t_traj, arm=arm, blocking=blocking)

  '''Stop motion such as FollowQTraj.
    arm: arm id, or None (==currarm). '''
  def StopMotion(self, arm=None):
    raise NotImplementedError('StopMotion is not implemented for:',self.Name)


  '''Open a gripper.
    arm: arm id, or None (==currarm).
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def OpenGripper(self, arm=None, blocking=False):
    pass

  '''Close a gripper.
    arm: arm id, or None (==currarm).
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def CloseGripper(self, arm=None, blocking=False):
    pass

  '''Get a fingertip height offset in meter.
    The fingertip trajectory of some grippers has a rounded shape.
    This function gives the offset from the highest (longest) point (= closed fingertip position),
    and the offset is always negative.
    NOTE: In the previous versions (before 2019-12-10), this offset was from the opened fingertip position.
      pos: Gripper position to get the offset. None: Current position.
      arm: arm id, or None (==currarm).'''
  def FingertipOffset(self, pos=None, arm=None):
    raise NotImplementedError('FingertipOffset is not implemented for:',self.Name)


'''Common robot control class for dual-arm robots such as PR2 and Baxter.'''
class TDualArmRobot(TMultiArmRobot):
  def __init__(self, name):
    super(TDualArmRobot,self).__init__(name)

  @property
  def NumArms(self):
    return 2

  def ArmStr(self, arm=None):
    if arm is None:  arm= self.Arm
    return LRToStr(arm)

  def ArmStrS(self, arm=None):
    if arm is None:  arm= self.Arm
    return LRToStrS(arm)

  def ArmStrs(self, arm=None):
    if arm is None:  arm= self.Arm
    return LRToStrs(arm)


class TFakeRobot(TMultiArmRobot):
  def __init__(self):
    super(TFakeRobot,self).__init__(name='NoRobot')

  '''Answer to a query q by {True,False}. e.g. Is('Robotiq').'''
  def Is(self, q):
    if q in ('NoRobot'):  return True
    return False



'''2 finger 1 DoF gripper interface'''
class TGripper2F1(TROSUtil):
  def __init__(self):
    super(TGripper2F1,self).__init__()

  '''Answer to a query q by {True,False}. e.g. Is('Robotiq').'''
  def Is(self, q):
    if q in ('Gripper','Gripper2F1'):  return True
    return False

  '''Range of gripper position.'''
  def PosRange(self):
    return None

  '''Get a fingertip height offset in meter.
    The fingertip trajectory of some grippers has a rounded shape.
    This function gives the offset from the highest (longest) point (= closed fingertip position),
    and the offset is always negative.
      pos: Gripper position to get the offset. '''
  def FingertipOffset(self, pos):
    return 0.0

  '''Get current position.'''
  def Position(self):
    return None

  '''Activate gripper (torque is enabled).
    Return success or not.'''
  def Activate(self):
    return False

  '''Deactivate gripper (torque is disabled).
    Return success or not.'''
  def Deactivate(self):
    return False

  '''Open a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Open(self, blocking=False):
    pass

  '''Close a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Close(self, blocking=False):
    pass

  '''Control a gripper.
    pos: target position.
    max_effort: maximum effort to control.
    speed: speed of the movement.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Move(self, pos, max_effort=None, speed=None, blocking=False):
    pass

  '''Stop the gripper motion. '''
  def Stop(self):
    pass


'''Fake (dummy) gripper.
FIXME This class should be defined as a subclass of general (not 2 finger) gripper class.'''
class TFakeGripper(TGripper2F1):
  def __init__(self):
    super(TFakeGripper,self).__init__()

  '''Answer to a query q by {True,False}. e.g. Is('Robotiq').'''
  def Is(self, q):
    if q in ('NoGripper'):  return True
    return False


'''Simulated 2 finger 1 DoF gripper'''
class TSimGripper2F1(TGripper2F1):
  def __init__(self,is_=('Gripper','Gripper2F1'),pos_range=[0.0, 1.0],pos_open=None,pos_close=None):
    super(TSimGripper2F1,self).__init__()

    #Configurable parameters:
    self.is_= is_
    self.pos_range= pos_range
    self.pos_open= pos_range[1] if pos_open is None else pos_open
    self.pos_close= pos_range[0] if pos_close is None else pos_close

    #Current position:
    self.pos= 0.0

  '''Answer to a query q by {True,False}. e.g. Is('Robotiq').'''
  def Is(self, q):
    if q in self.is_:  return True
    if q in ('SimGripper','SimGripper2F1'):  return True
    if q in ('sim','SIM'):  return True
    return super(TSimGripper2F1,self).Is(q)

  '''Range of gripper position.'''
  def PosRange(self):
    return self.pos_range

  '''Get current position.'''
  def Position(self):
    return self.pos

  '''Activate gripper (torque is enabled).
    Return success or not.'''
  def Activate(self):
    return True

  '''Deactivate gripper (torque is disabled).
    Return success or not.'''
  def Deactivate(self):
    return True

  '''Open a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Open(self, blocking=False):
    self.Move(self.pos_open)

  '''Close a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Close(self, blocking=False):
    self.Move(self.pos_close)

  '''Control a gripper.
    pos: target position.
    max_effort: maximum effort to control.
    speed: speed of the movement.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Move(self, pos, max_effort=None, speed=None, blocking=False):
    self.pos= pos
    if self.pos<self.pos_range[0]:  self.pos= self.pos_range[0]
    if self.pos>self.pos_range[1]:  self.pos= self.pos_range[1]
