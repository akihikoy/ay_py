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
    return False

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

  @property
  def ArmStr(self):
    return 'Arm'+('A','B','C','D','E','F','G')[self.Arm]

  @property
  def ArmStrS(self):
    return ('A','B','C','D','E','F','G')[self.Arm]

  @property
  def ArmStrs(self):
    return ('a','b','c','d','e','f','g')[self.Arm]

  @property
  def BaseFrame(self):
    pass

  '''End link of an arm.'''
  def EndLink(self, arm):
    pass

  '''Names of joints of an arm.'''
  def JointNames(self, arm):
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
  def EndEff(self, arm):
    pass

  '''Return joint angles of an arm.
    arm: arm id, or None (==currarm). '''
  def Q(self, arm=None):
    pass

  '''Return joint velocities of an arm.
    arm: arm id, or None (==currarm). '''
  def DQ(self, arm=None):
    pass

  '''Compute a forward kinematics of an arm.
  Return self.EndLink(arm) pose on self.BaseFrame.
    return: x, res;  x: pose (None if failure), res: FK status.
    arm: arm id, or None (==currarm).
    q: list of joint angles, or None (==self.Q(arm)).
    x_ext: a local pose on self.EndLink(arm) frame.
      If not None, the returned pose is x_ext on self.BaseFrame.
    with_st: whether return FK status. '''
  def FK(self, q=None, x_ext=None, arm=None, with_st=False):
    pass

  '''Compute a Jacobian matrix of an arm.
  Return J of self.EndLink(arm).
    return: J, res;  J: Jacobian (None if failure), res: status.
    arm: arm id, or None (==currarm).
    q: list of joint angles, or None (==self.Q(arm)).
    x_ext: a local pose (i.e. offset) on self.EndLink(arm) frame.
      If not None, we do not consider an offset.
    with_st: whether return the solver status. '''
  def J(self, q=None, x_ext=None, arm=None, with_st=False):
    pass

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
    pass

  '''Follow a joint angle trajectory.
    arm: arm id, or None (==currarm).
    q_traj: joint angle trajectory [q0,...,qD]*N.
    t_traj: corresponding times in seconds from start [t1,t2,...,tN].
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN. '''
  def FollowQTraj(self, q_traj, t_traj, arm=None, blocking=False):
    pass

  '''Follow a self.EndLink(arm)-pose trajectory.
    arm: arm id, or None (==currarm).
    x_traj: self.EndLink(arm)-pose trajectory [x,y,z,qx,qy,qz,qw]*N.
    t_traj: corresponding times in seconds from start [t1,t2,...,tN].
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.
    x_ext: a local pose on the self.EndLink(arm) frame.
      If not None, the final joint angles q satisfies self.FK(q,x_ext,arm)==x_trg. '''
  def FollowXTraj(self, x_traj, t_traj, x_ext=None, arm=None, blocking=False):
    if arm is None:  arm= self.Arm

    q_curr= self.Q(arm)
    q_traj= XTrajToQTraj(lambda x,q_start: self.IK(x, x_ext=x_ext, start_angles=q_start, arm=arm),
                         x_traj, start_angles=q_curr)
    if q_traj is None:
      raise ROSError('ik','FollowXTraj: IK failed')

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
    dt: duration time in seconds.
    inum: number of interpolation points.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.
    x_ext: a local pose on the self.EndLink(arm) frame.
      If not None, the final joint angles q satisfies self.FK(q,x_ext,arm)==x_trg. '''
  def MoveToXI(self, x_trg, dt=4.0, x_ext=None, inum=30, arm=None, blocking=False):
    if arm is None:  arm= self.Arm

    x_curr= self.FK(q=None, x_ext=x_ext, arm=arm)
    x_traj= XInterpolation(x_curr,x_trg,inum)
    t_traj= TimeTraj(dt,inum)

    self.FollowXTraj(x_traj, t_traj, x_ext=x_ext, arm=arm, blocking=blocking)

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


'''Common robot control class for dual-arm robots such as PR2 and Baxter.'''
class TDualArmRobot(TMultiArmRobot):
  def __init__(self, name):
    super(TDualArmRobot,self).__init__(name)

  @property
  def NumArms(self):
    return 2

  @property
  def ArmStr(self):
    return LRToStr(self.Arm)

  @property
  def ArmStrS(self):
    return LRToStrS(self.Arm)

  @property
  def ArmStrs(self):
    return LRToStrs(self.Arm)


class TFakeRobot(TDualArmRobot):
  def __init__(self):
    super(TFakeRobot,self).__init__(name='NoRobot')



'''2 finger 1 DoF gripper interface'''
class TGripper2F1(TROSUtil):
  def __init__(self):
    super(TGripper2F1,self).__init__()

  '''Answer to a query q by {True,False}. e.g. Is('Robotiq').'''
  def Is(self, q):
    if q in ('Gripper','Gripper2F1'):  return True
    return False

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

