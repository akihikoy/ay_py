#! /usr/bin/env python
#Robot controller for Robotiq-No-Body.
from const import *
if ROS_ROBOT not in ('ANY','RobotiqNB'):
  raise ImportError('Stop importing: ROS_ROBOT is not RobotiqNB')
#if ROS_DISTRO not in ('groovy','hydro','indigo'):  return

from robot import TMultiArmRobot
from rbt_rq import TRobotiq

'''Robot control class for RobotiqNB.
  This is defined as a subclass of TMultiArmRobot,
  but actually it does not have a body (only Robotiq gripper).
  This virtual body is designed for a compatibility of programs.'''
class TRobotRobotiqNB(TMultiArmRobot):
  def __init__(self, name='RobotiqNB'):
    super(TRobotRobotiqNB,self).__init__(name=name)
    self.currarm= 0

    #Gripper command-position conversions.
    #rqg: Robotiq gripper.
    self.rqg_cmd2pos= lambda cmd: -0.00041*cmd+0.09249   #effective cmd in [12,230] ([0,255])
    self.rqg_pos2cmd= lambda pos: -(pos-0.09249)/0.00041 #pos in [0.0,0.0855] meter
    self.rqg_range= [0.0,0.0855]

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    self.robotiq= TRobotiq()  #Robotiq controller
    self.grippers= [self.robotiq, self.robotiq]

    print 'Initializing and activating Robotiq gripper...'
    ra(self.robotiq.Init())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    super(TRobotRobotiqNB,self).Cleanup()

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('RobotiqNB',):  return True
    return super(TRobotRobotiqNB,self).Is(q)

  @property
  def BaseFrame(self):
    return 'world'

  '''Return range of gripper.
    arm: arm id, or None (==currarm). '''
  def GripperRange(self, arm=None):
    return self.rqg_range

  '''End effector of an arm.'''
  def EndEff(self, arm):
    if arm is None:  arm= self.Arm
    return self.grippers[arm]

  #Dummy FK.
  def FK(self, q=None, x_ext=None, arm=None, with_st=False):
    x_res= [0,0,0, 0,0,0,1]
    return (x_res, True) if with_st else x_res

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

    gripper= self.grippers[arm]
    if isinstance(gripper, TRobotiq):
      clip= lambda c: max(0.0,min(255.0,c))
      cmd= clip(self.rqg_pos2cmd(pos))
      max_effort= clip(max_effort*(255.0/100.0))
      speed= clip(speed*(255.0/100.0))
      with self.control_locker:
        gripper.Move(cmd, max_effort, speed, blocking=blocking)

  '''Get a gripper position in meter.
    arm: arm id, or None (==currarm). '''
  def GripperPos(self, arm=None):
    if arm is None:  arm= self.Arm

    gripper= self.grippers[arm]
    if isinstance(gripper, TRobotiq):
      with self.sensor_locker:
        pos= self.rqg_cmd2pos(gripper.Position())
      return pos

  '''Get fingertip offset in meter.
    The fingertip trajectory of Robotiq gripper has a round shape.
    This function gives the offset from the opening posture.
      pos: Gripper position to get the offset. None: Current position.
      arm: arm id, or None (==currarm).'''
  def FingertipOffset(self, pos=None, arm=None):
    if arm is None:  arm= self.Arm
    gripper= self.grippers[arm]
    if isinstance(gripper, TRobotiq):
      if pos is None:  pos= self.GripperPos(arm)
      return -0.701*pos**3 - 2.229*pos**2 + 0.03*pos + 0.128 - 0.113

