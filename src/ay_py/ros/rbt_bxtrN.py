#! /usr/bin/env python
#Robot controller for BaxterN.
from const import *
if ROS_ROBOT not in ('ANY','BaxterN'):
  raise ImportError('Stop importing: ROS_ROBOT is not BaxterN')
#if ROS_DISTRO not in ('groovy','hydro','indigo'):  return

from rbt_bxtr import *
from rbt_rq import TRobotiq

'''Robot control class for Baxter.'''
class TRobotBaxterN(TRobotBaxter):
  def __init__(self, name='BaxterN'):
    super(TRobotBaxterN,self).__init__(name=name)

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    self.default_face= 'config/baymax.jpg'
    self.ChangeFace(self.default_face)

    self.limbs= [None,None]
    self.limbs[RIGHT]= baxter_interface.Limb(LRTostr(RIGHT))
    self.limbs[LEFT]=  baxter_interface.Limb(LRTostr(LEFT))

    self.kin= [None,None]
    self.kin[RIGHT]= baxter_kinematics(LRTostr(RIGHT))
    self.kin[LEFT]=  baxter_kinematics(LRTostr(LEFT))  #tip_link=_gripper(default),_wrist,_hand

    self.head= baxter_interface.Head()

    ra(self.AddActC('r_traj', '/robot/limb/right/follow_joint_trajectory',
                    control_msgs.msg.FollowJointTrajectoryAction, time_out=3.0))
    ra(self.AddActC('l_traj', '/robot/limb/left/follow_joint_trajectory',
                    control_msgs.msg.FollowJointTrajectoryAction, time_out=3.0))

    self.robotiq= [None,None]
    self.robotiq[RIGHT]= TRobotiqN('gripper_right')
    self.robotiq[LEFT]= TRobotiqN('gripper_left')
    self.grippers= [self.robotiq[RIGHT], self.robotiq[LEFT]]

    print 'Enabling the robot...'
    baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()

    print 'Initializing Robotiq grippers...'
    ra(self.robotiq[RIGHT].Init())
    ra(self.robotiq[LEFT].Init())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('Baxter','Baxter_SIM','BaxterN'):  return True
    return super(TRobotBaxterN,self).Is(q)


'''Robotiq Gripper (NihonBinary package/USB) utility class'''
class TRobotiqN(TRobotiq):
  def __init__(self, dev_name='gripper_left'):
    super(TRobotiqN,self).__init__()
    self.status= {'pos':None, 'curr':None}
    self.SensorCallback= None
    self.DevName= dev_name

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    for key,topic in (('pos','position'), ('spd','speed'), ('force','force')):
      ra(self.AddPub('cmd_'+key, '/robotiq/{dev_name}/command/{topic}'.format(dev_name=self.DevName,topic=topic),
                     std_msgs.msg.Int32))

    for key,topic in (('pos','position'), ('curr','current')):
      ra(self.AddSub('st_'+key, '/robotiq/{dev_name}/current/{topic}'.format(dev_name=self.DevName,topic=topic),
                     std_msgs.msg.Int32, lambda msg,key=key:self.SensorHandler(key,msg)))

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    super(TRobotiqN,self).Cleanup()

  def SensorHandler(self,key,msg):
    self.status[key]= msg.data
    if self.SensorCallback is not None:
      self.SensorCallback(self.status)

  def Status(self):
    return self.status

  '''Get current position.'''
  def Position(self):
    return self.status['pos']

  def Current(self):
    return self.status['curr']

  '''Open a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Open(self, blocking=False):
    self.Move(pos=0, max_effort=100, blocking=blocking)

  '''Close a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Close(self, blocking=False):
    self.Move(pos=255, max_effort=100, blocking=blocking)

  '''Control a gripper.
    pos: target position; 0 (open), 255 (close).
    max_effort: maximum effort to control; 0~50 (weak), 200 (strong), 255 (maximum).
    speed: speed of the movement; 0 (minimum), 255 (maximum).
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.
      WARNING: max_effort is not implemented.  '''
  def Move(self, pos, max_effort=None, speed=255, blocking=False):
    pos= max(0,min(255,int(pos)))
    self.pub.cmd_pos.publish(std_msgs.msg.Int32(pos))
    if blocking:
      prev_PO= None
      CMAX= 500
      counter= CMAX
      tol= 20
      while abs(pos-self.status['pos'])>tol and not rospy.is_shutdown():
        if self.status['pos']==prev_PO:  counter-= 1
        else:  counter= CMAX
        if counter==0:  break
        prev_PO= self.status['pos']
        rospy.sleep(0.001)
