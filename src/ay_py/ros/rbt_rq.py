#! /usr/bin/env python
#Robot controller for Robotiq.
from const import *
#if ROS_ROBOT not in ('ANY','Baxter','Baxter_SIM','BaxterN','RobotiqNB','Motoman','Motoman_SIM'):
  #raise ImportError('Stop importing: ROS_ROBOT does not have Robotiq')
#if ROS_DISTRO not in ('groovy','hydro','indigo'):  return
try:
  import roslib
  import rospkg
  roslib.load_manifest('robotiq_c_model_control')
  import robotiq_c_model_control.msg as robotiq_msgs
except rospkg.common.ResourceNotFound:
  print 'Module not found: robotiq_c_model_control'
except ImportError:
  print 'Cannot import: robotiq_c_model_control.msg'
finally:
  #Define TRobotiq for class structure.
  if 'robotiq_msgs' not in globals():
    class TRobotiq(TGripper2F1):
      pass
    raise ImportError('Stop importing: robotiq_c_model_control')

import rospy
import time

from robot import TGripper2F1


'''Robotiq Gripper utility class'''
class TRobotiq(TGripper2F1):
  def __init__(self, cmd_topic='/rq1/command', st_topic='/rq1/status'):
    super(TRobotiq,self).__init__()
    self.status= None
    self.SensorCallback= None
    self.CmdTopic= cmd_topic
    self.StTopic= st_topic

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    ra(self.AddPub('grip', self.CmdTopic, robotiq_msgs.CModel_robot_output))
    ra(self.AddSub('grip', self.StTopic, robotiq_msgs.CModel_robot_input, self.SensorHandler))

    rospy.sleep(0.2)
    self.Activate()

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    self.Deactivate()
    super(TRobotiq,self).Cleanup()

  '''Answer to a query q by {True,False}. e.g. Is('Robotiq').'''
  def Is(self, q):
    if q in ('Robotiq',):  return True
    return super(TRobotiq,self).Is(q)

  def SensorHandler(self,msg):
    self.status= msg
    if self.SensorCallback is not None:
      self.SensorCallback(self.status)

  @staticmethod
  def PrintStatus(st):
    print 'Flags(ACT,GTO,STA,OBJ,FLT):',st.gACT,st.gGTO,st.gSTA,st.gOBJ,st.gFLT,
    print 'State(PR,PO,CU):',st.gPR,st.gPO,st.gCU

  def Status(self):
    return self.status

  '''Get current position.'''
  def Position(self):
    return self.status.gPO

  def Current(self):
    return self.status.gCU

  '''Activate gripper (torque is enabled).
    Return success or not.'''
  def Activate(self):
    cmd= robotiq_msgs.CModel_robot_output();
    cmd.rACT= 1
    cmd.rGTO= 1
    cmd.rSP= 255  #SPeed
    cmd.rFR= 150  #FoRce
    self.pub.grip.publish(cmd)

  '''Deactivate gripper (torque is disabled).
    Return success or not.'''
  def Deactivate(self):
    cmd= robotiq_msgs.CModel_robot_output();
    cmd.rACT= 0
    self.pub.grip.publish(cmd)

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
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Move(self, pos, max_effort, speed=255, blocking=False):
    pos= max(0,min(255,int(pos)))
    cmd= robotiq_msgs.CModel_robot_output();
    cmd.rACT= 1
    cmd.rGTO= 1
    cmd.rPR= pos  #Position Request
    cmd.rSP= speed
    cmd.rFR= max_effort
    self.pub.grip.publish(cmd)
    if blocking:
      while pos!=self.status.gPR and not rospy.is_shutdown():
        #self.PrintStatus(self.status)
        rospy.sleep(0.001)
      prev_PO= None
      CMAX= 500
      counter= CMAX
      while not (self.status.gGTO==0 or self.status.gOBJ==3) and not rospy.is_shutdown():
        #self.PrintStatus(self.status)
        if self.status.gPO==prev_PO:  counter-= 1
        else:  counter= CMAX
        if counter==0:  break
        prev_PO= self.status.gPO
        rospy.sleep(0.001)
      #self.Stop()

  '''Stop the gripper motion. '''
  def Stop(self):
    cmd= robotiq_msgs.CModel_robot_output();
    cmd.rACT= 1
    cmd.rGTO= 0
    self.pub.grip.publish(cmd)



