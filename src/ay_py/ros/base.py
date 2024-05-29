#! /usr/bin/env python
#ROS basic tools.
import rospy
import actionlib as al
import geometry_msgs.msg
import trajectory_msgs.msg
import actionlib_msgs.msg
import control_msgs.msg
import dynamic_reconfigure.client
import tf

from ..core.util import *

'''Exception class for ROS operation (e.g. FK, IK).
    ROSError.Kind: 'fk','ik','ctrl',etc. '''
class ROSError(Exception):
  def __init__(self, kind, msg):
    self.Kind= kind
    self.Msg= msg
  def __str__(self):
    return 'ROSError({kind},{msg})'.format(kind=repr(self.Kind), msg=repr(self.Msg))
  def __repr__(self):
    return 'ROSError({kind},{msg})'.format(kind=repr(self.Kind), msg=repr(self.Msg))

ACTC_STATE_TO_STR= {getattr(actionlib_msgs.msg.GoalStatus,key):key for key in ('PENDING', 'ACTIVE', 'RECALLED', 'REJECTED', 'PREEMPTED', 'ABORTED', 'SUCCEEDED', 'LOST')}
ACTC_RESULT_TO_STR= {getattr(control_msgs.msg.FollowJointTrajectoryResult,key):key for key in ('GOAL_TOLERANCE_VIOLATED', 'INVALID_GOAL', 'INVALID_JOINTS', 'OLD_HEADER_TIMESTAMP', 'PATH_TOLERANCE_VIOLATED', 'SUCCESSFUL')}

'''Block the execution of action client.
  act_client: action client that is executing an action.
  blocking: False: not block, True: wait until action ends, 'time': wait until duration.
  duration: duration of the action in seconds.
  accuracy: accuracy to check duration.
  timeout_offset: offset from duration for timeout in seconds (timeout=duration+offset).
'''
def BlockAction(act_client, blocking, duration, accuracy=0.02, timeout_offset=1.5):
  if blocking==False:  return
  if blocking=='time':
    end_time= rospy.Time.now() + rospy.Duration(duration)
    dt= duration*accuracy
    while rospy.Time.now() < end_time:
      if act_client.get_state()==actionlib_msgs.msg.GoalStatus.SUCCEEDED:
        #CPrint(4,'BlockAction: act_client ends with state==SUCCEEDED')
        break
      if act_client.get_state() not in (actionlib_msgs.msg.GoalStatus.PENDING, actionlib_msgs.msg.GoalStatus.ACTIVE):
        #CPrint(4,'BlockAction: act_client state is not normal: [{}, {}].'.format(act_client.get_state(), ACTC_STATE_TO_STR[act_client.get_state()]))
        raise ROSError('ctrl','BlockAction: act_client state is not normal: [{}, {}].'.format(act_client.get_state(), ACTC_STATE_TO_STR[act_client.get_state()]))
      time.sleep(dt)
    return
  if blocking==True:
    if not act_client.wait_for_result(timeout=rospy.Duration(duration+timeout_offset)):
      raise ROSError('ctrl','BlockAction: act_client.wait_for_result finished anomaly: [state:{}/{}].'.format(act_client.get_state(), ACTC_STATE_TO_STR[act_client.get_state()]))
    res= act_client.get_result()
    if res is None:
      raise ROSError('ctrl','BlockAction: act_client.wait_for_result could not get a result within timeout (duration+timeout_offset={}s) [state:{}/{}].'.format(duration+timeout_offset, act_client.get_state(), ACTC_STATE_TO_STR[act_client.get_state()]))
    if res.error_code!=0:  #cf. control_msgs/FollowJointTrajectoryActionResult
      #CPrint(4,'BlockAction: act_client finished anomaly: [{}:{},{}].'.format(res.error_code,ACTC_RESULT_TO_STR[res.error_code],res.error_string))
      raise ROSError('ctrl','BlockAction: act_client finished anomaly: [{}:{},{}].'.format(res.error_code,ACTC_RESULT_TO_STR[res.error_code],res.error_string))
    if act_client.get_state()!=actionlib_msgs.msg.GoalStatus.SUCCEEDED:
      #CPrint(4,'BlockAction: act_client state is not succeeded: [{}, {}].'.format(act_client.get_state(), ACTC_STATE_TO_STR[act_client.get_state()]))
      raise ROSError('ctrl','BlockAction: act_client state is not succeeded: [{}, {}].'.format(act_client.get_state(), ACTC_STATE_TO_STR[act_client.get_state()]))
    return
  raise Exception('BlockAction: invalid blocking type: %r'%blocking)


def SetupServiceProxy(name, srv_type, persistent=False, time_out=None):
  print 'Waiting for %s... (t/o: %r)' % (name, time_out)
  try:
    rospy.wait_for_service(name, time_out)
  except rospy.exceptions.ROSException as e:
    print 'Failed to connect the service %s' % name
    print '  Error:',str(e)
    return None
  srvp= rospy.ServiceProxy(name, srv_type, persistent=persistent)
  return srvp


'''Setup an instance of SimpleActionClient.
  e.g.
    SimpleActionClient(name, act_type, time_out=5.0, num_wait=None)
      Waiting for [name]... (up to 5.0s)
      Waiting for [name]... (up to 5.0s)
      (until connection is established)
    SimpleActionClient(name, act_type, time_out=5.0)
      Waiting for [name]... (up to 5.0s)
      (return None if connection is not established)
'''
def SetupSimpleActionClient(name, act_type, time_out=None, num_wait=1):
  actc= al.SimpleActionClient(name, act_type)
  if time_out is None:  time_out= rospy.Duration()
  elif not isinstance(time_out, rospy.Duration):  time_out= rospy.Duration(time_out)
  while num_wait is None or num_wait>0:
    print 'Waiting for %s... (t/o: %r, #: %r)' % (name, time_out.to_sec(), num_wait)
    if actc.wait_for_server(time_out):
      return actc
    num_wait-= 1
  print 'Failed to connect the action service %s' % name


def SetupDynamicReconfigureClient(name, time_out=None):
  print 'Waiting for %s... (t/o: %r)' % (name, time_out)
  try:
    client= dynamic_reconfigure.client.Client(name, timeout=time_out)
    return client
  except rospy.exceptions.ROSException as e:
    print 'Failed to connect the dynamic_reconfigure.client %s' % name
    print '  Error:',str(e)
    return None


def TopicArrives(port_name, port_type, time_out=5.0):
  try:
    res= rospy.wait_for_message(port_name, port_type, time_out)
    return res is not None
  except rospy.ROSException:
    return False


#Convert p to geometry_msgs/Point
def PToGPoint(p):
  point= geometry_msgs.msg.Point()
  point.x= p[0]
  point.y= p[1]
  point.z= p[2]
  return point

#Convert p to geometry_msgs/Vector3
def PToGVector3(p):
  vec= geometry_msgs.msg.Vector3()
  vec.x= p[0]
  vec.y= p[1]
  vec.z= p[2]
  return vec

#Convert x to geometry_msgs/Pose
def XToGPose(x):
  pose= geometry_msgs.msg.Pose()
  pose.position.x= x[0]
  pose.position.y= x[1]
  pose.position.z= x[2]
  pose.orientation.x= x[3]
  pose.orientation.y= x[4]
  pose.orientation.z= x[5]
  pose.orientation.w= x[6]
  return pose

#Convert geometry_msgs/Pose to x
def GPoseToX(pose):
  x= [0]*7
  x[0]= pose.position.x
  x[1]= pose.position.y
  x[2]= pose.position.z
  x[3]= pose.orientation.x
  x[4]= pose.orientation.y
  x[5]= pose.orientation.z
  x[6]= pose.orientation.w
  return x

#Convert x to geometry_msgs/Transform
def XToGTransform(x):
  tfm= geometry_msgs.msg.Transform()
  tfm.translation.x= x[0]
  tfm.translation.y= x[1]
  tfm.translation.z= x[2]
  tfm.rotation.x= x[3]
  tfm.rotation.y= x[4]
  tfm.rotation.z= x[5]
  tfm.rotation.w= x[6]
  return tfm

#Convert x to geometry_msgs/Quaternion
def QToGQuaternion(x):
  rot= geometry_msgs.msg.Quaternion()
  rot.x= x[0]
  rot.y= x[1]
  rot.z= x[2]
  rot.w= x[3]
  return rot

#Assign v (list of 3 elements) to [d.x,d.y,d.z]
def VecToXYZ(v, d):
  d.x,d.y,d.z= v

#Return [d.x,d.y,d.z]
def XYZToVec(d):
  return [d.x,d.y,d.z]


'''Support function to generate trajectory_msgs/JointTrajectoryPoint.
    q: Joint positions, t: Time from start, dq: Joint velocities.'''
def ROSGetJTP(q,t,dq=None):
  jp= trajectory_msgs.msg.JointTrajectoryPoint()
  jp.positions= q
  jp.time_from_start= rospy.Duration(t)
  if dq is not None:  jp.velocities= dq
  return jp

'''Get trajectory_msgs/JointTrajectory from a joint angle trajectory.
  joint_names: joint names.
  q_traj: joint angle trajectory [q0,...,qD]*N.
  t_traj: corresponding times in seconds from start [t1,t2,...,tN].
  dq_traj: corresponding velocity trajectory [dq0,...,dqD]*N. '''
def ToROSTrajectory(joint_names, q_traj, t_traj, dq_traj=None):
  assert(len(q_traj)==len(t_traj))
  if dq_traj is not None:  (len(dq_traj)==len(t_traj))
  traj= trajectory_msgs.msg.JointTrajectory()
  traj.joint_names= joint_names
  if dq_traj is not None:
    traj.points= [ROSGetJTP(q,t,dq) for q,t,dq in zip(q_traj, t_traj, dq_traj)]
  else:
    traj.points= [ROSGetJTP(q,t) for q,t in zip(q_traj, t_traj)]
  traj.header.stamp= rospy.Time.now()
  return traj

#Make a list of JointTolerance(pos, vel, acc) for all joint_names.
def GetSimpleJointTol(joint_names, pos, vel, acc):
  return [control_msgs.msg.JointTolerance(jname, pos, vel, acc) for jname in joint_names]


'''One time tf listening.  Useful when obtaining static transformation.
  trg_frame, src_frame: Target and source frame ids. '''
def TfOnce(trg_frame, src_frame, time_out=4.0):
  listener= tf.TransformListener()
  try:
    listener.waitForTransform(trg_frame, src_frame, rospy.Time(), rospy.Duration(time_out))
    (trans,rot)= listener.lookupTransform(trg_frame, src_frame, rospy.Time(0))
  except Exception:
    CPrint(0,'transformation not found:', trg_frame, src_frame)
    return None
  trans_rot= trans+rot
  return trans_rot


'''Basic ROS utility class.'''
class TROSUtil(object):
  def __init__(self):
    self._is_initialized= False

    debug= False
    #Container for Publishers
    self.pub= TContainer(debug=debug)
    #Container for Subscribers
    self.sub= TContainer(debug=debug)
    #Container for Service proxies
    self.srvp= TContainer(debug=debug)
    #Container for Service
    self.srv= TContainer(debug=debug)
    #Container for SimpleActionClient
    self.actc= TContainer(debug=debug)
    #Container for SimpleActionServer
    self.acts= TContainer(debug=debug)
    #Container for dynamic reconfigure client
    self.dynconfig= TContainer(debug=debug)

  def __del__(self):
    self.Cleanup()

  def Init(self):
    self._is_initialized= False
    '''Example:
    res= []
    ra= lambda r: res.append(r)

    ra(self.AddPub(...))
    ra(self.AddPub(...))
    ra(self.AddSrvP(...))

    if False not in res:  self._is_initialized= True
    return self._is_initialized
    '''

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency

    for k in self.acts.keys():
      print 'Delete action server %r...' % k,
      del self.acts[k]
      print 'ok'

    for k in self.sub.keys():
      print 'Stop subscribing %r...' % k,
      self.sub[k].unregister()
      del self.sub[k]
      print 'ok'

    for k in self.pub.keys():
      print 'Stop publishing %r...' % k,
      self.pub[k].publish()
      self.pub[k].unregister()
      del self.pub[k]
      print 'ok'

    for k in self.srvp.keys():
      print 'Delete service proxy %r...' % k,
      del self.srvp[k]
      print 'ok'

    for k in self.srv.keys():
      print 'Shutdown service %r...' % k,
      self.srv[k].shutdown()
      del self.srv[k]
      print 'ok'

    for k in self.actc.keys():
      print 'Delete action client %r...' % k,
      del self.actc[k]
      print 'ok'

    for k in self.dynconfig.keys():
      print 'Delete dynamic reconfigure client %r...' % k,
      del self.dynconfig[k]
      print 'ok'

  def IsInitialized(self):
    return self._is_initialized

  #Add a subscriber.
  def AddSub(self, name, port_name, port_type, call_back, callback_args=None, queue_size=None, buff_size=65536, tcp_nodelay=False):
    if name not in self.sub:
      self.sub[name]= rospy.Subscriber(port_name, port_type, call_back, callback_args, queue_size, buff_size, tcp_nodelay)
    return True

  #Add a subscriber and wait for the first message.
  def AddSubW(self, name, port_name, port_type, call_back, callback_args=None, queue_size=None, buff_size=65536, tcp_nodelay=False, time_out=None):
    if name not in self.sub:
      try:
        print 'Waiting for %s... (t/o: %r)' % (port_name, time_out)
        msg= rospy.wait_for_message(port_name, port_type, time_out)
        if callback_args is None:  call_back(msg)
        else:                      call_back(msg,callback_args)
        self.sub[name]= rospy.Subscriber(port_name, port_type, call_back, callback_args, queue_size, buff_size, tcp_nodelay)
      except rospy.ROSException:
        raise Exception('Failed to receive the message: {port_name}'.format(port_name=port_name))
    return True

  #Add a publisher.
  def AddPub(self, name, port_name, port_type, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=10):
    if name not in self.pub:
      self.pub[name]= rospy.Publisher(port_name, port_type, subscriber_listener, tcp_nodelay, latch, headers, queue_size)
    return True

  #Add a service proxy.
  def AddSrvP(self, name, port_name, port_type, persistent=False, time_out=None):
    if name not in self.srvp:
      srvp= SetupServiceProxy(port_name, port_type, persistent, time_out)
      if srvp is None:  return False
      else:  self.srvp[name]= srvp
    return True

  #Add a service.
  def AddSrv(self, name, port_name, port_type, handler, buff_size=65536, error_handler=None):
    if name not in self.srv:
      self.srv[name]= rospy.Service(port_name, port_type, handler, buff_size=buff_size, error_handler=error_handler)
    return True

  #Add an action client.
  def AddActC(self, name, port_name, port_type, time_out=None, num_wait=1):
    if name not in self.actc:
      actc= SetupSimpleActionClient(port_name, port_type, time_out, num_wait)
      if actc is None:  return False
      else:  self.actc[name]= actc
    return True

  #Add an action server.
  def AddActS(self, name, port_name, port_type, auto_start=True):
    if name not in self.acts:
      acts= actionlib.SimpleActionServer(port_name, port_type, execute_cb=execute_cb, auto_start=auto_start)
      if acts is None:  return False
      else:  self.acts[name]= acts
    return True

  #Add a dynamic reconfigure client.
  def AddDynConfig(self, name, node_name, time_out=None):
    if name not in self.dynconfig:
      client= SetupDynamicReconfigureClient(node_name, time_out=time_out)
      if client is None:  return False
      else:  self.dynconfig[name]= client
    return True

  def DelSub(self, name):
    if name in self.sub:
      self.sub[name].unregister()
      del self.sub[name]

  def DelPub(self, name):
    if name in self.pub:
      self.pub[name].publish()
      self.pub[name].unregister()
      del self.pub[name]

  def DelSrvP(self, name):
    if name in self.srvp:
      del self.srvp[name]

  def DelSrv(self, name):
    if name in self.srv:
      self.srv[name].shutdown()
      del self.srv[name]

  def DelActC(self, name):
    if name in self.actc:
      del self.actc[name]

  def DelActS(self, name):
    if name in self.acts:
      del self.acts[name]

  def DelDynConfig(self, name):
    if name in self.dynconfig:
      del self.dynconfig[name]

