#! /usr/bin/env python
#Basic tools (trajectory).
from __future__ import print_function
from __future__ import absolute_import
import numpy as np
import numpy.linalg as la
import math
import random
import copy
from .util import *
from .geom import *

#Get a sequence of times, from 0 to dt including inum points (0 is not included).
def TimeTraj(dt, inum):
  return FRange1(0.0, dt, inum)[1:]

'''
Generate t_traj (time trajectory) from x_traj (trajectory of position+quaternion) and speed.
t_traj[0] is zero.
'''
def TTrajFromXTraj(x_traj, linear_speed, angular_speed):
  def duration(x0,x1):
    diff= DiffX(x1,x0)
    #print 'duration:',la.norm(diff[:3])/linear_speed,la.norm(diff[3:])/angular_speed
    return max(la.norm(diff[:3])/linear_speed,la.norm(diff[3:])/angular_speed)
  t_traj= [0.0]
  for x0,x1 in zip(x_traj[:-1],x_traj[1:]):
    t_traj.append(t_traj[-1]+duration(x0,x1))
  return t_traj

'''
Generate t_traj (time trajectory) from x_traj (trajectory of position+quaternion) and speed and acceleration.
t_traj[0] is zero.
'''
def TTrajFromXTraj2(x_traj, linear_speed, angular_speed, linear_acceleration):
  lin_ang_ratio= linear_speed / angular_speed
  def dist(x0,x1):
    diff= DiffX(x1,x0)
    return la.norm([la.norm(diff[:3]), lin_ang_ratio*la.norm(diff[3:])])
  d_traj= [0.0]
  for x0,x1 in zip(x_traj[:-1],x_traj[1:]):
    d_traj.append(d_traj[-1]+dist(x0,x1))
  d_T3= d_traj[-1]
  if linear_acceleration*d_T3 > linear_speed*linear_speed:
    T1= linear_speed / linear_acceleration
    T2= d_T3 / linear_speed
    T3= T2 + T1
    d_T1= linear_speed*linear_speed / (2.0*linear_acceleration)
    d_T2= d_T3 - d_T1
    def t_from_start(d):
      if d<d_T1:    return np.sqrt(2.0*d/linear_acceleration)
      elif d<d_T2:  return d/linear_speed + linear_speed/(2.0*linear_acceleration)
      else:         return T3 - np.sqrt(2.0*max(0.0,d_T3-d)/linear_acceleration)
    t_traj= map(t_from_start, d_traj)
    #print [('T1' if d1<d_T1 else ('T2' if d1<d_T2 else 'T3'), t1-t0) for t0,t1,d0,d1 in zip(t_traj[:-1],t_traj[1:],d_traj[:-1],d_traj[1:])]
  else:  #There is no constant-velocity phase.
    T1= np.sqrt(d_T3/linear_acceleration)
    T3= 2.0*T1
    d_T1= d_T3 / 2.0
    def t_from_start(d):
      if d<d_T1:    return np.sqrt(2.0*d/linear_acceleration)
      else:         return T3 - np.sqrt(2.0*max(0.0,d_T3-d)/linear_acceleration)
    t_traj= map(t_from_start, d_traj)
    #print [('_T1' if d1<d_T1 else '_T3', t1-t0) for t0,t1,d0,d1 in zip(t_traj[:-1],t_traj[1:],d_traj[:-1],d_traj[1:])]
  return t_traj

'''
Generate t_traj (time trajectory) from a set of trajectories x_trajs in different spaces, and speed and acceleration.
t_traj[0] is zero.
x_trajs: List of trajectories.  Each trajectory should have the same length, i.e. map(len,x_trajs)=[len(x_trajs[0])]*len(x_trajs).
speed_ratios: Speed ratios of x_trajs[1:] components over the speed of x_trajs[0].
  Speed of x_trajs[0]: speed.
  Speed of x_trajs[i]: speed*speed_ratios[i-1].
  len(x_trajs)==len(speed_ratios)+1.
speed, acceleration: Speed and acceleration of x_trajs[0].
dist_criteria: List of criteria to calculate distances for each points on trajectories.
  dist_criteria should have the same length as x_trajs, and each element can be:
  'E': Euclidean distance (L2 norm).
  'Q': Quaternion distance (DiffQ).
'''
def TTrajFromXTraj3(x_trajs, speed, speed_ratios, acceleration, dist_criteria):
  if len(x_trajs)==0:  return []
  speed_ratios= [1.0]+list(speed_ratios)
  def dist(x0,x1,dist_criterion):
    if dist_criterion=='E':  return Dist(x0,x1)
    if dist_criterion=='Q':  return np.linalg.norm(DiffQ(x0,x1))
  d_traj= [0.0]
  for k in range(len(x_trajs[0])-1):
    dists= [dist(x_traj[k],x_traj[k+1],dc)/sr for x_traj,sr,dc in zip(x_trajs,speed_ratios,dist_criteria)]
    d_traj.append(d_traj[-1]+max(dists))
    #d_traj.append(d_traj[-1]+np.linalg.norm(dists))
  d_T3= d_traj[-1]
  if acceleration*d_T3 > speed*speed:
    T1= speed / acceleration
    T2= d_T3 / speed
    T3= T2 + T1
    d_T1= speed*speed / (2.0*acceleration)
    d_T2= d_T3 - d_T1
    def t_from_start(d):
      if d<d_T1:    return np.sqrt(2.0*d/acceleration)
      elif d<d_T2:  return d/speed + speed/(2.0*acceleration)
      else:         return T3 - np.sqrt(2.0*max(0.0,d_T3-d)/acceleration)
    t_traj= map(t_from_start, d_traj)
    #print [('T1' if d1<d_T1 else ('T2' if d1<d_T2 else 'T3'), t1-t0) for t0,t1,d0,d1 in zip(t_traj[:-1],t_traj[1:],d_traj[:-1],d_traj[1:])]
  else:  #There is no constant-velocity phase.
    T1= np.sqrt(d_T3/acceleration)
    T3= 2.0*T1
    d_T1= d_T3 / 2.0
    def t_from_start(d):
      if d<d_T1:    return np.sqrt(2.0*d/acceleration)
      else:         return T3 - np.sqrt(2.0*max(0.0,d_T3-d)/acceleration)
    t_traj= map(t_from_start, d_traj)
    #print [('_T1' if d1<d_T1 else '_T3', t1-t0) for t0,t1,d0,d1 in zip(t_traj[:-1],t_traj[1:],d_traj[:-1],d_traj[1:])]
  return t_traj

#Remove radian jumping in a joint angle trajectory.
def SmoothQTraj(q_traj):
  if len(q_traj)==0:  return
  q_prev= np.array(q_traj[0])
  q_offset= np.array([0]*len(q_prev))
  for q in q_traj:
    q_diff= np.array(q) - q_prev
    for d in range(len(q_prev)):
      if q_diff[d]<-math.pi:  q_offset[d]+=1
      elif q_diff[d]>math.pi:  q_offset[d]-=1
    q_prev= copy.deepcopy(q)
    q[:]= q+q_offset*2.0*math.pi

'''Limit joint velocities with acceleration and deceleration phases.
Sequence of times t_traj will be directly modified.
  q_start: joint angles at t=0.
  q_traj: joint angle trajectory [q0,...,qD]*N.
  t_traj: corresponding times in seconds from start [t1,t2,...,tN].
  qvel_limits: limit of velocities.
  acc_phase: number of points in acceleration and deceleration phases (acc_phase>=1).
  Note: limit of velocities at i-th point in the acceleration phase are given by:
    qvel_limits*i/acc_phase '''
def LimitQTrajVel(q_start, q_traj, t_traj, qvel_limits, acc_phase=9):
  assert(len(q_traj)==len(t_traj))
  if len(q_traj)==0:  return

  def sub_proc(qt, i, t_offset, t_prev, q_prev, vel_limits):
    q= qt[0][i]
    qt[1][i]+= t_offset
    qd= [AngleMod1(q[d]-q_prev[d]) / (qt[1][i]-t_prev) for d in range(len(q))]
    s= max([abs(qd[d])/vel_limits[d] for d in range(len(qd))])
    if s>1.0:
      diff= (qt[1][i]-t_prev)*float(s-1.0)
      t_offset+= diff
      qt[1][i]+= diff
    #CPrint(2,'s=',s,qt[1][i]-t_prev,qd)
    return t_offset, qt[1][i], q

  t_offset= 0.0
  t_prev= 0.0
  q_prev= q_start
  i_start= 0
  if t_traj[0]<1.0e-6 and Dist(q_start,q_traj[0])<1.0e-6:
    i_start= 1
  qt= (q_traj,t_traj)

  i_term= acc_phase if len(q_traj)>2*acc_phase else len(q_traj)/2
  i_middle= len(q_traj)-2*i_term
  dv= 1.0/float(i_term)
  for i in range(i_start,i_term):
    vel_limits= [v*dv*float(i+1) for v in qvel_limits]
    t_offset,t_prev,q_prev= sub_proc(qt, i, t_offset, t_prev, q_prev, vel_limits)
  for i in range(i_middle):
    t_offset,t_prev,q_prev= sub_proc(qt, i_term+i, t_offset, t_prev, q_prev, qvel_limits)
  for i in range(i_term):
    vel_limits= [v*dv*float(i_term-i) for v in qvel_limits]
    t_offset,t_prev,q_prev= sub_proc(qt, i_term+i_middle+i, t_offset, t_prev, q_prev, vel_limits)

#DEPRECATED:
#  IKTrajCheck was deprecated due to its inaccuracy.
#  Use CheckXQTrajValidity instead.
##Check the velocity consistency between x_traj and q_traj
##  where x_traj is target Cartesian trajectory,
##  q_traj is corresponding joint angle trajectory solved by IK.
##If the norm of inconsistency is greater than err_thresh,
##  return None immediately, otherwise, return the maximum of the norms.
#def IKTrajCheck(x_traj, q_traj, err_thresh=0.01):
  ##Compute velocities
  #xd_traj= [Vec(x_traj[i])-Vec(x_traj[i-1]) for i in range(1,len(x_traj))]
  #qd_traj= [Vec(q_traj[i])-Vec(q_traj[i-1]) for i in range(1,len(q_traj))]
  #max_err= 0.0
  #for i in range(3,len(xd_traj)):
    ##Roughly estimate the next Cartesian velocity
    #xd_est= np.dot(Vec([xd_traj[i-3],xd_traj[i-2],xd_traj[i-1]]).T,
                   #np.dot(la.pinv(Vec([qd_traj[i-3],qd_traj[i-2],qd_traj[i-1]]).T),
                          #qd_traj[i]) )
    #err= la.norm(xd_traj[i]-xd_est)
    #if err>err_thresh:  return None
    #if err>max_err:  max_err= err
  #return max_err

'''Check the validity of the corresponding joint and Cartesian trajectories.
Validity: For corresponding trajectories q_traj and x_traj (f_fk(q_traj)==x_traj),
  the interpolated trajectories q_traj_int and x_traj_int also should satisfy
  f_fk(q_traj_int)~x_traj_int where we allow the position and orientation errors dp_lim, dq_lim.
Return True if the trajectories are valid.
f_fK: Forward kinematics (x=f_fk(q)).
N_int: Number of interpolation.
dp_lim, dq_lim: Max position and orientation errors (meters, radians).
TODO:FIXME: Especially the mapping of FK takes long compt. time (map(f_fk, q_traj_int)).
'''
def CheckXQTrajValidity(q_traj, x_traj, f_fk, dp_lim=np.inf, dq_lim=0.1, N_int=5):
  if len(q_traj)<=1:  return True
  x_traj_int= [list(x_traj[0])] + sum([XInterpolation(x1,x2,N_int) for x1,x2 in zip(x_traj[:-1],x_traj[1:])], [])
  q_traj_int= [q_traj[0]] + sum([(q1+(np.array(q2)-q1)*np.linspace(0,1,N_int+1)[1:].reshape(-1,1)).tolist() for q1,q2 in zip(q_traj[:-1],q_traj[1:])], [])
  x_traj_est= map(f_fk, q_traj_int)
  x_diff= map(lambda d: np.linalg.norm(d[:3])<dp_lim and np.linalg.norm(d[3:])<dq_lim, [DiffX(xi,xe) for xi,xe in zip(x_traj_int,x_traj_est)])
  return all(x_diff)

#Return the interpolation from x1 to x2 with N points
#p1 is not included
def XInterpolation(x1,x2,N):
  p1,R1= XToPosRot(x1)
  p2,R2= XToPosRot(x2)
  dp= (p2-p1)/float(N)
  trans_R= np.dot(R2,R1.T)
  w= InvRodrigues(trans_R)
  traj=[]
  for t in range(N):
    R= np.dot(Rodrigues(float(t+1)/float(N)*w),R1)
    p1= p1+dp
    traj.append(PosRotToX(p1,R))
  return traj

#Return the interpolation from q1 to q2 with N points
#q1 is not included
def QInterpolation(q1,q2,N):
  R1= QToRot(q1)
  R2= QToRot(q2)
  trans_R= np.dot(R2,R1.T)
  w= InvRodrigues(trans_R)
  traj=[]
  for t in range(N):
    R= np.dot(Rodrigues(float(t+1)/float(N)*w),R1)
    traj.append(RotToQ(R))
  return traj

'''Transform a Cartesian trajectory to joint angle trajectory.
  func_ik: IK function (x, q_start).
  x_traj: pose sequence [x1, x2, ...].
  start_angles: joint angles used for initial pose of first IK.  '''
def XTrajToQTraj(func_ik, x_traj, start_angles):
  N= len(x_traj)
  q_prev= start_angles
  q_traj= None
  for x,n in zip(x_traj, range(N)):
    q= func_ik(x, q_prev)
    if q is None:  return None
    if q_traj is None:  q_traj= [[0.0]*len(q) for i in range(N)]
    q_traj[n][:]= q
    q_traj[n][:]= q
    q_prev= q
  SmoothQTraj(q_traj)
  return q_traj


#Generate a cubic Hermite spline from a key points.
#Key points: [[t0,x0],[t1,x1],[t2,x2],...].
class TCubicHermiteSpline:
  class TKeyPoint:
    T= 0.0  #Input
    X= 0.0  #Output
    M= 0.0  #Gradient
    def __str__(self):
      return '['+str(self.T)+', '+str(self.X)+', '+str(self.M)+']'

  class TParam: pass

  def __init__(self):
    self.idx_prev= 0
    self.Param= self.TParam()

  def FindIdx(self, t, idx_prev=0):
    idx= idx_prev
    if idx>=len(self.KeyPts): idx= len(self.KeyPts)-1
    while idx+1<len(self.KeyPts) and t>self.KeyPts[idx+1].T:  idx+=1
    while idx>=0 and t<self.KeyPts[idx].T:  idx-=1
    return idx

  #Return interpolated value at t.
  #with_tan: If True, both x and dx/dt are returned.
  #with_dd: If True, x,dx/dt,ddx/ddt are returned (with_tan is ignored).
  def Evaluate(self, t, with_tan=False, with_dd=False):
    idx= self.FindIdx(t,self.idx_prev)
    if abs(t-self.KeyPts[-1].T)<1.0e-6:  idx= len(self.KeyPts)-2
    if idx<0 or idx>=len(self.KeyPts)-1:
      print('WARNING: Given t= %f is out of the key points (index: %i)' % (t,idx))
      if idx<0:
        idx= 0
        t= self.KeyPts[0].T
      else:
        idx= len(self.KeyPts)-2
        t= self.KeyPts[-1].T

    h00= lambda t: t*t*(2.0*t-3.0)+1.0
    h10= lambda t: t*(t*(t-2.0)+1.0)
    h01= lambda t: t*t*(-2.0*t+3.0)
    h11= lambda t: t*t*(t-1.0)

    self.idx_prev= idx
    p0= self.KeyPts[idx]
    p1= self.KeyPts[idx+1]
    dT= p1.T-p0.T
    tr= (t-p0.T) / dT
    x= h00(tr)*p0.X + h10(tr)*dT*p0.M + h01(tr)*p1.X + h11(tr)*dT*p1.M
    if not with_tan and not with_dd:  return x

    dh00= lambda t: t*(6.0*t-6.0)
    dh10= lambda t: t*(3.0*t-4.0)+1.0
    dh01= lambda t: t*(-6.0*t+6.0)
    dh11= lambda t: t*(3.0*t-2.0)
    dx= (dh00(tr)*p0.X + dh10(tr)*dT*p0.M + dh01(tr)*p1.X + dh11(tr)*dT*p1.M) / dT
    if not with_dd:  return x,dx

    ddh00= lambda t: 12*t - 6
    ddh10= lambda t: 6*t - 4
    ddh01= lambda t: -12*t + 6
    ddh11= lambda t: 6*t - 2
    ddx= (ddh00(tr)*p0.X + ddh10(tr)*dT*p0.M + ddh01(tr)*p1.X + ddh11(tr)*dT*p1.M) / (dT*dT)
    return x,dx,ddx

  #Compute a phase information (n, tp) for a cyclic spline curve.
  #n:  n-th occurrence of the base wave
  #tp: phase (time in the base wave)
  def PhaseInfo(self, t):
    t0= self.KeyPts[0].T
    te= self.KeyPts[-1].T
    T= te-t0
    mod= Mod(t-t0,T)
    tp= t0+mod  #Phase
    n= (t-t0-mod)/T
    return n, tp

  #Return interpolated value at t (cyclic version).
  #pi: Phase information.
  #with_tan: If True, both x and dx/dt are returned.
  #with_dd: If True, x,dx/dt,ddx/ddt are returned (with_tan is ignored).
  def EvaluateC(self, t, pi=None, with_tan=False, with_dd=False):
    if pi is None:
      n, tp= self.PhaseInfo(t)
    else:
      n, tp= pi
    if with_dd:  x,dx,ddx= self.Evaluate(tp, with_dd=with_dd)
    else:
      if with_tan:  x,dx= self.Evaluate(tp, with_tan=with_tan)
      else:        x= self.Evaluate(tp)
    x= x + n*(self.KeyPts[-1].X - self.KeyPts[0].X)
    return (x,dx,ddx) if with_dd else ( (x,dx) if with_tan else x )

  #data= [[t0,x0],[t1,x1],[t2,x2],...]
  FINITE_DIFF=0  #Tangent method: finite difference method
  CARDINAL=1  #Tangent method: Cardinal spline (c is used)
  ZERO= 0  #End tangent: zero
  GRAD= 1  #End tangent: gradient (m is used)
  CYCLIC= 2  #End tangent: treating data as cyclic (KeyPts[-1] and KeyPts[0] are considered as an identical point)
  def Initialize(self, data, tan_method=CARDINAL, end_tan=GRAD, c=0.0, m=1.0):
    if data != None:
      self.KeyPts= [self.TKeyPoint() for i in range(len(data))]
      for idx in range(len(data)):
        self.KeyPts[idx].T= data[idx][0]
        self.KeyPts[idx].X= data[idx][1]

    #Store parameters for future use / remind parameters if not given
    if tan_method is None:  tan_method= self.Param.TanMethod
    else:                   self.Param.TanMethod= tan_method
    if end_tan is None:  end_tan= self.Param.EndTan
    else:                self.Param.EndTan= end_tan
    if c is None:  c= self.Param.C
    else:          self.Param.C= c
    if m is None:  m= self.Param.M
    else:          self.Param.M= m

    grad= lambda idx1,idx2: (self.KeyPts[idx2].X-self.KeyPts[idx1].X)/(self.KeyPts[idx2].T-self.KeyPts[idx1].T)

    if tan_method == self.FINITE_DIFF:
      for idx in range(1,len(self.KeyPts)-1):
        self.KeyPts[idx].M= 0.5*grad(idx,idx+1) + 0.5*grad(idx-1,idx)
    elif tan_method == self.CARDINAL:
      for idx in range(1,len(self.KeyPts)-1):
        self.KeyPts[idx].M= (1.0-c)*grad(idx-1,idx+1)

    if end_tan == self.ZERO:
      self.KeyPts[0].M= 0.0
      self.KeyPts[-1].M= 0.0
    elif end_tan == self.GRAD:
      self.KeyPts[0].M= m*grad(0,1)
      self.KeyPts[-1].M= m*grad(-2,-1)
    elif end_tan == self.CYCLIC:
      if tan_method == self.FINITE_DIFF:
        grad_p1= grad(0,1)
        grad_n1= grad(-2,-1)
        M= 0.5*grad_p1 + 0.5*grad_n1
        self.KeyPts[0].M= M
        self.KeyPts[-1].M= M
      elif tan_method == self.CARDINAL:
        T= self.KeyPts[-1].T - self.KeyPts[0].T
        X= self.KeyPts[-1].X - self.KeyPts[0].X
        grad_2= (X+self.KeyPts[1].X-self.KeyPts[-2].X)/(T+self.KeyPts[1].T-self.KeyPts[-2].T)
        M= (1.0-c)*grad_2
        self.KeyPts[0].M= M
        self.KeyPts[-1].M= M

  def Update(self):
    self.Initialize(data=None, tan_method=None, end_tan=None, c=None, m=None)



'''Convert joint angle trajectory to joint velocity trajectory.'''
def QTrajToDQTraj(q_traj, t_traj):
  dof= len(q_traj[0])

  #Modeling the trajectory with spline.
  splines= [TCubicHermiteSpline() for d in range(dof)]
  for d in range(len(splines)):
    data_d= [[t,q[d]] for q,t in zip(q_traj,t_traj)]
    splines[d].Initialize(data_d, tan_method=splines[d].CARDINAL, c=0.0, m=0.0)

  #NOTE: We don't have to make spline models as we just want velocities at key points.
  #  They can be obtained by computing tan_method, which will be more efficient.

  dq_traj= []
  for t in t_traj:
    dq= [splines[d].Evaluate(t,with_tan=True)[1] for d in range(dof)]
    dq_traj.append(dq)
  return dq_traj



#Modify the velocity of a given trajectory (base routine).
#t0: Current internal time maintaining a playing point of the trajectory
#v: Target speed
#traj: Function to map time t --> point x
#time_step: Control time step (actual value)
#T: Maximum internal time
#num_iter_l: Limit of iteration number (linear search)
#num_iter_e: Limit of iteration number (exponential search)
#norm: Function to compute norm of two vectors
#diff: Function to compute difference of two vectors
#is_samedir: Function to judge if two vectors have the same direction
def modify_traj_velocity_base(t0, v, traj, time_step, T, num_iter_l, num_iter_e, norm, diff, is_samedir):
  num_iter_l= int(num_iter_l)
  num_iter_e= int(num_iter_e)
  ddt= time_step/(float(num_iter_l)/5.0)
  v_dt= v*time_step
  x0= traj(t0)
  t1= t0
  x1= x0
  s1= None
  cross= False
  #print '-----------'
  while num_iter_l>0 and num_iter_e>0 and t1<T:
    t2= t1+ddt
    if t2>T:  t2= T
    x2= traj(t2)
    dx= norm(x2,x0)
    s2= diff(x2,x1)
    if s1 is None:  s1= s2
    #print t2,dx/time_step,s1,s2
    samedir= is_samedir(s1,s2)
    if dx<v_dt and samedir:
      t1= t2
      x1= x2
      num_iter_l-= 1
    else:
      if dx<v_dt and not samedir:
        #print 'cross#',t2,x2
        cross= True
        over_t2= t2
        over_x2= x2
      ddt*= 0.5
      num_iter_e-= 1
  if cross:
    #print 'cross',over_t2,over_x2,num_iter_l,num_iter_e
    t1= over_t2
    x1= over_x2
  #print t1,t1-t0, x1, norm(x1,x0)/time_step, v,num_iter_l, num_iter_e
  return t1, x1, norm(x1,x0)/time_step

#Modify the velocity of a given trajectory (1-d version).
#t0: Current internal time maintaining a playing point of the trajectory
#v: Target speed
#traj: Function to map time t --> point x
#time_step: Control time step (actual value)
#T: Maximum internal time
#num_iter_l: Limit of iteration number (linear search)
#num_iter_e: Limit of iteration number (exponential search)
def ModifyTrajVelocity(t0, v, traj, time_step, T, num_iter_l=50, num_iter_e=6):
  norm= lambda x2,x1: abs(x2-x1)
  diff= lambda x2,x1: x2-x1
  is_samedir= lambda s1,s2: Sign(s1)==Sign(s2)
  return modify_traj_velocity_base(t0, v, traj, time_step, T, num_iter_l, num_iter_e, norm, diff, is_samedir)

#Modify the velocity of a given trajectory (N-d version).
#t0: Current internal time maintaining a playing point of the trajectory
#v: Target speed
#trajs: List of functions to map time t --> point x
#time_step: Control time step (actual value)
#T: Maximum internal time
#num_iter_l: Limit of iteration number (linear search)
#num_iter_e: Limit of iteration number (exponential search)
def ModifyTrajVelocityV(t0, v, traj, time_step, T, num_iter_l=50, num_iter_e=6):
  diff= lambda x2,x1: [x2d-x1d for (x2d,x1d) in zip(x2,x1)]
  norm= lambda x2,x1: la.norm(diff(x2,x1))
  is_samedir= lambda s1,s2: np.dot(s1,s2) > 0
  return modify_traj_velocity_base(t0, v, traj, time_step, T, num_iter_l, num_iter_e, norm, diff, is_samedir)
