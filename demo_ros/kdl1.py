#!/usr/bin/python
#\file    kdl1.py
#\brief   certain python script
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Nov.01, 2017

from _path import *
from ay_py.core import *
from ay_py.ros import *
import sys

if __name__=='__main__':
  base_link= sys.argv[1] if len(sys.argv)>1 else 'base_link'
  end_link= sys.argv[2] if len(sys.argv)>2 else 'link_t'
  print 'Testing TKinematics (using robot_description).'
  print 'Before executing this script, upload URDF to the robot_description parameter.'
  print 'e.g.'
  print '  rosparam load `rospack find motoman_sia10f_support`/urdf/sia10f.urdf robot_description'
  kin= TKinematics(base_link=base_link, end_link=end_link)
  kin.print_robot_description()

  dof= len(kin.joint_names)
  q0= [0.0]*dof
  angles= {joint:q0[j] for j,joint in enumerate(kin.joint_names)}  #Deserialize
  x0= kin.forward_position_kinematics(angles)
  print 'q0=',q0
  print 'x0= FK(q0)=',x0

  import random
  q1= [3.0*(random.random()-0.5) for j in range(dof)]
  angles= {joint:q1[j] for j,joint in enumerate(kin.joint_names)}  #Deserialize
  x1= kin.forward_position_kinematics(angles)
  print 'q1=',q1
  print 'x1= FK(q1)=',x1

  seed= [0.0]*dof
  #seed= [3.0*(random.random()-0.5) for j in range(dof)]
  q2= kin.inverse_kinematics(x1[:3], x1[3:], seed=seed, maxiter=2000, eps=1.0e-4)  #, maxiter=500, eps=1.0e-6
  print 'q2= IK(x1)=',q2
  if q2 is not None:
    angles= {joint:q2[j] for j,joint in enumerate(kin.joint_names)}  #Deserialize
    x2= kin.forward_position_kinematics(angles)
    print 'x2= FK(q2)=',x2
    print 'x2==x1?', np.allclose(x2,x1)
    print '|x2-x1|=',np.linalg.norm(x2-x1)
  else:
    print 'Failed to solve IK.'
