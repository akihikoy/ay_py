#!/usr/bin/python
#\file    viz2.py
#\brief   certain python script
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Aug.12, 2022
from _path import *
from ay_py.core import *
from ay_py.ros import *
import rospy
import numpy as np
from viz1 import GetPoly, GetRandomX


if __name__=='__main__':
  rospy.init_node('viz1')
  viz= TSimpleVisualizerArray(rospy.Duration(3), name_space='visualizer_demo', frame='base_link')
  viz.DeleteAllMarkers()
  viz.Reset()

  t_start= rospy.Time.now()
  rate_adjuster= rospy.Rate(30)
  dt_sum= 0.0
  dt_cnt= 0
  while not rospy.is_shutdown():
    t_0= time.time()
    N= np.random.randint(0,100)
    #viz.DeleteAllMarkers()
    for mid in range(N+2, viz.curr_id):
      viz.DeleteMarker(mid)
    t= (rospy.Time.now()-t_start).to_sec()
    mid= 0
    mid= viz.AddPolygon(GetPoly(n=2,offset=[0,0,np.sin(t)]), scale=[0.005,0.005], alpha=0.5, rgb=viz.ICol(1), mid=mid)
    mid= viz.AddPolygon(GetPoly(n=2,offset=[0.5,0,0],radius=[0.0,np.sin(t),np.sin(t)]), scale=[0.005,0.005], alpha=0.5, rgb=viz.ICol(2), mid=mid)
    for i in range(N):
      scale= np.random.uniform(0.1,0.5), np.random.uniform(0.01,0.05)
      col= np.random.uniform(0,1,3)
      mid= viz.AddArrow(GetRandomX(), scale=[scale[0],scale[1],scale[1]], rgb=col, alpha=0.8, mid=mid)
    viz.Publish()
    dt= time.time()-t_0
    dt_sum+= dt
    dt_cnt+= 1
    print 'N, Computational time [ms]:',N,dt*1000
    rate_adjuster.sleep()

  print 'Average computational time [ms]:',dt_sum/dt_cnt*1000


