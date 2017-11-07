#! /usr/bin/env python
#Basic tools (advanced stuff).
import numpy as np
import numpy.linalg as la
import math
import random
from scipy.spatial import ConvexHull as scipy_ConvexHull
from scipy.optimize import minimize as scipy_minimize
from util import *
from geom import *

#Compute the sensor pose x_sensor on the robot frame from data
#  marker_data: a sequence of pose vector [0-2]: position x,y,z, [3-6]: orientation qx,qy,qz,qw
#  gripper_data: corresponding gripper pose sequence (on the robot frame)
#  x_g2m: marker pose on the gripper's local frame
def CalibrateSensorPose(marker_data, gripper_data, x_g2m):
  assert(len(marker_data)==len(gripper_data))
  #Marker poses on the robot frame
  robot_marker_data= map(lambda x: Transform(x,x_g2m), gripper_data)
  #Sensor poses
  x_sensor_data= [TransformRightInv(robot_marker_data[d], marker_data[d]) for d in range(len(marker_data))]
  #Average sensor poses to get x_sensor
  x_sensor= AverageXData(x_sensor_data)
  #x_sensor= x_sensor_data[0]
  print '##--------------##'
  print 'la.norm(x_sensor[3:]):',la.norm(x_sensor[3:])
  print '##gripper_data[0]:',gripper_data[0]
  print '##robot_marker_data[0]:',robot_marker_data[0]
  print '##Transform(gripper_data[0],x_g2m):',Transform(gripper_data[0],x_g2m)
  print '##marker_data[0]:',marker_data[0]
  print '##x_sensor_data[0]:',x_sensor_data[0]
  print '##Transform(x_sensor_data[0],marker_data[0]):',Transform(x_sensor_data[0],marker_data[0])
  print '##Transform(x_sensor,marker_data[0]):',Transform(x_sensor,marker_data[0])
  print '##--------------##'
  #for i in range(len(x_sensor_data)):
    #x= x_sensor_data[i]
    #print x, rostf.euler_from_quaternion(x[3:7]), marker_data[i]
  print '##--------------##'
  #print x_sensor
  err= [0.0]*7
  for d in range(len(robot_marker_data)):
    err= [err[k]+abs(Transform(x_sensor,marker_data[d])[k]-robot_marker_data[d][k]) for k in range(7)]
    #err+= la.norm(np.array(Transform(x_sensor,marker_data[d]))-np.array(robot_marker_data[d]))
  err= [err[k]/float(len(robot_marker_data)) for k in range(7)]
  print 'Error:',err
  return x_sensor
