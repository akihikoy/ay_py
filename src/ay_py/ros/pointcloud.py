#! /usr/bin/env python
#ROS tools (pointcloud).
import roslib; roslib.load_manifest('rospy')
import rospy
import sensor_msgs.msg
from sensor_msgs import point_cloud2
import numpy as np

'''
Convert img_depth and img_rgb to point cloud, and return the message.
img_depth and img_rgb should be aligned.
proj_mat: Camera projection matrix.
header: Header for the pointcloud message.
'''
def DepthRGBImgsToPointCloud(img_depth, img_rgb, proj_mat, header, xstep=2, ystep=2):
  #points= [(np.array(InvProjectFromImage([x,y],proj_mat))*(img_depth[y,x]*1.0e-3)).tolist()+[struct.unpack('I',struct.pack('BBBB',*(img_rgb[y,x].tolist()+[255])))[0]] for y,x in itertools.product(xrange(0,img_depth.shape[0],ystep),xrange(0,img_depth.shape[1],xstep))]
  Fx,Fy,Cx,Cy= proj_mat[0,0],proj_mat[1,1],proj_mat[0,2],proj_mat[1,2]
  col= np.squeeze(np.concatenate((img_rgb[::ystep,::xstep].astype(np.uint8), np.full(img_rgb[::ystep,::xstep].shape[:-1]+(1,), 255, dtype=np.uint8)), axis=-1).view(np.uint32))
  func= lambda x,y,z,col: [(x-Cx)/Fx*z, (y-Cy)/Fy*z, z, col]
  indices= np.indices(img_depth.shape)[:,::ystep,::xstep]
  merged= np.dstack(func(indices[1], indices[0], img_depth[::ystep,::xstep].astype(np.float64)*1.0e-3, col))
  points= merged.reshape((-1, 4)).tolist()

  fields= [
    #name,offset,datatype,count
    sensor_msgs.msg.PointField('x',0,sensor_msgs.msg.PointField.FLOAT32,1),
    sensor_msgs.msg.PointField('y',4,sensor_msgs.msg.PointField.FLOAT32,1),
    sensor_msgs.msg.PointField('z',8,sensor_msgs.msg.PointField.FLOAT32,1),
    sensor_msgs.msg.PointField('rgba',12,sensor_msgs.msg.PointField.UINT32,1)]
  pc_msg= point_cloud2.create_cloud(header, fields, points)
  return pc_msg

'''
Convert img_depth to point cloud, and return the message.
proj_mat: Camera projection matrix.
header: Header for the pointcloud message.
'''
def DepthImgToPointCloud(img_depth, proj_mat, header, xstep=2, ystep=2):
  #dmin= np.min(img_depth)
  #points= [(np.array(InvProjectFromImage([x,y],proj_mat))*(img_depth[y,x]*1.0e-3)).tolist()+[struct.unpack('I',struct.pack('BBBB',*(d2c(x,y,img_depth[y,x]))))[0]] for y,x in itertools.product(xrange(0,img_depth.shape[0],ystep),xrange(0,img_depth.shape[1],xstep))]
  xs= lambda a,v: np.full(a.shape, v, dtype=np.uint8)
  #d2c= lambda x,y,z: [0,int(max(0,min(255,255+10*(dmin-z)))),0,255]
  #d2c= lambda x,y,z: [x%256,z%256,y%256,255]
  d2c= lambda x,y,z: [xs(z,0),((5*z)%256).astype(np.uint8),xs(z,0),xs(z,255)]
  Fx,Fy,Cx,Cy= proj_mat[0,0],proj_mat[1,1],proj_mat[0,2],proj_mat[1,2]
  indices= np.indices(img_depth.shape)[:,::ystep,::xstep]
  col= np.squeeze(np.dstack(d2c(indices[1], indices[0], img_depth[::ystep,::xstep])).view(np.uint32))
  func= lambda x,y,z,col: [(x-Cx)/Fx*z, (y-Cy)/Fy*z, z, col]
  merged= np.dstack(func(indices[1], indices[0], img_depth[::ystep,::xstep].astype(np.float64)*1.0e-3, col))
  points= merged.reshape((-1, 4)).tolist()

  fields= [
    #name,offset,datatype,count
    sensor_msgs.msg.PointField('x',0,sensor_msgs.msg.PointField.FLOAT32,1),
    sensor_msgs.msg.PointField('y',4,sensor_msgs.msg.PointField.FLOAT32,1),
    sensor_msgs.msg.PointField('z',8,sensor_msgs.msg.PointField.FLOAT32,1),
    sensor_msgs.msg.PointField('rgba',12,sensor_msgs.msg.PointField.UINT32,1)]
  pc_msg= point_cloud2.create_cloud(header, fields, points)
  return pc_msg

