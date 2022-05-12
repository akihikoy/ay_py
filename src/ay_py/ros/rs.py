#! /usr/bin/env python
#RealSense utility.
import roslib; roslib.load_manifest('rospy')
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import os
import cv2
import threading
import six.moves.cPickle as pickle
import numpy as np
from ..core.util import TContainer

def GetCameraProjectionMatrix(cam_info_topic='/camera/aligned_depth_to_color/camera_info'):
  try:
    cam_info= rospy.wait_for_message(cam_info_topic, sensor_msgs.msg.CameraInfo, 5.0)
    proj_mat= np.array(cam_info.P).reshape(3,4) #get camera projection matrix from ros topic
    return proj_mat
  except (rospy.ROSException, rospy.ROSInterruptException):
    raise Exception('Failed to read topic: {cam_info_topic}'.format(cam_info_topic=cam_info_topic))

#Return an empty container object to store RealSense observation.
#with_helper: Return a helper container together,
#    which includes a converter from ROS message to images.
def GetEmptyRSContainer(with_helper=True):
  rs= TContainer()
  rs.options= None  #Dict of RealSense utility options.
  rs.proj_mat= None  #Projection matrix. Use GetCameraProjectionMatrix().
  rs.msg_depth= None  #Original depth message.
  rs.img_depth= None  #Depth image (for OpenCV).
  rs.stamp_depth= None  #Stamp of the depth message.
  rs.msg_rgb= None  #Original rgb message.
  rs.img_rgb= None  #RGB image (for OpenCV).
  rs.stamp_rgb= None  #Stamp of the rgb message.

  '''
  Frames and poses (each pose is a standard 7-d vector containing xyz+quaternion):
    frame: Frame where the RealSense is attached.
    camera_link: Frame of the RealSense.
    camera_color_optical_frame: Frame of the optical module of RealSense.
  '''
  rs.frame= None  #Frame id (string) where the RealSense is attached.
  rs.lx= None  #Local pose of camera_color_optical_frame in rs.frame.
  rs.lw_x_camera_link= None  #Local pose of camera_link in rs.frame.
  rs.xw= None  #Pose of rs.frame in the global frame at the observation.
  #Note: If both depth and rgb are observed via different ROS topics, xw is measured only when depth is observed.

  if not with_helper:
    return rs
  else:
    helper= TContainer()
    helper.cvbridge= CvBridge()
    helper.thread_locker= threading.RLock()
    return rs,helper

'''
Save an object d (e.g. dict) into a file filepath as a pickle format with RealSense data reduction.
When a RealSense reference rs included in d is given, we try to reduce the storage size by:
  Keeping header of rs.msg_depth and rs.msg_rgb as rs.msg_depth_header, rs.msg_rgb_header respectively,
  removing rs.msg_depth and rs.msg_rgb,
  saving rs.img_depth and rs.img_rgb as png files in the same directory when save_rs_imgs_separately is True.
         filenames are assigned to these variables.
Note: Content of rs is modified. Use copy.deepcopy to preserve the original.
rs should be a reference to an element in d.
  e.g.
    d=dict(rs=copy.deepcopy(rs), x=x, y=y)
    SaveAsPickleRS(filepath,d,rs=d['rs'])
rs can be a list of RealSense references.  In this case, the reduction is applied for all items in rs.
compress: If True, the data file is compressed with gzip.
'''
def SaveAsPickleRS(filepath, d, rs=None, save_rs_imgs_separately=True, compress=False):
  if rs is not None:
    if isinstance(rs,list):  rs_list= rs
    else:                    rs_list= [rs]
    path_dir,path_name_ext= os.path.split(filepath)
    path_name,path_ext= os.path.splitext(path_name_ext)
    for i_rs,rs in enumerate(rs_list):
      #Reduce the data size to store.
      rs.msg_depth_header= rs.msg_depth.header
      rs.msg_rgb_header= rs.msg_rgb.header
      rs.msg_depth= None
      rs.msg_rgb= None
      if rs.img_depth is not None and save_rs_imgs_separately:
        img_name= '{}-rs{}-img_depth{}'.format(path_name,i_rs,'.png')
        img_path= str(os.path.join(path_dir, img_name))
        cv2.imwrite(img_path, rs.img_depth)
        rs.img_depth= img_name
      if rs.img_rgb is not None and save_rs_imgs_separately:
        img_name= '{}-rs{}-img_rgb{}'.format(path_name,i_rs,'.png')
        img_path= str(os.path.join(path_dir, img_name))
        cv2.imwrite(img_path, rs.img_rgb)
        rs.img_rgb= img_name
  with open(filepath, 'w') as data_fp:
    pickle.dump(d,data_fp)
  if compress:
    #The gzip command of the os is used.
    #Using gzip in Python module would be better for portability,
    #but it takes more computation time.
    os.system('gzip {0}'.format(data_file_path))  #Compressing the data

'''
Reconstruct RealSense data where a part of the elements were removed or saved separately.
rs: RealSense data (TContainer) loaded from a pickle file.
    Note: elements in rs are modified.
parent_filepath: File path of the pickle file.
generate_rgb_from_depth: If an RGB image is not saved, we generate it from a depth image.
Return: rs (the same reference as rs).
'''
def ReconstructRS(rs, parent_filepath='', generate_rgb_from_depth=True):
  #Reload separately-stored images.
  parent_dir= os.path.dirname(parent_filepath)
  if isinstance(rs.img_depth,str):
    rs.img_depth= cv2.imread(os.path.join(parent_dir,rs.img_depth),cv2.IMREAD_ANYDEPTH)
  if isinstance(rs.img_rgb,str):
    rs.img_rgb= cv2.imread(os.path.join(parent_dir,rs.img_rgb))

  #Note: We do not reconstruct these elements (although it is possible):
  #rs.msg_depth
  #rs.msg_rgb

  if rs.img_rgb is None and rs.img_depth is not None:
    #If no RGB data is contained, we generate it from the depth image.
    #.reshape(rs.img_depth.shape[:2]+(1,))
    rs.img_rgb= np.ones(rs.img_depth.shape[:2]+(3,), np.uint8)*np.array((255,255,255), np.uint8)
    rs.img_rgb[:,:,1]= ( rs.img_rgb[:,:,0]*(0.5+0.5*np.cos(rs.img_depth*0.01)) ).astype(np.uint8)
    rs.img_rgb[:,:,2]= ( rs.img_rgb[:,:,0]*(0.5+0.5*np.sin(rs.img_depth*0.01)) ).astype(np.uint8)
    rs.stamp_rgb= rs.stamp_depth
    rs.msg_rgb_header= rs.msg_depth_header

  return rs

