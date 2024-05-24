#! /usr/bin/env python
#ROS computer vision utility.
import roslib; roslib.load_manifest('sensor_msgs')
import rospy
import sensor_msgs.msg
import numpy as np

#Get the image encoding of a ROS image topic.
#If convert_cv is true, the encoding is converted for OpenCV image conversion.
def GetImageEncoding(img_topic, convert_cv=False, time_out=5.0):
  try:
    msg= rospy.wait_for_message(img_topic, sensor_msgs.msg.Image, time_out)
    encoding= msg.encoding
    if not convert_cv:  return encoding
    if encoding=="rgb8":  return "bgr8"
    if encoding=="RGB8":  return "BGR8"
    #TODO: Add more conversion if necessary.
    return encoding;
  except (rospy.ROSException, rospy.ROSInterruptException):
    raise Exception('Failed to receive the image topic: {}'.format(img_topic))


#Get a camera projection matrix from a ROS topic.
def GetCameraProjectionMatrix(cam_info_topic='/camera/aligned_depth_to_color/camera_info', time_out=5.0):
  try:
    cam_info= rospy.wait_for_message(cam_info_topic, sensor_msgs.msg.CameraInfo, time_out)
    proj_mat= np.array(cam_info.P).reshape(3,4) #get camera projection matrix from ros topic
    return proj_mat
  except (rospy.ROSException, rospy.ROSInterruptException):
    raise Exception('Failed to read topic: {cam_info_topic}'.format(cam_info_topic=cam_info_topic))



#Get detailed camera parameters (P,K,D,R) from a ROS topic.
def GetCameraInfo(cam_info_topic='/camera/aligned_depth_to_color/camera_info', time_out=5.0):
  try:
    cam_info= rospy.wait_for_message(cam_info_topic, sensor_msgs.msg.CameraInfo, time_out)
    P= np.array(cam_info.P).reshape(3,4)
    K= np.array(cam_info.K).reshape(3,3)
    D= np.array(cam_info.D)
    R= np.array(cam_info.R).reshape(3,3)
    return P,K,D,R
  except (rospy.ROSException, rospy.ROSInterruptException):
    raise Exception('Failed to read topic: {cam_info_topic}'.format(cam_info_topic=cam_info_topic))

