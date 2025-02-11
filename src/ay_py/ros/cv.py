#! /usr/bin/env python3
#ROS computer vision utility.
import roslib; roslib.load_manifest('sensor_msgs')
import rospy
import sensor_msgs.msg
import numpy as np
from .base import IsTopicAvailable

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

#Wait for a camera info topic and return after receiving a message.
def WaitForCameraInfoTopic(cam_info_topic='/camera/aligned_depth_to_color/camera_info', time_out=5.0):
  try:
    cam_info= rospy.wait_for_message(cam_info_topic, sensor_msgs.msg.CameraInfo, time_out)
    return cam_info
  except (rospy.ROSException, rospy.ROSInterruptException):
    raise Exception('Failed to read topic: {cam_info_topic}'.format(cam_info_topic=cam_info_topic))

#Wait for multiple camera info topics and return when a message is received.
#Assuming that the all topics have the same information (i.e. aligned).
#  dt_pole: Interval to pole topics.
#  time_out: Time out (None to wait forever).
def WaitForCameraInfoTopics(cam_info_topics, dt_pole=0.02, time_out=5.0):
  t_start= rospy.Time.now()
  t_end= t_start+rospy.Duration(time_out) if time_out is not None else None
  cam_info_type= sensor_msgs.msg.CameraInfo
  while not rospy.is_shutdown():
    for topic in cam_info_topics:
      if IsTopicAvailable(topic, cam_info_type):
        time_out= max(0, (t_end-rospy.Time.now()).to_sec()) if t_end is not None else None
        cam_info= WaitForCameraInfoTopic(topic, time_out)
        return cam_info
    if t_end is not None and t_end<=rospy.Time.now():
      raise Exception('Failed to receive camera info topics from:', cam_info_topics)
    rospy.sleep(dt_pole)

#Get a camera projection matrix from a ROS topic.
#cam_info_topic can be a list of topics. In this case, the camera information is obtained from one of them.
def GetCameraProjectionMatrix(cam_info_topic='/camera/aligned_depth_to_color/camera_info', time_out=5.0):
  if isinstance(cam_info_topic, list):
    cam_info= WaitForCameraInfoTopics(cam_info_topic, time_out=time_out)
  else:
    cam_info= WaitForCameraInfoTopic(cam_info_topic, time_out=time_out)
  proj_mat= np.array(cam_info.P).reshape(3,4) #get camera projection matrix from ros topic
  return proj_mat

#Get detailed camera parameters (P,K,D,R) from a ROS topic.
#cam_info_topic can be a list of topics. In this case, the camera information is obtained from one of them.
def GetCameraInfo(cam_info_topic='/camera/aligned_depth_to_color/camera_info', time_out=5.0):
  if isinstance(cam_info_topic, list):
    cam_info= WaitForCameraInfoTopics(cam_info_topic, time_out=time_out)
  else:
    cam_info= WaitForCameraInfoTopic(cam_info_topic, time_out=time_out)
  P= np.array(cam_info.P).reshape(3,4)
  K= np.array(cam_info.K).reshape(3,3)
  D= np.array(cam_info.D)
  R= np.array(cam_info.R).reshape(3,3)
  return P,K,D,R

