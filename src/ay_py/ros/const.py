#! /usr/bin/env python
#Basic tools (constant values).
import os

USING_ROS= True if 'ROS_ROOT' in os.environ else False
#ROS_ROBOT= os.environ['ROS_ROBOT'] if 'ROS_ROBOT' in os.environ else None
ROS_DISTRO= os.environ['ROS_DISTRO'] if 'ROS_DISTRO' in os.environ else None

#if ROS_ROBOT in ('PR2','PR2_SIM'):
  #ROS_DEFAULT_FRAME= 'torso_lift_link'
#elif ROS_ROBOT in ('Baxter','Baxter_SIM'):
  #ROS_DEFAULT_FRAME= 'torso'
  ##ROS_DEFAULT_FRAME= 'base'
#elif ROS_ROBOT in ('Motoman','Motoman_SIM'):
  #ROS_DEFAULT_FRAME= 'base_link'
#else:
  #ROS_DEFAULT_FRAME= 'base'


#Attribute key for the current (temporary) situation
CURR='*'

#Database structure:
#database= [situation, inferred_data, assessment] * N1
#situation= {dict}
#inferred_data= [keys, value] * N2
#assessment= {dict}
DB_SITUATION= 0
DB_INFERRED= 1
DB_ASSESSMENT= 2
DB_INFERRED_KEYS= 0
DB_INFERRED_VALUE= 1

#Database file name
DATABASE_FILE= '.database.yaml'

#Memory file name
MEMORY_FILE= '.memory.yaml'

#Arm/gripper ID
RIGHT=0
LEFT=1
ID_A=0
ID_B=1
ID_C=2
ID_D=3
ID_E=4
ID_F=5
ID_G=6

def LRToStr(whicharm):
  return ('Right','Left')[whicharm]

def LRTostr(whicharm):
  return ('right','left')[whicharm]

def LRToStrS(whicharm):
  return ('R','L')[whicharm]

def LRToStrs(whicharm):
  return ('r','l')[whicharm]

def StrToLR(whicharm_str):
  try:
    return {'r':RIGHT,'R':RIGHT,'right':RIGHT,'Right':RIGHT,'RIGHT':RIGHT,
            'l':LEFT,'L':LEFT,'left':LEFT,'Left':LEFT,'LEFT':LEFT}[whicharm_str]
  except KeyError:
    return None

def IDToStr(whicharm):
  return ('A','B','C','D','E','F','G')[whicharm]

def IDTostr(whicharm):
  return ('a','b','c','d','e','f','g')[whicharm]

def StrToID(whicharm_str):
  try:
    return {'A':ID_A,'B':ID_B,'C':ID_C,'D':ID_D,'E':ID_E,'F':ID_F,'G':ID_G,
            'a':ID_A,'b':ID_B,'c':ID_C,'d':ID_D,'e':ID_E,'f':ID_F,'g':ID_G}[whicharm_str]
  except KeyError:
    return None
