#!/usr/bin/python
#\file    dxl_dxlpo2.py
#\brief   Control module of DxlpO2 gripper.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jan.28, 2020
#\version 0.2
#\date    May.21, 2022
#         Refactored the code.
from dxl_gripper import TDxlGripperBase

'''DxlpO2 gripper utility class'''
class TDxlpO2(TDxlGripperBase):
  def __init__(self, dev='/dev/ttyUSB0', finger_type=None):
    super(TDxlpO2,self).__init__(dxl_type='PH54-200-S500',dev=dev)

    if finger_type is None:
      raise Exception('TDxlpO2: finger_type is not specified. Set finger type such as Straight1.')

    elif finger_type=='Straight1':
      self.CmdMax= 0  #Gripper opened widely.
      self.CmdMin= -250962  #Gripper closed.
      self.CmdOpen= -150000  #Gripper opened.
      self.CmdClose= -250962  #Gripper closed.
      #Gripper range in meter:
      self.GrpClose= 0.0
      self.GrpOpen= 0.300
      self.GrpMin= 0.0
      self.GrpMax= 0.300  #FIXME: 0.3 is inaccurate.

    elif finger_type=='SRound1':  #Small, round finger (yellow)
      self.CmdMax= 0  #Gripper opened widely.
      self.CmdMin= -250962  #Gripper closed.
      self.CmdOpen= -150000  #Gripper opened.
      self.CmdClose= -250962  #Gripper closed.
      #Gripper range in meter:
      self.GrpClose= 0.0
      self.GrpOpen= 0.1950
      self.GrpMin= 0.0
      self.GrpMax= 0.1950

    #elif finger_type=='Fork1':
      #self.CmdMax= 0  #Gripper opened widely.
      #self.CmdMin= -200962  #Gripper closed.
      #self.CmdOpen= -150000  #Gripper opened.
      #self.CmdClose= -200962  #Gripper closed.
      ##Gripper range in meter:
      #self.GrpClose= 0.03
      #self.GrpOpen= 0.23
      #self.GrpMin= 0.03
      #self.GrpMax= 0.230

    #elif finger_type=='Fork1':  #TODO:FIXME:Change this name to Fork2
      #self.CmdMax= 0        #Gripper opened widely.  #DxlPo2f1
      ##self.CmdMin= -234000  #Gripper closed.
      #self.CmdMin= -243160  #Gripper closed.
      ##self.CmdOpen= -124100  #Gripper opened.
      #self.CmdOpen= -134500  #Gripper opened.
      ##self.CmdClose= -234000  #Gripper closed.
      #self.CmdClose= -243160  #Gripper closed.
      ##Gripper range in meter:
      #self.GrpClose= 0.0
      #self.GrpOpen= 0.23
      #self.GrpMin= 0.0
      #self.GrpMax= 0.23

    elif finger_type=='Fork1':  #TODO:FIXME:Change this name to Fork2cross
      self.CmdMax=  150000  #Gripper opened widely.  #DxlPo2f1
      self.CmdMin=   10000  #Gripper over-closed (crossed).
      self.CmdOpen= 111181  #Gripper opened.
      self.CmdClose= 18732  #Gripper closed.
      #Gripper range in meter:
      self.GrpClose= 0.0
      self.GrpOpen= 0.20
      self.GrpMin= -0.0189  #-0.008 is obtained by gripper_cmd2pos(CmdMin) w/o gripper_range.
      self.GrpMax= 0.20
      #FIXME:Overwirting for the tong attachment:
      self.CmdMin=   -9500  #Gripper over-closed (crossed).
      self.CmdClose= -2805  #Gripper closed.
      self.CmdOpen=  75678  #Gripper opened.
      self.GrpMin= -0.017  #min is obtained by gripper_cmd2pos(CmdMin) w/o gripper_range.
      self.GrpMax= 0.20

    elif finger_type=='???':
      self.CmdMax= 0  #Gripper opened widely.
      self.CmdMin= -160000  #Gripper closed.
      self.CmdOpen= -150000  #Gripper opened.
      self.CmdClose= -160000  #Gripper closed.
      #Gripper range in meter:
      self.GrpClose= 0.0
      self.GrpOpen= 0.2200
      self.GrpMin= 0.0
      self.GrpMax= 0.2200

    else:
      raise Exception('TDxlpO2: Unknown finger_type: {finger_type}'.format(finger_type=finger_type))

