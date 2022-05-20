#!/usr/bin/python
#\file    dxl_ezg.py
#\brief   Control module of SAKE EZGripper Gen2.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jul.12, 2019
#\version 0.2
#\date    May.21, 2022
#         Refactored the code.
#         NOTE: Dynamixel MX-64AR of EZGripper is originally protocol 1.0; update the firmware.
from dxl_gripper import TDxlGripperBase

'''SAKE EZGripper Gen2 utility class'''
class TEZG(TDxlGripperBase):
  def __init__(self, dev='/dev/ttyUSB0'):
    super(TEZG,self).__init__(dxl_type='MX-64AR',dev=dev)
    self.op_mode= 'CURRPOS'

    self.CmdMax= 2104  #Gripper opened widely.
    self.CmdMin= 50  #Gripper closed strongly.
    self.CmdOpen= 809  #Gripper opened.
    self.CmdClose= 233  #Gripper closed.
    #Gripper range in meter:
    self.GrpClose= 0.0
    self.GrpOpen= 0.150
    self.GrpMin= 0.0
    self.GrpMax= 0.150

