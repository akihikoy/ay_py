#!/usr/bin/python
#\file    dxl_ezg.py
#\brief   Control module of DxlpY1.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    May.21, 2022
from dxl_gripper import TDxlGripperBase

'''DxlpY1 utility class'''
class TDxlpY1(TDxlGripperBase):
  def __init__(self, dev='/dev/ttyUSB0'):
    super(TDxlpY1,self).__init__(dxl_type='XD540-T270',dev=dev)

    self.CmdMax= 2060  #Gripper tightly closed.
    self.CmdMin= 1600  #Gripper opened widely.
    self.CmdOpen= 1800
    self.CmdClose= 2052
    #Gripper range in meter:
    self.GrpClose= 0.0
    self.GrpOpen= 0.133
    self.GrpMin= 0.0
    self.GrpMax= 0.133
