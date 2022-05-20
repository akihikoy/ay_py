#!/usr/bin/python
#\file    dxl_rhp12rn.py
#\brief   Control module of RH-P12-RN (Thormang3 gripper).
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Nov.20, 2018
#\version 0.2
#\date    May.21, 2022
#         Refactored the code.
from dxl_gripper import TDxlGripperBase

'''RH-P12-RN Gripper (Thormang3 gripper) utility class'''
class TRHP12RN(TDxlGripperBase):
  def __init__(self, dev='/dev/ttyUSB0'):
    super(TRHP12RN,self).__init__(dxl_type='RH-P12-RN',dev=dev)

    self.CmdMax= 776  #Gripper closed.
    self.CmdMin= 26  #Gripper opened.
    self.CmdOpen= 26
    self.CmdClose= 776
    #Gripper range in meter:
    self.GrpClose= 0.0
    self.GrpOpen= 0.109
    self.GrpMin= 0.0
    self.GrpMax= 0.109

