#!/usr/bin/python
#\file    dxl_rhp12rn.py
#\brief   Control module of RH-P12-RN (Thormang3 gripper).
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Nov.20, 2018
#\version 0.2
#\date    May.21, 2022
#         Refactored the code.
#\version 0.3
#\date    Sep.19, 2022
#         Added RH-P12-RN(A).
from dxl_gripper import TDxlGripperBase

'''
RH-P12-RN Gripper (Thormang3 gripper) utility class.
For RH-P12-RN(A), specify type='(A)'.
'''
class TRHP12RN(TDxlGripperBase):
  def __init__(self, dev='/dev/ttyUSB0', type=''):
    super(TRHP12RN,self).__init__(dxl_type='RH-P12-RN'+type,dev=dev)

    self.CmdMax= 740
    self.CmdMin= 0  #Gripper opened.
    self.CmdOpen= 0
    self.CmdClose= 730  #Gripper closed.
    #Gripper range in meter:
    self.GrpClose= 0.0
    self.GrpOpen= 0.1055
    self.GrpMin= -0.0014452054794520546
    self.GrpMax= 0.1055

