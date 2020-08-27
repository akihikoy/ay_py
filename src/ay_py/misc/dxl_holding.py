#!/usr/bin/python
#\file    dxl_holding.py
#\brief   Holding controller for Dynamixel.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jan.30, 2018
import threading
from ..core.util import TRate, CPrint

'''
Holding controller for Dynamixel.
It uses a virtual offset to increase position control power (like PI).
The effect is similar to increasing POS_I_GAIN but this offset is zero when the servo
is moving; it is more stable (i.e. less vibration/chatter).

Example:
  port_locker= threading.RLock()
  def holding_observer():
    with port_locker:
      pos,vel,pwm= dxl.Position(), dxl.Velocity(), dxl.PWM()
    return pos,vel,pwm

  def holding_controller(target_position):
    with port_locker:
      dxl.MoveTo(target_position, wait=False)

  holding= TDxlHolding()
  holding.observer= holding_observer
  holding.controller= holding_controller
  holding.SetTarget(TARGET_POS, MAX_PWM)
  holding.Start()

  user-defined-loop:
    ...
    holding.SetTarget(TARGET_POS, MAX_PWM)
    ...

  holding.Stop()

'''
class TDxlHolding(object):
  def __init__(self, rate=30):
    self.trg_pos= 2048
    self.max_pwm= 100
    self.is_running= False

    self.th_p= 3
    self.th_v= 3
    self.ostep= 3
    self.ctrl_rate= rate

    self.observer= None  #Should be: pos,vel,pwm= observer()
    self.controller= None  #Should be: controller(target_position)

  #Set target position.
  #  trg_pos: Target position.
  #  max_pwm: Maximum effort; when pwm exceeds this value, we don't increase the offset.
  def SetTarget(self, trg_pos, max_pwm=None):
    self.trg_pos= trg_pos
    self.max_pwm= max_pwm if max_pwm is not None else self.max_pwm

  def Start(self):
    self.Stop()
    self.thread= threading.Thread(name='holding', target=self.Loop)
    self.is_running= True
    self.thread.start()

  def Stop(self):
    if self.is_running:
      self.is_running= False
      self.thread.join()

  def Loop(self):
    sign= lambda x: 1 if x>0 else -1 if x<0 else 0

    rate= TRate(self.ctrl_rate)

    #Virtual offset:
    self.trg_offset= 0.0

    while self.is_running:
      pos,vel,pwm= self.observer()

      if self.trg_offset!=0 and sign(self.trg_pos-pos)!=sign(self.trg_offset):
        self.trg_pos= self.trg_pos+self.trg_offset
        self.trg_offset= 0.0
      elif abs(vel)>=self.th_v:
        self.trg_pos= self.trg_pos+self.trg_offset
        self.trg_offset= 0.0
      elif abs(self.trg_pos-pos)>self.th_p and abs(vel)<self.th_v and abs(pwm)<self.max_pwm:
        self.trg_offset= self.trg_offset + self.ostep*sign(self.trg_pos-pos)
      #print pos,vel,pwm,'--',self.trg_pos,self.trg_offset

      #print 'dxl_holding:Loop:',int(self.trg_pos+self.trg_offset)
      self.controller(int(self.trg_pos+self.trg_offset))

      rate.sleep()
      #print 'dxl_holding:Loop:rate.remaining:',rate.remaining()

