#!/usr/bin/python
#\file    toy_push1.py
#\brief   Toy example: push an object with a funnel-like gripper.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Mar.03, 2016
import math,random

def GenerateObjs(num,p1,p2,probs,stddevs):
  p_set= []
  p12= (p1,p2)
  for i in range(num):
    rand1= random.random()
    if rand1<probs[0]:  obj= 0
    else:               obj= 1
    p_set.append([ random.gauss(p12[obj][0],stddevs[obj]), random.gauss(p12[obj][1],stddevs[obj]) ])
  return p_set

#Transform po to gripper frame
def TransformToLocal(po,pg,theta):
  lpo= [po[0]-pg[0],po[1]-pg[1]]
  cos,sin= math.cos(theta), math.sin(theta)
  lpo= [cos*lpo[0]+sin*lpo[1], -sin*lpo[0]+cos*lpo[1]]
  return lpo

#Transform lpo from gripper frame to global frame
def TransformToGlobal(lpo,pg,theta):
  cos,sin= math.cos(-theta), math.sin(-theta)
  po= [cos*lpo[0]+sin*lpo[1], -sin*lpo[0]+cos*lpo[1]]
  po= [po[0]+pg[0],po[1]+pg[1]]
  return po

def PushFwdDyn(x_in):
  #In=['po','pg','theta','dm','rg']
  #Out=['lpo','dpo']
  po,pg,theta,dm,rg= x_in[:2],x_in[2:4],x_in[4],x_in[5],x_in[6]
  lpo2,pg2,dpo= DoPush(po,pg,theta,dm,rg)
  return lpo2+[dpo]

def DoPush(po,pg,theta,dm,rg):
  assert(dm>=0.0)
  lpo= TransformToLocal(po,pg,theta)
  sign= lambda x: 1.0 if x>=0.0 else -1.0
  norm= lambda p1,p2: math.sqrt(sum((p1[d]-p2[d])**2 for d in xrange(2)))
  lpo2= None
  if lpo[0]-abs(lpo[1])<0.0 or abs(lpo[1])>rg:  lpo2= [lpo[0]-dm, lpo[1]]  #not touched
  elif dm<=lpo[0]-abs(lpo[1]):                  lpo2= [lpo[0]-dm, lpo[1]]  #not touched
  elif lpo[0]<dm:                               lpo2= [0.0, 0.0]
  elif lpo[0]-abs(lpo[1])<dm and dm<=lpo[0]:    lpo2= [lpo[0]-dm, sign(lpo[1])*(lpo[0]-dm)]
  cos,sin= math.cos(theta), math.sin(theta)
  pg2= [pg[0]+dm*cos, pg[1]+dm*sin]
  #dpo= norm(TransformToGlobal(lpo2,pg2,theta),po)  #|po2-po|; how much object is moved
  if sum(map(abs,lpo2))<1.0e-6 and lpo[0]<dm:  dpo= dm-lpo[0]  #how much object is pushed by [0,0]
  else:  dpo= 0.0
  #print dpo
  return lpo2, pg2, dpo

def VizGripper(pg,theta,rg,fp):
  lp_set= ((0.0,0.0),(rg,rg),(-0.3*rg,0.0),(-0.6*rg,0.0),(-0.3*rg,0.0),(rg,-rg),(0.0,0.0))
  for lp in lp_set:
    p= TransformToGlobal(lp,pg,theta)
    fp.write('%f %f\n'%(p[0],p[1]))
  fp.write('\n')

def ExecuteAndViz(file_format,po,pg,theta,dm,rg,p1,p2,probs,stddevs):
  lpo2, pg2, dpo= DoPush(po,pg,theta,dm,rg)
  po2= TransformToGlobal(lpo2,pg2,theta)

  p_set= GenerateObjs(500,p1,p2,probs,stddevs)
  fp= open(file_format.format(kind='pset'),'w')
  for po1 in p_set:  fp.write('%f %f\n'%(po1[0],po1[1]))
  fp.close()

  fp= open(file_format.format(kind='po1'),'w')
  fp.write('%f %f\n'%(po[0],po[1]))
  fp.close()
  fp= open(file_format.format(kind='g1'),'w')
  VizGripper(pg,theta,rg,fp)
  fp.close()

  fppo= open(file_format.format(kind='po15'),'w')
  dm15= 0.0
  while dm15<dm:
    lpo15, pg15, dpo15= DoPush(po,pg,theta,dm15,rg)
    po15= TransformToGlobal(lpo15,pg15,theta)
    fppo.write('%f %f\n'%(po15[0],po15[1]))
    dm15+= 0.01
  fppo.close()

  fpg= open(file_format.format(kind='g15'),'w')
  dm15= 0.0
  while dm15<dm:
    lpo15, pg15, dpo15= DoPush(po,pg,theta,dm15,rg)
    VizGripper(pg15,theta,rg,fpg)
    dm15+= 0.1
  fpg.close()

  fp= open(file_format.format(kind='po2'),'w')
  fp.write('%f %f\n'%(po2[0],po2[1]))
  fp.close()
  fp= open(file_format.format(kind='g2'),'w')
  VizGripper(pg2,theta,rg,fp)
  fp.close()

def Main():
  p1= [0.5,0.5]  #position of Gaussian-1
  p2= [0.8,0.3]  #position of Gaussian-2
  probs= [0.5,0.5]  #probabilities of p1,p2
  stddevs= [0.1,0.1]  #standard deviation of p1,p2

  po= GenerateObjs(1,p1,p2,probs,stddevs)[0]  #actual position

  pg= [0.2,0.8]  #position to start push
  theta= -0.25*math.pi  #orientation of gripper
  dm= 0.7  #distance to push
  rg= 0.3  #size of gripper
  #pg= [0.4,0.0]
  #theta= 0.25*math.pi
  #pg= [random.random()*0.8+0.1, random.random()*0.8+0.1]
  #theta= random.random()*6.28-3.14

  file_format= '/tmp/push1_{kind}.dat'
  ExecuteAndViz(file_format,po,pg,theta,dm,rg,p1,p2,probs,stddevs)

def PlotGraphs():
  print 'Plotting graphs..'
  import os
  commands=[
    '''qplot -x2 aaa
      -s 'set size ratio -1'
      -s 'set xrange [-0.1:1.1]'
      -s 'set yrange [-0.1:1.1]'
      /tmp/push1_pset.dat w p ps 1 pt 6
      /tmp/push1_po15.dat w p ps 2 pt 7 lt 5
      /tmp/push1_g15.dat w l lw 1 lt 5
      /tmp/push1_po1.dat w p ps 4 pt 7 lt 2
      /tmp/push1_po2.dat w p ps 3 pt 7 lt 3
      /tmp/push1_g1.dat w l lw 2 lt 2
      /tmp/push1_g2.dat w l lw 3 lt 3
      &
      ''',
    '''''',
    '''''',
    ]
  for cmd in commands:
    if cmd!='':
      cmd= ' '.join(cmd.splitlines())
      print '###',cmd
      os.system(cmd)

  print '##########################'
  print '###Press enter to close###'
  print '##########################'
  raw_input()
  os.system('qplot -x2kill aaa')

if __name__=='__main__':
  import sys
  if len(sys.argv)>1 and sys.argv[1] in ('p','plot','Plot','PLOT'):
    PlotGraphs()
    sys.exit(0)
  Main()
