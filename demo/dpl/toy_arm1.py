#!/usr/bin/python
#\file    toy_arm1.py
#\brief   Arm robot test.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jan.20, 2016
from _path import *
from ay_py.core import *

class TArm(object):
  def __init__(self,N=3,L=1.0):
    self.N= N
    self.Len= L

  def FK2D(self, q, with_J=False, viz=None):
    f_red= lambda x: [x[0],x[2]]
    viz_red= (lambda x: viz(f_red(x))) if viz is not None else None
    if not with_J:
      x= self.FK3D(q,with_J=with_J,viz=viz_red)
      return f_red(x)
    else:
      x,J= self.FK3D(q,with_J=with_J,viz=viz_red)
      return f_red(x), J[(0,2),:]

  #with_J: whether compute Jacobian matrix.
  #with_full: whether compute full joint poses (otherwise we just compute end effector pose).
  def FK3D(self, q, with_J=False, viz=None, with_full=False):
    N= self.N
    l_link= self.Len/float(N)
    unit_axes= ((1.0,0.0,0.0), (0.0,1.0,0.0), (0.0,0.0,1.0))
    ax= lambda j: unit_axes[1]
    ztrans= lambda z: [0.0,0.0,z, 0.0,0.0,0.0,1.0]
    qtrans= lambda axis,theta: [0.0,0.0,0.0]+QFromAxisAngle(axis,theta).tolist()

    dT= [None]*(N)
    T= [None]*(N+1)
    for j in range(0,N):
      dT[j]= Transform(qtrans(ax(j),q[j]), ztrans(l_link))
    T[0]= [0.0,0.0,0.0, 0.0,0.0,0.0,1.0]
    if viz is not None:  viz(T[0])
    for j in range(0,N):
      T[j+1]= Transform(T[j],dT[j])
      if viz is not None:  viz(T[j+1])
    xe= T[-1] if not with_full else T

    if with_J:
      if not with_full:
        J= np.zeros((6,N))
        pe= xe[:3]
        for j in range(N):
          p,R= XToPosRot(T[j])
          a= np.dot(R,ax(j))
          J[:3,j]= np.cross(a, pe-p)
          J[3:,j]= a
      else:  #with_full
        J= np.zeros((6*(N+1),N))
        for i in range(N+1):
          pe= T[i][:3]
          for j in range(i-1):
            p,R= XToPosRot(T[j])
            a= np.dot(R,ax(j))
            J[(3*i):(3*(i+1)),j]= np.cross(a, pe-p)
            J[(3*(N+1+i)):(3*(N+2+i)):,j]= a
      return xe, J
    else:
      return xe

def Visualize2D(arm, q1, q2, x_trg, file_name, n_div=1):
  fp= open(file_name,'w')
  for r in FRange1(0.0,1.0,n_div):
    q= (1.0-r)*np.array(q1) + r*np.array(q2)
    arm.FK2D(q,viz=lambda x:fp.write('{x} {z} {xt} {zt}\n'.format(x=x[0],z=x[1],xt=x_trg[0],zt=x_trg[1])))
    fp.write('\n')
  fp.close()

def VisualizeTraj2D(arm, q_traj, x_trg, file_name):
  fp= open(file_name,'w')
  for q in q_traj:
    arm.FK2D(q,viz=lambda x:fp.write('{x} {z} {xt} {zt}\n'.format(x=x[0],z=x[1],xt=x_trg[0],zt=x_trg[1])))
    fp.write('\n')
  fp.close()

def PlotGraphs():
  print 'Plotting graphs..'
  import os
  commands=[
    '''qplot -x2 aaa
          -s 'set xlabel "x";set ylabel "z";'
          -s 'set xrange [-1.2:1.2];set yrange[-1.2:1.2];'
          -s 'set size square;'
          /tmp/arm/arm.dat u 3:4 w p ps 3 pt 5 lt 1
          /tmp/arm/arm.dat u 1:2:-1 w l lw 3 lt 3 lc var
          , '"< tail -2 /tmp/arm/arm.dat"' w p ps 3 pt 6 lw 3 lt 3 &''',
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
  print 'f:FK, i:IK'
  cmd= AskGen('f','i')
  #FK test
  if cmd=='f':
    arm= TArm(5)
    fp= open('/tmp/arm/arm.dat','w')
    for theta in FRange1(0.0,math.pi*0.5,10):
      q= [theta for j in range(arm.N)]
      arm.FK2D(q,viz=lambda x:fp.write('{x} {z}\n'.format(x=x[0],z=x[1])))
      fp.write('\n')
    fp.close()

  #IK test
  if cmd=='i':
    arm= TArm(5)
    #x_trg= np.array([-0.5,-0.5])
    #q= np.array([0.1 for j in range(arm.N)])
    x_trg= np.array(arm.FK2D(np.array([Rand(-math.pi,math.pi) for d in range(arm.N)])))
    q= np.array([Rand(-math.pi,math.pi) for d in range(arm.N)])
    fp= open('/tmp/arm/arm.dat','w')
    for i in range(20):
      x,J= arm.FK2D(q,with_J=True,viz=lambda x:fp.write('{x} {z}\n'.format(x=x[0],z=x[1])))
      dq= 0.2*np.dot(la.pinv(J),x_trg-x)
      #dq= 0.2*np.dot(J.T,x_trg-x)
      mx= max([abs(th) for th in dq])
      if mx>0.5:  dq/= mx
      #print x, dq
      #print x_trg-x, Dist(x_trg,x)**2
      q+= dq
      fp.write('\n')
    fp.close()
