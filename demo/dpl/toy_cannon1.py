#!/usr/bin/python
#\file    toy_cannon1.py
#\brief   Simple 2D cannon example.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Feb.29, 2016
import math
import numpy as np

#Float version of range
def FRange1(x1,x2,num_div):
  return [x1+(x2-x1)*x/float(num_div) for x in range(num_div+1)]

def CannonForward(x_in,with_grad):
  #In=['p0','pe','v0','theta']
  #Out=['th','pyh']
  p0,pe,v0,theta= x_in[:2],x_in[2:4],x_in[4],x_in[5]
  A= 9.8/2.0 * ((pe[0]-p0[0])**2)/(v0**2)
  B= pe[0]-p0[0]
  th= B/(v0*math.cos(theta))
  sin= math.sin(theta)
  cos= math.cos(theta)
  pyh= p0[1] + B*sin/cos - A/(cos*cos)
  if not with_grad:
    return [th,pyh]
  else:
    dth_dtheta= B/v0 * sin/(cos*cos)
    dpyh_dtheta= B/(cos*cos) - 2.0*A*sin/(cos**3)
    D= np.zeros((6,2))
    D[5,0]= dth_dtheta
    D[5,1]= dpyh_dtheta
    return [th,pyh], D

def GetTh(p0, pe, v0, theta):
  return (pe[0]-p0[0])/(v0*math.cos(theta))

def SolveAnalytically(p0, pe, v0):
  A= 9.8/2.0 * ((pe[0]-p0[0])**2)/(v0**2)
  B= pe[0]-p0[0]
  C= pe[1]-p0[1]
  #A - B*sin(theta)*cos(theta) + C*cos(theta)*cos(theta) = 0
  rt= B**2-4.0*A*(A+C)
  if rt<0.0:
    #print 'No solution.'
    return None
  theta1= math.atan((B+math.sqrt(rt))/(2.0*A))
  theta2= math.atan((B-math.sqrt(rt))/(2.0*A))
  another_sol= lambda theta: theta+math.pi if theta<0.0 else theta-math.pi
  theta= [theta1, another_sol(theta1), theta2, another_sol(theta2)]
  theta= [theta_i for theta_i in theta if theta_i<0.5*math.pi and theta_i>-0.5*math.pi]
  th= []
  for theta_i in theta:
    th.append( (pe[0]-p0[0])/(v0*math.cos(theta_i)) )
  th_theta= zip(th,theta)
  th_theta.sort()
  th,theta= th_theta[0][0], th_theta[0][1]
  #print 'th_theta=',th_theta
  #print 'theta=',theta
  #print 'th=',th
  #print 'A-B*sin(theta)*cos(theta)+C*cos(theta)*cos(theta)=', A-B*math.sin(theta)*math.cos(theta)+C*math.cos(theta)*math.cos(theta)
  return theta

def Simulate(p0, pe, v0, theta, fp):
  th= (pe[0]-p0[0])/(v0*math.cos(theta))
  for t in FRange1(0.0,th,50):
    p= [p0[i] + v0*t*(math.cos(theta),math.sin(theta))[i] + 0.5*t*t*(0.0,-9.8)[i] for i in (0,1)]
    fp.write('{x} {y}\n'.format(x=p[0],y=p[1]))

def Main1():
  p0= [0.0, 0.8]
  pe= [1.3, 0.3]
  #p0= [0.0, 0.0]
  #pe= [0.5407, 0.3]
  #p0= [0.0, 0.8]
  #pe= [1.327, 0.3]
  v0= 3.0
  theta= SolveAnalytically(p0, pe, v0)
  print 'theta=', theta
  print 'th,pyh=', CannonForward(p0+pe+[v0]+[theta],with_grad=False) if theta is not None else None
  if theta is not None:
    fp= open('/tmp/cannon1.dat','w')
    Simulate(p0, pe, v0, theta, fp)
    fp.close()
  else:
    raise Exception('No solution.')

def Main2():
  p1= [0.0, 0.0]
  p2= [0.0, 0.8]
  pe= [0.225, 0.5]
  v0= 3.5
  theta1,theta2= SolveAnalytically(p1, pe, v0), SolveAnalytically(p2, pe, v0)
  print 'theta1,2=', theta1,theta2
  print 'th1,pyh1=', CannonForward(p1+pe+[v0]+[theta1],with_grad=False)
  print 'th2,pyh2=', CannonForward(p2+pe+[v0]+[theta2],with_grad=False)
  theta= 2.29605378903
  print '(dpl)theta=', theta
  print '(dpl)th1,pyh1=', CannonForward(p1+pe+[v0]+[theta],with_grad=False)
  print '(dpl)th2,pyh2=', CannonForward(p2+pe+[v0]+[theta],with_grad=False)

def Main():
  Main1()
  #Main2()


def PlotGraphs():
  print 'Plotting graphs..'
  import os
  commands=[
    '''qplot -x2 aaa /tmp/cannon1.dat w l &''',
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
