#!/usr/bin/python
#\file    cma1.py
#\brief   CMA example
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Apr.20, 2017
from _path import *
from ay_py.core import *

def Fobj(x, f_rand=0.0):
  return f_rand*random.random() + (x[0]-1.0)**2 + (x[1]+0.5)**2

def Main():
  opt= TContOptNoGrad()
  options= {}
  options['bounds']= [[-2.0]*2,[2.0]*2]
  options['tolfun']= 1.0e-4
  options['scale0']= 0.05
  #options['popsize']= 3
  options['parameters0']= [0.0]*2
  options['maxfevals']= 1000
  opt.Init({'options':options})

  fp= OpenW('/tmp/cma1_lc.dat')
  while not opt.Stopped():
    x= opt.Select()
    f= Fobj(x)
    opt.Update(-f)
    fp.write('%f %f %f\n'%(x[0],x[1],f))
  fp.close
  print 'Result=',opt.Result()

  fp= OpenW('/tmp/cma1_true.dat')
  #X= np.mgrid[-2:2:0.1, -2:2:0.1]
  #for x in zip(X[0].ravel(),X[1].ravel()):
  X= np.ogrid[-2:2:0.1, -2:2:0.1]
  for x0 in X[0].ravel():
    for x1 in X[1].ravel():
      x= [x0,x1]
      f= Fobj(x)
      fp.write('%f %f %f\n'%(x[0],x[1],f))
    fp.write('\n')
  fp.close

def PlotGraphs():
  print 'Plotting graphs..'
  import os
  commands=[
    '''qplot -x2 aaa -3d
          /tmp/cma1_true.dat w l
          /tmp/cma1_lc.dat w lp
          &''',
    '''qplot -x2 aaa
          /tmp/cma1_lc.dat u 3 w l
          &''',
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
