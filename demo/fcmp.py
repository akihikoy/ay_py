#!/usr/bin/python
#Test of TCombinedFuncApprox (combined function approximator) that combines
#some TFunctionApprox objects.
from _path import *
from ay_py.core import *

def Main():
  #def PrintEq(s):  print '%s= %r' % (s, eval(s))

  load_model= False

  options= {}
  options['kernel']= 'maxg'
  options['c_min']= 0.01
  options['f_reg']= 0.0001
  cmodels= [TLWR(),TLWR()]
  cmodels[0].Load({'options':options})
  cmodels[1].Load({'options':options})
  if load_model:
    cmodels[0].Load(LoadYAML('/tmp/lwr/0_lwr_model.yaml'), '/tmp/lwr/0_')
    cmodels[1].Load(LoadYAML('/tmp/lwr/1_lwr_model.yaml'), '/tmp/lwr/1_')
  model= TCombinedFuncApprox(3,2,([0,1,2],[0],cmodels[0]),([0,1,2],[1],cmodels[1]))
  #model= TCombinedFuncApprox(3,2,([0,1,2],[0],cmodels[0]),([0],[1],cmodels[1]))
  model.Init()

  src_file= demo_dir+'data/ode_f3_smp.dat'; dim= [3,2]
  #assess= lambda y: 5.0*y[0]+y[1]

  if not load_model:
    fp= file(src_file)
    while True:
      line= fp.readline()
      if not line: break
      data= line.split()
      model.Update(map(float,data[0:dim[0]]),
                   map(float,data[dim[0]:sum(dim)]))
    SaveYAML(cmodels[0].Save('/tmp/lwr/0_'), '/tmp/lwr/0_lwr_model.yaml')
    SaveYAML(cmodels[1].Save('/tmp/lwr/1_'), '/tmp/lwr/1_lwr_model.yaml')

  DataX= cmodels[0].DataX
  mi= [min([x[d] for x in DataX]) for d in range(len(DataX[0]))]
  ma= [max([x[d] for x in DataX]) for d in range(len(DataX[0]))]
  #me= [Median([x[d] for x in DataX]) for d in range(len(DataX[0]))]
  f_reduce=lambda x:[x[0],x[2]]
  f_repair=lambda x,mi,ma,me:[x[0],me[1],x[1]]
  DumpPlot(model, f_reduce=f_reduce, f_repair=f_repair, file_prefix='/tmp/lwr/f3', x_var=[0.,0.,0.], bounds=[mi,ma])
  #NOTE: f3_smp.dat is not saved as model itself doesn't have DataX and DataY.
  fp= open('/tmp/lwr/f3_sample.dat','w')
  for x1,y1,y2 in zip(DataX, cmodels[0].DataY, cmodels[1].DataY):
    y= [y1[0],y2[0]]
    fp.write('%s\n' % ToStr(f_reduce(x1),x1,y))
  fp.close()

def PlotGraphs():
  print 'Plotting graphs..'
  import os
  commands=[
    '''qplot -x2 aaa -3d
          -s 'set xlabel "x[0]";set ylabel "x[2]";set title "y[0]";set ticslevel 0;'
          /tmp/lwr/f3_est.dat u 1:2:6 w l /tmp/lwr/f3_sample.dat u 1:2:6 ps 3 &''',
    '''qplot -x2 aaa -3d
          -s 'set xlabel "x[0]";set ylabel "x[2]";set title "y[1]";set ticslevel 0;'
          /tmp/lwr/f3_est.dat u 1:2:7 w l /tmp/lwr/f3_sample.dat u 1:2:7 ps 3 &''',
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
