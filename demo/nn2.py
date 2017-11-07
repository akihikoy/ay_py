#!/usr/bin/python
#\file    nn1.py
#\brief   Train NN for IRIS classification;
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    May.29, 2017
from _path import *
from ay_py.core import *

modeldir= '/tmp/c2/'

def TrainNN():
  data_x= [[5.1, 3.5, 1.4, 0.2], [4.9, 3.0, 1.4, 0.2], [4.7, 3.2, 1.3, 0.2], [4.6, 3.1, 1.5, 0.2], [5.0, 3.6, 1.4, 0.2], [5.4, 3.9, 1.7, 0.4], [4.6, 3.4, 1.4, 0.3], [5.0, 3.4, 1.5, 0.2], [4.4, 2.9, 1.4, 0.2], [4.9, 3.1, 1.5, 0.1], [5.4, 3.7, 1.5, 0.2], [4.8, 3.4, 1.6, 0.2], [4.8, 3.0, 1.4, 0.1], [4.3, 3.0, 1.1, 0.1], [5.8, 4.0, 1.2, 0.2], [5.7, 4.4, 1.5, 0.4], [5.4, 3.9, 1.3, 0.4], [5.1, 3.5, 1.4, 0.3], [5.7, 3.8, 1.7, 0.3], [5.1, 3.8, 1.5, 0.3], [5.4, 3.4, 1.7, 0.2], [5.1, 3.7, 1.5, 0.4], [4.6, 3.6, 1.0, 0.2], [5.1, 3.3, 1.7, 0.5], [4.8, 3.4, 1.9, 0.2], [5.0, 3.0, 1.6, 0.2], [5.0, 3.4, 1.6, 0.4], [5.2, 3.5, 1.5, 0.2], [5.2, 3.4, 1.4, 0.2], [4.7, 3.2, 1.6, 0.2], [4.8, 3.1, 1.6, 0.2], [5.4, 3.4, 1.5, 0.4], [5.2, 4.1, 1.5, 0.1], [5.5, 4.2, 1.4, 0.2], [4.9, 3.1, 1.5, 0.1], [5.0, 3.2, 1.2, 0.2], [5.5, 3.5, 1.3, 0.2], [4.9, 3.1, 1.5, 0.1], [4.4, 3.0, 1.3, 0.2], [5.1, 3.4, 1.5, 0.2], [5.0, 3.5, 1.3, 0.3], [4.5, 2.3, 1.3, 0.3], [4.4, 3.2, 1.3, 0.2], [5.0, 3.5, 1.6, 0.6], [5.1, 3.8, 1.9, 0.4], [4.8, 3.0, 1.4, 0.3], [5.1, 3.8, 1.6, 0.2], [4.6, 3.2, 1.4, 0.2], [5.3, 3.7, 1.5, 0.2], [5.0, 3.3, 1.4, 0.2], [7.0, 3.2, 4.7, 1.4], [6.4, 3.2, 4.5, 1.5], [6.9, 3.1, 4.9, 1.5], [5.5, 2.3, 4.0, 1.3], [6.5, 2.8, 4.6, 1.5], [5.7, 2.8, 4.5, 1.3], [6.3, 3.3, 4.7, 1.6], [4.9, 2.4, 3.3, 1.0], [6.6, 2.9, 4.6, 1.3], [5.2, 2.7, 3.9, 1.4], [5.0, 2.0, 3.5, 1.0], [5.9, 3.0, 4.2, 1.5], [6.0, 2.2, 4.0, 1.0], [6.1, 2.9, 4.7, 1.4], [5.6, 2.9, 3.6, 1.3], [6.7, 3.1, 4.4, 1.4], [5.6, 3.0, 4.5, 1.5], [5.8, 2.7, 4.1, 1.0], [6.2, 2.2, 4.5, 1.5], [5.6, 2.5, 3.9, 1.1], [5.9, 3.2, 4.8, 1.8], [6.1, 2.8, 4.0, 1.3], [6.3, 2.5, 4.9, 1.5], [6.1, 2.8, 4.7, 1.2], [6.4, 2.9, 4.3, 1.3], [6.6, 3.0, 4.4, 1.4], [6.8, 2.8, 4.8, 1.4], [6.7, 3.0, 5.0, 1.7], [6.0, 2.9, 4.5, 1.5], [5.7, 2.6, 3.5, 1.0], [5.5, 2.4, 3.8, 1.1], [5.5, 2.4, 3.7, 1.0], [5.8, 2.7, 3.9, 1.2], [6.0, 2.7, 5.1, 1.6], [5.4, 3.0, 4.5, 1.5], [6.0, 3.4, 4.5, 1.6], [6.7, 3.1, 4.7, 1.5], [6.3, 2.3, 4.4, 1.3], [5.6, 3.0, 4.1, 1.3], [5.5, 2.5, 4.0, 1.3], [5.5, 2.6, 4.4, 1.2], [6.1, 3.0, 4.6, 1.4], [5.8, 2.6, 4.0, 1.2], [5.0, 2.3, 3.3, 1.0], [5.6, 2.7, 4.2, 1.3], [5.7, 3.0, 4.2, 1.2], [5.7, 2.9, 4.2, 1.3], [6.2, 2.9, 4.3, 1.3], [5.1, 2.5, 3.0, 1.1], [5.7, 2.8, 4.1, 1.3], [6.3, 3.3, 6.0, 2.5], [5.8, 2.7, 5.1, 1.9], [7.1, 3.0, 5.9, 2.1], [6.3, 2.9, 5.6, 1.8], [6.5, 3.0, 5.8, 2.2], [7.6, 3.0, 6.6, 2.1], [4.9, 2.5, 4.5, 1.7], [7.3, 2.9, 6.3, 1.8], [6.7, 2.5, 5.8, 1.8], [7.2, 3.6, 6.1, 2.5], [6.5, 3.2, 5.1, 2.0], [6.4, 2.7, 5.3, 1.9], [6.8, 3.0, 5.5, 2.1], [5.7, 2.5, 5.0, 2.0], [5.8, 2.8, 5.1, 2.4], [6.4, 3.2, 5.3, 2.3], [6.5, 3.0, 5.5, 1.8], [7.7, 3.8, 6.7, 2.2], [7.7, 2.6, 6.9, 2.3], [6.0, 2.2, 5.0, 1.5], [6.9, 3.2, 5.7, 2.3], [5.6, 2.8, 4.9, 2.0], [7.7, 2.8, 6.7, 2.0], [6.3, 2.7, 4.9, 1.8], [6.7, 3.3, 5.7, 2.1], [7.2, 3.2, 6.0, 1.8], [6.2, 2.8, 4.8, 1.8], [6.1, 3.0, 4.9, 1.8], [6.4, 2.8, 5.6, 2.1], [7.2, 3.0, 5.8, 1.6], [7.4, 2.8, 6.1, 1.9], [7.9, 3.8, 6.4, 2.0], [6.4, 2.8, 5.6, 2.2], [6.3, 2.8, 5.1, 1.5], [6.1, 2.6, 5.6, 1.4], [7.7, 3.0, 6.1, 2.3], [6.3, 3.4, 5.6, 2.4], [6.4, 3.1, 5.5, 1.8], [6.0, 3.0, 4.8, 1.8], [6.9, 3.1, 5.4, 2.1], [6.7, 3.1, 5.6, 2.4], [6.9, 3.1, 5.1, 2.3], [5.8, 2.7, 5.1, 1.9], [6.8, 3.2, 5.9, 2.3], [6.7, 3.3, 5.7, 2.5], [6.7, 3.0, 5.2, 2.3], [6.3, 2.5, 5.0, 1.9], [6.5, 3.0, 5.2, 2.0], [6.2, 3.4, 5.4, 2.3], [5.9, 3.0, 5.1, 1.8]]
  data_x= [x[1:3] for x in data_x]
  data_y= [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]

  dim_in= 2
  dim_out= 3
  options={
    'base_dir': modeldir+'data/iris1_model/',
    'n_units': [dim_in] + [200,200] + [dim_out],
    'name': 'Firis1',
    'loss_stddev_stop': 1.0e-6,
    'num_max_update': 2000,
    #'dropout': False,
    #'dropout_ratio': 0.001,
    #'batchsize': 100,
    }
  prefix= modeldir+'data/iris1_model/Firis1'
  NN= TNNClassification()
  NN.Load(data={'options':options})
  NN.Init()

  NN.UpdateBatch(data_x, data_y)
  SaveYAML(NN.Save(prefix), prefix+'.yaml')

def Main(logdir='/tmp/c2/', options_in={}):
  #Uncomment this to train the model:
  print 'Run TrainNN?'
  if AskYesNo():  TrainNN()
  #return

  #Load existing model
  NN= TNNClassification()
  prefix= modeldir+'data/iris1_model/Firis1'
  NN.Load(LoadYAML(prefix+'.yaml'), prefix)
  NN.Init()

  fp= OpenW(logdir+'data.dat')
  for x,y in zip(NN.DataX, NN.DataY):
    fp.write('%s %s\n'%(ToStr(x),str(y)))
  fp.close()
  fp= OpenW(logdir+'nn_est.dat')
  for x1 in np.mgrid[2.0:4.5:51j]:
    for x2 in np.mgrid[1.0:7.0:51j]:
      x= [x1,x2]
      pred= NN.Predict(x,x_var=0.0**2,with_var=False,with_grad=False)
      y= pred.Y.tolist()
      ymax= y.index(max(y))
      fp.write('%s %s %s\n'%(ToStr(x),ymax,ToStr(y)))
    fp.write('\n')
  fp.close()

def PlotGraphs(argv):
  print 'Plotting graphs..'
  import os
  logdir= argv[0] if len(argv)>0 else '/tmp/c2/'
  qopt= argv[1] if len(argv)>1 else ''
  commands=[
    '''qplot -x2 aaa
          -s 'set xlabel "x1";set ylabel "x2";set title "";'
          -s 'set encoding utf8;symbol(z)="+xo%#"[int(z):int(z)];'
          /tmp/c2/nn_est.dat u 1:2:'(symbol($3+1))' w labels textcolor lt 3 t '"Final epoch"'
          /tmp/c2/data.dat u 1:2:'(symbol($3+1))' w labels textcolor lt 1 t '"sample"'  &''',
    '''qplot -x2 aaa -3d
          -s 'set xlabel "x1";set ylabel "x2";set title "Class 0";'
          -s 'set pm3d;unset surface;set view map;'
          -s 'set encoding utf8;symbol(z)="+xo%#"[int(z):int(z)];'
          /tmp/c2/nn_est.dat u 1:2:4 t '""'
          /tmp/c2/data.dat u 1:2:'(0.0)':'(symbol($3+1))' w labels textcolor lt 1 t '"sample"'  &''',
    '''qplot -x2 aaa -3d
          -s 'set xlabel "x1";set ylabel "x2";set title "Class 1";'
          -s 'set encoding utf8;symbol(z)="+xo%#"[int(z):int(z)];'
          -s 'set pm3d;unset surface;set view map;'
          /tmp/c2/nn_est.dat u 1:2:5 t '""'
          /tmp/c2/data.dat u 1:2:'(0.0)':'(symbol($3+1))' w labels textcolor lt 1 t '"sample"'  &''',
    '''qplot -x2 aaa -3d
          -s 'set xlabel "x1";set ylabel "x2";set title "Class 2";'
          -s 'set pm3d;unset surface;set view map;'
          -s 'set encoding utf8;symbol(z)="+xo%#"[int(z):int(z)];'
          /tmp/c2/nn_est.dat u 1:2:6 t '""'
          /tmp/c2/data.dat u 1:2:'(0.0)':'(symbol($3+1))' w labels textcolor lt 1 t '"sample"'  &''',
    '''''',
    ]
  for cmd in commands:
    if cmd!='':
      cmd= ' '.join(cmd.splitlines()).replace('/tmp/c2/',logdir)
      if qopt!='':
        cmd= cmd.replace('qplot -x2 aaa','qplot '+qopt)
        if cmd[-1]=='&':  cmd= cmd[:-1]
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
    PlotGraphs(sys.argv[2:])
    sys.exit(0)
  Main()
