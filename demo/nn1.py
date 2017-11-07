#!/usr/bin/python
#\file    nn1.py
#\brief   Train NN with 1d catapult data (simulation);
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Apr.14, 2017
from _path import *
from ay_py.core import *

modeldir= '/tmp/c1/'

def TrainNN():
  #data= [[e['action']['pos_target'],e['result']['loc_land']] for e in LoadYAML('data/catapult1.yaml')]
  #data= map(lambda row:map(float,row.split()), open('data/catapult1.dat','r').read().split('\n'))
  data= [map(float,row.split()) for row in open(demo_dir+'data/catapult1.dat','r').read().split('\n') if row!='']

  dim_in= 1
  dim_out= 1
  options={
    'base_dir': modeldir+'data/catapult1_model/',
    'n_units': [dim_in] + [200,200] + [dim_out],
    'name': 'FCatapult1',
    'loss_stddev_stop': 1.0e-6,
    'num_max_update': 40000,
    #'dropout': False,
    #'dropout_ratio': 0.001,
    'batchsize': 100,
    }
  prefix= modeldir+'data/catapult1_model/FCatapult1'
  NN= TNNRegression()
  NN.Load(data={'options':options})
  NN.Init()

  for x,y in data:
    NN.Update([x], [y], not_learn=True)
  NN.UpdateBatch()
  SaveYAML(NN.Save(prefix), prefix+'.yaml')

def Main(logdir='/tmp/c1/', options_in={}):
  #Uncomment this to train the model:
  print 'Run TrainNN?'
  if AskYesNo():  TrainNN()
  #return

  #Load existing model
  NN= TNNRegression()
  prefix= modeldir+'data/catapult1_model/FCatapult1'
  NN.Load(LoadYAML(prefix+'.yaml'), prefix)
  NN.Init()

  fp= OpenW(logdir+'data.dat')
  for x,y in zip(ToList(NN.DataX),ToList(NN.DataY)):
    fp.write('%f %f\n'%(x,y))
  fp.close()
  fp= OpenW(logdir+'nn_est.dat')
  for x in np.arange(0.0,2.4,0.01):
    pred= NN.Predict([x],x_var=0.0**2,with_var=True,with_grad=False)
    y= pred.Y[0,0]
    y_var= pred.Var[0,0]
    fp.write('%f %f %f\n'%(x,y,y_var))
  fp.close()

def PlotGraphs(argv):
  print 'Plotting graphs..'
  import os
  logdir= argv[0] if len(argv)>0 else '/tmp/c1/'
  qopt= argv[1] if len(argv)>1 else ''
  commands=[
    '''qplot -x2 aaa
      -s 'unset key'
      /tmp/c1/nn_est.dat w yerrorbar
      /tmp/c1/nn_est.dat w l lw 3
      /tmp/c1/data.dat w p
      &
      ''',
    '''''',
    ]
  for cmd in commands:
    if cmd!='':
      cmd= ' '.join(cmd.splitlines()).replace('/tmp/c1/',logdir)
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
