#!/usr/bin/python
#\file    dpl4c.py
#\brief   Test dpl4.py with toy_arm1 and multiple targets.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jan.21, 2016
from _path import *
from ay_py.core import *
from toy_arm1 import *

from dpl4b import (
  Ffk_dFfk,
  Fmv_dFmv
  )

def Main(logdir='/tmp/arm/', options_in={}):
  arm= TArm(10)

  Nsel= 2
  domain= TGraphDynDomain()
  SP= TCompSpaceDef
  domain.SpaceDefs={
    'q_cmd': SP('state',arm.N),
    'Q_cmd': SP('action',arm.N*Nsel,min=[-5.0*math.pi]*arm.N*Nsel,max=[5.0*math.pi]*arm.N*Nsel),
    's': SP('select',num=Nsel),
    'k_rep': SP('state',1),
    'q': SP('state',arm.N),
    'x': SP('state',2),
    'x_trg': SP('state',2),
    'X_trg': SP('state',2*Nsel),
    REWARD_KEY:  SP('state',1),
    }
  def Delta1(dim,s):
    assert(abs(s-int(s))<1.0e-6)
    p= [0.0]*dim
    p[int(s)]= 1.0
    return p
  def Order(s,k_rep):
    if s==0:  return (0,1)[k_rep]
    if s==1:  return (1,0)[k_rep]
  def Fsel(X):
    Q_cmd,X_trg,k_rep,s= X[0:arm.N*Nsel],X[arm.N*Nsel:arm.N*Nsel+2*Nsel],X[arm.N*Nsel+2*Nsel],X[arm.N*Nsel+2*Nsel+1]
    assert(abs(k_rep-int(k_rep))<1.0e-6)
    order= Order(s,int(k_rep))
    q_cmd= Q_cmd[arm.N*order:arm.N*(order+1)]
    x_trg= X_trg[2*order:2*(order+1)]
    k_rep= k_rep+1
    Y= [0.0]*(arm.N+2+1)
    Y[0:arm.N]= q_cmd
    Y[arm.N:arm.N+2]= x_trg
    Y[arm.N+2]= k_rep
    return Y
  def dFsel(X):
    k_rep,s= X[arm.N*Nsel+2*Nsel],X[arm.N*Nsel+2*Nsel+1]
    assert(abs(k_rep-int(k_rep))<1.0e-6)
    order= Order(s,int(k_rep))
    grad= np.zeros((arm.N*Nsel+2*Nsel+2, arm.N+2+1))
    #corresponding q_cmd:
    grad[arm.N*order:arm.N*(order+1), 0:arm.N]= np.eye(arm.N,arm.N)
    #corresponding x_trg:
    grad[arm.N*Nsel+2*order:arm.N*Nsel+2*(order+1), arm.N:arm.N+2]= np.eye(2,2)
    return grad
  domain.Models={
    #key:[In,Out,F],
    'Fnone': [[],[], None],
    'Ffk': [['q_cmd'],['x'], TLocalLinear(arm.N,2,FdF=lambda q,with_grad:Ffk_dFfk(arm, q, with_grad))],
    #'Ffk': [['q_cmd'],['x'], None],
    'Fmv': [['q','q_cmd'],['q'], TLocalLinear(arm.N*2,arm.N,FdF=lambda q_qcmd,with_grad:Fmv_dFmv(arm,q_qcmd,with_grad))],
    'Reng': [['q','q_cmd'],[REWARD_KEY], TQuadratic2(arm.N,-1.0e-6)],
    'Rfk': [['x','x_trg'],[REWARD_KEY], TQuadratic2(2,-1.0)],
    'Fsel': [['Q_cmd','X_trg','k_rep','s'],['q_cmd','x_trg','k_rep'], TLocalLinear(arm.N*Nsel+2*Nsel+2,arm.N+2+1,lambda x:Fsel(x),lambda x:dFsel(x))],
    'Ps': [['s'],[PROB_KEY], TLocalLinear(0,Nsel,lambda s:Delta1(Nsel,s[0]),lambda s:[0.0]*Nsel)],
    'P1':  [[],[PROB_KEY], TLocalLinear(0,1,lambda x:[1.0],lambda x:[0.0])],
    'P3':  [[],[PROB_KEY], TLocalLinear(0,3,lambda x:[1.0]*3,lambda x:[0.0]*3)],
    }
  domain.Graph={
    'n0': TDynNode(None,'Ps',('Fnone','n10'),('Fnone','n20')),

    'n10': TDynNode('n0','P1',('Fsel','n11')),
    'n11': TDynNode('n10','P3',('Ffk','n11fk'),('Fmv','n12'),('Reng','n11er')),
    'n11fk': TDynNode('n11','P1',('Rfk','n11fkr')),
    'n11er': TDynNode('n11'),
    'n11fkr': TDynNode('n11fk'),

    'n12': TDynNode('n11','P1',('Fsel','n13')),
    'n13': TDynNode('n12','P3',('Ffk','n13fk'),('Fmv','n14'),('Reng','n13er')),
    'n13fk': TDynNode('n13','P1',('Rfk','n13fkr')),
    'n13er': TDynNode('n13'),
    'n13fkr': TDynNode('n13fk'),
    'n14': TDynNode('n13'),

    'n20': TDynNode('n0','P1',('Fsel','n21')),
    'n21': TDynNode('n20','P3',('Ffk','n21fk'),('Fmv','n22'),('Reng','n21er')),
    'n21fk': TDynNode('n21','P1',('Rfk','n21fkr')),
    'n21er': TDynNode('n21'),
    'n21fkr': TDynNode('n21fk'),

    'n22': TDynNode('n21','P1',('Fsel','n23')),
    'n23': TDynNode('n22','P3',('Ffk','n23fk'),('Fmv','n24'),('Reng','n23er')),
    'n23fk': TDynNode('n23','P1',('Rfk','n23fkr')),
    'n23er': TDynNode('n23'),
    'n23fkr': TDynNode('n23fk'),
    'n24': TDynNode('n23'),
    }

  dpl_options= {}
  dpl_options['base_dir']= logdir
  dpl= TGraphDynPlanLearn(domain)
  dpl.Load({'options':dpl_options})
  dpl.Init()

  for i in range(1):
    dpl.NewEpisode()

    xs0= {}  #XSSA
    xs0['k_rep']= SSA([0])
    xs0['q']= SSA([0.0]*arm.N)
    #xs0['x_trg']= SSA([0.5,0.5])
    #xs0['q']= SSA([Rand(-math.pi,math.pi) for d in range(arm.N)])
    xs0['X_trg']= SSA([0.0]*2*Nsel)
    for trg in range(Nsel):
      x_trg= MCVec([Rand(-1.0,1.0),Rand(-1.0,1.0)])
      if Norm(x_trg)>0.9*arm.Len:  x_trg*= 0.9*arm.Len/Norm(x_trg)
      xs0['X_trg'].X[2*trg:2*(trg+1)]= MCVec(x_trg)

    #dpl.Update(0,xs0)
    #x1= ToList(dpl.Select(0,x0))
    #xs0['Q_cmd']= SSA([0.0]*arm.N*Nsel)  #Init guess
    xs0['Q_cmd']= SSA(ToList(xs0['q'].X)*Nsel)
    res= dpl.Plan('n0', xs0)
    print 'xs0(after plan)=',xs0
    #db_n0= dpl.DB.AddToSeq(parent=None,name='n0',xs=xs0)

    ptree= res.PTree
    CPrint(1,'R=',dpl.Value(ptree))
    CPrint(1,'X_trg=',xs0['X_trg'])
    if xs0['s'].X[0]==0:
      CPrint(1,'x10=',ptree.Tree[TPair('n11fkr',0)].XS['x'])
      CPrint(1,'x11=',ptree.Tree[TPair('n13fkr',0)].XS['x'])
      xs1= ptree.Tree[TPair('n13',0)].XS
      Visualize2D(arm, ToList(xs0['q'].X), ToList(xs0['Q_cmd'].X)[0:arm.N], ToList(xs0['X_trg'].X)[0:2], '/tmp/arm/arm1.dat')
      Visualize2D(arm, ToList(xs0['q'].X), ToList(xs0['Q_cmd'].X)[0:arm.N], ToList(xs0['X_trg'].X)[0:2], '/tmp/arm/arm1_{count}.dat'.format(count=i))
      Visualize2D(arm, ToList(xs1['q'].X), ToList(xs0['Q_cmd'].X)[arm.N:arm.N*2], ToList(xs0['X_trg'].X)[2:2*2], '/tmp/arm/arm2.dat')
      Visualize2D(arm, ToList(xs1['q'].X), ToList(xs0['Q_cmd'].X)[arm.N:arm.N*2], ToList(xs0['X_trg'].X)[2:2*2], '/tmp/arm/arm2_{count}.dat'.format(count=i))
    elif xs0['s'].X[0]==1:
      CPrint(1,'x20=',ptree.Tree[TPair('n21fkr',0)].XS['x'])
      CPrint(1,'x21=',ptree.Tree[TPair('n23fkr',0)].XS['x'])
      xs1= ptree.Tree[TPair('n23',0)].XS
      Visualize2D(arm, ToList(xs0['q'].X), ToList(xs0['Q_cmd'].X)[arm.N:arm.N*2], ToList(xs0['X_trg'].X)[2:2*2], '/tmp/arm/arm1.dat')
      Visualize2D(arm, ToList(xs0['q'].X), ToList(xs0['Q_cmd'].X)[arm.N:arm.N*2], ToList(xs0['X_trg'].X)[2:2*2], '/tmp/arm/arm1_{count}.dat'.format(count=i))
      Visualize2D(arm, ToList(xs1['q'].X), ToList(xs0['Q_cmd'].X)[0:arm.N], ToList(xs0['X_trg'].X)[0:2], '/tmp/arm/arm2.dat')
      Visualize2D(arm, ToList(xs1['q'].X), ToList(xs0['Q_cmd'].X)[0:arm.N], ToList(xs0['X_trg'].X)[0:2], '/tmp/arm/arm2_{count}.dat'.format(count=i))


    dpl.EndEpisode()
    #xar_data= dpl.DB.Entry[-1].Dump()
    #fp.write(xar_data+'\n')
    #print i,xar_data

  #SaveYAML(dpl.MM.Save(dpl.MM.Options['base_dir']), dpl.MM.Options['base_dir']+'model_mngr.yaml')
  #SaveYAML(dpl.DB.Save(), logdir+'database.yaml')
  SaveYAML(dpl.Save(), logdir+'dpl.yaml')


def PlotGraphs():
  print 'Plotting graphs..'
  import os
  commands=[
    '''qplot -x2 aaa
          -s 'set xlabel "x";set ylabel "z";'
          -s 'set xrange [-1.2:1.2];set yrange[-1.2:1.2];'
          -s 'set size square;'
          /tmp/arm/arm1.dat u 3:4 w p ps 3 pt 5 lt 4
          /tmp/arm/arm1.dat w l lw 6 lt 2
          , '"< tail -2 /tmp/arm/arm1.dat"' w p ps 3 pt 6 lw 3 lt 2
          /tmp/arm/arm2.dat u 3:4 w p ps 3 pt 5 lt 1
          /tmp/arm/arm2.dat w l lw 3 lt 3
          , '"< tail -2 /tmp/arm/arm2.dat"' w p ps 3 pt 6 lw 3 lt 3
      &''',
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
  import sys as sys
  if len(sys.argv)>1 and sys.argv[1] in ('p','plot','Plot','PLOT'):
    PlotGraphs()
    sys.exit(0)
  Main()
