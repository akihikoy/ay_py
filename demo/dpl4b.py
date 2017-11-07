#!/usr/bin/python
#\file    dpl4b.py
#\brief   Test dpl4.py with toy_arm1.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jan.20, 2016
from _path import *
from ay_py.core import *
from toy_arm1 import *

def Ffk_dFfk(arm, q, with_grad):
  if not with_grad:
    return arm.FK2D(q)
  else:
    x,J= arm.FK2D(q,with_J=True)
    return x, J.T

def Fmv_dFmv(arm, q_qcmd, with_grad):
  q= q_qcmd[arm.N:]
  if not with_grad:  return q
  grad= np.zeros((arm.N*2,arm.N))
  grad[arm.N:,:]= np.eye(arm.N,arm.N)
  return q, grad

def Main(logdir='/tmp/arm/', options_in={}):
  arm= TArm(10)

  domain= TGraphDynDomain()
  SP= TCompSpaceDef
  domain.SpaceDefs={
    'q_cmd': SP('action',arm.N,min=[-5.0*math.pi]*arm.N,max=[5.0*math.pi]*arm.N),
    'q': SP('state',arm.N),
    'x': SP('state',2),
    'x_trg': SP('state',2),
    REWARD_KEY:  SP('state',1),
    }
  domain.Models={
    #key:[In,Out,F],
    #'Ffk': [['q_cmd'],['x'], TLocalLinear(arm.N,2,lambda q:arm.FK2D(q),lambda q:arm.FK2D(q,with_J=True)[1].T)],
    'Ffk': [['q_cmd'],['x'], TLocalLinear(arm.N,2,FdF=lambda q,with_grad:Ffk_dFfk(arm, q, with_grad))],
    #'Ffk': [['q_cmd'],['x'], None],
    #'Fmv': [['q','q_cmd'],['q'], TLocalLinear(arm.N*2,arm.N,lambda q_qcmd:q_qcmd[arm.N:],lambda q_qcmd:dFmv(q_qcmd))],
    'Fmv': [['q','q_cmd'],['q'], TLocalLinear(arm.N*2,arm.N,FdF=lambda q_qcmd,with_grad:Fmv_dFmv(arm,q_qcmd,with_grad))],
    'Reng': [['q','q_cmd'],[REWARD_KEY], TQuadratic2(arm.N,-1.0e-6)],
    'Rfk': [['x','x_trg'],[REWARD_KEY], TQuadratic2(2,-1.0)],
    'P1':  [[],[PROB_KEY], TLocalLinear(0,1,lambda x:[1.0],lambda x:[0.0])],
    'P3':  [[],[PROB_KEY], TLocalLinear(0,3,lambda x:[1.0]*3,lambda x:[0.0]*3)],
    }
  domain.Graph={
    'n0': TDynNode(None,'P3',('Ffk','n0fk'),('Fmv','n0mv'),('Reng','n0er')),
    'n0fk': TDynNode('n0','P1',('Rfk','n0fkr')),
    'n0mv': TDynNode('n0'),
    'n0er': TDynNode('n0'),
    'n0fkr': TDynNode('n0fk'),
    }

  dpl_options= {}
  dpl_options['base_dir']= logdir
  dpl= TGraphDynPlanLearn(domain)
  dpl.Load({'options':dpl_options})
  dpl.Init()

  for i in range(1):
    dpl.NewEpisode()

    xs0= {}  #XSSA
    xs0['q']= SSA([0.0]*arm.N)
    #xs0['x_trg']= SSA([0.5,0.5])
    #xs0['q']= SSA([Rand(-math.pi,math.pi) for d in range(arm.N)])
    #xs0['x_trg']= SSA(arm.FK2D(np.array([Rand(-math.pi,math.pi) for d in range(arm.N)])))
    xs0['x_trg']= SSA([Rand(-1.0,1.0),Rand(-1.0,1.0)])
    if Norm(xs0['x_trg'].X)>0.9*arm.Len:  xs0['x_trg'].X*= 0.9*arm.Len/Norm(xs0['x_trg'].X)

    #xs0['q_cmd']= SSA([0.0]*arm.N)  #Init guess
    xs0['q_cmd']= copy.deepcopy(xs0['q'])  #Init guess
    res= dpl.Plan('n0', xs0)
    print 'xs0(after plan)=',xs0
    #db_n0= dpl.DB.AddToSeq(parent=None,name='n0',xs=xs0)

    ptree= res.PTree
    CPrint(1,'R=',dpl.Value(ptree))
    CPrint(1,'x_trg=',ptree.Tree[TPair('n0fkr',0)].XS['x_trg'])
    CPrint(1,'x=',ptree.Tree[TPair('n0fkr',0)].XS['x'])

    dpl.EndEpisode()

    Visualize2D(arm, ToList(xs0['q'].X), ToList(xs0['q_cmd'].X), ToList(xs0['x_trg'].X), '/tmp/arm/arm.dat')
    Visualize2D(arm, ToList(xs0['q'].X), ToList(xs0['q_cmd'].X), ToList(xs0['x_trg'].X), '/tmp/arm/arm{count}.dat'.format(count=i))

  #SaveYAML(dpl.MM.Save(dpl.MM.Options['base_dir']), dpl.MM.Options['base_dir']+'model_mngr.yaml')
  #SaveYAML(dpl.DB.Save(), logdir+'database.yaml')
  SaveYAML(dpl.Save(), logdir+'dpl.yaml')


if __name__=='__main__':
  import sys as sys
  if len(sys.argv)>1 and sys.argv[1] in ('p','plot','Plot','PLOT'):
    PlotGraphs()
    sys.exit(0)
  Main()
