#!/usr/bin/python
#\file    dpl4e.py
#\brief   Test dpl4.py with toy_arm1, trajectory optimization, collision check.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jan.25, 2016
from _path import *
from ay_py.core import *
from toy_arm1 import *

from dpl4d import (
  GetParameterizedSplines,
  Ftraj,
  Ftrfk_dF,
  Rtreng,
  Rtrfk,
  VizTraj,
  LinePointDist,
  )

def CheckCollision(arm,Nobs,X_obs,q,r_obs=0.3):
  X= []
  arm.FK2D(q,viz=lambda x:X.append(x))
  X= np.array(X).reshape(arm.N+1,2)
  X_obs= np.array(X_obs).reshape(Nobs,2)
  Rd= 0.0
  for n in range(arm.N):
    x1,x2= X[n:n+2]
    for x_o in X_obs:
      d= LinePointDist(x1,x2,x_o)
      if d<r_obs:
        Rd+= d*d
  return Rd

def VizObstacles(Nobs, X_obs, file_name, r_obs=0.3):
  fp= open(file_name,'w')
  X_obs= np.array(X_obs).reshape(Nobs,2)
  for x_o in X_obs:
    for th in FRange1(0.0,6.3,50):
      x= x_o+np.array([r_obs*math.cos(th),r_obs*math.sin(th)])
      fp.write('{x} {z}\n'.format(x=x[0],z=x[1]))
    fp.write('\n')
  fp.close()

def Rcol(arm,Ntraj,Nobs,q_traj_X_obs,w):
  q_traj= q_traj_X_obs[:arm.N*Ntraj]
  X_obs= q_traj_X_obs[arm.N*Ntraj:]
  #q_traj= np.array(q_traj).reshape(Ntraj,arm.N)
  #X_obs= np.array(X_obs).reshape(Nobs,2)
  R= 0.0
  for q in np.array(q_traj).reshape(Ntraj,arm.N):
    R+= w*CheckCollision(arm,Nobs,X_obs,q)
  return [R]

def Main(logdir='/tmp/arm/', options_in={}):
  arm= TArm(6)
  Ntraj= 10
  Nobs= 3

  domain= TGraphDynDomain()
  SP= TCompSpaceDef
  domain.SpaceDefs={
    #'q_cmd': SP('action',arm.N,min=[-5.0*math.pi]*arm.N,max=[5.0*math.pi]*arm.N),
    'pqf': SP('action',arm.N,min=[-5.0*math.pi]*arm.N,max=[5.0*math.pi]*arm.N),
    'pvia': SP('action',arm.N*2,min=[-math.pi,-math.pi]*arm.N,max=[math.pi,math.pi]*arm.N),
    'q': SP('state',arm.N),
    'q_traj': SP('state',arm.N*Ntraj),
    'x_traj': SP('state',2*Ntraj),
    'x_trg': SP('state',2),
    'X_obs': SP('state',2*Nobs),  #Positions of obstacles
    REWARD_KEY:  SP('state',1),
    }
  def dFmv(q_qcmd):
    grad= np.zeros((arm.N*2,arm.N))
    grad[arm.N:,:]= np.eye(arm.N,arm.N)
    return grad
  #FIXME:TODO:Calculate gradients of: Ftraj, Rtreng, Rtrfk, Rcol
  dRcol= lambda x:np.zeros((arm.N*Ntraj+2*Nobs,1))  #FIXME:Since Taylor expansion of Rcol is too heavy, we just use a zero mat.
  domain.Models={
    #key:[In,Out,F],
    'Ftraj': [['q','pqf','pvia'],['q_traj'], TLocalLinear(arm.N+arm.N+arm.N*2,arm.N*Ntraj,lambda qp:Ftraj(arm,Ntraj,qp),None)],
    'Ftrfk': [['q_traj'],['x_traj'], TLocalLinear(arm.N*Ntraj,2*Ntraj,FdF=lambda q_traj,with_grad:Ftrfk_dF(arm,Ntraj,q_traj,with_grad))],
    'Rtreng': [['q_traj'],[REWARD_KEY], TLocalLinear(arm.N*Ntraj,1,lambda q_traj:Rtreng(Ntraj,q_traj,-1.0e-6),None)],
    'Rtrfk': [['x_traj','x_trg'],[REWARD_KEY], TLocalLinear(2*Ntraj+2,1,lambda x_traj_trg:Rtrfk(Ntraj,x_traj_trg,-1.0,-1.0e-2),None)],
    'Rcol': [['q_traj','X_obs'],[REWARD_KEY], TLocalLinear(arm.N*Ntraj+2*Nobs,1,lambda q_traj_X_obs:Rcol(arm,Ntraj,Nobs,q_traj_X_obs,-1.0),dRcol)],
    'P1':  [[],[PROB_KEY], TLocalLinear(0,1,lambda x:[1.0],lambda x:[0.0])],
    'P2':  [[],[PROB_KEY], TLocalLinear(0,2,lambda x:[1.0]*2,lambda x:[0.0]*2)],
    'P3':  [[],[PROB_KEY], TLocalLinear(0,3,lambda x:[1.0]*3,lambda x:[0.0]*3)],
    }
  domain.Graph={
    'n0': TDynNode(None,'P1',('Ftraj','n1')),
    'n1': TDynNode(None,'P3',('Ftrfk','n1fk'),('Rtreng','n1er'),('Rcol','n1cr')),
    'n1fk': TDynNode('n1','P1',('Rtrfk','n1fkr')),
    'n1er': TDynNode('n1'),
    'n1cr': TDynNode('n1'),
    'n1fkr': TDynNode('n1fk'),
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

    while True:
      X_obs= [Rand(-1.0,1.0) for io in range(Nobs*2)]
      if CheckCollision(arm,Nobs,X_obs,xs0['q'].X)<=1.0e-6:  break
    xs0['X_obs']= SSA(X_obs)

    #dpl.Update(0,xs0)
    #x1= ToList(dpl.Select(0,x0))
    #xs0['q_cmd']= SSA([0.0]*arm.N)  #Init guess
    xs0['pqf']= copy.deepcopy(xs0['q'])  #Init guess
    xs0['pvia']= SSA([0.0]*(arm.N*2))  #Init guess
    res= dpl.Plan('n0', xs0)
    print 'xs0(after plan)=',xs0
    #db_n0= dpl.DB.AddToSeq(parent=None,name='n0',xs=xs0)

    CPrint(1,'R=',dpl.Value(res.PTree))
    VizTraj(arm, ToList(xs0['q'].X), ToList(xs0['pqf'].X), ToList(xs0['pvia'].X), ToList(xs0['x_trg'].X), '/tmp/arm/arm.dat')
    VizTraj(arm, ToList(xs0['q'].X), ToList(xs0['pqf'].X), ToList(xs0['pvia'].X), ToList(xs0['x_trg'].X), '/tmp/arm/4e_arm{count}.dat'.format(count=i))
    VizObstacles(Nobs, X_obs, '/tmp/arm/obs.dat')
    VizObstacles(Nobs, X_obs, '/tmp/arm/4e_obs{count}.dat'.format(count=i))
    #CPrint(1,'x_trg=',ptree.Tree[TPair('n0fkr',0)].XS['x_trg'])
    #CPrint(1,'x=',ptree.Tree[TPair('n0fkr',0)].XS['x'])

    dpl.EndEpisode()

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
          /tmp/arm/obs.dat w l lt 2
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
  import sys as sys
  if len(sys.argv)>1 and sys.argv[1] in ('p','plot','Plot','PLOT'):
    PlotGraphs()
    sys.exit(0)
  Main()
