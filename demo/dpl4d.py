#!/usr/bin/python
#\file    dpl4d.py
#\brief   Test dpl4.py with toy_arm1, trajectory optimization.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jan.25, 2016
from _path import *
from ay_py.core import *
from toy_arm1 import *

#Get parameterized splines from initial and final points x0, xf, duration tf,
#and parameters param.
#x0 and xf can be any dimensionality Nd, tf is a positive scalar.
#param=[[p1,p2],...], 2 parameters for each dimension, [0.0,0.0] is a line.
def GetParameterizedSplines(x0=[0.0],xf=[1.0],tf=1.0,param=[[0.0,0.0]]):
  Nd= len(x0)
  data= [[0.0]+[None]*Nd,
         [0.333*tf]+[None]*Nd,
         [0.666*tf]+[None]*Nd,
         [tf]+[None]*Nd]
  for d in range(Nd):
    a= (xf[d]-x0[d])/tf
    data[0][1+d]= x0[d]
    data[1][1+d]= x0[d]+a*data[1][0] + param[d][0]
    data[2][1+d]= x0[d]+a*data[2][0] + param[d][1]
    data[3][1+d]= xf[d]

  splines= [TCubicHermiteSpline() for d in range(Nd)]
  for d in range(len(splines)):
    data_d= [[x[0],x[d+1]] for x in data]
    splines[d].Initialize(data_d, tan_method=splines[d].CARDINAL, c=0.0, m=0.0)
  return splines

def Ftraj(arm,Ntraj,qp):
  #q: joint angles
  #pqf: final joint angles
  #pvia: via points parameters; [p1,p2]*arm.N
  q,pqf,pvia= qp[:arm.N], qp[arm.N:arm.N*2], np.mat(qp[arm.N*2:]).reshape(arm.N,2)
  splines= GetParameterizedSplines(x0=q, xf=pqf, param=pvia.tolist())
  #q_traj: joint angle trajectory; arm.N*Ntraj
  q_traj= []
  for t in FRange1(0.0,1.0,Ntraj-1):
    q_traj+= [splines[d].Evaluate(t) for d in range(len(splines))]
  return q_traj

def Ftrfk_dF(arm,Ntraj,q_traj,with_grad):
  #q_traj: joint angle trajectory; arm.N*Ntraj
  #x_traj: Cartesian trajectory; 2*Ntraj
  x_traj= [0.0]*(2*Ntraj)
  if not with_grad:
    for k,q in enumerate(np.array(q_traj).reshape(Ntraj,arm.N)):
      x= arm.FK2D(q)
      x_traj[2*k:2*(k+1)]= x
    return x_traj
  else:  #with_grad
    dx_traj= np.zeros((arm.N*Ntraj,2*Ntraj))
    for k,q in enumerate(np.array(q_traj).reshape(Ntraj,arm.N)):
      x,J= arm.FK2D(q,with_J=True)
      x_traj[2*k:2*(k+1)]= x
      dx_traj[arm.N*k:arm.N*(k+1),2*k:2*(k+1)]= J.T
    return x_traj, dx_traj

def Rtreng(Ntraj,q_traj,w):
  armN= len(q_traj)/Ntraj
  q_prev= None
  for q in np.array(q_traj).reshape(Ntraj,armN):
    if q_prev is None:  R= 0.0
    else:  R+= w*la.norm(q_prev-q)**2
    q_prev= q
  return [R]

#Distance between a line p1--p2 and a point px.
def LinePointDist(p1,p2,px):
  p1= np.array(p1)
  p2= np.array(p2)
  px= np.array(px)
  a= p2-p1
  lp12= la.norm(a)
  a= a/lp12
  r= np.dot(px-p1,a)
  if r<0.0:   return la.norm(px-p1)
  if r>lp12:  return la.norm(px-p2)
  return la.norm(px-(p1+r*a))

def Rtrfk(Ntraj,x_traj_trg,wf,wi):
  #x_traj_trg: x_traj,x_trg
  x_traj= x_traj_trg[:-2]
  x_trg= x_traj_trg[-2:]
  x_0= x_traj[:2]
  Rf= wf*Dist(x_traj[-2:],x_trg)**2  #Terminal cost
  x_ideal= [(1.0-t)*np.array(x_0)+t*np.array(x_trg) for t in FRange1(0.0,1.0,Ntraj-1)]
  Ri= 0.0
  for xi,xi_trg in zip(np.array(x_traj).reshape(Ntraj,2), x_ideal):
    #Ri+= wi*LinePointDist(x_0,x_trg,xi)**2
    Ri+= wi*Dist(xi,xi_trg)**2
  #print 'Ri',Ri
  return [Rf+Ri]

def VizTraj(arm,q,pqf,pvia,x_trg,file_name):
  #q: joint angles
  #pqf: final joint angles
  #pvia: via points parameters; [p1,p2]*arm.N
  splines= GetParameterizedSplines(x0=q, xf=pqf, param=np.array(pvia).reshape(arm.N,2).tolist())
  #q_traj: joint angle trajectory; arm.N*Ntraj
  q_traj= []
  for t in FRange1(0.0,1.0,20):
    q_traj.append([splines[d].Evaluate(t) for d in range(len(splines))])
  VisualizeTraj2D(arm, q_traj, x_trg, file_name)

def Main(logdir='/tmp/arm/', options_in={}):
  arm= TArm(6)
  Ntraj= 10

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
    REWARD_KEY:  SP('state',1),
    }
  def dFmv(q_qcmd):
    grad= np.zeros((arm.N*2,arm.N))
    grad[arm.N:,:]= np.eye(arm.N,arm.N)
    return grad
  #FIXME:TODO:Calculate gradients of: Ftraj, Rtreng, Rtrfk
  domain.Models={
    #key:[In,Out,F],
    'Ftraj': [['q','pqf','pvia'],['q_traj'], TLocalLinear(arm.N+arm.N+arm.N*2,arm.N*Ntraj,lambda qp:Ftraj(arm,Ntraj,qp),None)],
    #'Ftrfk': [['q_traj'],['x_traj'], TLocalLinear(arm.N*Ntraj,2*Ntraj,lambda q_traj:Ftrfk(arm,Ntraj,q_traj),lambda q_traj:dFtrfk(arm,Ntraj,q_traj))],
    'Ftrfk': [['q_traj'],['x_traj'], TLocalLinear(arm.N*Ntraj,2*Ntraj,FdF=lambda q_traj,with_grad:Ftrfk_dF(arm,Ntraj,q_traj,with_grad))],
    'Rtreng': [['q_traj'],[REWARD_KEY], TLocalLinear(arm.N*Ntraj,1,lambda q_traj:Rtreng(Ntraj,q_traj,-1.0e-6),None)],
    'Rtrfk': [['x_traj','x_trg'],[REWARD_KEY], TLocalLinear(2*Ntraj+2,1,lambda x_traj_trg:Rtrfk(Ntraj,x_traj_trg,-1.0,-1.0e-2),None)],
    'P1':  [[],[PROB_KEY], TLocalLinear(0,1,lambda x:[1.0],lambda x:[0.0])],
    'P2':  [[],[PROB_KEY], TLocalLinear(0,2,lambda x:[1.0]*2,lambda x:[0.0]*2)],
    'P3':  [[],[PROB_KEY], TLocalLinear(0,3,lambda x:[1.0]*3,lambda x:[0.0]*3)],
    }
  domain.Graph={
    'n0': TDynNode(None,'P1',('Ftraj','n1')),
    'n1': TDynNode(None,'P2',('Ftrfk','n1fk'),('Rtreng','n1er')),
    'n1fk': TDynNode('n1','P1',('Rtrfk','n1fkr')),
    'n1er': TDynNode('n1'),
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
    VizTraj(arm, ToList(xs0['q'].X), ToList(xs0['pqf'].X), ToList(xs0['pvia'].X), ToList(xs0['x_trg'].X), '/tmp/arm/4d_arm{count}.dat'.format(count=i))
    #CPrint(1,'x_trg=',ptree.Tree[TPair('n0fkr',0)].XS['x_trg'])
    #CPrint(1,'x=',ptree.Tree[TPair('n0fkr',0)].XS['x'])

    dpl.EndEpisode()

  #SaveYAML(dpl.MM.Save(dpl.MM.Options['base_dir']), dpl.MM.Options['base_dir']+'model_mngr.yaml')
  #SaveYAML(dpl.DB.Save(), logdir+'database.yaml')
  SaveYAML(dpl.Save(), logdir+'dpl.yaml')


if __name__=='__main__':
  import sys as sys
  if len(sys.argv)>1 and sys.argv[1] in ('p','plot','Plot','PLOT'):
    PlotGraphs()
    sys.exit(0)
  Main()
