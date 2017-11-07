#!/usr/bin/python
#\file    dpl4g2.py
#\brief   Test dpl4.py with toy_push1.
#         Based on test_dpl4g.py, design of the graph structure is modified.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.07, 2017
from _path import *
from ay_py.core import *
from toy_push1 import PushFwdDyn, GenerateObjs, ExecuteAndViz
from dpl4g import modeldir, LearnFwdDyn, LearnRwd, Reward

"""
'''A generator function y=F(), y: m-dim vector.  There is no input.
This function class divides a constant array, and returns a part of it.
Precisely, it returns A[i1:i2].'''
class TArrayDivider(TFunctionApprox):
  #Number of x-dimensions
  @property
  def Dx(self):
    return 0

  #Number of y-dimensions
  #WARNING: Works only when i2>=i1>=0.
  @property
  def Dy(self):
    return self.i2-self.i1

  def __init__(self, i1, i2, A=None):
    TFunctionApprox.__init__(self)
    self.i1= i1
    self.i2= i2
    self.A= A

  #Whether prediction is available (False if the model is not learned).
  def IsPredictable(self):
    return True  #This class does not learn anything.

  '''
  Do prediction.
    Return a TPredRes instance.
    x_var: Covariance of x.  If a scholar is given, we use diag(x_var,x_var,..).
    with_var: Whether compute a covariance matrix of error at the query point as well.
    with_grad: Whether compute a gradient at the query point as well.
  '''
  def Predict(self, x, x_var=0.0, with_var=False, with_grad=False):
    x_var, var_is_zero= RegularizeCov(x_var, len(x))
    y= self.A[self.i1:self.i2]
    dy= np.zeros((0,len(y)))
    var= np.zeros((len(y),len(y)))
    res= self.TPredRes()
    res.Y= y
    if with_var:  res.Var= var
    if with_grad:  res.Grad= dy
    return res
"""

'''A generator function y=F().  There is no input.
This function class selects an XSSA from an array of XSSA.
i.e. it returns A[i].'''
class TArraySelector(TFunctionApprox):
  #Number of x-dimensions
  @property
  def Dx(self):
    return 0

  #Number of y-dimensions
  @property
  def Dy(self):
    return self.dy

  def __init__(self, dy, i, A=None):
    TFunctionApprox.__init__(self)
    self.i= i
    self.dy= dy
    self.A= A

  #Whether prediction is available (False if the model is not learned).
  def IsPredictable(self):
    return True  #This class does not learn anything.

  '''
  Do prediction.
    Return a TPredRes instance.
    x_var: Covariance of x.  If a scholar is given, we use diag(x_var,x_var,..).
    with_var: Whether compute a covariance matrix of error at the query point as well.
    with_grad: Whether compute a gradient at the query point as well.
  '''
  def Predict(self, x, x_var=0.0, with_var=False, with_grad=False):
    #x_var, var_is_zero= RegularizeCov(x_var, len(x))
    ys= self.A[self.i]
    dy= np.zeros((0,len(ys.X)))
    var,_= RegularizeCov(ys.Cov, len(ys.X))
    res= self.TPredRes()
    res.Y= ys.X
    if with_var:  res.Var= var
    if with_grad:  res.Grad= dy
    return res


def Main(logdir='/tmp/p1/', options_in={}):
  #Create NN model of PushFwdDyn:
  print 'Run LearnFwdDyn?'
  if AskYesNo():  LearnFwdDyn()
  #return
  #Create NN model of Reward:
  print 'Run LearnRwd?'
  if AskYesNo():  LearnRwd()
  #return
  #Load NN model of PushFwdDyn:
  FPush= TNNRegression()
  prefix= modeldir+'data/p1_model/FPush2'
  FPush.Load(LoadYAML(prefix+'.yaml'), prefix)
  FPush.Init()
  #Load NN model of Reward:
  FRwd= TNNRegression()
  prefix= modeldir+'data/p1_model/FRwd'
  FRwd.Load(LoadYAML(prefix+'.yaml'), prefix)
  FRwd.Init()

  #Analytical model of PushFwdDyn
  FPushAnl= TLocalLinear(7,3,F=lambda x_in: PushFwdDyn(x_in))
  FPushAnl.Load({'options':{'h':0.05}})

  p_array= [None]*2

  probs= [0.5,0.5]  #probabilities of p1,p2

  domain= TGraphDynDomain()
  SP= TCompSpaceDef
  domain.SpaceDefs={
    'pg': SP('action',2,min=[0.0,0.0],max=[1.0,1.0]),  #position to start push
    'theta': SP('action',1,min=[-math.pi],max=[math.pi]),  #orientation of gripper to push
    'dm': SP('action',1,min=[0.05],max=[0.8]),  #distance to push
    'rg': SP('state',1),  #size of gripper
    'p': SP('state',2),  #position Gaussian (common of p1,p2)
    'po': SP('state',2),  #actual position of object
    'lpo': SP('state',2),  #actual position of object in gripper local frame
    'dpo': SP('state',1),  #distance of object movement
    REWARD_KEY:  SP('state',1),
    }
  domain.Models={
    #key:[In,Out,F],
    #NN model:
    'F': [['p','pg','theta','dm','rg'],['lpo','dpo'],FPush],
    'R':  [['lpo','dpo','rg','dm'],[REWARD_KEY],FRwd],
    'Pm2': [[],[PROB_KEY], TLocalLinear(0,2,lambda x:[probs[0],probs[1]],lambda x:[0.0,0.0])],
    'P1': [[],[PROB_KEY], TLocalLinear(0,1,lambda x:[1.0],lambda x:[0.0])],
    #Variable generator:
    #'G_0': [[],['p'],TArraySelector(2,0,p_array)],
    #'G_1': [[],['p'],TArraySelector(2,1,p_array)],
    }
  domain.Graph={
    #'n0': TDynNode(None,'Pm2',('G_0','n1_0'),('G_1','n1_1')),
    #'n1_0': TDynNode('n0','P1',('F','n2_0')),
    #'n1_1': TDynNode('n0','P1',('F','n2_1')),
    #'n2_0': TDynNode('n1_0','P1',('R','n2r_0')),
    #'n2_1': TDynNode('n1_1','P1',('R','n2r_1')),
    #'n2r_0': TDynNode('n2_0'),
    #'n2r_1': TDynNode('n2_1'),
    }
  #Program to generate graph and models:
  domain.Graph['n0']= TDynNode(None,'Pm2',*(('G_%d'%i,'n1_%d'%i) for i in range(2)))
  for i in range(2):
    domain.Models['G_%d'%i]= [[],['p'],TArraySelector(2,i,p_array)]
    domain.Graph['n1_%d'%i]= TDynNode('n0','P1',('F','n2_%d'%i))
    domain.Graph['n2_%d'%i]= TDynNode('n1_%d'%i,'P1',('R','n2r_%d'%i))
    domain.Graph['n2r_%d'%i]= TDynNode('n2_%d'%i)

  dpl_options= {
    'opt_log_name': None,  #Not save optimization log.
    'ddp_sol': {'f_reward_ucb': 0.0,},  #Try 100.0
    }
  dpl_options['base_dir']= logdir
  #InsertDict(options, options_in)

  dpl= TGraphDynPlanLearn(domain)
  dpl.Load({'options':dpl_options})
  dpl.Init()

  CopyFile(__file__,logdir+os.path.basename(__file__))

  fp= OpenW(logdir+'dpl_p1.dat')
  for i in range(1):
    dpl.NewEpisode()

    rg= 0.3  #size of gripper
    p1= [Rand(0.3,0.7), Rand(0.3,0.7)]
    p2= [Rand(0.3,0.7), Rand(0.3,0.7)]
    #p1= [0.5, 0.5]
    #p2= [0.8, 0.3]
    stddevs= [Rand(0.0,0.1),Rand(0.0,0.1)]  #standard deviation of p1,p2
    #stddevs= [0.1,0.1]  #standard deviation of p1,p2
    #stddevs= [0.01,0.01]  #standard deviation of p1,p2
    po= GenerateObjs(1,p1,p2,probs,stddevs)[0]  #actual position
    #p1= [0.6354168369476203, 0.6533908977455765]
    #p2= [0.3977549751591069, 0.3127670785454465]
    ##p2= [0.2977549751591069, 0.2127670785454465]
    #po= [0.347402, 0.374530]
    xs0= {
      'rg': SSA([rg]),
      }
    p_array[0]= SSA(p1,(1.0*stddevs[0])**2)
    p_array[1]= SSA(p2,(1.0*stddevs[1])**2)

    #print 'xs0(before plan)=',xs0
    #print dpl.Value(dpl.GetPTree('n0', xs0))

    res= dpl.Plan('n0', xs0)
    print 'xs0(after plan)=',xs0
    res.PTree.Dump()

    pg= ToList(xs0['pg'].X)
    theta= xs0['theta'].X[0,0]
    dm= xs0['dm'].X[0,0]

    print '------------'
    print 'n2r_0.lpo=',res.PTree.Tree[TPair('n2r_0',0)].XS['lpo']
    print 'n2r_1.lpo=',res.PTree.Tree[TPair('n2r_1',0)].XS['lpo']
    print 'FPush.Predict(p1)=',ToList(FPush.Predict(p1+pg+[theta,dm,rg]).Y)
    print 'FPush.Predict(p2)=',ToList(FPush.Predict(p2+pg+[theta,dm,rg]).Y)
    print 'FPush.Predict(po)=',ToList(FPush.Predict(po+pg+[theta,dm,rg]).Y)
    print 'PushFwdDyn(p1)=',PushFwdDyn(p1+pg+[theta,dm,rg])
    print 'PushFwdDyn(p2)=',PushFwdDyn(p2+pg+[theta,dm,rg])
    print 'PushFwdDyn(po)=',PushFwdDyn(po+pg+[theta,dm,rg])

    file_format= '{logdir}{i}-push1_{kind}.dat'.format(logdir=logdir,i=i,kind='{kind}')
    ExecuteAndViz(file_format,po,pg,theta,dm,rg,p1,p2,probs,stddevs)

    S= lambda l: ' '.join(map(str,l))
    fp.write('{res} # {po} # {pg} # {theta} # {dm} # {rg} # {p1} # {p2} # {probs} # {stddevs}\n'.format(
      res=S(PushFwdDyn(po+pg+[theta,dm,rg])),
      po=S(po),pg=S(pg),theta=theta,dm=dm,rg=rg,
      p1=S(p1),p2=S(p2),probs=S(probs),stddevs=S(stddevs)))
    fp.flush()

    dpl.EndEpisode()
    #print i,dpl.DB.DumpOne()

  fp.close()

  #SaveYAML(dpl.MM.Save(dpl.MM.Options['base_dir']), dpl.MM.Options['base_dir']+'model_mngr.yaml')
  #SaveYAML(dpl.DB.Save(), logdir+'database.yaml')
  #SaveYAML(dpl.Save(), logdir+'dpl.yaml')


def PlotGraphs(argv):
  print 'Plotting graphs..'
  import os
  logdir= argv[0] if len(argv)>0 else '/tmp/p1/'
  qopt= argv[1] if len(argv)>1 else ''
  commands=[
    '''qplot -x2 aaa
      -s 'unset key'
      -s 'set size ratio -1'
      -s 'set xrange [-0.1:1.1]'
      -s 'set yrange [-0.1:1.1]'
      /tmp/p1/{i}-push1_pset.dat w p ps 1 pt 6
      /tmp/p1/{i}-push1_po15.dat w p ps 2 pt 7 lt 5
      /tmp/p1/{i}-push1_g15.dat w l lw 1 lt 5
      /tmp/p1/{i}-push1_po1.dat w p ps 4 pt 7 lt 2
      /tmp/p1/{i}-push1_po2.dat w p ps 3 pt 7 lt 3
      /tmp/p1/{i}-push1_g1.dat w l lw 2 lt 2
      /tmp/p1/{i}-push1_g2.dat w l lw 3 lt 3
      &
      '''.format(i=0),
    '''''',
    ]
  for cmd in commands:
    if cmd!='':
      cmd= ' '.join(cmd.splitlines()).replace('/tmp/p1/',logdir)
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
