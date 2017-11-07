#!/usr/bin/python
#\file    dpl4f.py
#\brief   Test dpl4.py with toy_cannon1.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Feb.29, 2016
from _path import *
from ay_py.core import *
from toy_cannon1 import CannonForward, SolveAnalytically, GetTh

def Reward(x_in):
  #In=['th','pyh','pe']
  th,pyh,pex,pey= x_in
  return -(pyh-pey)**2 - 1.0e-3*th*th

def Main(logdir='/tmp/c1/', options_in={}):
  def Delta1(dim,s):
    assert(abs(s-int(s))<1.0e-6)
    p= [0.0]*dim
    p[int(s)]= 1.0
    return p
  domain= TGraphDynDomain()
  SP= TCompSpaceDef
  domain.SpaceDefs={
    'theta': SP('action',1,min=[-1.57],max=[1.57]),  #Cannon angle
    's': SP('select',num=2),  #Cannon selection (#1,#2)
    'v0': SP('state',1),  #Cannon initial velocity
    'pe': SP('state',2),  #Enemy position
    'p1': SP('state',2),  #Cannon position #1
    'p2': SP('state',2),  #Cannon position #2
    'th': SP('state',1),  #Hit time
    'pyh': SP('state',1),  #Hit y position
    REWARD_KEY:  SP('state',1),
    }
  domain.Models={
    #key:[In,Out,F],
    'F1': [['p1','pe','v0','theta'],['th','pyh'],TLocalLinear(6,2,FdF=lambda x_in,with_grad: CannonForward(x_in,with_grad))],
    'F2': [['p2','pe','v0','theta'],['th','pyh'],TLocalLinear(6,2,FdF=lambda x_in,with_grad: CannonForward(x_in,with_grad))],
    'R':  [['th','pyh','pe'],[REWARD_KEY],TLocalQuad(4,lambda x_in:Reward(x_in))],
    'P1': [[],[PROB_KEY], TLocalLinear(0,1,lambda x:[1.0],lambda x:[0.0])],
    'Ps': [['s'],[PROB_KEY], TLocalLinear(1,2,lambda s:Delta1(2,s[0]),lambda s:[0.0,0.0])],
    }
  domain.Graph={
    'n0': TDynNode(None,'Ps',('F1','n1'),('F2','n2')),
    'n1': TDynNode('n0','P1',('R','n1r')),
    'n2': TDynNode('n0','P1',('R','n2r')),
    'n1r': TDynNode('n1'),
    'n2r': TDynNode('n2'),
    }

  dpl_options= {
    'ddp_sol': {'f_reward_ucb': 0.0,},  #Try 100.0
    }
  dpl_options['base_dir']= logdir
  #InsertDict(options, options_in)

  dpl= TGraphDynPlanLearn(domain)
  dpl.Load({'options':dpl_options})
  dpl.Init()

  fp= OpenW(logdir+'dpl_cn1.dat')
  pex_set= FRange1(0.0,1.5,40)
  pex_set= pex_set[1:]
  for i in range(len(pex_set)):
    dpl.NewEpisode()

    p1= [0.0, 0.0]
    p2= [0.0, 0.8]
    pe= [pex_set[i], 0.3]
    v0= 3.0
    xs0= {
      'v0': SSA([v0]),
      'pe': SSA(pe),
      'p1': SSA(p1),
      'p2': SSA(p2),
      }

    #ptree= dpl.GetPTree('n0', xs0)
    #print dpl.Value(ptree)

    res= dpl.Plan('n0', xs0)
    print 'xs0(after plan)=',xs0
    #res.PTree.Dump()
    theta1a= SolveAnalytically(p1, pe, v0)
    theta2a= SolveAnalytically(p2, pe, v0)
    th1a= GetTh(p1, pe, v0, theta1a) if theta1a is not None else None
    th2a= GetTh(p1, pe, v0, theta2a) if theta2a is not None else None
    if th1a is None and th2a is None:  sa= None
    else:  sa= 0 if th1a is not None and th1a<th2a else 1
    fp.write('{pex} {theta1a} {th1a} {theta2a} {th2a} {sa} {s} {theta} {th}\n'.format(
      pex= xs0['pe'].X[0,0],
      theta1a= theta1a, th1a= th1a,
      theta2a= theta2a, th2a= th2a,
      sa= sa,
      s= xs0['s'].X[0,0],
      theta= xs0['theta'].X[0,0],
      th= GetTh(p1, pe, v0, xs0['theta'].X[0,0]) ))
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
  logdir= argv[0] if len(argv)>0 else '/tmp/c1/'
  qopt= argv[1] if len(argv)>1 else ''
  commands=[
    '''f=/tmp/c1/dpl_cn1.dat &&
      qplot -x2 aaa
        -s 'set key right top;'
        -s 'set xlabel "X-target position";'
        -s 'set ylabel "Theta";'
        -s 'set yrange [*:*] reverse;'
        $f u 1:2 w l lw 3 t '"Analytical theta (p1)"'
        $f u 1:4 w l lw 3 t '"Analytical theta (p2)"'
        $f u 1:'($6==$7 ? $8 : 1/0)' w p pt 6 ps 2 t '"DDP (correct)"'
        $f u 1:'($6!=$7 ? $8 : 1/0)' w p pt 2 ps 2 t '"DDP (wrong)"'
        $f u 1:'($1>1.32 ? $8 : 1/0)' w p pt 8 ps 2 t '"DDP (approx)"'
      &''',
    '''f=/tmp/c1/dpl_cn1.dat &&
      qplot -x2 aaa
        -s 'set key right bottom;'
        -s 'set xlabel "X-target position";'
        -s 'set ylabel "Time to hit";'
        -s 'set yrange [*:*];'
        $f u 1:3 w l lw 3 t '"Analytical t_hit (p1)"'
        $f u 1:5 w l lw 3 t '"Analytical t_hit (p2)"'
        $f u 1:'($6==$7 ? $9 : 1/0)' w p pt 6 ps 2 t '"DDP (correct)"'
        $f u 1:'($6!=$7 ? $9 : 1/0)' w p pt 2 ps 2 t '"DDP (wrong)"'
        $f u 1:'($1>1.32 ? $9 : 1/0)' w p pt 8 ps 2 t '"DDP (approx)"'
      &''',
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
