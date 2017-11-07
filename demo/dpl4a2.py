#!/usr/bin/python
#\file    dpl4a2.py
#\brief   Test dpl4.py w.r.t. a simple selection.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jan.21, 2016
from _path import *
from ay_py.core import *
from toy1 import *

def Main(logdir='/tmp/g2/', options_in={}):
  domain= TGraphDynDomain()
  SP= TCompSpaceDef
  domain.SpaceDefs={
    's': SP('select',num=2),
    'x': SP('state',1),
    REWARD_KEY:  SP('state',1),
    }
  def Delta1(dim,s):
    assert(abs(s-int(s))<1.0e-6)
    p= [0.0]*dim
    p[int(s)]= 1.0
    return p
  domain.Models={
    #key:[In,Out,F],
    'R0':  [['x'],[REWARD_KEY],TLocalLinear(1,1,lambda x:[x[0]],lambda x:[1.0])],
    'R1':  [['x'],[REWARD_KEY],TLocalLinear(1,1,lambda x:[-x[0]+1.0],lambda x:[-1.0])],
    'Ps': [['s'],[PROB_KEY], TLocalLinear(1,2,lambda s:Delta1(2,s[0]),lambda s:[0.0,0.0])],
    }
  domain.Graph={
    'n0': TDynNode(None,'Ps',('R0','n10'),('R1','n11')),
    'n10': TDynNode('n0'),
    'n11': TDynNode('n0'),
    }

  mm_options= {}
  mm_options['base_dir']= logdir+'models/'
  mm= TModelManager(domain.SpaceDefs, domain.Models)
  mm.Load({'options':mm_options})
  db= TGraphEpisodeDB()

  dpl_options= {}
  dpl_options['base_dir']= logdir

  dpl= TGraphDynPlanLearn(domain, db, mm)
  dpl.Load({'options':dpl_options})
  mm.Init()
  dpl.Init()
  fp= OpenW(logdir+'dpl_log.dat')

  for i in range(40):
    dpl.NewEpisode()

    xs0= {}  #XSSA
    xs0['x']= SSA([Rand(0.0,1.0)])

    res= dpl.Plan('n0', xs0)
    print 'xs0(after plan)=',xs0
    db_n0= dpl.DB.AddToSeq(parent=None,name='n0',xs=xs0)

    ptree= res.PTree
    CPrint(1,'R=',dpl.Value(ptree))
    CPrint(1,'s=',xs0['s'])
    if xs0['s'].X[0]==0:
      msg= 'RIGHT!' if ptree.Tree[TPair('n10',0)].XS[REWARD_KEY].X[0] > ptree.Tree[TPair('n11',0)].XS[REWARD_KEY].X[0] else 'WRONG!'
    else:
      msg= 'RIGHT!' if ptree.Tree[TPair('n10',0)].XS[REWARD_KEY].X[0] < ptree.Tree[TPair('n11',0)].XS[REWARD_KEY].X[0] else 'WRONG!'
    CPrint(1 if msg=='RIGHT!' else 4,msg,ptree.Tree[TPair('n10',0)].XS[REWARD_KEY],ptree.Tree[TPair('n11',0)].XS[REWARD_KEY])
    if msg!='RIGHT!':  break
    #TODO: Log n10 or n11 to DB.
    #TODO: Log XSSA after rewards are computed.

    dpl.EndEpisode()
    fp.write(dpl.DB.DumpOneYAML())
    print i,dpl.DB.DumpOne()
  fp.close()

  SaveYAML(dpl.MM.Save(dpl.MM.Options['base_dir']), dpl.MM.Options['base_dir']+'model_mngr.yaml')
  SaveYAML(dpl.DB.Save(), logdir+'database.yaml')
  SaveYAML(dpl.Save(), logdir+'dpl.yaml')


def PlotGen(logdir='/tmp/g2/'):
  db= TGraphEpisodeDB()
  db.Load(LoadYAML(logdir+'dpl_log.dat'))
  db.SDump(open(logdir+'dpl_sdump.dat','w'), ('n0',0,'x','s'))

def PlotGraphs(logdir='/tmp/g2/'):
  print 'Plotting graphs..'
  import os
  PlotGen(logdir)
  commands=[
    '''qplot -x2 g1 -cs 'u 1:4 w lp' /tmp/g2/dpl_sdump.dat &''',
    '''''',
    '''''',
    ]
  for cmd in commands:
    if cmd!='':
      cmd= (' '.join(cmd.splitlines())).replace('/tmp/g2/',logdir)
      print '###',cmd
      os.system(cmd)

  print '##########################'
  print '###Press enter to close###'
  print '##########################'
  raw_input()
  os.system('qplot -x2kill g1')

if __name__=='__main__':
  import sys as sys
  if len(sys.argv)>1 and sys.argv[1] in ('p','plot','Plot','PLOT'):
    PlotGraphs()
    sys.exit(0)
  Main()
