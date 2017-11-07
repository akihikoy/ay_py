#!/usr/bin/python
#\file    dpl4a.py
#\brief   Test dpl4.py with toy1.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jan.18, 2016
from _path import *
from ay_py.core import *
from toy1 import *

def Main(logdir='/tmp/g1/', options_in={}):
  domain= TGraphDynDomain()
  SP= TCompSpaceDef
  domain.SpaceDefs={
    'x1': SP('action',2,min=[0.3,0.3],max=[0.7,0.7]),
    'x2': SP('state',3),
    'y':  SP('state',1),
    REWARD_KEY:  SP('state',1),
    }
  domain.Models={
    #key:[In,Out,F],
    'F1': [['x1'],['x2'],None],
    'F2': [['x2'],['y'],None],
    'R':  [['y'],[REWARD_KEY],TLocalQuad(1,lambda x:x[0])],
    'P1': [[],[PROB_KEY], TLocalLinear(0,1,lambda x:[1.0],lambda x:[0.0])],
    }
  domain.Graph={
    'n0': TDynNode(None,'P1',('F1','n1')),
    'n1': TDynNode('n0','P1',('F2','n2')),
    'n2': TDynNode('n1','P1',('R','n3')),
    'n3': TDynNode('n2'),
    }

  load_model= False
  #load_model= True
  #load_model= 'persistent'  #Always keep updating the model.

  mm_options= {}
  #mm_options['type']= 'lwr'
  mm_options['base_dir']= logdir+'models/'
  mm= TModelManager(domain.SpaceDefs, domain.Models)
  mm.Load({'options':mm_options})
  if load_model is True:
    mm.Load(LoadYAML('/tmp/g1/m1/model_mngr.yaml'), '/tmp/g1/m1/')
    mm.Options['base_dir']= mm_options['base_dir']
  elif load_model is 'persistent':
    mm_options['base_dir']= '/tmp/g1/m1/'
    mm.Load(LoadYAML('/tmp/g1/m1/model_mngr.yaml'), '/tmp/g1/m1/')
    mm.Options['base_dir']= mm_options['base_dir']
  db= TGraphEpisodeDB()

  dpl_options= {
    'ddp_sol': {'f_reward_ucb': 0.0,},  #Try 100.0
    }
  dpl_options['base_dir']= logdir
  #InsertDict(options, options_in)

  dpl= TGraphDynPlanLearn(domain, db, mm)
  dpl.Load({'options':dpl_options})
  mm.Init()
  dpl.Init()
  fp= OpenW(logdir+'dpl_log.dat')

  tsys.n_dyn= 0
  for i in range(40):
    dpl.NewEpisode()

    xs0= {}  #XSSA

    res= dpl.Plan('n0', xs0)
    print 'xs0(after plan)=',xs0
    #ptree= ddp_sol.GetPTree('n0', xs0, max_visits=ddp_sol.Options['max_visits'])
    #print '########', [ddp_sol.Value(ptree)]
    res.PTree.Dump()
    db_n0= dpl.DB.AddToSeq(parent=None,name='n0',xs=xs0)

    xs1= CopyXSSA(xs0)
    xs1['x2']= SSA(F1(ToList(xs0['x1'].X)))

    dpl.MM.Update('F1',xs0,xs1)
    res= dpl.Plan('n1', xs1)  #No actions to be planned.
    print 'xs1(after plan)=',xs1
    db_n1= dpl.DB.AddToSeq(parent=db_n0,name='n1',xs=xs1)

    xs2= CopyXSSA(xs1)
    xs2['y']= SSA(F2(ToList(xs1['x2'].X)))

    dpl.MM.Update('F2',xs1,xs2)
    db_n2= dpl.DB.AddToSeq(parent=db_n1,name='n2',xs=xs2)

    xs3= dpl.Forward('R',xs2)
    print 'xs3=',xs3
    db_n3= dpl.DB.AddToSeq(parent=db_n2,name='n3',xs=xs3)
    CPrint(1,'R=',xs3[REWARD_KEY])

    dpl.EndEpisode()
    fp.write(dpl.DB.DumpOneYAML())
    print i,dpl.DB.DumpOne()
  fp.close()

  SaveYAML(dpl.MM.Save(dpl.MM.Options['base_dir']), dpl.MM.Options['base_dir']+'model_mngr.yaml')
  SaveYAML(dpl.DB.Save(), logdir+'database.yaml')
  SaveYAML(dpl.Save(), logdir+'dpl.yaml')

  #Plot of the estimated whole system with LWR
  ptree= dpl.GetPTree('n0', {})
  fp= open(logdir+'whole_est2.dat','w')
  for y in FRange1(tsys.bound[0][0],tsys.bound[1][0],25):
    for z in FRange1(tsys.bound[0][1],tsys.bound[1][1],25):
      x1= [y,z]
      ptree.StartNode.XS= {'x1':SSA(x1)}
      ptree.ResetFlags()
      e= [dpl.Value(ptree)]
      fp.write('%s\n' % ToStr(x1,e))
    fp.write('\n')
  fp.close()
  #print '''qplot -x -3d -s 'set xlabel "y";set ylabel"z"' /tmp/g1/whole.dat w l /tmp/g1/whole_est2.dat w l -cs 'w l' /tmp/g1/opt*.dat'''

  dpl.MM.PlotModel('F1',file_prefix=logdir+'f{key}',f_reduce=lambda xa:[xa[0],xa[1]],f_repair=lambda xa,mi,ma,me:[xa[0],xa[1]])
  dpl.MM.PlotModel('F2',file_prefix=logdir+'f{key}',f_reduce=lambda xa:[xa[0],xa[2]],f_repair=lambda xa,mi,ma,me:[xa[0],me[1],xa[1]])

def PlotGen(logdir='/tmp/g1/'):
  db= TGraphEpisodeDB()
  db.Load(LoadYAML(logdir+'dpl_log.dat'))
  db.SDump(open(logdir+'dpl_sdump.dat','w'), ('n0',0,'x1'))

  #def PrintEq(s):  print '%s= %r' % (s, eval(s))
  #Plot of the whole system
  fp= open(logdir+'whole.dat','w')
  for y in FRange1(tsys.bound[0][0],tsys.bound[1][0],50):
    for z in FRange1(tsys.bound[0][1],tsys.bound[1][1],50):
      x1= [y,z]
      fp.write('%s\n' % ToStr(x1,F12(x1)))
    fp.write('\n')
  fp.close()
  #print 'qplot -x -3d -s \'set xlabel "y";set ylabel"z"\' /tmp/g1/whole.dat w l'

  fp= open(logdir+'f1.dat','w')
  for y in FRange1(tsys.bound[0][0],tsys.bound[1][0],50):
    for z in FRange1(tsys.bound[0][1],tsys.bound[1][1],50):
      x1= [y,z]
      fp.write('%s\n' % ToStr(x1,x1,F1(x1)))
    fp.write('\n')
  fp.close()
  #print '''qplot -x -3d -cs 'u 1:2:5' /tmp/g1/f0.dat w l /tmp/g1/f0_est.dat w l /tmp/g1/f0_smp.dat'''

  fp= open(logdir+'f2.dat','w')
  for cy in FRange1(tsys.bound2[0][0],tsys.bound2[1][0],50):
    for w in FRange1(tsys.bound2[0][2],tsys.bound2[1][2],50):
      x2= [cy,0.5*(tsys.bound2[0][1]+tsys.bound2[1][1]),w]
      fp.write('%s\n' % ToStr([x2[0],x2[2]],x2,F2(x2)))
    fp.write('\n')
  fp.close()
  #print '''qplot -x -3d -cs 'u 1:2:6' /tmp/g1/f1.dat w l /tmp/g1/f1_est.dat w l /tmp/g1/f1_smp.dat -cs '' /tmp/g1/f1_ideals.dat u 1:2:'(0.0)' '''

def PlotGraphs(logdir='/tmp/g1/'):
  print 'Plotting graphs..'
  import os
  PlotGen(logdir)
  optpr='003'
  commands=[
    '''qplot -x2 g1 -cs 'u 1:4 w lp' /tmp/g1/dpl_sdump.dat &''',
    '''qplot -x2 g1 -3d -cs 'u 1:2:5' /tmp/g1/f1.dat w l lt 2 /tmp/g1/fF1_est.dat w l lt 1 /tmp/g1/fF1_smp.dat &''',
    '''qplot -x2 g1 -3d -cs 'u 1:2:6' /tmp/g1/f2.dat w l lt 2 /tmp/g1/fF2_est.dat w l lt 1 /tmp/g1/fF2_smp.dat &''',
    '''qplot -x2 g1 -3d -s 'set xlabel "y";set ylabel"z"'
          /tmp/g1/whole.dat w l lt 2 /tmp/g1/whole_est2.dat w l lt 1
          -cs 'u 2:3:4 w lp' /tmp/g1/dpl_sdump.dat &''',
    '''''',
    '''''',
    ]
  for cmd in commands:
    if cmd!='':
      cmd= (' '.join(cmd.splitlines())).replace('/tmp/g1/',logdir)
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
