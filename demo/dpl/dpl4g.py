#!/usr/bin/python
#\file    dpl4g.py
#\brief   Test dpl4.py with toy_push1.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Mar.03, 2016
from _path import *
from ay_py.core import *
from toy_push1 import PushFwdDyn, GenerateObjs, ExecuteAndViz

modeldir= '/tmp/p1/'

def LearnFwdDyn():
  #Here we train a NN to learn PushFwdDyn.
  #I/O: ['po','pg','theta','dm','rg'],['lpo','dpo']
  dim_in= 7
  dim_out= 3
  options={
    'base_dir': modeldir+'data/p1_model/',
    'n_units': [dim_in] + [200,200] + [dim_out],
    'name': 'FPush2',
    'loss_stddev_stop': 1.0e-6,
    'num_max_update': 10000,
    }
  prefix= modeldir+'data/p1_model/FPush2'
  FPush= TNNRegression()
  FPush.Load(data={'options':options})
  if os.path.exists(prefix+'.yaml'):
    FPush.Load(LoadYAML(prefix+'.yaml'), prefix)
    FPush.Load(data={'options':options}, base_dir=prefix)
  FPush.Init()

  for i in range(2000):
    po= [Rand(0.2,0.8), Rand(0.2,0.8)]
    pg= [Rand(0.0,1.0), Rand(0.0,1.0)]
    theta= Rand(-math.pi,math.pi)
    dm= Rand(0.01,2.0)
    rg= 0.3
    x_in= po+pg+[theta,dm,rg]
    x_out= PushFwdDyn(x_in)
    FPush.Update(x_in, x_out, not_learn=True)
  FPush.UpdateBatch()

  SaveYAML(FPush.Save(prefix), prefix+'.yaml')

def LearnRwd():
  #Here we train a NN to learn Reward.
  #I/O: ['lpo','dpo','rg','dm'],[REWARD_KEY]
  dim_in= 5
  dim_out= 1
  options={
    'base_dir': modeldir+'data/p1_model/',
    'n_units': [dim_in] + [200,200] + [dim_out],
    'name': 'FRwd',
    'loss_stddev_stop': 1.0e-6,
    'loss_stddev_init': 2000.0,
    'num_max_update': 10000,
    }
  prefix= modeldir+'data/p1_model/FRwd'
  FRwd= TNNRegression()
  FRwd.Load(data={'options':options})
  if os.path.exists(prefix+'.yaml'):
    FRwd.Load(LoadYAML(prefix+'.yaml'), prefix)
    FRwd.Load(data={'options':options}, base_dir=prefix)
  FRwd.Init()

  for i in range(2000):
    po= [Rand(0.2,0.8), Rand(0.2,0.8)]
    pg= [Rand(0.0,1.0), Rand(0.0,1.0)]
    theta= Rand(-math.pi,math.pi)
    dm= Rand(0.05,0.7)
    rg= 0.3
    fwd_out= PushFwdDyn(po+pg+[theta,dm,rg])
    lpo2,dpo= fwd_out[:2],fwd_out[2]
    x_in= lpo2+[dpo,rg,dm]
    x_out= [Reward(x_in)]
    FRwd.Update(x_in, x_out, not_learn=True)
  FRwd.UpdateBatch()

  SaveYAML(FRwd.Save(prefix), prefix+'.yaml')


#def Reward(x_in):
  ##In=['lpo','dpo','rg','dm']
  #lpox,lpoy,dpo,rg,dm= x_in
  ##return -1000.0*(lpox**2+lpoy**2)
  #if dpo>0.0:
    #R= 1.0+10.0*max(0.0,dpo)**2
  #elif -1.0e-6<=lpox and lpox<rg and abs(lpoy)<=lpox+1.0e-6:
    ##R= 0.0
    #R= max(0.0, 1.0-100.0*(lpox**2+lpoy**2))
  #else:
    #R= -1000.0*(lpox**2+lpoy**2)
  ##R+= -0.001*dm*dm
  ##print dpo,lpox,lpoy,R
  #return R

#def Reward(x_in):
  ##In=['lpo','dpo','rg','dm']
  #lpox,lpoy,dpo,rg,dm= x_in
  ##R= 1.0+10.0*max(0.0,dpo)
  ##R= 1.0*math.log(1.0+max(0.0,dpo))
  #R= 1.0*math.atan(5.0*max(0.0,dpo))
  ##R+= -100.0*(lpox**2+lpoy**2)
  ##R+= -0.2*dm*dm
  #return R

def Reward(x_in):
  #In=['lpo','dpo','rg','dm']
  lpox,lpoy,dpo,rg,dm= x_in
  R= -1000.0*(lpox**2+lpoy**2)
  R+= -0.001*dm*dm
  return R

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

  probs= [0.5,0.5]  #probabilities of p1,p2

  domain= TGraphDynDomain()
  SP= TCompSpaceDef
  domain.SpaceDefs={
    'pg': SP('action',2,min=[0.0,0.0],max=[1.0,1.0]),  #position to start push
    'theta': SP('action',1,min=[-math.pi],max=[math.pi]),  #orientation of gripper to push
    'dm': SP('action',1,min=[0.05],max=[0.8]),  #distance to push
    'rg': SP('state',1),  #size of gripper
    'p1': SP('state',2),  #position Gaussian-1
    'p2': SP('state',2),  #position Gaussian-2
    'po': SP('state',2),  #actual position of object
    'lpo': SP('state',2),  #actual position of object in gripper local frame
    'dpo': SP('state',1),  #distance of object movement
    REWARD_KEY:  SP('state',1),
    }
  domain.Models={
    #key:[In,Out,F],
    #Analytical model (gradient is numerically obtained):
    #'F1': [['p1','pg','theta','dm','rg'],['lpo','dpo'],FPushAnl],
    #'F2': [['p2','pg','theta','dm','rg'],['lpo','dpo'],FPushAnl],
    'R':  [['lpo','dpo','rg','dm'],[REWARD_KEY],TLocalQuad(5,lambda x_in:Reward(x_in))],
    #NN model:
    'F1': [['p1','pg','theta','dm','rg'],['lpo','dpo'],FPush],
    'F2': [['p2','pg','theta','dm','rg'],['lpo','dpo'],FPush],
    #'R':  [['lpo','dpo','rg','dm'],[REWARD_KEY],FRwd],
    'Pm2': [[],[PROB_KEY], TLocalLinear(0,2,lambda x:[probs[0],probs[1]],lambda x:[0.0,0.0])],
    'P1': [[],[PROB_KEY], TLocalLinear(0,1,lambda x:[1.0],lambda x:[0.0])],
    }
  domain.Graph={
    'n0': TDynNode(None,'Pm2',('F1','n1'),('F2','n2')),
    'n1': TDynNode('n0','P1',('R','n1r')),
    'n2': TDynNode('n0','P1',('R','n2r')),
    'n1r': TDynNode('n1'),
    'n2r': TDynNode('n2'),
    }

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
      'p1': SSA(p1,(1.0*stddevs[0])**2),  #tune cautiousness
      'p2': SSA(p2,(1.0*stddevs[1])**2),
      #'p1': SSA(p1,100.0*stddevs[0]),
      #'p2': SSA(p2,100.0*stddevs[1]),
      #Heuristic initial guess:
      #'pg': SSA(1.2*Vec(p1)-0.2*Vec(p2)),
      #'theta': SSA([math.atan2(p2[1]-p1[1],p2[0]-p1[0])]),
      #'dm': SSA([0.7]),
      }

    #print 'xs0(before plan)=',xs0
    #print dpl.Value(dpl.GetPTree('n0', xs0))

    res= dpl.Plan('n0', xs0)
    print 'xs0(after plan)=',xs0
    res.PTree.Dump()

    pg= ToList(xs0['pg'].X)
    theta= xs0['theta'].X[0,0]
    dm= xs0['dm'].X[0,0]

    print '------------'
    print 'n1r.lpo=',res.PTree.Tree[TPair('n1r',0)].XS['lpo']
    print 'n2r.lpo=',res.PTree.Tree[TPair('n2r',0)].XS['lpo']
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
