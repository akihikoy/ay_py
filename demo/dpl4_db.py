#!/usr/bin/python
#\file    dpl4_db.py
#\brief   Test src/base/base_dpl4.py, TGraphEpisodeDB.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jan.27, 2016
from _path import *
from ay_py.core import *
from toy1 import *

def Main():
  t_start= time.time()
  db= TGraphEpisodeDB()
  db.Load(LoadYAML('/tmp/g1/database.yaml'))
  '''
  for i,eps in enumerate(db.Entry):
    #TEST-1
    #print i,eps.Dump()
    #TEST-2
    #keys= (('n0',0),('n1',0),('n2',0),('n3',0))
    #nodes= eps.Find(*keys)
    #print i,
    #for k,n in zip(keys,nodes):
      #print k,':',n,
    #print ''
    #TEST-3
    #n0,n1,n2,n3= eps.Find(('n0',0),('n1',0),('n2',0),('n3',0))
    #print i, ToStr(ToList(n0.XS['x1'].X), ToList(n1.XS['x2'].X), ToList(n2.XS['y'].X), ToList(n3.XS[REWARD_KEY].X))
    #TEST-4
    print i, eps.SDump(('n0',0,'x1'),('n1',0,'x2'),('n2',0,'y'),('n3',0,REWARD_KEY))
  #'''
  #db.Dump(sys.stdout)
  ##The same results in different interfaces:
  #db.SDump(sys.stdout, ('n0',0,'x1'),('n1',0,'x2'),('n2',0,'y'),('n3',0,REWARD_KEY))
  #db.SDump2(sys.stdout,
      #('n0',0,lambda n:n.XS['x1'].X),
      #('n1',0,lambda n:n.XS['x2'].X),
      #('n2',0,lambda n:n.XS['y'].X),
      #('n3',0,lambda n:n.XS[REWARD_KEY].X) )
  #db.SDump2(sys.stdout,
      #('n0',None,lambda n:n[0].XS['x1'].X),
      #('n1',None,lambda n:n[0].XS['x2'].X),
      #('n2',None,lambda n:n[0].XS['y'].X),
      #('n3',None,lambda n:n[0].XS[REWARD_KEY].X) )

  ##Another test:
  #db.SDump2(sys.stdout,
      #('n0',None,lambda n:[len(n)]),
      #('nx',None,lambda n:[len(n)]) )
  calc= lambda r: math.cos(r)*r
  db.SDump2(sys.stdout,
      ('n3',None,lambda n:[calc(n[0].XS[REWARD_KEY].X[0,0])]) )

  print '###Took %f[sec]'%(time.time()-t_start)

if __name__=='__main__':
  Main()
