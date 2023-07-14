#!/usr/bin/python
'''
Dynamic programming and learning over graph-dynamical system.
'''
from __future__ import print_function
from __future__ import absolute_import
from .util import *
from .ml import *
from .ml_lwr import TLWR
from .ml_dnn import TNNRegression
from .opt import *

import multiprocessing as mp
import Queue as MPQueue

#Hashable pair (a,b). type(a) and type(b) should be hashable.
#Do not modify the content.
class TPair(object):
  def __init__(self,a,b):
    self.__a= a
    self.__b= b
  def __key(self):
    return (self.__a,self.__b)
  def __str__(self):
    return str(self.__key())
  def __repr__(self):
    return str(self.__key())
  def __eq__(x, y):
    if type(x)!=type(y):  return False
    return x.__key()==y.__key()
  def __ne__(x, y):
    if type(x)!=type(y):  return True
    return x.__key()!=y.__key()
  def __gt__(x, y):
    return x.__key()>y.__key()
  def __ge__(x, y):
    return x.__key()>=y.__key()
  def __lt__(x, y):
    return x.__key()<y.__key()
  def __le__(x, y):
    return x.__key()<=y.__key()
  def __iter__(self):
    return self.__key().__iter__()
  def __hash__(self):
    return hash(self.__key())
  @property
  def A(self):
    return self.__a
  @property
  def B(self):
    return self.__b

#Get a type of the first element of a dict x,
#return that type or default_type if x is empty.
def GetDictElemType(x, default_type=None):
  try:
    return type(x.itervalues().next())
  except StopIteration:
    return default_type


'''Define a discrete action (selection) space. '''
class TSelectDef(object):
  def __init__(self,num=None):
    self.N= num

'''Define a composite space.
  Choose Type from:
    'state': state space.
      D,Min,Max: the same as TSpaceDef, N=None
    'action': (continuous) action space.
      D,Min,Max: the same as TSpaceDef, N=None
    'select': discrete action space (selection).
      N: the same as TSelectDef, D=1,Min=0,Max=N-1 '''
class TCompSpaceDef(TSpaceDef,TSelectDef):
  def __init__(self,type=None,dim=0,min=None,max=None,num=0):
    self.Type= type
    if self.Type in ('state','action'):
      TSpaceDef.__init__(self,dim=dim,min=min,max=max)
      TSelectDef.__init__(self)
    elif self.Type==('select'):
      TSpaceDef.__init__(self,dim=1,min=[0] if num>0 else None,max=[num-1] if num>0 else None)
      TSelectDef.__init__(self,num=num)

  #Check if self.Type is type.
  def Is(self,type):
    return self.Type==type


'''Super state action dictionary (XSSA).
  XSSA is defined as: xs= {key: value,...}
    key: a string identifier.  Use the same format as variables in C/C++/Python.
      An identifier starting with dot is reserved for special purposes.
    value: a state or an action; an instance of TSSA or its subclasses.
      X: mean vector of a state or an action (continuous vector type),
          or FIXME: an integer value of an action (selection type).
      Cov: covariance matrix when X is a mean vector.
          Cov may be None, a float, or a matrix.
  XSSA-gradient is a dictionary to store dys/dxs where xs,ys are gradients.
    grads= {TPair(key_y,key_x): value,...}
      key_y,key_x: a key of ys and xs.
      value: a matrix.
'''
class TSSA(object):
  def __init__(self,x=None,cov=None):
    self.X= x
    self.Cov= cov
  def __repr__(self):
    return '({mean}, {cov})'.format(mean=ToList(self.X) if isinstance(self.X,np.ndarray) else self.X,
                                    cov=self.Cov.tolist() if isinstance(self.Cov,np.ndarray) else self.Cov)

#FIXME: Move to a good place to define
REWARD_KEY= '.r'  #A special key of XSSA to represent a reward.
PROB_KEY= '.p'  #A special key of XSSA to represent bifurcation probabilities.

#Shortcut to get a TSSA instance.
def SSA(x=None,cov=None):
  return TSSA(x=MCVec(x) if x is not None else x, cov=cov)

#Get a list of number of dimensions of state/action keys.
def DimsXSSA(space_defs,keys):
  return [space_defs[key].D for key in keys]

#Serialize some elements in an XSSA xs.
def SerializeXSSA(space_defs, xs, keys):
  dims= DimsXSSA(space_defs,keys)
  D= sum(dims)
  x_e= [0.0]*D
  cov_e= np.zeros((D,D))
  i= 0
  for key,dim in zip(keys,dims):
    i2= i+dim
    x_e[i:i2]= ToList(xs[key].X)
    cov_k, cov_k_is_zero= RegularizeCov(xs[key].Cov, dim)
    if not cov_k_is_zero:  cov_e[i:i2,i:i2]= cov_k
    i= i2
  return x_e,cov_e,dims

#Map serialized elements (x_e,cov_e) to an XSSA xs.
#WARNING: This function ignores a part of non-diagonal elements in cov_e.
def MapToXSSA(space_defs, x_e, cov_e, keys, xs):
  ssatype= GetDictElemType(xs, default_type=TSSA)
  dims= DimsXSSA(space_defs,keys)
  i= 0
  for key,dim in zip(keys,dims):
    i2= i+dim
    xs[key]= ssatype(x=x_e[i:i2], cov=cov_e[i:i2,i:i2])
    i= i2
  return dims

##Serialize XSSA-gradient to a matrix.
#def SerializeXSSAGrad(space_defs, keys_y, keys_x, grads):
  #dims_x= DimsXSSA(space_defs,keys_x)
  #dims_y= DimsXSSA(space_defs,keys_y)
  #grad_e

'''Initialize XSSA-gradient grads.
Initialize grads[TPair(key_y,key_x)]= 1.0 for all key_x==key_y
 since if F does nothing, a state/action in xs is kept in ys
 (actually that gradient is an identity matrix, but 1.0 works in the backward computation). '''
def InitXSSAGrad(space_defs, keys):
  return {TPair(key_xy,key_xy):1.0 for key_xy in keys}

#Map serialized gradients grad_e to an XSSA-gradient grads.
#grads should be initialized.
def MapToXSSAGrad(space_defs, grad_e, keys_y, keys_x, grads):
  dims_x= DimsXSSA(space_defs,keys_x)
  dims_y= DimsXSSA(space_defs,keys_y)
  iy= 0
  for key_y,dim_y in zip(keys_y,dims_y):
    iy2= iy+dim_y
    ix= 0
    for key_x,dim_x in zip(keys_x,dims_x):
      ix2= ix+dim_x
      grads[TPair(key_y,key_x)]= grad_e[ix:ix2,iy:iy2]
      ix= ix2
    iy= iy2
  return grads


#Make a copy of an XSSA xs that is a different dict object
#but has the same-type objects (i.e. references of original elements).
def CopyXSSA(xs):
  ys= {}
  for key,value in xs.iteritems():
    ys[key]= value
  return ys

#Copy XSSA elemens specified by keys from xs to ys.
#If keys is None, all elements of xs are copied.
#Modified ys is returned.
#Use with CopyXSSA, e.g. zs= PartialCopyXSSA(xs,CopyXSSA(ys),keys)
def PartialCopyXSSA(xs, ys, keys=None):
  ssatype= GetDictElemType(ys, default_type=TSSA)
  if keys is None:  keys= xs.iterkeys()
  for key in keys:
    ys[key]= xs[key] if key in xs else ssatype()
  return ys

#Extract a part of elements of XSSA.
def ExtractXSSA(xs, keys):
  ssatype= GetDictElemType(xs, default_type=TSSA)
  ys= {}
  for key in keys:
    ys[key]= xs[key] if key in xs else ssatype()
  return ys

#Convert from a standard dictionary representation d to XSSA xs.
#d would be obtained by ToStdType(xs).
def StdDictToXSSA(d):
  xs= {}
  for key,value in d.iteritems():
    xs[key]= TSSA(x=MCVec(value['X']),cov=value['Cov'])
  return xs

#XSSA version of TFunctionApprox.
#FIXME:TODO: Replace (In,Out,F) with this class.
#For some cases, this would be much efficient to compute gradients.
#Perhaps good for generalization.
class TFuncApproxXSSA(object):
  def __init__(self,In,Out):
    self.In= In
    self.Out= Out


#An episode log of a graph-dynamical system.
class TGraphEpisode(object):
  class TNode(object):
    def __init__(self,parent=None,name=None,xs=None):
      self.Parent= parent  #Index of previous stage (an index of TGraphEpisode.Seq).
      self.Name= name      #Node name (a key of TGraphDynDomain.Graph).
      self.XS= xs          #XSSA.
    def __repr__(self):
      return '{parent} {name} {xs}'.format(parent=self.Parent,name=self.Name,xs=self.XS)

  def __init__(self):
    self.Seq= []  #Each element is an TGraphEpisode.TNode.
    self.R= None  #Sum of rewards in an episode.

  @property
  def Len(self):
    return len(self.Seq)

  #Count how many times the agent visited a node "name" (a key of TGraphDynDomain.Graph).
  def NumVisits(self,name):
    return sum(1 for n in self.Seq if n.Name==name)

  '''Make a map from key_graph (a node name) to a list of nodes.
    Return key_to_nodes.
      key_to_nodes[key_graph][num_visits] gives a node object.
        key_graph is a key of TGraphDynDomain.Graph (str), i.e. a node,
        num_visits is number of visits (start from 0). '''
  def MakeKeyToNodes(self):
    key_to_nodes= {}  #Map from key_graph to list of nodes.
    for node in self.Seq:
      if node.Name in key_to_nodes:  key_to_nodes[node.Name].append(node)
      else:                          key_to_nodes[node.Name]= [node]
    return key_to_nodes

  '''Find nodes specified by keys=key,key,...; key=(key_graph,num_visits).
    key_graph is a key of TGraphDynDomain.Graph (str), i.e. a node,
    num_visits is number of visits (start from 0).
      num_visits can take a negative value similar to a list index (index from the last).
      If key_graph is None, num_visits indicates the index of self.Seq.
      If num_visits is None, all nodes with key_graph are found (as a list). '''
  def Find(self, *keys):
    if len(keys)==0:  return ()
    found= [None]*len(keys)
    '''
    keys_graph= tuple(key for key,num in keys)
    nums_visits= [num for key,num in keys]
    for node in self.Seq:
      if node.Name in keys_graph:
        i= keys_graph.index(node.Name)
        if nums_visits[i]==0:
          found[i]= node
          if None not in found:  return found
        nums_visits[i]-= 1
    '''
    key_to_nodes= self.MakeKeyToNodes()
    for i,(key,num) in enumerate(keys):
      try:
        if num is None:            found[i]= key_to_nodes[key]
        elif key is None:          found[i]= self.Seq[num]
        elif key in key_to_nodes:  found[i]= key_to_nodes[key][num]
      except (IndexError, KeyError):
        found[i]= None if num is not None else []
    return found

  def Dump(self):
    if self.R==None:  line= '#BROKEN#'
    else:  line= ''
    delim= ''
    for n,node in enumerate(self.Seq):
      line+= '{delim}{n}: {node}'.format(delim=delim,n=n,node=node)
      delim= ' '
    line+= '{delim}R: {R}'.format(delim=delim,R=self.R)
    #delim= ' '
    return line

  '''Serialized dump.  Serialization structure is given by
    keys=key,key,...; key=(key_graph,num_visits,key_xssa,key_xssa,...).
    key_graph is a key of TGraphDynDomain.Graph (str), i.e. a node,
    num_visits is number of visits (start from 0).
      num_visits can take a negative value similar to a list index (index from the last).
      If key_graph is None, num_visits indicates the index of self.Seq.
      Do not use None as num_visits.
    key_xssa is a key of XSSA; number of key_xssa is arbitrary. '''
  def SDump(self,*keys):
    keys_find= tuple(key[:2] for key in keys)
    keys_xssa= tuple(key[2:] for key in keys)
    nodes= self.Find(*keys_find)
    line= ''
    delim= ''
    for node,keys in zip(nodes,keys_xssa):
      for key in keys:
        line+= delim+ToStr(ToList(node.XS[key].X))
        delim= ' '
    line+= delim+str(self.R)
    #delim= ' '
    return line

  '''Serialized dump 2.  Similar to SDump, but this is more flexible.
    Serialization structure is given by
    keys=key,key,...
      key=(key_graph,num_visits,expr)
    expr=lambda X:... is a lambda expression where
      X is a node (TGraphEpisode.TNode) or a list of nodes.
      A node is identified by (key_graph,num_visits).
      (key_graph,None) gives all nodes with key_graph to X.
      expr should return a list or an array.
    key_graph is a key of TGraphDynDomain.Graph (str), i.e. a node,
    num_visits is number of visits (start from 0).
      num_visits can take a negative value similar to a list index (index from the last).
      If key_graph is None, num_visits indicates the index of self.Seq.
      If num_visits is None, all nodes with key_graph are used as X. '''
  def SDump2(self,*keys):
    keys_find= tuple(key[:2] for key in keys)
    exprs= tuple(key[2] for key in keys)
    nodes= self.Find(*keys_find)
    line= ''
    delim= ''
    for node,expr in zip(nodes,exprs):
      line+= delim+ToStr(ToList(expr(node)))
      delim= ' '
    line+= delim+str(self.R)
    #delim= ' '
    return line


'''Database of episodes where each episode is an instance of TGraphEpisode.'''
class TGraphEpisodeDB(object):
  def __init__(self):
    self.Entry= []  #List of episodes.

  #Save into data (dict).
  def Save(self):
    ST= ToStdType
    data= {}
    data['Entry']= ST(self.Entry)
    return data

  #Load from data (dict).
  def Load(self, data):
    if data is None:  return
    if 'Entry' in data:
      self.Entry= [None]*len(data['Entry'])
      for i,eps in enumerate(data['Entry']):
        self.Entry[i]= TGraphEpisode()
        self.Entry[i].Seq= [None]*len(eps['Seq'])
        for n,node in enumerate(eps['Seq']):
          self.Entry[i].Seq[n]= TGraphEpisode.TNode(parent=node['Parent'],name=node['Name'],xs=StdDictToXSSA(node['XS']))
        self.Entry[i].R= eps['R']

  @property
  def CurrId(self):
    return Len(self.Entry)-1

  #Add a new entry.
  def NewEntry(self):
    self.Entry.append(TGraphEpisode())

  #Update total rewards R of the current entry.
  #Actually we sum all REWARD_KEY of XSSA in Seq.
  def UpdateR(self):
    assert(self.CurrId>=0)
    if len(self.Entry[-1].Seq)>0:
      self.Entry[-1].R= sum([node.XS[REWARD_KEY].X[0,0] for node in self.Entry[-1].Seq if REWARD_KEY in node.XS])

  '''Add an node (TGraphEpisode.TNode) to the sequence of the last entry.
    parent: Index of previous stage (an index of TGraphEpisode.Seq of the same entry).
    name:   Node name (a key of TGraphDynDomain.Graph).
    xs:     XSSA.  '''
  def AddToSeq(self,parent,name,xs):
    assert(self.CurrId>=0)
    self.Entry[-1].Seq.append(TGraphEpisode.TNode(parent=parent,name=name,xs=xs))
    return len(self.Entry[-1].Seq)-1

  #Return an episode specified by index.
  def GetEpisode(self, index):
    return self.Entry[index]

  #Return indexes of episodes that satisfy condition(episode)={True,False}.
  def SearchIf(self, condition):
    return [i for i,eps in enumerate(self.Entry) if condition(eps)]

  #Return indexes of episodes that have biggest key(episode);
  #find num episodes and sort them in descending order w.r.t. key.
  def SearchTopN(self, key, num):
    keys= [key(eps) for eps in self.Entry]  #may include None
    topn= np.argsort(keys)[::-1][:num].tolist()
    return [i for i in topn if keys[i] is not None]

  #Dump to a file pointer fp (e.g. sys.stdout).
  def Dump(self, fp):
    for i,eps in enumerate(self.Entry):
      fp.write('{i} {eps}\n'.format(i=i, eps=eps.Dump()))

  #Serialized dump to a file pointer fp (e.g. sys.stdout).
  #See TGraphEpisode.SDump.
  def SDump(self, fp, *keys):
    for i,eps in enumerate(self.Entry):
      fp.write('{i} {eps}\n'.format(i=i, eps=eps.SDump(*keys)))

  #Serialized dump to a file pointer fp (e.g. sys.stdout).
  #See TGraphEpisode.SDump2.
  def SDump2(self, fp, *keys):
    for i,eps in enumerate(self.Entry):
      fp.write('{i} {eps}\n'.format(i=i, eps=eps.SDump2(*keys)))

  def DumpOne(self, index=-1):
    return self.Entry[index].Dump()

  def DumpOneYAML(self, index=-1):
    if len(self.Entry)==0:  return ''
    index= index if index>=0 else len(self.Entry)+index
    if index==0:  return yamldump({'Entry':[ToStdType(self.Entry[index])]}, Dumper=YDumper)
    else:         return yamldump([ToStdType(self.Entry[index])], Dumper=YDumper)


'''Implementations for a graph-dynamical system.  This class defines the domain.
Elements:
  Super state action dictionaries (XSSA):
    xs= {key: value,...}
    key: a string identifier.
    value: a state or an action; an instance of TSSA.
    See the explanation of TSSA.
  Dynamics and reward models:
    xs'= Fd(xs)
    xs,xs': super state action dictionary (XSSA).
    Fd: a dynamics or reward model.
      Fd uses a part of xs, and returns xs' with modifying a part of xs and/or adding new entries.
      Fd.In: a list of keys used by Fd.
      Fd.Out: a list of keys generated by Fd.
      If Fd is a reward model, Fd.Out= [REWARD_KEY] and xs'[REWARD_KEY] should be a single-element vector.
  Bifurcation probability models:
    [p_0,p_1,...,p_Nb-1]= Fp(xs)
    xs: super state action dictionary (XSSA).
    p_b: probability of b-th bifurcation, should be in [0,1].
    Nb: number of bifurcations.
    Note: sum(p_0,...,p_Nb-1) >= 1, e.g. p_0=p_1=1, which means we allow a parallel execution.
    Fp: bifurcation probability model.
      Fp uses a part of xs, and returns [p_0,...,p_Nb-1].
      Fp.In: a list of keys used by Fp.
  Graph structure:
    {key:node,...}
    key: node identifier (string).
    node= (parent, next, Fp, {Fd_0,...,Fd_Nb-1})
    parent: a parent node key.
    next= [next_0,...,next_Nb-1]: a list of succeeding node names.
    Nb= size of next, i.e. a number of bifurcations.
    Fp: a bifurcation probability model.
    {Fd_0,...,Fd_Nb-1}: dynamics or reward models of bifurcations.
    NOTE: ONLY WHEN next_b is a terminal node, Fd_b is a reward model
      (so, Fd_b.Out should contain REWARD_KEY).
    Otherwise Fd_b is a dynamics model.

For a given XSSA xs (states are filled) and a current node,
we plan actions in xs.

[System description]
SpaceDefs= {key:def, ...}
  State/action space definitions.
  def: an instance of TCompSpaceDef ('state','action',or 'select').
Models= {key:(In,Out,F), ...}
  Dynamics/reward models and bifurcation models.
  F is an instance of a subclass of TFunctionApprox.
  In: a list of input state/action keys.
  Out: a list of output state keys.
Graph= {key:node,...}
  Graph structure definitions.
  node is an instance of TDynNode.
'''
class TGraphDynDomain(object):
  def __init__(self):
    self.SpaceDefs= None   #{key:def, ...}, space definitions, key is a string (a key of XSSA), def is a TCompSpaceDef
    self.Models= None      #{key:(In,Out,F), ...}, dynamics/reward/bifurcation prob models, key is a unique string, F is a TFunctionApprox, In and Out: a list/tuple of keys of XSSA
    self.Graph= None       #{key:node,...}, graph structure, key is a unique string, node is a TDynNode
  #Check the consistency.
  def Check(self):
    #TODO: implement the consistency-check code.
    #for key_F,(In,Out,F) in self.Models.iteritems():
      #if not all(key in self.SpaceDefs for key in In+Out):
        #CPrint(4, 'Some key(s) in TGraphDynDomain.Models are not defined in .SpaceDefs:')
        #print 'Model:',key_F
        #print {key:key in self.SpaceDefs for key in In+Out}
        #return False
    #for key_graph,node in self.Graph.iteritems():
      #node.Parent= parent  #Parent node (a key of TGraphDynDomain.Graph)
      #node.Fp= Fp          #Bifurcation probability model (a key of TGraphDynDomain.Models)
      #node.Next= []        #Succeeding nodes, i.e. bifurcations (keys of TGraphDynDomain.Graph)
      #node.Fd= []          #Dynamics or reward models of bifurcations (keys of TGraphDynDomain.Models)
    return True

'''A node of a graph structure of a dynamical system.
  node= (parent, next, Fp, {Fd_0,...,Fd_Nb-1})
  Parent: a parent node key.  None means no parent.
  Next= [next_0,...,next_Nb-1]: a list of succeeding node names.
  Nb= size of next, i.e. a number of bifurcations.
  Fp: a bifurcation probability model (specify by a key).
  {Fd_0,...,Fd_Nb-1}: dynamics or reward models of bifurcations (specify by keys).
  If next_b is a terminal node, Fd_b is a reward model.
  Otherwise Fd_b is a dynamics model.

  Example:
    node= TDynNode('n0','sel0',('dyn01','n11'),('dyn02','n12'),('dyn03','n13'))
'''
class TDynNode(object):
  def __init__(self,parent=None,Fp=None,*Fd_next):
    self.Parent= parent  #Parent node (a key of TGraphDynDomain.Graph)
    self.Fp= Fp          #Bifurcation probability model (a key of TGraphDynDomain.Models)
    self.Next= []        #Succeeding nodes, i.e. bifurcations (keys of TGraphDynDomain.Graph)
    self.Fd= []          #Dynamics or reward models of bifurcations (keys of TGraphDynDomain.Models)
    for Fd,key in Fd_next:
      self.Next.append(key)
      self.Fd.append(Fd)


'''A node of a tree structure of TPlanningTree.
  Parent: a parent node key.  None means no parent.
  Next= [next_0,...,next_Nb-1]: a list of succeeding node names.
  Nb= size of next, i.e. a number of bifurcations.
  P= [p_0,...,p_Nb-1]: probabilities of bifurcations.
  XS= XSSA: super state action dictionary of this node.
  dFd,dFp,dJ: gradients of Fd[] (dynamics or reward model), Fp (bifurcation probability model), J (value function).
'''
class TPlanningNode(object):
  def __init__(self):  #FIXME:,parent=None,Fp=None,*next_Fd
    self.Parent= None  #Parent node (key of TPlanningTree.Tree)
    self.Next= []      #Next nodes (keys of TPlanningTree.Tree)
    self.P= []         #Probabilities of next nodes
    self.XS= None      #XSSA of this node
    self.J= 0.0        #Value of this node
    self.dFd= []       #[Gradients of Fd]: each is a dict of a matrix; [TPair(key_y,key_x)]= dFy/dx
    self.dFp= None     #Gradients of Fp: dict of a matrix; [key_x]= dFp/dx
    self.dJ= None      #Gradients of J: dict of a matrix; [key_x]= dJ/dx
    self.Data= None    #Data of optimizer; [key_x]= <<data>>
  def Dump(self):
    def convert(x):
      if isinstance(x,np.ndarray): return x.tolist()
      if isinstance(x,list): return [convert(value) for value in x]
      if isinstance(x,dict): return {key:convert(value) for key,value in x.iteritems()}
      return x
    line= '{'
    delim= ''
    for key,value in self.__dict__.iteritems():
      line+= '%s%s:%s'%(delim,key,str(convert(value)))
      delim= ', '
    line+= '}'
    return line

#Storing a snapshot of states/gradients like a nominal trajectory.
class TPlanningTree(object):
  def __init__(self):
    self.Start= None      #The key of a start node (a key of self.Tree)
    self.Tree= {}         #{key:node,...}, key is TPair(key_graph,num_visits), node is a TPlanningNode
    #Note: key_graph is a key of TGraphDynDomain.Graph (str), num_visits is number of visits (start from 0).
    self.Terminal= []     #Terminal nodes (a list of keys of self.Tree)
    self.BwdOrder= []     #Order of backward computation (a list of keys of self.Tree)
    self.Actions= []      #[key_xssa,...], actions to be planned, key_xssa is a key of XSSA
    self.Selections= []   #[key_xssa,...], selections to be planned, key_xssa is a key of XSSA
    self.Models= []       #[key_F,...], models used, key_F is a key of TGraphDynDomain.Models (str)
    self.FlagFwd= 0  #Forward is 0:Not computed, 1:Computed without gradients, 2:Computed with gradients.
    self.FlagBwd= 0  #Backward is 0:Not computed, 1:Computed.
  def Dump(self):
    for key in ('Start','Terminal','BwdOrder','Actions','Selections','Models','FlagFwd','FlagBwd'):
      print('%s:%s'%(key,str(self.__dict__[key])))
    print('Tree:')
    for key,node in self.Tree.iteritems():
      print('  %s:%s'%(key,node.Dump()))

  #Return a start node (TPlanningNode).
  #WARNING: After modifying the node (e.g. StartNode.XS), execute ResetFlags().
  @property
  def StartNode(self):
    return self.Tree[self.Start] if self.Start is not None else None

  #Return a value of the planning tree.
  def Value(self):
    if self.FlagFwd==0:  return 0.0
    return self.StartNode.J

  def ResetFlags(self):
    self.FlagFwd= 0
    self.FlagBwd= 0

  #Return TPlanningTree with the same structure.
  def Blank(self, xs_start=None):
    blank= TPlanningTree()
    blank.Start= self.Start
    blank.Tree= {}
    blank.Terminal= self.Terminal
    blank.BwdOrder= self.BwdOrder
    blank.Actions= self.Actions
    blank.Selections= self.Selections
    blank.Models= self.Models
    for key,node in self.Tree.iteritems():
      n_blank= TPlanningNode()
      n_blank.Parent= node.Parent
      n_blank.Next= node.Next
      blank.Tree[key]= n_blank
    if xs_start is not None:
      blank.StartNode.XS= xs_start
    return blank

'''Obtain a TPlanningTree from a graph (e.g. TGraphDynDomain.Graph) with a specific start node (a key).
  graph= {key:node,...}, key is a str, node at least contains node.Next (a list of keys of adjacent nodes).
  Loops are unrolled (max_visits indicates how many visits are allowed to each node).
  This is a breadth-first search algorithm.'''
def GraphToPTree(graph, start, max_visits):
  #ng_*: key of node on Graph
  #nt_*: key of node on Tree
  ng_start= start
  tree= TPlanningTree()
  tree.Start= TPair(ng_start,0)
  num_visits= {key:0 for key in graph.iterkeys()}
  queue= [(None,ng_start)]  #Stack of (nt_parent,ng_curr)
  while len(queue)>0:
    nt_parent,ng_curr= queue.pop(0)
    #print ng_curr, num_visits[ng_curr]
    if num_visits[ng_curr]<max_visits:
      #Add ng_curr to the tree:
      nt_curr= TPair(ng_curr,num_visits[ng_curr])
      t_node= TPlanningNode()
      t_node.Parent= nt_parent
      tree.Tree[nt_curr]= t_node
      num_visits[ng_curr]+= 1
      #Done.  Prepare for the next:
      for ng_next in graph[ng_curr].Next:
        queue.append((nt_curr,ng_next))
        #Add to the Next list; if num_visits exceeds the threshold, None is added to keep the size of Next.
        t_node.Next.append(TPair(ng_next,num_visits[ng_next]) if num_visits[ng_next]<max_visits else None)
  #Get terminal nodes:
  for key,t_node in tree.Tree.iteritems():
    if len(t_node.Next)==0:
      tree.Terminal.append(key)
  #Get order of backward computation:
  tree.BwdOrder= []
  processed= [None]+[key for key in tree.Terminal]
  queue= [tree.Tree[key].Parent for key in tree.Terminal if tree.Tree[key].Parent is not None]
  while len(queue)>0:
    key= queue.pop(0)
    if key in processed:  continue
    if all([key_next in processed for key_next in tree.Tree[key].Next]):
      tree.BwdOrder.append(key)
      processed.append(key)
      queue.append(tree.Tree[key].Parent)
    else:
      queue.append(key)  #This node (key) is not ready to compute backward
  return tree


'''Utility class of TGraphDynDomain.'''
class TGraphDynUtil(object):
  @staticmethod
  def DefaultOptions():
    Options= {}
    Options['f_reward_ucb']= 0.0  #Scale factor of UCB (Upper Confidence Bound; to compute a value J, instead of reward, we use reward+f*std_dev).
    #Options['use_prob_in_pred']= True  #Using a covariance of states/actions in prediction with forward models.
    return Options
  #@staticmethod
  #def DefaultParams():
    #Params= {}
    #return Params
  def CheckOptions(self,options):
    res= True
    defaults= self.DefaultOptions()
    for key,val in options.iteritems():
      if not key in defaults:
        print('Invalid option: %s'%(key))
        res= False
    return res

  def __init__(self):
    self.Options= {}
    #self.Params= {}
    #self.Load(data={'options':self.DefaultOptions(), 'params':self.DefaultParams()})
    self.Load(data={'options':self.DefaultOptions()})

  #Save into data (dict):  {'options':{options}}
  def Save(self):
    data= {}
    data['options']= self.Options
    return data

  #Load from data (dict):  {'options':{options}}
  def Load(self, data):
    if data!=None and 'options' in data:
      assert(self.CheckOptions(data['options']))
      InsertDict(self.Options, data['options'])
    #if data!=None and 'params' in data: InsertDict(self.Params, data['params'])

  #Initialize the utility.  We set a domain which should be an instance of TGraphDynDomain.
  def Init(self,domain):
    assert(isinstance(domain,TGraphDynDomain))
    assert(domain.Check())
    self.d= domain


  #Randomly generate actions, return as XSSA.
  def RandActions(self, actions):
    xs= {}
    for key in actions:
      ad= self.d.SpaceDefs[key]
      assert(ad.Type=='action')
      xs[key]= TSSA(MCVec(RandB(ad.Bounds)))
    return xs

  #Randomly generate selections, return as XSSA.
  def RandSelections(self, selections):
    xs= {}
    for key in selections:
      ad= self.d.SpaceDefs[key]
      assert(ad.Type=='select')
      xs[key]= TSSA(MCVec([RandI(ad.N)]))
    return xs

  #Randomly generate action noise, return as XSSA.
  def ActionNoise(self, actions, var):
    xs= {}
    for key in actions:
      ad= self.d.SpaceDefs[key]
      assert(ad.Type=='action')
      sigma= np.diag([(var*(amax-amin))**2 for amin,amax in zip(ad.Min,ad.Max)])
      va_n= np.random.multivariate_normal([0.0]*ad.D, sigma)
      xs[key]= TSSA(MCVec(va_n))
    return xs

  #Evaluate value of ptree.
  def Value(self, ptree, with_grad=False):
    #if ptree.FlagFwd==0:  self.ForwardTree(ptree, with_grad=with_grad)
    #FIXME:TODO:This is inefficient. Make a function to compute only StartNode.J, something like BackwardJ
    if ptree.FlagBwd==0:  self.BackwardTree(ptree)
    return ptree.Value()

  '''
  Criteria is DEPRECATED.
  Use self.Value(ptree) instead, which is equivalent to criteria=('sum','none').
  '''

  #Compute the forward model specified with key from an input XSSA xs,
  #  return ys (ys: output XSSA).
  #  If with_grad:  ys, grad (grad: dict of dict of a matrix; [TPair(key_y,key_x)]= dFy/dx).
  def Forward(self, key, xs, with_grad=False):
    In,Out,Fd= self.d.Models[key]
    if len(Out)==0:
      if not with_grad:  return CopyXSSA(xs)
      else:              return CopyXSSA(xs), InitXSSAGrad(self.d.SpaceDefs, xs.iterkeys())
    x_in,cov_in,dims_in= SerializeXSSA(self.d.SpaceDefs, xs, In)
    pred= Fd.Predict(x_in, cov_in, with_var=True, with_grad=with_grad)
    ys= CopyXSSA(xs)  #Note: references are copied (efficient).
    dims_out= MapToXSSA(self.d.SpaceDefs, MCVec(pred.Y), Mat(pred.Var), Out, ys)
    if not with_grad:
      return ys
    else:
      #Compute grads[TPair(key_y,key_x)]= dFy/dx.
      #This initialization assigns grads[TPair(key_xy,key_xy)]=1 for all key_xy in xy.keys()
      #since usually elements are kept as far as F does nothing.
      grads= InitXSSAGrad(self.d.SpaceDefs, xs.iterkeys())
      #Map gradients in pred to grads.
      grads= MapToXSSAGrad(self.d.SpaceDefs, Mat(pred.Grad), Out, In, grads)
      return ys, grads

  #Compute the bifurcation probability model specified with key from an input XSSA xs,
  #  return p (p: output probability list).
  #  If with_grad:  p, grad (grad: dict of a matrix; [key_x]= dFp/dx).
  def ForwardP(self, key, xs, with_grad=False):
    In,Out,Fp= self.d.Models[key]
    x_in,cov_in,dims_in= SerializeXSSA(self.d.SpaceDefs, xs, In)
    pred= Fp.Predict(x_in, cov_in, with_var=True, with_grad=with_grad)
    p= MCVec(pred.Y)
    #FIXME: How to use: pred.Var
    if not with_grad:
      return p
    else:
      grad= {}  #[key_x]= dFp/dx
      pred_Grad= Mat(pred.Grad)
      ix= 0
      for key_x,dim_x in zip(In,dims_in):
        ix2= ix+dim_x
        grad[key_x]= pred_Grad[ix:ix2,:]
        ix= ix2
      return p, grad

  '''Do forward computation of a TPlanningTree ptree.
    We start from ptree.Start (ptree.Tree[ptree.Start].XS should be given) and
    propagate XSSA with breadth-first order.
    with_grad: Whether computing gradients or not. '''
  def ForwardTree(self, ptree, with_grad=False):
    graph= self.d.Graph
    queue= [ptree.Start]
    while len(queue)>0:
      n_curr= queue.pop(0)
      tnode= ptree.Tree[n_curr]
      gnode= graph[n_curr.A]
      #Doing something with current node:
      #Compute next bifurcation probabilities:
      key_Fp= gnode.Fp
      if key_Fp is not None:
        if with_grad:  tnode.P,tnode.dFp= self.ForwardP(key_Fp, tnode.XS, with_grad=True)
        else:          tnode.P= self.ForwardP(key_Fp, tnode.XS, with_grad=False)
      tnode.dFd= [None]*len(tnode.Next)
      #Done.  Prepare for the next:
      for b,n_next in enumerate(tnode.Next):
        if n_next is not None:
          queue.append(n_next)
          #Doing something with current to next edge:
          tnode_nx= ptree.Tree[n_next]
          #Compute next XSSA:
          key_Fd= gnode.Fd[b]
          if key_Fd is not None:
            if with_grad:  tnode_nx.XS,tnode.dFd[b]= self.Forward(key_Fd, tnode.XS, with_grad=True)
            else:          tnode_nx.XS= self.Forward(key_Fd, tnode.XS, with_grad=False)
          #Done.
        else:  #i.e. n_next is None
          #In this case, we force the probability zero
          tnode.P[b]= 0.0
          if with_grad:
            for key_x,dFp in tnode.dFp.iteritems():
              dFp[:,b]= 0.0
    ptree.FlagFwd= 2 if with_grad else 1
    ptree.FlagBwd= 0
    return ptree  #Return ptree for convenience (input ptree is modified).

  '''Do backward computation of a TPlanningTree ptree.
    We compute J (value function) and its gradients dJ for all nodes in ptree.'''
  def BackwardTree(self, ptree):
    if ptree.FlagFwd in (0,1):
      self.ForwardTree(ptree, with_grad=True)
    for n_curr in ptree.Terminal:
      tnode= ptree.Tree[n_curr]
      if REWARD_KEY in tnode.XS:
        tnode.J= tnode.XS[REWARD_KEY].X[0,0]
        if self.Options['f_reward_ucb']!=0.0:
          tnode.J+= self.Options['f_reward_ucb'] * tnode.XS[REWARD_KEY].Cov[0,0]
        tnode.dJ= {REWARD_KEY:1.0}
      else:
        tnode.J= 0.0
        tnode.dJ= {REWARD_KEY:0.0}
    mplus= lambda a,b: b if a is None else a+b
    for n_curr in ptree.BwdOrder:
      tnode= ptree.Tree[n_curr]
      tnode_next= [ptree.Tree[n_next] for n_next in tnode.Next]
      J_next= MCVec([tnode_nx.J for tnode_nx in tnode_next])
      tnode.J= np.dot(ToList(tnode.P),ToList(J_next))
      tnode.dJ= {}
      for key_x in tnode.XS.iterkeys():
        dJ_dx= None
        if key_x in tnode.dFp:  dJ_dx= mplus(dJ_dx, tnode.dFp[key_x]*J_next)
        for tnode_nx,p_nx,b in zip(tnode_next,ToList(tnode.P),xrange(len(tnode_next))):
          dj= None
          dFd= tnode.dFd[b]
          for key_y in tnode_nx.XS.iterkeys():
            if TPair(key_y,key_x) in dFd and key_y in tnode_nx.dJ:
              dj= mplus(dj, dFd[TPair(key_y,key_x)]*tnode_nx.dJ[key_y])
          if dj is not None:  dJ_dx= mplus(dJ_dx, p_nx*dj)
        if dJ_dx is not None:
          tnode.dJ[key_x]= dJ_dx
    ptree.FlagBwd= 1

  #Get a planning tree at a node n_start, with XSSA xs_start if given.
  def GetPTree(self, n_start, xs_start=None, max_visits=3):
    #Unroll the dynamical graph, obtain a planning tree TPlanningTree.
    ptree= GraphToPTree(self.d.Graph, n_start, max_visits=max_visits)
    ptree.Tree[ptree.Start].XS= xs_start
    #Get actions and selections to be planned:
    models= set()  #Models used in ptree.
    for key,t_node in ptree.Tree.iteritems():
      #self.d.Graph[key.A].{Fp,Fd[0,1,...]}
      models.update({self.d.Graph[key.A].Fp})
      models.update(self.d.Graph[key.A].Fd)
    models.remove(None)
    ptree.Models= models
    actions= set()  #Actions to be planned
    selections= set()  #Selections to be planned
    for model in models:
      In,Out,F= self.d.Models[model]
      actions.update([ssa  for ssa in In if self.d.SpaceDefs[ssa].Type=='action'])
      selections.update([ssa  for ssa in In if self.d.SpaceDefs[ssa].Type=='select'])
    ptree.Actions= list(actions)
    ptree.Selections= list(selections)
    return ptree



'''Optimizer of reference vectors in graph-dynamical system.
FIXME: We need to derive equations.
'''
#class TGraphRefOpt(TGraphDynUtil):


class TGraphDDPRes(object):
  #Pseudo constants for ResCode:
  UNKNOWN       = 0
  OK            = 1  #Success
  UNNECESSARY   = 2  #Planning was unnecessary because of no actions, no selections
  FAILED        = -1
  NO_SOLUTION   = -2
  MODEL_MISSING = -3
  BAD_QUALITY   = -4
  def __init__(self, ptree, res_code=0):
    self.PTree= ptree
    self.XS= ptree.StartNode.XS
    self.ResCode= res_code
  def Msg(self):
    if self.ResCode==self.UNKNOWN       : return 'Unknown result'
    if self.ResCode==self.OK            : return 'Success'
    if self.ResCode==self.UNNECESSARY   : return 'Unnecessary to plan (no actions, no selections)'
    if self.ResCode==self.FAILED        : return 'Plan failed'
    if self.ResCode==self.NO_SOLUTION   : return 'No solution'
    if self.ResCode==self.MODEL_MISSING : return 'Some of models are missing (actions and selections have init-guess values)'
    if self.ResCode==self.BAD_QUALITY   : return 'Solution quality is bad'
    return 'Unknown error code (%d)'%self.ResCode



""" DEPRECATED: USE TGraphDDPSolver3

'''Stochastic differential dynamic programming for a graph-dynamical system.
See the system description of TGraphDynDomain.
WARNING: This solver does not plan selections (i.e. discrete actions).
WARNING: Tested in toy1, but some functions such as logging are not tested yet.

[Solver helper]
References= TODO: We will use in future.
Database= database
  database is storing previously executed episodes:
    {episode,...}, episode is an instance of TGraphEpisode or its subclass.
  database should provide following methods.
  These functions should be specialized for the current problem setup
  (note that the database may contain episodes of different dynamical systems).
  database.SearchIf(condition):
    returns indexes that episode matches condition(episode)={True,False},
    where episode should have an interface of TGraphEpisode.
  database.GetEpisode(index):
    returns an episode specified by index.  index is one of SearchIf results.
  TGraphEpisodeDB is an implementation with these interfaces.
LogFP= File pointer to a log file (can be None).
'''
#FIXME:TODO: Should be able to adjust a given action sequence (i.e. start from a given action sequence)
class TGraphDDPSolver1(TGraphDynUtil):
  class THelper:
    def __init__(self):
      self.Database= None    #Reference to a database
      self.LogFP= None       #Pointer to a log file descriptor
    #Check the consistency.
    def Check(self,domain):
      return True

  @staticmethod
  def DefaultOptions():
    #Options= {}
    Options= TGraphDynUtil.DefaultOptions()
    Options['max_visits']= 3  #Used to unroll loops in graph, indicating how many visits are allowed to each node.

    Options['initguess_using_db']= True  #Use database in initial guess.
    Options['initguess_R_min']= 0.3  #In init guess, only consider samples whose R > this value.
    Options['initguess_num']= 10  #In init guess, how many samples do we generate.

    Options['grad_act_noise']= 0.2  #Search noise used in PlanGrad.
    Options['grad_max_iter']= 40  #Max number of iterations of each gradient descent DP.
    Options['grad_min_iter']= 3   #Min number of iterations of each gradient descent DP.
    Options['grad_max_stages']= 8  #If requested quality is not achieved with grad_criterias, grad_criterias[-1] is repeated until the total # of stages reaches grad_max_stages.
    #Options['grad_term_quality_ratio']= 0.7  #Requested quality: evaluation with criteria is greater than this rate * rewards of references.
    #TODO:FIXME: 'grad_term_quality_ratio' might cause bad behaviors.

    Options['optimizer']= 'adadelta'  #Gradient descent algorithm.
    '''Options of 'optimizer'
      'gd': Standard gradient descent.
      'adadelta': Ada Delta.
    '''
    Options['gd_alpha']= 0.03
    Options['gd_nz_grad']= True  #Whether normalize gradient.
    Options['ad_rho']= 0.95       #Parameter of Ada Delta.
    Options['ad_eps']= 1.0e-3     #Parameter of Ada Delta.
    Options['ad_nz_grad']= False  #Whether normalize gradient.

    return Options
  #@staticmethod
  #def DefaultParams():
    #Params= {}
    #return Params


  def __init__(self):
    TGraphDynUtil.__init__(self)

  #Initialize the planner.  We set a domain which should be an instance of TGraphDynDomain.
  def Init(self,domain,helper):
    TGraphDynUtil.Init(self,domain)
    assert(isinstance(helper,self.THelper))
    assert(helper.Check(domain))
    self.h= helper


  '''Initial guess for gradient based search.
    Return a new TPlanningTree based on ptree, filling actions in ptree.StartNode.XS.
    ptree will be a result of self.GetPTree() '''
  def InitGuess(self, ptree):
    xs_start= ptree.StartNode.XS
    fa_data= []
    num= self.Options['initguess_num']
    if self.Options['initguess_using_db'] and self.h.Database is not None:
      #Initial guess from database (past log)
      R_min= self.Options['initguess_R_min']
      randidx= self.h.Database.SearchIf(lambda eps: eps.R is not None and eps.R>R_min)
      random.shuffle(randidx)
      for i in randidx[:num]:
        eps= self.h.Database.GetEpisode(i)
        ptree2= ptree.Blank(xs_start=PartialCopyXSSA(eps.Seq[0].XS, CopyXSSA(xs_start), ptree.Actions))
        value= self.Value(ptree2)
        fa_data.append([value,ptree2])
    for i in range(num-len(fa_data)):
      ptree2= ptree.Blank(xs_start=PartialCopyXSSA(self.RandActions(ptree.Actions), CopyXSSA(xs_start)))
      value= self.Value(ptree2)
      fa_data.append([value,ptree2])
    fa_data.sort(reverse=True,key=lambda x:x[0])
    return fa_data[0][1]


  '''Plan actions, return TPlanningTree whose start is TPair(n_start,0).
    n_start: current node (a key of TGraphDynDomain.Graph).
    xs: XSSA (actions are filled automatically). '''
  def Plan(self, n_start, xs):
    tol= 1.0e-4  #FIXME:TODO:Should be an option.
    diff_value= lambda ptree1,ptree2: self.Value(ptree1) - self.Value(ptree2)
    ptree= self.GetPTree(n_start, xs, max_visits=self.Options['max_visits'])
    ptree= self.InitGuess(ptree)
    ptree_best= None
    fp= self.h.LogFP
    if self.h.LogFP is not None:
      log= lambda ptree: self.h.LogFP.write('%f # %s # %s\n'%(
            self.Value(ptree),
            ExtractXSSA(ptree.StartNode.XS, ptree.Actions),  #Actions
            {key:ToList(ptree.StartNode.dJ[key]) for key in ptree.Actions}  #Gradients; [key_x]= dJ/dx
            ))
      log_br= lambda: self.h.LogFP.write('\n')
    else:
      log= lambda ptree: None
      log_br= lambda: None
    Ni= self.Options['grad_max_iter']
    Ni_min= self.Options['grad_min_iter']
    #R_refs= None
    for j in range(self.Options['grad_max_stages']):
      opt= self.InitOpt(ptree)
      for i in range(Ni):
        log(ptree)
        ptree_new= self.StepOpt(opt, ptree)
        dvalue= diff_value(ptree_new, ptree)
        if dvalue > 0.0: ptree= ptree_new
        #print dvalue
        if i >= Ni_min-1 and dvalue < tol:  break
      log(ptree)
      if ptree_best==None or diff_value(ptree, ptree_best)>0.0:
        ptree_best= ptree
      #if j>=Nj-1 and j<self.Options['grad_max_stages']-1 and R_refs is not None:
        #if self.Value(ptree_best) > self.Options['grad_term_quality_ratio']*R_refs:
          #break
      if j<self.Options['grad_max_stages']-1:
        #Random jump for next optimization stage.
        a_noise= self.ActionNoise(ptree_best.Actions,var=self.Options['grad_act_noise'])
        actions= {key:TSSA(ConstrainN(self.d.SpaceDefs[key].Bounds, ptree_best.StartNode.XS[key].X + a_noise[key].X)) for key in ptree_best.Actions}
        ptree= ptree_best.Blank(xs_start=PartialCopyXSSA(actions, CopyXSSA(ptree_best.StartNode.XS)))
      log_br()
    ptree= ptree_best
    log(ptree)
    return ptree

  #Initialize an optimizer for actions in start node of TPlanningTree ptree.StartNode.
  #Internal state of optimizer is also initialized and stored in ptree.StartNode.
  def InitOpt(self, ptree):
    if self.Options['optimizer']=='gd':
      opt= TGradientAscent(alpha=self.Options['gd_alpha'], normalize_grad=self.Options['gd_nz_grad'])
    elif self.Options['optimizer']=='adadelta':
      opt= TAdaDeltaMax(rho=self.Options['ad_rho'], eps=self.Options['ad_eps'], normalize_grad=self.Options['ad_nz_grad'])
    xs_start= ptree.StartNode.XS
    ptree.StartNode.Data= {key:opt.Init(xs_start[key].X) for key in ptree.Actions}
    return opt

  #One step update of actions in start node of TPlanningTree ptree.StartNode using a gradient descent.
  #Updated actions are stored in returned TPlanningTree.
  def StepOpt(self, opt, ptree):
    if ptree.FlagBwd==0: self.BackwardTree(ptree)
    pt_start= ptree.StartNode
    actions= {key:TSSA() for key in ptree.Actions}
    data= {}
    for key in ptree.Actions:
      grad= pt_start.dJ[key]
      actions[key].X,data[key]= opt.Step(pt_start.XS[key].X, grad, pt_start.Data[key])
      actions[key].X= ConstrainN(self.d.SpaceDefs[key].Bounds, actions[key].X)
    ptree2= ptree.Blank(xs_start=PartialCopyXSSA(actions, CopyXSSA(ptree.StartNode.XS)))
    ptree2.StartNode.Data= data
    return ptree2


'''Stochastic differential dynamic programming for a graph-dynamical system.
See the system description of TGraphDynDomain.
This solver plans both actions (continuous vectors) and selections (discrete actions).

[Solver helper]
References= TODO: We will use in future.
Database= database
  database is storing previously executed episodes:
    {episode,...}, episode is an instance of TGraphEpisode or its subclass.
  database should provide following methods.
  These functions should be specialized for the current problem setup
  (note that the database may contain episodes of different dynamical systems).
  database.SearchIf(condition):
    returns indexes that episode matches condition(episode)={True,False},
    where episode should have an interface of TGraphEpisode.
  database.GetEpisode(index):
    returns an episode specified by index.  index is one of SearchIf results.
  TGraphEpisodeDB is an implementation with these interfaces.
LogFP= File pointer to a log file (can be None).
'''
#FIXME:TODO: Should be able to adjust a given action sequence (i.e. start from a given action sequence)
class TGraphDDPSolver2(TGraphDynUtil):
  class THelper:
    def __init__(self):
      self.Database= None    #Reference to a database
      self.LogFP= None       #Pointer to a log file descriptor
    #Check the consistency.
    def Check(self,domain):
      return True

  @staticmethod
  def DefaultOptions():
    #Options= {}
    Options= TGraphDynUtil.DefaultOptions()
    Options['max_visits']= 3  #Used to unroll loops in graph, indicating how many visits are allowed to each node.

    Options['ptree_num']= 'auto'  #In multi-point search, how many samples do we generate.  If 'auto', automatically decided.
    Options['ptree_num_base']= 20  #Used with 'ptree_num'=='auto'.
    Options['db_init_ratio']= 0.5  #How much ratio of samples we generate from the database (remaining samples are randomly generated).
    Options['db_init_R_min']= 0.3  #In samples of database, we only use samples whose R > this value.

    Options['disc_opt']= {'alpha':0.8,'tol_same_opt':10e7}  #Options of discrete optimizer (TDiscOptProb).
    Options['max_total_iter']= 2000  #Max total-iterations.

    #Options['grad_max_iter']= 40  #Max number of iterations of each gradient descent DP.
    Options['grad_act_noise']= 0.2  #Search noise used in PlanGrad.

    Options['optimizer']= 'adadelta'  #Gradient descent algorithm.
    '''Options of 'optimizer'
      'gd': Standard gradient descent.
      'adadelta': Ada Delta.
    '''
    Options['gd_alpha']= 0.03
    Options['gd_nz_grad']= True  #Whether normalize gradient.
    Options['ad_rho']= 0.95       #Parameter of Ada Delta.
    Options['ad_eps']= 1.0e-3     #Parameter of Ada Delta.
    Options['ad_nz_grad']= False  #Whether normalize gradient.

    return Options
  #@staticmethod
  #def DefaultParams():
    #Params= {}
    #return Params


  def __init__(self):
    TGraphDynUtil.__init__(self)

  #Initialize the planner.  We set a domain which should be an instance of TGraphDynDomain.
  def Init(self,domain,helper):
    TGraphDynUtil.Init(self,domain)
    assert(isinstance(helper,self.THelper))
    assert(helper.Check(domain))
    self.h= helper


  '''Initial guess for gradient based search.
    Return a set of TPlanningTree instances whose actions in StartNode.XS are filled.
    ptree will be a result of self.GetPTree() '''
  def InitGuess(self, ptree, only_one=False):
    xs_start= ptree.StartNode.XS
    ptree_set= []
    num= self.Options['ptree_num'] if not only_one else 1
    if num=='auto':
      num= self.Options['ptree_num_base'] + sum(DimsXSSA(self.d.SpaceDefs,ptree.Selections))
    if self.Options['db_init_ratio']>0.0 and self.h.Database is not None:
      #Initial guess from database (past log)
      if num>1:
        num_db= int(num*self.Options['db_init_ratio'])
      else:
        num_db= 1 if random.random()<self.Options['db_init_ratio'] else 0  #Probabilistic decision
      R_min= self.Options['db_init_R_min']
      randidx= self.h.Database.SearchIf(lambda eps: eps.R is not None and eps.R>R_min)
      random.shuffle(randidx)
      for i in randidx[:num_db]:
        eps= self.h.Database.GetEpisode(i)
        ptree2= ptree.Blank(xs_start=PartialCopyXSSA(eps.Seq[0].XS, CopyXSSA(xs_start), ptree.Actions+ptree.Selections))
        value= self.Value(ptree2)
        ptree_set.append(ptree2)
    for i in range(num-len(ptree_set)):
      xs_start2= PartialCopyXSSA(self.RandActions(ptree.Actions), CopyXSSA(xs_start))
      PartialCopyXSSA(self.RandSelections(ptree.Selections), xs_start2)
      ptree2= ptree.Blank(xs_start=xs_start2)
      value= self.Value(ptree2)
      ptree_set.append(ptree2)
    return ptree_set


  '''Plan actions, return TPlanningTree whose start is TPair(n_start,0).
    n_start: current node (a key of TGraphDynDomain.Graph).
    xs: XSSA (actions are filled automatically). '''
  def Plan(self, n_start, xs):
    diff_value= lambda ptree1,ptree2: self.Value(ptree1) - self.Value(ptree2)
    ptree= self.GetPTree(n_start, xs, max_visits=self.Options['max_visits'])
    if len(ptree.Actions)+len(ptree.Selections)==0:  return ptree
    ptree_set= self.InitGuess(ptree)
    #print 'BEFORE:',[ptree.StartNode.XS for ptree in ptree_set]
    fp= self.h.LogFP
    if self.h.LogFP is not None:
      log= lambda ptree: self.h.LogFP.write('%f # %s # %s\n'%(
            self.Value(ptree),
            ExtractXSSA(ptree.StartNode.XS, ptree.Actions),  #Actions
            {key:ToList(ptree.StartNode.dJ[key]) for key in ptree.Actions}  #Gradients; [key_x]= dJ/dx
            ))
      log_br= lambda: self.h.LogFP.write('\n')
    else:
      log= lambda ptree: None
      log_br= lambda: None

    disc_opt= TDiscOptProb()
    options= copy.deepcopy(self.Options['disc_opt'])
    options['mapped_values']= range(len(ptree_set))
    disc_opt.Init({'options':options})

    if self.Options['optimizer']=='gd':
      grad_opt= TGradientAscent(alpha=self.Options['gd_alpha'], normalize_grad=self.Options['gd_nz_grad'])
    elif self.Options['optimizer']=='adadelta':
      grad_opt= TAdaDeltaMax(rho=self.Options['ad_rho'], eps=self.Options['ad_eps'], normalize_grad=self.Options['ad_nz_grad'])

    tol= 1.0e-6  #FIXME:TODO:Should be an option.
    stopped= [False]*len(ptree_set)
    last_values= [None]*len(ptree_set)
    for count in range(self.Options['max_total_iter']):
      if disc_opt.Stopped() or all(stopped):  break
      pt= disc_opt.Select()
      if last_values[pt] is None:
        self.InitOpt(grad_opt, ptree_set[pt])
        last_values[pt]= self.Value(ptree_set[pt])
      if not stopped[pt]:
        ptree_new= self.StepOpt(grad_opt, ptree_set[pt])
        value= self.Value(ptree_new)
        if value-last_values[pt] < 0.0:  #Get stuck?
          if pt==disc_opt.Result()[0]:  #If this is the best, we initialize other stopped ones again.
            for pt2 in range(len(ptree_set)):
              if not stopped[pt2]:  continue
              ptree_set[pt2]= self.InitGuess(ptree,only_one=True)[0]
              last_values[pt2]= None
              stopped[pt2]= False
            stopped[pt]= True
          else:                         #Else, we do init guess again
            #ptree2= ptree_set[pt]
            #a_noise= self.ActionNoise(ptree2.Actions,var=self.Options['grad_act_noise'])
            #actions= {key:TSSA(ConstrainN(self.d.SpaceDefs[key].Bounds, ptree2.StartNode.XS[key].X + a_noise[key].X)) for key in ptree2.Actions}
            ##ptree_set[pt]= ptree2.Blank(xs_start=PartialCopyXSSA(actions, CopyXSSA(ptree2.StartNode.XS)))
            #PartialCopyXSSA(actions, ptree2.StartNode.XS)
            #ptree2.ResetFlags()
            ptree_set[pt]= self.InitGuess(ptree,only_one=True)[0]
            last_values[pt]= None
        else:
          if value-last_values[pt] < tol:
            stopped[pt]= True
          ptree_set[pt]= ptree_new
          last_values[pt]= value
        #print pt,stopped[pt],disc_opt.Stopped(),count,value-last_values[pt]
      disc_opt.Update(last_values[pt])
      #print pt,stopped[pt],disc_opt.Stopped(),count,last_values[pt],disc_opt.Params['means'][pt]
      log(ptree_set[pt])
    #print 'AFTER:',[ptree.StartNode.XS for ptree in ptree_set]
    #print 'last_values:',last_values,'disc_opt.Result():',disc_opt.Result()

    return ptree_set[disc_opt.Result()[0]]

  '''
  #Optimize Actions (not Selections) in start node of TPlanningTree ptree.StartNode.
  #Initial guess of Actions and Selections should be done.
  #This is a single optimization process such as gradient descent.
  def OptPTree(self, ptree, log=lambda ptree: None):
    tol= 1.0e-6  #FIXME:TODO:Should be an option.
    opt= self.InitOpt(ptree)
    value= self.Value(ptree)
    for i in range(self.Options['grad_max_iter']):
      log(ptree)
      ptree_new= self.StepOpt(opt, ptree)
      value_new= self.Value(ptree_new)
      if value_new-value < tol:  break
      ptree= ptree_new
      value= value_new
    log(ptree)
  '''

  #Initialize an optimizer for actions in start node of TPlanningTree ptree.StartNode.
  #Internal state of optimizer is also initialized and stored in ptree.StartNode.
  def InitOpt(self, opt, ptree):
    xs_start= ptree.StartNode.XS
    ptree.StartNode.Data= {key:opt.Init(xs_start[key].X) for key in ptree.Actions}

  #One step update of actions in start node of TPlanningTree ptree.StartNode using a gradient descent.
  #Updated actions are stored in returned TPlanningTree.
  def StepOpt(self, opt, ptree):
    if ptree.FlagBwd==0: self.BackwardTree(ptree)
    pt_start= ptree.StartNode
    actions= {key:TSSA() for key in ptree.Actions}
    data= {}
    for key in ptree.Actions:
      grad= pt_start.dJ[key]
      actions[key].X,data[key]= opt.Step(pt_start.XS[key].X, grad, pt_start.Data[key])
      actions[key].X= ConstrainN(self.d.SpaceDefs[key].Bounds, actions[key].X)
    ptree2= ptree.Blank(xs_start=PartialCopyXSSA(actions, CopyXSSA(ptree.StartNode.XS)))
    ptree2.StartNode.Data= data
    return ptree2

"""


'''Stochastic differential dynamic programming for a graph-dynamical system.
See the system description of TGraphDynDomain.
This solver plans both actions (continuous vectors) and selections (discrete actions).

[Solver helper]
References= TODO: We will use in future.
Database= database
  database is storing previously executed episodes:
    {episode,...}, episode is an instance of TGraphEpisode or its subclass.
  database should provide following methods.
  These functions should be specialized for the current problem setup
  (note that the database may contain episodes of different dynamical systems).
  database.SearchIf(condition):
    returns indexes that episode matches condition(episode)={True,False},
    where episode should have an interface of TGraphEpisode.
  database.GetEpisode(index):
    returns an episode specified by index.  index is one of SearchIf results.
  TGraphEpisodeDB is an implementation with these interfaces.
LogFP= File pointer to a log file (can be None).
'''
class TGraphDDPSolver3(TGraphDynUtil):
  class THelper:
    def __init__(self):
      self.Database= None    #Reference to a database
      self.LogFP= None       #Pointer to a log file descriptor
    #Check the consistency.
    def Check(self,domain):
      return True

  @staticmethod
  def DefaultOptions():
    #Options= {}
    Options= TGraphDynUtil.DefaultOptions()
    Options['max_visits']= 3  #Used to unroll loops in graph, indicating how many visits are allowed to each node.

    Options['ptree_num']= 'auto'  #In multi-point search, how many trainee samples do we generate.  If 'auto', automatically decided.
    Options['ptree_num_base']= 20  #Used with 'ptree_num'=='auto'.
    Options['db_init_ratio']= 0.5  #How much ratio of samples we generate from the database (remaining samples are randomly generated).
    Options['db_init_R_min']= -1.0  #In samples of database, we only use samples whose R > this value.

    Options['max_total_iter']= 2000  #Max total-iterations.

    Options['prob_update_best']= 0.4  #In multi-point search, probability to update a best sample in trainee.
    Options['prob_update_rand']= 0.3   #In multi-point search, probability to update a randomly-chosen sample in trainee.
    #In multi-point search, with probability 1-prob_update_best-prob_update_rand, we update a best sample in finished (noise is added before updating).
    Options['grad_max_iter']= 50  #Max number of iterations of each gradient descent DP.
    Options['grad_act_noise']= 0.001  #Search noise used in PlanGrad.
    Options['grad_tol']= 1.0e-6  #Gradient descent tolerance (stops if value_new-value<grad_tol).
    Options['grad_max_bounce']= 10  #Gradient descent stops if number of bounce (value_new-value < 0.0) reaches this value.

    Options['optimizer']= 'adadelta'  #Gradient descent algorithm.
    '''Options of 'optimizer'
      'gd': Standard gradient descent.
      'adadelta': Ada Delta.
    '''
    Options['gd_alpha']= 0.03
    Options['gd_nz_grad']= True  #Whether normalize gradient.
    Options['ad_rho']= 0.98       #Parameter of Ada Delta.
    Options['ad_eps']= 1.0e-6     #Parameter of Ada Delta.
    Options['ad_nz_grad']= False  #Whether normalize gradient.

    return Options
  #@staticmethod
  #def DefaultParams():
    #Params= {}
    #return Params


  def __init__(self):
    TGraphDynUtil.__init__(self)

  #Initialize the planner.  We set a domain which should be an instance of TGraphDynDomain.
  def Init(self,domain,helper):
    TGraphDynUtil.Init(self,domain)
    assert(isinstance(helper,self.THelper))
    assert(helper.Check(domain))
    self.h= helper

  def GetPTreeNum(self, ptree):
    num= self.Options['ptree_num']
    if num=='auto':
      num= self.Options['ptree_num_base'] + sum(DimsXSSA(self.d.SpaceDefs,ptree.Selections))
    return num

  '''Initial guess for gradient based search.
    Return a set of TPlanningTree instances whose actions in StartNode.XS are filled.
    ptree will be a result of self.GetPTree() '''
  def InitGuess(self, ptree, only_one=False, actions=None, selections=None):
    if actions is None:  actions= ptree.Actions
    if selections is None:  selections= ptree.Selections
    xs_start= ptree.StartNode.XS
    ptree_set= []
    num= self.GetPTreeNum(ptree) if not only_one else 1
    if self.Options['db_init_ratio']>0.0 and self.h.Database is not None:
      #Initial guess from database (past log)
      if num>1:
        num_db= int(num*self.Options['db_init_ratio'])
      else:
        num_db= 1 if random.random()<self.Options['db_init_ratio'] else 0  #Probabilistic decision
      R_min= self.Options['db_init_R_min']
      randidx= self.h.Database.SearchIf(lambda eps: eps.R is not None and eps.R>R_min)
      random.shuffle(randidx)
      for i in randidx[:num_db]:
        eps= self.h.Database.GetEpisode(i)
        ptree2= ptree.Blank(xs_start=PartialCopyXSSA(eps.Seq[0].XS, CopyXSSA(xs_start), actions+selections))
        #value= self.Value(ptree2)
        ptree_set.append(ptree2)
    for i in range(num-len(ptree_set)):
      xs_start2= PartialCopyXSSA(self.RandActions(actions), CopyXSSA(xs_start))
      PartialCopyXSSA(self.RandSelections(selections), xs_start2)
      ptree2= ptree.Blank(xs_start=xs_start2)
      #value= self.Value(ptree2)
      ptree_set.append(ptree2)
    return ptree_set

  '''Plan actions, return TPlanningTree ptree whose start is TPair(n_start,0).
    ptree.StartNode.XS contains all planned actions and selections.
    n_start: current node (a key of TGraphDynDomain.Graph).
    xs: XSSA (actions and selections can be omitted; if given, they are used as init-guess).
      xs is not modified.
    return: TGraphDDPRes. '''
  def Plan(self, n_start, xs):
    diff_value= lambda ptree1,ptree2: self.Value(ptree1) - self.Value(ptree2)
    ptree= self.GetPTree(n_start, xs, max_visits=self.Options['max_visits'])
    actions_in_xs= [key for key in ptree.Actions if key in xs]
    selections_in_xs= [key for key in ptree.Selections if key in xs]
    actions_to_plan= [key for key in ptree.Actions if key not in actions_in_xs]
    selections_to_plan= [key for key in ptree.Selections if key not in selections_in_xs]

    if len(ptree.Actions)+len(ptree.Selections)==0:
      return TGraphDDPRes(ptree, TGraphDDPRes.UNNECESSARY)
    if not all(F.IsPredictable() for In,Out,F in (self.d.Models[model] for model in ptree.Models) if F is not None):
      ptree2= self.InitGuess(ptree, actions=actions_to_plan, selections=selections_to_plan, only_one=True)[0]
      return TGraphDDPRes(ptree2, TGraphDDPRes.MODEL_MISSING)

    ptree_set= self.InitGuess(ptree, actions=actions_to_plan, selections=selections_to_plan)
    if len(actions_in_xs)>0:
      for ptree2 in ptree_set:
        a_noise= self.ActionNoise(actions_in_xs,var=self.Options['grad_act_noise'])
        actions= {key:TSSA(ConstrainN(self.d.SpaceDefs[key].Bounds, xs[key].X + a_noise[key].X)) for key in actions_in_xs}
        PartialCopyXSSA(actions, ptree2.StartNode.XS)
    if len(selections_in_xs)>0:
      pass
      #TODO: ADD NOISE ON SELECTIONS???

    #if all([key in xs for key in ptree.Actions+ptree.Selections]):
      #ptree_set= []
      #for i in range(self.GetPTreeNum(ptree)):
        #a_noise= self.ActionNoise(ptree.Actions,var=self.Options['grad_act_noise'])
        #actions= {key:TSSA(ConstrainN(self.d.SpaceDefs[key].Bounds, xs[key].X + a_noise[key].X)) for key in ptree.Actions}
        #ptree2= ptree.Blank(xs_start=PartialCopyXSSA(actions, CopyXSSA(xs)))
        #ptree_set.append(ptree2)
    #else:
      #ptree_set= self.InitGuess(ptree, actions=actions_to_plan, selections=selections_to_plan)
    #print 'BEFORE:',[ptree.StartNode.XS for ptree in ptree_set]

    if self.h.LogFP is not None:
      log= lambda ptree: self.h.LogFP.write('%f # %s # %s\n'%(
            self.Value(ptree),
            ExtractXSSA(ptree.StartNode.XS, ptree.Actions+ptree.Selections),  #Actions
            {key:ToList(ptree.StartNode.dJ[key]) for key in ptree.Actions}  #Gradients; [key_x]= dJ/dx
            ))
      log_br= lambda: self.h.LogFP.write('\n')
    else:
      log= lambda ptree: None
      log_br= lambda: None

    ptree_set= [(ptree2, self.Value(ptree2)) for ptree2 in ptree_set]
    ptree_finished= []
    count= 0
    while len(ptree_set)>0 and count<self.Options['max_total_iter']:
      p= random.random()
      p1= self.Options['prob_update_best']
      p12= p1 + self.Options['prob_update_rand']
      if p<p1:
        pt_pop= ptree_set.index(max(ptree_set,key=lambda x:x[1]))
        ptree2,value= ptree_set.pop(pt_pop)
      elif p<p12:
        pt_pop= RandI(len(ptree_set))
        ptree2,value= ptree_set.pop(pt_pop)
      else:
        ptree_set.pop(ptree_set.index(min(ptree_set,key=lambda x:x[1])))  #Just throw away.
        ptree2= max(ptree_finished,key=lambda x:x[1])[0] if len(ptree_finished)>0 else max(ptree_set,key=lambda x:x[1])[0]
        a_noise= self.ActionNoise(ptree2.Actions,var=self.Options['grad_act_noise'])
        actions= {key:TSSA(ConstrainN(self.d.SpaceDefs[key].Bounds, ptree2.StartNode.XS[key].X + a_noise[key].X)) for key in ptree2.Actions}
        ptree2= ptree2.Blank(xs_start=PartialCopyXSSA(actions, CopyXSSA(ptree2.StartNode.XS)))
      ptree2,res_type,count_sub= self.OptPTree(ptree2, log)
      last_value= self.Value(ptree2)
      log_br()
      #Result type; 'good': tolerance satisfied, 'timeout': num of iterations reached max, 'bouncing': bouncing
      if res_type=='good':  #tolerance satisfied
        ptree_finished.append((ptree2, self.Value(ptree2)))
      elif res_type=='timeout':  #num of iterations reached max
        if len(ptree_finished)==0 or self.Value(ptree2) > max(ptree_finished,key=lambda x:x[1])[1]:
          ptree_finished.append((ptree2, self.Value(ptree2)))
        ptree_set.append((ptree2,self.Value(ptree2)))
      elif res_type=='bouncing':  #bouncing
        if len(ptree_finished)==0 or self.Value(ptree2) > max(ptree_finished,key=lambda x:x[1])[1]:
          ptree_finished.append((ptree2, self.Value(ptree2)))
          #ptree3= self.InitGuess(ptree,only_one=True)[0]
          #ptree_set.append((ptree3,self.Value(ptree3)))
        #else:
          #ptree3= max(ptree_finished,key=lambda x:x[1])[0]
          #a_noise= self.ActionNoise(ptree3.Actions,var=self.Options['grad_act_noise'])
          #actions= {key:TSSA(ConstrainN(self.d.SpaceDefs[key].Bounds, ptree3.StartNode.XS[key].X + a_noise[key].X)) for key in ptree3.Actions}
          #ptree3= ptree3.Blank(xs_start=PartialCopyXSSA(actions, CopyXSSA(ptree3.StartNode.XS)))
          #ptree_set.append((ptree3,self.Value(ptree3)))
        ptree3= self.InitGuess(ptree, actions=actions_to_plan, selections=selections_to_plan, only_one=True)[0]
        #FIXME: ADD NOISE TO actions_in_xs
        ptree_set.append((ptree3,self.Value(ptree3)))
      count+= count_sub
      print('DDP:', count, len(ptree_finished), len(ptree_set), max(ptree_finished,key=lambda x:x[1])[1] if len(ptree_finished)>0 else None, last_value, res_type, end=' ')
      CPrint(0,{key:ToList(ptree2.StartNode.XS[key].X) for key in ptree.Actions+ptree.Selections})

    if len(ptree_finished)>0:
      return TGraphDDPRes(max(ptree_finished,key=lambda x:x[1])[0], TGraphDDPRes.OK)
    else:
      return TGraphDDPRes(max(ptree_set,key=lambda x:x[1])[0], TGraphDDPRes.BAD_QUALITY)

  #Optimize Actions (not Selections) in start node of TPlanningTree ptree.StartNode.
  #Initial guess of Actions and Selections should be done.
  #This is a single optimization process such as gradient descent.
  def OptPTree(self, ptree, log=lambda ptree: None):
    tol= self.Options['grad_tol']
    if self.Options['optimizer']=='gd':
      opt= TGradientAscent(alpha=self.Options['gd_alpha'], normalize_grad=self.Options['gd_nz_grad'])
    elif self.Options['optimizer']=='adadelta':
      opt= TAdaDeltaMax(rho=self.Options['ad_rho'], eps=self.Options['ad_eps'], normalize_grad=self.Options['ad_nz_grad'])
    self.InitOpt(opt, ptree)
    value= self.Value(ptree)
    res_type= 'timeout'  #Result type; 'good': tolerance satisfied, 'timeout': num of iterations reached max, 'bouncing': bouncing
    bounce_patience= self.Options['grad_max_bounce']
    for count in range(self.Options['grad_max_iter']):
      log(ptree)
      ptree_new= self.StepOpt(opt, ptree)
      value_new= self.Value(ptree_new)
      #print '  ',value_new
      #print count, bounce_patience, value_new, value_new-value
      if value_new-value < 0.0:  #Get stuck?
        bounce_patience-= 1
        if bounce_patience<=0:
          res_type= 'bouncing'
          break
      elif value_new-value < tol:
        ptree= ptree_new
        res_type= 'good'
        break
      ptree= ptree_new
      value= value_new
    log(ptree)
    return ptree, res_type, count

  #Initialize an optimizer for actions in start node of TPlanningTree ptree.StartNode.
  #Internal state of optimizer is also initialized and stored in ptree.StartNode.
  def InitOpt(self, opt, ptree):
    xs_start= ptree.StartNode.XS
    ptree.StartNode.Data= {key:opt.Init(xs_start[key].X) for key in ptree.Actions}

  #One step update of actions in start node of TPlanningTree ptree.StartNode using a gradient descent.
  #Updated actions are stored in returned TPlanningTree.
  def StepOpt(self, opt, ptree):
    if ptree.FlagBwd==0: self.BackwardTree(ptree)
    pt_start= ptree.StartNode
    actions= {key:TSSA() for key in ptree.Actions}
    data= {}
    for key in ptree.Actions:
      grad= pt_start.dJ[key]
      actions[key].X,data[key]= opt.Step(pt_start.XS[key].X, grad, pt_start.Data[key])
      actions[key].X= ConstrainN(self.d.SpaceDefs[key].Bounds, actions[key].X)
    ptree2= ptree.Blank(xs_start=PartialCopyXSSA(actions, CopyXSSA(ptree.StartNode.XS)))
    ptree2.StartNode.Data= data
    return ptree2



'''Stochastic differential dynamic programming for a graph-dynamical system.
See the system description of TGraphDynDomain.
This solver plans both actions (continuous vectors) and selections (discrete actions).
Optimization is a multi-point search with multi-process programming.

[Solver helper]
References= TODO: We will use in future.
Database= database
  database is storing previously executed episodes:
    {episode,...}, episode is an instance of TGraphEpisode or its subclass.
  database should provide following methods.
  These functions should be specialized for the current problem setup
  (note that the database may contain episodes of different dynamical systems).
  database.SearchIf(condition):
    returns indexes that episode matches condition(episode)={True,False},
    where episode should have an interface of TGraphEpisode.
  database.GetEpisode(index):
    returns an episode specified by index.  index is one of SearchIf results.
  TGraphEpisodeDB is an implementation with these interfaces.
LogFP= File pointer to a log file (can be None).
'''
class TGraphDDPSolver4(TGraphDynUtil):
  class THelper:
    def __init__(self):
      self.Database= None    #Reference to a database
      self.LogFP= None       #Pointer to a log file descriptor
    #Check the consistency.
    def Check(self,domain):
      return True

  @staticmethod
  def DefaultOptions():
    #Options= {}
    Options= TGraphDynUtil.DefaultOptions()
    Options['max_visits']= 3  #Used to unroll loops in graph, indicating how many visits are allowed to each node.

    Options['ptree_num']= 'auto'  #In multi-point search, how many trainee samples do we generate.  If 'auto', automatically decided.
    Options['ptree_num_base']= 20  #Used with 'ptree_num'=='auto'.
    Options['db_init_ratio']= 0.5  #How much ratio of samples we generate from the database (remaining samples are randomly generated).
    Options['db_init_R_min']= -1.0  #In samples of database, we only use samples whose R > this value.

    Options['num_finished']= 20  #Stop optimization when the number of optimized points reaches this value.
    Options['num_proc']= 12  #Number of optimization processes.
    Options['max_total_iter']= 2000  #Max total-iterations.

    Options['prob_update_best']= 0.4  #In multi-point search, probability to update a best sample in trainee.
    Options['prob_update_rand']= 0.3   #In multi-point search, probability to update a randomly-chosen sample in trainee.
    #In multi-point search, with probability 1-prob_update_best-prob_update_rand, we update a best sample in finished (noise is added before updating).
    Options['grad_max_iter']= 50  #Max number of iterations of each gradient descent DP.
    Options['grad_act_noise']= 0.001  #Search noise used in PlanGrad.
    Options['grad_tol']= 1.0e-6  #Gradient descent tolerance (stops if value_new-value<grad_tol).
    Options['grad_max_bounce']= 10  #Gradient descent stops if number of bounce (value_new-value < 0.0) reaches this value.

    Options['optimizer']= 'adadelta'  #Gradient descent algorithm.
    '''Options of 'optimizer'
      'gd': Standard gradient descent.
      'adadelta': Ada Delta.
    '''
    Options['gd_alpha']= 0.03
    Options['gd_nz_grad']= True  #Whether normalize gradient.
    Options['ad_rho']= 0.98       #Parameter of Ada Delta.
    Options['ad_eps']= 1.0e-6     #Parameter of Ada Delta.
    Options['ad_nz_grad']= False  #Whether normalize gradient.

    return Options
  #@staticmethod
  #def DefaultParams():
    #Params= {}
    #return Params


  def __init__(self):
    TGraphDynUtil.__init__(self)

  #Initialize the planner.  We set a domain which should be an instance of TGraphDynDomain.
  def Init(self,domain,helper):
    TGraphDynUtil.Init(self,domain)
    assert(isinstance(helper,self.THelper))
    assert(helper.Check(domain))
    self.h= helper

  def GetPTreeNum(self, ptree):
    num= self.Options['ptree_num']
    if num=='auto':
      num= self.Options['ptree_num_base'] + sum(DimsXSSA(self.d.SpaceDefs,ptree.Selections))
    return num

  '''Initial guess for gradient based search.
    Return a set of TPlanningTree instances whose actions in StartNode.XS are filled.
    ptree will be a result of self.GetPTree() '''
  def InitGuess(self, ptree, only_one=False, actions=None, selections=None):
    if actions is None:  actions= ptree.Actions
    if selections is None:  selections= ptree.Selections
    xs_start= ptree.StartNode.XS
    ptree_set= []
    num= self.GetPTreeNum(ptree) if not only_one else 1
    if self.Options['db_init_ratio']>0.0 and self.h.Database is not None:
      #Initial guess from database (past log)
      if num>1:
        num_db= int(num*self.Options['db_init_ratio'])
      else:
        num_db= 1 if random.random()<self.Options['db_init_ratio'] else 0  #Probabilistic decision
      R_min= self.Options['db_init_R_min']
      randidx= self.h.Database.SearchIf(lambda eps: eps.R is not None and eps.R>R_min)
      random.shuffle(randidx)
      for i in randidx[:num_db]:
        eps= self.h.Database.GetEpisode(i)
        ptree2= ptree.Blank(xs_start=PartialCopyXSSA(eps.Seq[0].XS, CopyXSSA(xs_start), actions+selections))
        #value= self.Value(ptree2)
        ptree_set.append(ptree2)
    for i in range(num-len(ptree_set)):
      xs_start2= PartialCopyXSSA(self.RandActions(actions), CopyXSSA(xs_start))
      PartialCopyXSSA(self.RandSelections(selections), xs_start2)
      ptree2= ptree.Blank(xs_start=xs_start2)
      #value= self.Value(ptree2)
      ptree_set.append(ptree2)
    return ptree_set

  '''Plan actions, return TPlanningTree ptree whose start is TPair(n_start,0).
    ptree.StartNode.XS contains all planned actions and selections.
    n_start: current node (a key of TGraphDynDomain.Graph).
    xs: XSSA (actions and selections can be omitted; if given, they are used as init-guess).
      xs is not modified.
    return: TGraphDDPRes. '''
  def Plan(self, n_start, xs):
    diff_value= lambda ptree1,ptree2: self.Value(ptree1) - self.Value(ptree2)
    ptree= self.GetPTree(n_start, xs, max_visits=self.Options['max_visits'])
    actions_in_xs= [key for key in ptree.Actions if key in xs]
    selections_in_xs= [key for key in ptree.Selections if key in xs]
    actions_to_plan= [key for key in ptree.Actions if key not in actions_in_xs]
    selections_to_plan= [key for key in ptree.Selections if key not in selections_in_xs]

    if len(ptree.Actions)+len(ptree.Selections)==0:
      return TGraphDDPRes(ptree, TGraphDDPRes.UNNECESSARY)
    if not all(F.IsPredictable() for In,Out,F in (self.d.Models[model] for model in ptree.Models) if F is not None):
      ptree2= self.InitGuess(ptree, actions=actions_to_plan, selections=selections_to_plan, only_one=True)[0]
      return TGraphDDPRes(ptree2, TGraphDDPRes.MODEL_MISSING)

    ptree_set= self.InitGuess(ptree, actions=actions_to_plan, selections=selections_to_plan)
    if len(actions_in_xs)>0:
      for ptree2 in ptree_set:
        a_noise= self.ActionNoise(actions_in_xs,var=self.Options['grad_act_noise'])
        actions= {key:TSSA(ConstrainN(self.d.SpaceDefs[key].Bounds, xs[key].X + a_noise[key].X)) for key in actions_in_xs}
        PartialCopyXSSA(actions, ptree2.StartNode.XS)
    if len(selections_in_xs)>0:
      pass
      #TODO: ADD NOISE ON SELECTIONS???


    if self.h.LogFP is not None:
      log= lambda ptree: '%f # %s # %s\n'%(
            self.Value(ptree),
            ExtractXSSA(ptree.StartNode.XS, ptree.Actions+ptree.Selections),  #Actions
            {key:ToList(ptree.StartNode.dJ[key]) for key in ptree.Actions}  #Gradients; [key_x]= dJ/dx
            )
    else:
      log= lambda ptree: None

    ptree_set= [(ptree2, self.Value(ptree2)) for ptree2 in ptree_set]
    ptree_finished= []
    count= 0
    queue_cmd= mp.Queue()
    queue_out= mp.Queue()
    pid= 0  #process ID
    processes= {}  #pid:process
    while len(ptree_finished)<self.Options['num_finished'] and count<self.Options['max_total_iter']:
      while len(ptree_set)>0 and len(processes)<self.Options['num_proc']:
        p= random.random()
        p1= self.Options['prob_update_best']
        p12= p1 + self.Options['prob_update_rand']
        if p<p1:
          pt_pop= ptree_set.index(max(ptree_set,key=lambda x:x[1]))
          ptree2,value= ptree_set.pop(pt_pop)
        elif p<p12:
          pt_pop= RandI(len(ptree_set))
          ptree2,value= ptree_set.pop(pt_pop)
        else:
          ptree_set.pop(ptree_set.index(min(ptree_set,key=lambda x:x[1])))  #Just throw away.
          ptree2= max(ptree_finished,key=lambda x:x[1])[0] if len(ptree_finished)>0 else max(ptree_set,key=lambda x:x[1])[0]
          a_noise= self.ActionNoise(ptree2.Actions,var=self.Options['grad_act_noise'])
          actions= {key:TSSA(ConstrainN(self.d.SpaceDefs[key].Bounds, ptree2.StartNode.XS[key].X + a_noise[key].X)) for key in ptree2.Actions}
          ptree2= ptree2.Blank(xs_start=PartialCopyXSSA(actions, CopyXSSA(ptree2.StartNode.XS)))
        new_proc= mp.Process(target=self.OptPTree, args=(pid,queue_cmd,queue_out,ptree2,log))
        processes[pid]= new_proc
        processes[pid].start()
        pid+= 1

      pid_out,ptree2,res_type,count_sub,log_lines= queue_out.get()
      processes[pid_out].join()
      del processes[pid_out]

      last_value= self.Value(ptree2)
      if self.h.LogFP is not None:
        for line in log_lines:
          self.h.LogFP.write(line)
        self.h.LogFP.write('\n')

      #Result type; 'good': tolerance satisfied, 'timeout': num of iterations reached max, 'bouncing': bouncing
      if res_type=='good':  #tolerance satisfied
        ptree_finished.append((ptree2, self.Value(ptree2)))
      elif res_type=='timeout':  #num of iterations reached max
        if len(ptree_finished)==0 or self.Value(ptree2) > max(ptree_finished,key=lambda x:x[1])[1]:
          ptree_finished.append((ptree2, self.Value(ptree2)))
        ptree_set.append((ptree2,self.Value(ptree2)))
      elif res_type=='bouncing':  #bouncing
        if len(ptree_finished)==0 or self.Value(ptree2) > max(ptree_finished,key=lambda x:x[1])[1]:
          ptree_finished.append((ptree2, self.Value(ptree2)))
        ptree3= self.InitGuess(ptree, actions=actions_to_plan, selections=selections_to_plan, only_one=True)[0]
        #FIXME: ADD NOISE TO actions_in_xs
        ptree_set.append((ptree3,self.Value(ptree3)))
      count+= count_sub
      print('DDP:', count, len(ptree_finished), len(ptree_set), max(ptree_finished,key=lambda x:x[1])[1] if len(ptree_finished)>0 else None, last_value, res_type, end=' ')
      CPrint(0,{key:ToList(ptree2.StartNode.XS[key].X) for key in ptree.Actions+ptree.Selections})

    for i in xrange(len(processes)):  queue_cmd.put('stop')
    for i in xrange(len(processes)):  queue_out.get()
    for pid2,proc in processes.iteritems():  proc.join()

    if len(ptree_finished)>0:
      return TGraphDDPRes(max(ptree_finished,key=lambda x:x[1])[0], TGraphDDPRes.OK)
    else:
      return TGraphDDPRes(max(ptree_set,key=lambda x:x[1])[0], TGraphDDPRes.BAD_QUALITY)

  #Optimize Actions (not Selections) in start node of TPlanningTree ptree.StartNode.
  #Initial guess of Actions and Selections should be done.
  #This is a single optimization process such as gradient descent.
  def OptPTree(self, pid, queue_cmd, queue_out, ptree, log=lambda ptree: None):
    tol= self.Options['grad_tol']
    if self.Options['optimizer']=='gd':
      opt= TGradientAscent(alpha=self.Options['gd_alpha'], normalize_grad=self.Options['gd_nz_grad'])
    elif self.Options['optimizer']=='adadelta':
      opt= TAdaDeltaMax(rho=self.Options['ad_rho'], eps=self.Options['ad_eps'], normalize_grad=self.Options['ad_nz_grad'])
    self.InitOpt(opt, ptree)
    value= self.Value(ptree)
    res_type= 'timeout'  #Result type; 'good': tolerance satisfied, 'timeout': num of iterations reached max, 'bouncing': bouncing
    log_lines= []
    bounce_patience= self.Options['grad_max_bounce']
    for count in range(self.Options['grad_max_iter']):
      log_lines.append(log(ptree))
      ptree_new= self.StepOpt(opt, ptree)
      value_new= self.Value(ptree_new)
      #print '  ',value_new
      #print count, bounce_patience, value_new, value_new-value
      if value_new-value < 0.0:  #Get stuck?
        bounce_patience-= 1
        if bounce_patience<=0:
          res_type= 'bouncing'
          break
      elif value_new-value < tol:
        ptree= ptree_new
        res_type= 'good'
        break
      try:
        cmd= queue_cmd.get(block=False)
        if cmd=='stop':  break
      except MPQueue.Empty:
        pass
      ptree= ptree_new
      value= value_new
    log_lines.append(log(ptree))
    queue_out.put((pid, ptree, res_type, count, log_lines))

  #Initialize an optimizer for actions in start node of TPlanningTree ptree.StartNode.
  #Internal state of optimizer is also initialized and stored in ptree.StartNode.
  def InitOpt(self, opt, ptree):
    xs_start= ptree.StartNode.XS
    ptree.StartNode.Data= {key:opt.Init(xs_start[key].X) for key in ptree.Actions}

  #One step update of actions in start node of TPlanningTree ptree.StartNode using a gradient descent.
  #Updated actions are stored in returned TPlanningTree.
  def StepOpt(self, opt, ptree):
    if ptree.FlagBwd==0: self.BackwardTree(ptree)
    pt_start= ptree.StartNode
    actions= {key:TSSA() for key in ptree.Actions}
    data= {}
    for key in ptree.Actions:
      grad= pt_start.dJ[key]
      actions[key].X,data[key]= opt.Step(pt_start.XS[key].X, grad, pt_start.Data[key])
      actions[key].X= ConstrainN(self.d.SpaceDefs[key].Bounds, actions[key].X)
    ptree2= ptree.Blank(xs_start=PartialCopyXSSA(actions, CopyXSSA(ptree.StartNode.XS)))
    ptree2.StartNode.Data= data
    return ptree2




'''Utility class to manage models.  If necessary models are learned from samples.'''
class TModelManager(object):
  @staticmethod
  def DefaultOptions():
    Options= {}
    Options['type']= 'dnn'  #Model type.
    '''
      'lwr': Locally weighted regression (LWR).
      'dnn': Deep neural net (DNN).
    '''

    #Parameters for LWR:
    Options['lwr_options']= {'kernel':'l2g', 'c_min':0.01, 'f_reg':0.0001}  #Options for LWR.

    #Parameters for DNN:
    Options['dnn_hidden_units']= [200,200,200]  #Number of hidden units.
    Options['dnn_options']= {}  #Options for DNN. 'n_units' option is ignored. e.g. Options['dnn_options']['dropout_ratio']= 0.01

    Options['base_dir']= '/tmp/dpl/models/'  #Base directory.  Last '/' matters.
    return Options
  #@staticmethod
  #def DefaultParams():
    #Params= {}
    #return Params
  def CheckOptions(self,options):
    res= True
    defaults= self.DefaultOptions()
    for key,val in options.iteritems():
      if not key in defaults:
        print('Invalid option: %s'%(key))
        res= False
    return res

  '''Generate an instance.
      space_defs: the same as TGraphDynDomain.SpaceDefs.
      models: the same as TGraphDynDomain.Models.  F may be None; in this case F is learned.
        A reference of models is stored in this class, and modified. '''
  def __init__(self, space_defs, models):
    self.Options= {}
    #self.Params= {}
    #self.Load(data={'options':self.DefaultOptions(), 'params':self.DefaultParams()})
    self.Load(data={'options':self.DefaultOptions()})

    self.SpaceDefs= space_defs
    self.Models= models
    self.Learning= set()  #Memorizing learning models (keys in self.Models).

  #Save into data (dict):  {'options':{options}, 'params':{parameters}}
  #base_dir: used to store data into external data file(s); None for a default value.
  def Save(self, base_dir=None):
    if base_dir is None:  base_dir= self.Options['base_dir']
    data= {}
    data['options']= self.Options
    #data['params']= {}

    for key in self.Learning:
      In,Out,F= self.Models[key]
      prefix,path= self.GetFilePrefixPath(base_dir,key)
      SaveYAML(F.Save(prefix), path)
    return data

  #Load from data (dict):  {'options':{options}, 'params':{parameters}}
  #base_dir: where external data file(s) are stored; if None, data is not load.
  def Load(self, data, base_dir=None):
    if data!=None and 'options' in data:
      assert(self.CheckOptions(data['options']))
      InsertDict(self.Options, data['options'])
    #if data!=None and 'params' in data: InsertDict(self.Params, data['params'])
    if base_dir is None:  return

    self.CreateModels()
    for key in self.Learning:
      In,Out,F= self.Models[key]
      prefix,path= self.GetFilePrefixPath(base_dir,key)
      if os.path.exists(path):
        F.Load(LoadYAML(path), prefix)

  #Initialize planner/learner.  Should be executed before execution.
  def Init(self):
    self.CreateModels()
    for key in self.Learning:
      In,Out,F= self.Models[key]
      F.Options['base_dir']= self.Options['base_dir']
      F.Init()

  #Create learning models of self.Models[key] for key in keys.
  #If keys is None, learning models are created for all self.Models whose F is None and len(Out)>0.
  def CreateModels(self, keys=None):
    if keys is None:  keys= [key for key,(In,Out,F) in self.Models.iteritems() if F is None and len(Out)>0]
    if len(keys)==0:  return
    if self.Options['type']=='lwr':
      for key in keys:
        In,Out,F= self.Models[key]
        #dim_in= sum(DimsXSSA(self.SpaceDefs,In))
        #dim_out= sum(DimsXSSA(self.SpaceDefs,Out))
        options= copy.deepcopy(self.Options['lwr_options'])
        options['base_dir']= self.Options['base_dir']
        model= TLWR()
        model.Load(data={'options':options})
        #model.Importance= self.sample_importance  #Share importance in every model
        self.Models[key][2]= model
        self.Learning.update({key})
    elif self.Options['type']=='dnn':
      for key in keys:
        In,Out,F= self.Models[key]
        dim_in= sum(DimsXSSA(self.SpaceDefs,In))
        dim_out= sum(DimsXSSA(self.SpaceDefs,Out))
        options= copy.deepcopy(self.Options['dnn_options'])
        options['base_dir']= self.Options['base_dir']
        options['n_units']= [dim_in] + list(self.Options['dnn_hidden_units']) + [dim_out]
        options['name']= key
        model= TNNRegression()
        model.Load(data={'options':options})
        self.Models[key][2]= model
        self.Learning.update({key})

  #Return file prefix and path to save model data.
  #  key: a name of model (a key of self.Models).
  def GetFilePrefixPath(self, base_dir, key):
    if self.Options['type']=='lwr':    file_suffix= 'lwr.yaml'
    elif self.Options['type']=='dnn':  file_suffix= 'nn.yaml'
    prefix= '{base}{model}_'.format(base=base_dir, model=key)
    path= prefix+file_suffix
    return prefix,path

  '''Update a model.
    key: a name of model (a key of self.Models).
    xs: input XSSA.
    ys: output XSSA.
    not_learn: option for TFunctionApprox (if True, just storing samples; i.e. training is not performed). '''
  def Update(self, key, xs, ys, not_learn=False):
    if key not in self.Learning:  return
    In,Out,F= self.Models[key]
    x_in,cov_in,dims_in= SerializeXSSA(self.SpaceDefs, xs, In)
    x_out,cov_out,dims_out= SerializeXSSA(self.SpaceDefs, ys, Out)
    F.Update(x_in, x_out, not_learn=not_learn)

  #Dump data for plot into files.
  #file_prefix: prefix of the file names; {key} is replaced by the model name (a key of self.Models).
  #  {file_prefix}_est.dat, {file_prefix}_smp.dat are generated.
  def PlotModel(self,key,f_reduce,f_repair,file_prefix='/tmp/f{key}'):
    if key not in self.Models:
      raise Exception('PlotModel: Model not found: {key}'.format(key=key))
    In,Out,F= self.Models[key]
    if sum(DimsXSSA(self.SpaceDefs,In))==0 or sum(DimsXSSA(self.SpaceDefs,Out))==0:
      raise Exception('PlotModel: Model In/Out is zero: In={In}, Out={Out}'.format(In=In,Out=Out))
    DumpPlot(F, f_reduce=f_reduce, f_repair=f_repair, file_prefix=file_prefix.format(key=key))


#A simple test of above classes.
class TGraphDynPlanLearn(TGraphDynUtil):
  @staticmethod
  def DefaultOptions():
    #Options= {}
    Options= TGraphDynUtil.DefaultOptions()
    Options['ddp_ver']= 1
    #Options['ddp_sol']= TGraphDDPSolver3.DefaultOptions()  #Options of DDP solver (ver.1).
    Options['ddp_sol']= TGraphDDPSolver4.DefaultOptions()  #Options of DDP solver (ver.1).
    Options['base_dir']= '/tmp/dpl/'  #Base directory.  Last '/' matters.
    '''Format of log file name for each optimization.
        Set None to disable logging.
        i: i-th run, e: e-th plan execution, n: node on Graph, v: number of the node visits,
        base: Options['base_dir'].'''
    Options['opt_log_name']= '{base}seq/opt-{i:04d}-{e:03d}-{n}-{v:03d}.dat'
    return Options

  #@staticmethod
  #def DefaultParams():
    #Params= {}
    #return Params

  def __init__(self, domain, database=None, model_manager=None):
    self.own_mm= False
    TGraphDynUtil.__init__(self)
    TGraphDynUtil.Init(self,domain)

    self.database= TGraphEpisodeDB() if database is None else database
    self.model_manager= TModelManager(domain.SpaceDefs, domain.Models) if model_manager is None else model_manager
    self.own_mm= (model_manager is None)

    if self.own_mm:  self.model_manager.Options['base_dir']= self.Options['base_dir']+'models/'

  @property
  def DB(self):
    return self.database

  @property
  def MM(self):
    return self.model_manager

  #Save into data (dict):  {'options':{options}}
  #We do not save database and model_manager.
  def Save(self):
    data= {}
    data['options']= self.Options
    return data

  #Load from data (dict):  {'options':{options}}
  #We do not load database and model_manager.
  def Load(self, data):
    if data!=None and 'options' in data:
      assert(self.CheckOptions(data['options']))
      InsertDict(self.Options, data['options'])
      if self.own_mm:  self.model_manager.Options['base_dir']= self.Options['base_dir']+'models/'
    #if data!=None and 'params' in data: InsertDict(self.Params, data['params'])

  def Init(self):
    if self.own_mm:  self.model_manager.Init()

  #Get a DDP solver
  def GetDDPSol(self, logfp=None):
    domain= self.d
    #ddp_sol= TGraphDDPSolver1()
    #helper= TGraphDDPSolver1.THelper()
    #ddp_sol= TGraphDDPSolver2()
    #helper= TGraphDDPSolver2.THelper()
    #ddp_sol= TGraphDDPSolver3()
    #helper= TGraphDDPSolver3.THelper()
    ddp_sol= TGraphDDPSolver4()
    helper= TGraphDDPSolver4.THelper()
    helper.Database= self.database
    helper.LogFP= logfp
    ddp_sol.Load({'options':self.Options['ddp_sol']})
    ddp_sol.Init(domain, helper)
    return ddp_sol

  #Get a planning tree at a node n_start, with XSSA xs_start if given.
  def GetPTree(self, n_start, xs_start=None, max_visits=None):
    if max_visits is None:  max_visits= self.Options['ddp_sol']['max_visits']
    return TGraphDynUtil.GetPTree(self, n_start, xs_start, max_visits=max_visits)

  #Start a new episode.
  def NewEpisode(self):
    self.database.NewEntry()

  #End the current episode.
  def EndEpisode(self):
    self.database.UpdateR()

  '''Call DDP.Plan
    n_start: current node (a key of TGraphDynDomain.Graph).
    xs: XSSA (actions and selections can be omitted; if given, they are used as init-guess).
      Actions and selections of xs is modified by planned result.
    return: TGraphDDPRes. '''
  def Plan(self, n_start, xs):
    logfp= None
    if self.Options['opt_log_name'] is not None:
      logfp= OpenW(self.Options['opt_log_name'].format(
                  i=self.database.CurrId,
                  e=self.database.Entry[-1].NumVisits(n_start),
                  v=self.database.Entry[-1].Len,
                  n=n_start,base=self.Options['base_dir']),'w')
    ddp_sol= self.GetDDPSol(logfp)
    t_start= time.time()
    res= ddp_sol.Plan(n_start, xs)
    plan_time= time.time() - t_start  #TODO:Save plan_time into a database.
    if logfp is not None:  logfp.close()
    if res.ResCode>0 or res.ResCode in (res.MODEL_MISSING,res.BAD_QUALITY):
      if res.ResCode<=0:
        CPrint(3, 'Planning incomplete: ',res.Msg())
      #copy actions+selections into xs:
      PartialCopyXSSA(res.XS, xs, res.PTree.Actions+res.PTree.Selections)
    else:
      CPrint(4, 'Planning failed: ',res.Msg())
    return res




''' Designing a new model manager and DPL class.
Use case:
  During learning:
    dpl.UpdateModel('Fgrasp',l.xs.prev,l.xs.n1, not_learn=l.not_learn)
  Save learned stuff:
    mm.Save()
  Setup, Init and Load:
    mm_options= {
      #'type': 'lwr',
      'load_dir': l.logdir+'models/',
      'save_dir': l.logdir+'models/',
      }
    mm= TModelManager2(domain.SpaceDefs)
    mm.Load({'options':mm_options})
    mm.Load(LoadYAML(l.opt_conf['model_dir']+'model_mngr.yaml'))

    domain.Models={
      'Fgrasp': mm.Learner('Fgrasp',['gh_ratio'],['gh_abs']),
      'Fmvtorcv': mm.Combined(
          mm.Learner('Fmvtorcv_1',['ps_rcv','gh_abs','p_pour','p_pour_trg0'],['ps_rcv']),
          mm.Learner('Fmvtorcv_2',['ps_rcv','gh_abs','p_pour','p_pour_trg0'],['v_rcv']),
          ),
      'P1': mm.Manual([],[PROB_KEY], TLocalLinear(0,1,lambda x:[1.0],lambda x:[0.0])),
      ...
      }
'''

'''Utility class to manage models.  If necessary models are learned from samples.'''
class TModelManager2(object):
  @staticmethod
  def DefaultOptions():
    Options= {}
    #Options['base_dir']= '/tmp/dpl/models/'  #Base directory.  Last '/' matters.
    Options['load_dir']= '/tmp/dpl/models/'  #Load base directory (can be None).  Last '/' matters.
    Options['save_dir']= '/tmp/dpl/models/'  #Save base directory.  Last '/' matters.  Learned data and training logs will be stored.

    Options['type']= 'dnn'  #Learning model type.
    '''
      'lwr': Locally weighted regression (LWR).
      'dnn': Deep neural net (DNN).
    '''

    #Parameters for LWR:
    Options['lwr_options']= {'kernel':'l2g', 'c_min':0.01, 'f_reg':0.0001}  #Options for LWR.

    #Parameters for DNN:
    Options['dnn_hidden_units']= [200,200,200]  #Number of hidden units.
    Options['dnn_options']= {}  #Options for DNN. 'n_units' option is ignored. e.g. Options['dnn_options']['dropout_ratio']= 0.01
    return Options
  #@staticmethod
  #def DefaultParams():
    #Params= {}
    #return Params
  def CheckOptions(self,options):
    res= True
    defaults= self.DefaultOptions()
    for key,val in options.iteritems():
      if not key in defaults:
        print('Invalid option: %s'%(key))
        res= False
    return res

  '''Generate an instance.
      space_defs: the same as TGraphDynDomain.SpaceDefs. '''
  def __init__(self, space_defs):
    self.Options= {}
    #self.Params= {}
    #self.Load(data={'options':self.DefaultOptions(), 'params':self.DefaultParams()})
    self.Load(data={'options':self.DefaultOptions()})

    self.SpaceDefs= space_defs
    self.Learning= {}  #Memorizing learning models (key:[In,Out,F], key may be different from keys in domain.Models).

  #Load options from data (dict):  {'options':{options}, 'params':{parameters}}
  def Load(self, data):
    if data!=None and 'options' in data:
      assert(self.CheckOptions(data['options']))
      InsertDict(self.Options, data['options'])
    #if data!=None and 'params' in data: InsertDict(self.Params, data['params'])

  #Manually define a model.
  def Manual(self, In, Out, F):
    return [In,Out,F]

  #Make a learning model, and return a model triple [In,Out,F].
  #If Options['load_dir'] is not None and model files exist, the parameters are loaded.
  def Learner(self, key, In, Out):
    F= self.CreateModel(key,In,Out)

    if self.Options['load_dir'] is not None:
      prefix,path= self.GetFilePrefixPath(self.Options['load_dir'],key)
      if os.path.exists(path):
        F.Load(LoadYAML(path), prefix)

    F.Options['base_dir']= self.Options['save_dir']
    F.Init()

    return [In,Out,F]

  #Make a combined model combining models (list of [In,Out,F] tuples where F is a TFunctionApprox).
  def Combined(self, *models):
    if len(models)==0:  return [[],[], None]
    CmbIn= []  #Keys of whole input
    CmbOut= []  #Keys of whole output
    for In,Out,F in models:
      seen=set(CmbIn)
      CmbIn.extend(key for key in In if key not in seen and not seen.add(key))
      seen=set(CmbOut)
      CmbOut.extend(key for key in Out if key not in seen and not seen.add(key))

    cdims_in= DimsXSSA(self.SpaceDefs,CmbIn)
    cdims_out= DimsXSSA(self.SpaceDefs,CmbOut)
    cum= np.cumsum(cdims_in)
    k2i_in= {key:range(cum[i-1] if i>0 else 0,cum[i]) for i,key in enumerate(CmbIn)}  #Map of key to indexes in a whole input vector.
    cum= np.cumsum(cdims_out)
    k2i_out= {key:range(cum[i-1] if i>0 else 0,cum[i]) for i,key in enumerate(CmbOut)}  #Map of key to indexes in a whole output vector.

    cmodels= []  #List of [In2,Out2,F] tuples where In2,Out2 are indexes of whole input,output vector.
    for In,Out,F in models:
      In2= sum((k2i_in[key] for key in In),[])
      Out2= sum((k2i_out[key] for key in Out),[])
      cmodels.append([In2,Out2,F])

    CmbF= TCombinedFuncApprox(sum(cdims_in),sum(cdims_out),*cmodels)
    return [CmbIn,CmbOut,CmbF]

  #Save into data (dict):  {'options':{options}, 'params':{parameters}}
  #base_dir: used to store data of learned models into external data file(s); None for a default value (Options['save_dir']).
  def Save(self, base_dir=None):
    if base_dir is None:  base_dir= self.Options['save_dir']
    data= {}
    data['options']= self.Options
    #data['params']= {}

    for key,(In,Out,F) in self.Learning.iteritems():
      prefix,path= self.GetFilePrefixPath(base_dir,key)
      SaveYAML(F.Save(prefix), path)
    return data

  #Create a learning model.
  #The created model is stored in self.Learning.
  def CreateModel(self, key, In, Out):
    F= None
    if self.Options['type']=='lwr':
      #dim_in= sum(DimsXSSA(self.SpaceDefs,In))
      #dim_out= sum(DimsXSSA(self.SpaceDefs,Out))
      options= copy.deepcopy(self.Options['lwr_options'])
      options['base_dir']= self.Options['load_dir']
      F= TLWR()
      F.Load(data={'options':options})
      #F.Importance= self.sample_importance  #Share importance in every model
    elif self.Options['type']=='dnn':
      dim_in= sum(DimsXSSA(self.SpaceDefs,In))
      dim_out= sum(DimsXSSA(self.SpaceDefs,Out))
      options= copy.deepcopy(self.Options['dnn_options'])
      options['base_dir']= self.Options['load_dir']
      options['n_units']= [dim_in] + list(self.Options['dnn_hidden_units']) + [dim_out]
      options['name']= key
      F= TNNRegression()
      F.Load(data={'options':options})
    self.Learning[key]= [In,Out,F]
    return F

  #Return file prefix and path to save model data.
  #  key: a name of model.
  def GetFilePrefixPath(self, base_dir, key):
    if self.Options['type']=='lwr':    file_suffix= 'lwr.yaml'
    elif self.Options['type']=='dnn':  file_suffix= 'nn.yaml'
    prefix= '{base}{model}_'.format(base=base_dir, model=key)
    path= prefix+file_suffix
    return prefix,path


#A simple utility of Graph-DDP and model learning.
class TGraphDynPlanLearn2(TGraphDynUtil):
  @staticmethod
  def DefaultOptions():
    #Options= {}
    Options= TGraphDynUtil.DefaultOptions()
    Options['ddp_ver']= 1
    #Options['ddp_sol']= TGraphDDPSolver3.DefaultOptions()  #Options of DDP solver (ver.1).
    Options['ddp_sol']= TGraphDDPSolver4.DefaultOptions()  #Options of DDP solver (ver.1).
    Options['base_dir']= '/tmp/dpl/'  #Base directory.  Last '/' matters.
    '''Format of log file name for each optimization.
        Set None to disable logging.
        i: i-th run, e: e-th plan execution, n: node on Graph, v: number of the node visits,
        base: Options['base_dir'].'''
    Options['opt_log_name']= '{base}seq/opt-{i:04d}-{e:03d}-{n}-{v:03d}.dat'
    return Options

  #@staticmethod
  #def DefaultParams():
    #Params= {}
    #return Params

  def __init__(self, domain, database=None):
    TGraphDynUtil.__init__(self)
    TGraphDynUtil.Init(self,domain)

    self.database= TGraphEpisodeDB() if database is None else database

  @property
  def DB(self):
    return self.database

  #Save into data (dict):  {'options':{options}}
  #We do not save database.
  def Save(self):
    data= {}
    data['options']= self.Options
    return data

  #Load from data (dict):  {'options':{options}}
  #We do not load database.
  def Load(self, data):
    if data!=None and 'options' in data:
      assert(self.CheckOptions(data['options']))
      InsertDict(self.Options, data['options'])
    #if data!=None and 'params' in data: InsertDict(self.Params, data['params'])

  #Get a DDP solver
  def GetDDPSol(self, logfp=None):
    domain= self.d
    #ddp_sol= TGraphDDPSolver1()
    #helper= TGraphDDPSolver1.THelper()
    #ddp_sol= TGraphDDPSolver2()
    #helper= TGraphDDPSolver2.THelper()
    #ddp_sol= TGraphDDPSolver3()
    #helper= TGraphDDPSolver3.THelper()
    ddp_sol= TGraphDDPSolver4()
    helper= TGraphDDPSolver4.THelper()
    helper.Database= self.database
    helper.LogFP= logfp
    ddp_sol.Load({'options':self.Options['ddp_sol']})
    ddp_sol.Init(domain, helper)
    return ddp_sol

  #Get a planning tree at a node n_start, with XSSA xs_start if given.
  def GetPTree(self, n_start, xs_start=None, max_visits=None):
    if max_visits is None:  max_visits= self.Options['ddp_sol']['max_visits']
    return TGraphDynUtil.GetPTree(self, n_start, xs_start, max_visits=max_visits)

  #Start a new episode.
  def NewEpisode(self):
    self.database.NewEntry()

  #End the current episode.
  def EndEpisode(self):
    self.database.UpdateR()

  '''Call DDP.Plan
    n_start: current node (a key of TGraphDynDomain.Graph).
    xs: XSSA (actions and selections can be omitted; if given, they are used as init-guess).
      Actions and selections of xs is modified by planned result.
    return: TGraphDDPRes. '''
  def Plan(self, n_start, xs):
    logfp= None
    if self.Options['opt_log_name'] is not None:
      logfp= OpenW(self.Options['opt_log_name'].format(
                  i=self.database.CurrId,
                  e=self.database.Entry[-1].NumVisits(n_start),
                  v=self.database.Entry[-1].Len,
                  n=n_start,base=self.Options['base_dir']),'w')
    ddp_sol= self.GetDDPSol(logfp)
    t_start= time.time()
    res= ddp_sol.Plan(n_start, xs)
    plan_time= time.time() - t_start  #TODO:Save plan_time into a database.
    if logfp is not None:  logfp.close()
    if res.ResCode>0 or res.ResCode in (res.MODEL_MISSING,res.BAD_QUALITY):
      if res.ResCode<=0:
        CPrint(3, 'Planning incomplete: ',res.Msg())
      #copy actions+selections into xs:
      PartialCopyXSSA(res.XS, xs, res.PTree.Actions+res.PTree.Selections)
    else:
      CPrint(4, 'Planning failed: ',res.Msg())
    return res

  '''Update a model.
    key: a name of model (a key of self.d.Models).
    xs: input XSSA.
    ys: output XSSA.
    not_learn: option for TFunctionApprox (if True, just storing samples; i.e. training is not performed). '''
  def UpdateModel(self, key, xs, ys, not_learn=False):
    if key not in self.d.Models:  return
    In,Out,F= self.d.Models[key]
    x_in,cov_in,dims_in= SerializeXSSA(self.d.SpaceDefs, xs, In)
    x_out,cov_out,dims_out= SerializeXSSA(self.d.SpaceDefs, ys, Out)
    F.Update(x_in, x_out, not_learn=not_learn)



