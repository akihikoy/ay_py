#!/usr/bin/python
'''
Dynamic programming and learning over network dynamical system.
'''
from base_util import *
from base_ml import *
from base_ml_lwr2 import TLWR
from base_ml_dnn import TNNRegression
from base_dpl import OptCMA, OptCMA2, NormalizeGrad

'''
First we upgrade the dynamic programming and learning for
chain dynamical systems.
The purpose of this upgrade is:
Since TDynPlanLearn2 contains everything, i.e.
  1. dynamical model learner,
  2. reference vector optimizer,
  3. DDP,
  4. database (library of samples),
  5. logger,
  6. some crap codes (debug, test, old codes, etc.).
Some of them are used in DDP for dynamical system
with more complicated structures (e.g. bifurcations).
Thus we decompose the components into several classes
in order to make it easy to reuse.
2 and 3 are redesigned as optimizers for chain dynamical systems,
that will be a part of implementation in more complicated structures.
'''

class TXYAR(object):
  def __init__(self,x=None,y=None,a=None,r=None):
    self.x= x  #State
    self.y= y  #Output
    self.a= a  #Action
    self.r= r  #Reward

#x,y,a,r and n
class TIndexedXYAR(TXYAR):
  def __init__(self,n=None,x=None,y=None,a=None,r=None):
    TXYAR.__init__(self,x=x,y=y,a=a,r=r)
    self.n= n

class TChainEpisode(object):
  def __init__(self, seq_len=None):
    self.Seq= []  #Each element should have an interface of TXYAR.
    self.R= None  #Sum of rewards.
    if seq_len is not None:  self.Seq= [TXYAR() for i in range(seq_len)]

  @property
  def Len(self):
    return len(self.Seq)

  def Dump(self):
    if self.R==None:  return '#BROKEN_DATA'
    line= ''
    i_cols= 1
    delim= ''
    for n,xyar in enumerate(self.Seq):
      i_cols+= 7 + Len(xyar.x) + Len(xyar.y) + Len(xyar.a)
      line+= '{delim}{n} #x {x} #y {y} #a {a} #r {r} #{i}#'.format(
        delim=delim, n=n, x=ToStr(xyar.x), y=ToStr(xyar.y),
        a= ToStr(xyar.a), r= xyar.r if xyar.r is not None else 0.0, i=i_cols-1)
      delim= ' '
    i_cols+= 3
    line+= '{delim}#R {R} #{i}#'.format(delim=delim, R=self.R, i=i_cols-1)
    delim= ' '
    #line+= ' #%i# #Tplan %s '%(i_cols, ToStr(self.xar_log[idx].plan_time) )
    return line


'''Database of episodes where each episode is an instance of TChainEpisode.'''
class TChainEpisodeDB(object):
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
        self.Entry[i]= TChainEpisode()
        self.Entry[i].Seq= [None]*len(eps['Seq'])
        for n,xyar in enumerate(eps['Seq']):
          self.Entry[i].Seq[n]= TXYAR(x=xyar['x'],y=xyar['y'],a=xyar['a'],r=xyar['r'])
        self.Entry[i].R= eps['R']

  @property
  def CurrentId(self):
    return Len(self.Entry)-1

  #Add a new entry.
  def NewEntry(self, seq_len=None):
    self.Entry.append(TChainEpisode(seq_len))

  #Update total rewards R of the current entry.
  def UpdateR(self):
    assert(self.CurrentId>=0)
    self.Entry[-1].R= sum([xyar.r for xyar in self.Entry[-1].Seq if xyar.r!=None])

  #Modify n-th stage of the sequence of the last entry.
  def ModifySeq(self,n=None,x=None,y=None,a=None,r=None):
    assert(self.CurrentId>=0)
    while self.Entry[-1].Len<=n:  self.Entry[-1].Seq.append(TXYAR())
    seq= self.Entry[-1].Seq[n]
    if x is not None:  seq.x= copy.deepcopy(ToList(x))
    if y is not None:  seq.y= copy.deepcopy(ToList(y))
    if a is not None:  seq.a= copy.deepcopy(ToList(a))
    if r is not None:  seq.r= copy.deepcopy(r)

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

  def DumpOne(self, index=-1):
    self.Entry[index].Dump()


'''Implementations for a chain dynamical system.  This class defines the domain.
Assume a system:
  x_0,a_0 -> F_0 -> x_1,a_1 -> F_1 -> ... -> x_N-1,a_N-1 -> F_N-1 -> x_N
                 -> y_1                   -> y_N-1                -> y_N
  x_k: state at k-th stage, we also consider covx_k (covariance of Normal distr.),
  a_k: action at k-th stage,
  y_k: output values of x_k-1,a_k-1, we also consider covy_k (covariance of Normal distr.).
  Each dynamical model is: (x_k+1,covx_k+1,y_k+1,covy_k+1)= F_k(x_k,covx_k,a_k)
  Or: (x_k+1,covx_k+1)= Fx_k(x_k,covx_k,a_k), (y_k+1,covy_k+1)= Fy_k(x_k,covx_k,a_k)
x_N=null (don't care), x_0 can be null, a_k can be null, y_0= null (don't care).
There are N+1 stages and N (partial) dynamics in total.

We will plan {a_k,...,a_N-1} for a given x_k, covx_k.

[System description]
StateDefs= [def(x_0),...,def(x_N)]
  State definitions; def(x_k) must be an instance of TSpaceDef.
  def(x_0).D may be zero.  Other def(x_k).D (k>0) must be greater than 0.  def(x_N) is ignored.
  Note: If def(x_k).D==0, then separate the problem at that point.
OutDefs= [def(y_0),...,def(y_N)]
  Output definitions; def(y_k) must be an instance of TSpaceDef.
  def(y_0) is ignored.  Other def(y_k).D (k>0) may be zero.
ActionDefs= [def(a_0),...,def(a_N)]
  Action definitions; def(a_k) must be an instance of TSpaceDef.
  def(a_k).D can be zero, which means no action is taken.
ModelsX= [Fx_0(x_0,a_0),...,Fx_N-1(x_N-1,a_N-1)]
  Dynamical models(x); Fx_k should be an instance of TFunctionApprox.
ModelsY= [Fy_0(x_0,a_0),...,Fy_N-1(x_N-1,a_N-1)]
  Dynamical models(y); Fy_k should be an instance of TFunctionApprox or None.
Rewards= [R_0(y_0),...,R_N(y_N)]
  Reward models; R_k should be an instance of TFunctionApprox or None.
  k-th reward function: r_k= R_k(y_k); R_0 should be None (just ignored).
'''
class TChainDomainRL(object):
  def __init__(self):
    self.N= None           #Number of stages
    self.StateDefs= None   #List of N+1 TSpaceDef
    self.OutDefs= None     #List of N+1 TSpaceDef
    self.ActionDefs= None  #List of N+1 TSpaceDef
    self.Rewards= None     #List of N+1 TFunctionApprox
  #Check the consistency.
  def Check(self):
    if not self.N>0:  return False
    if not len(self.StateDefs)==self.N+1:  return False
    if not len(self.OutDefs)==self.N+1:  return False
    if not len(self.ActionDefs)==self.N+1:  return False
    if not len(self.Rewards)==self.N+1:  return False
    return True

class TChainDomainDP(TChainDomainRL):
  def __init__(self):
    TChainDomainRL.__init__(self)
    self.ModelsX= None     #List of N TFunctionApprox
    self.ModelsY= None     #List of N TFunctionApprox
  #Check the consistency.
  def Check(self):
    if not TChainDomainRL.Check(self):  return False
    if not len(self.ModelsX)==self.N:  return False
    if not len(self.ModelsY)==self.N:  return False
    return True

'''Utility class of TChainDomainRL.'''
class TChainRLUtil(object):
  @staticmethod
  def DefaultOptions():
    Options= {}
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
        print 'Invalid option: %s'%(key)
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

  #Initialize the utility.  We set a domain which should be an instance of TChainDomainRL.
  def Init(self,domain):
    assert(isinstance(domain,TChainDomainRL))
    assert(domain.Check())
    self.d= domain

  def Dx(self,n):
    return self.d.StateDefs[n].D

  def Dy(self,n):
    return self.d.OutDefs[n].D

  def Da(self,n):
    if n==self.d.N:  return 0
    return self.d.ActionDefs[n].D

  #Compute an immediate reward R_n(y_n) from an output distr. N(y_n, var_y_n)
  #  return R_n, var_R_n= E[R_n], var[R_n].
  #  If with_grad:  R_n, var_R_n, dR_n/dy_n
  def Reward(self, n, y_n, var_y_n=None, with_grad=False):
    if y_n is None or Len(y_n)==0 or self.d.Rewards[n] is None or self.Dy(n)==0:
      R_n= 0.0
      var_R_n= 0.0
      dR_dy= np.zeros((0,1))
    else:
      pred= self.d.Rewards[n].Predict(ToList(y_n), var_y_n, with_var=True, with_grad=with_grad)
      R_n= pred.Y[0,0]
      var_R_n= pred.Var[0,0]
      dR_dy= pred.Grad
    if not with_grad:  return R_n, var_R_n
    else:              return R_n, var_R_n, MCVec(dR_dy)


'''Utility class of TChainDomainDP.'''
class TChainDPUtil(TChainRLUtil):
  @staticmethod
  def DefaultOptions():
    #Options= {}
    Options= TChainRLUtil.DefaultOptions()
    Options['use_prob_in_pred']= True  #TEST: Using a covariance of x,a in prediction with forward models.
    return Options
  #@staticmethod
  #def DefaultParams():
    #Params= {}
    #return Params

  def __init__(self):
    TChainRLUtil.__init__(self)

  #Initialize the utility.  We set a domain which should be an instance of TChainDomainDP.
  def Init(self,domain):
    #TChainRLUtil.Init(self,domain)
    assert(isinstance(domain,TChainDomainDP))
    assert(domain.Check())
    self.d= domain

  #Get bounds of state space at stage n.
  def BoundsX(self,n):
    return self.d.StateDefs[n].Bounds

  #Get bounds of output space at stage n.
  def BoundsY(self,n):
    return self.d.OutDefs[n].Bounds

  #Get bounds of action space at stage n.
  def BoundsA(self,n):
    return self.d.ActionDefs[n].Bounds

  #Get bounds of state+action space at stage n.
  #return bounds=[xa_min, xa_max]
  def BoundsXA(self,n):
    bounds= self.BoundsX(n)
    if self.d.ActionDefs[n].D>0:
      return [bounds[0]+self.d.ActionDefs[n].Min, bounds[1]+self.d.ActionDefs[n].Max]
    return bounds

  #Get covariance from var_x_n and var_a_n for d.Models*[n].Predict.
  def XAVar(self, n, var_x_n=None, var_a_n=None):
    if not self.Options['use_prob_in_pred']:  return 0.0
    #Note: following works even when Dx_n==0 or Da_n==0.
    Dx_n= self.Dx(n); Da_n= self.Da(n)
    var_x, var_x_is_zero= RegularizeCov(var_x_n, Dx_n)
    var_a, var_a_is_zero= RegularizeCov(var_a_n, Da_n)
    var_xa= np.zeros((Dx_n+Da_n,Dx_n+Da_n))
    if not var_x_is_zero:  var_xa[:Dx_n,:Dx_n]= var_x
    if not var_a_is_zero:  var_xa[Dx_n:,Dx_n:]= var_a
    return var_xa

  #Compute the forward models from an input distr. N(x_n,var_x_n) and N(a_n,var_a_n)
  #  return x_next, var_x_next, y_next, var_y_next  (as np.matrix objects, x_next and y_next are column vectors).
  #  If with_grad:  x_next, var_x_next, y_next, var_y_next, dFx_n/dxa, dFy_n/dxa.
  def Forward(self, n, x_n=None, var_x_n=None, a_n=None, var_a_n=None, with_grad=False):
    xa_n= ToList(x_n)+ToList(a_n)
    var_xa_n= self.XAVar(n, var_x_n, var_a_n)
    pred_x= self.d.ModelsX[n].Predict(xa_n, var_xa_n, with_var=True, with_grad=with_grad)
    pred_y= self.d.ModelsY[n].Predict(xa_n, var_xa_n, with_var=True, with_grad=with_grad)
    #if self.d.OutDefs[n+1].D!=0 and self.d.ModelsY[n] is not None:
      #pred_y= self.d.ModelsY[n].Predict(xa_n, var_xa_n, with_var=True, with_grad=with_grad)
    #else:
      #pred_y= TFunctionApprox.TPredRes()
      #pred_y.Grad= np.zeros((Len(xa_n),0))
      ##Note: pred_y.Y==None which gives: MCVec(pred_y.Y)==np.zeros((0,1))
    if not with_grad:
      return MCVec(pred_x.Y), Mat(pred_x.Var), MCVec(pred_y.Y), Mat(pred_y.Var)
    else:
      return MCVec(pred_x.Y), Mat(pred_x.Var), MCVec(pred_y.Y), Mat(pred_y.Var), Mat(pred_x.Grad), Mat(pred_y.Grad)


'''Optimizer of reference vectors in chain dynamical system.
These reference vectors are expected to be used as TChainDDPSolver.THelper.References.
See the system description of TChainDomainDP.

The definition of reference vectors is:
References= [refs_0,...,refs_N]
  List of reference vectors; refs_k should be a list of TXYAR or its subclass (refs_k[i].r is internally used).
  k-th references: refs_k= [xyarref_k_0,xyarref_k_1,...] (good-->bad order).

[Optimizer helper]
Database= database
  database is storing previously executed episodes:
    {episode,...}, episode is an instance of TChainEpisode or its subclass.
  database should provide following methods.
  These functions should be specialized for the current problem setup
  (note that the database may contain episodes of different dynamical systems).
  database.SearchTopN(key,num):
    returns indexes of episodes that have biggest key(episode);
    extract num episodes and sort them in descending order.
  database.GetEpisode(index):
    returns an episode specified by index.  index is one of SearchTopN results.
'''
class TChainRefOpt(TChainDPUtil):
  class THelper:
    def __init__(self):
      self.Database= None    #Reference to a database
    #Check the consistency.
    def Check(self,domain):
      return True

  @staticmethod
  def DefaultOptions():
    #Options= {}
    Options= TChainDPUtil.DefaultOptions()
    Options['grad_iter']= 3  #Number of gradient descent iterations in optimizing references.
    Options['num_refs']= 6  #Number of references to be searched.
    Options['num_rpl_best']= 3  #Number of references to be replaced by xar_log_best in ModifyRefs.
    Options['num_rpl_rand']= 1  #Number of references to be replaced by random value in ModifyRefs.
    Options['reward_shape_f']= 0.1  #Factor of reward shape, i.e. blend ratio of reward and state error.
    return Options

  #@staticmethod
  #def DefaultParams():
    #Params= {}
    #return Params

  def __init__(self):
    TChainDPUtil.__init__(self)

  #Initialize the optimizer.  We set a domain which should be an instance of TChainDomainDP.
  def Init(self,domain,helper):
    TChainDPUtil.Init(self,domain)
    assert(isinstance(helper,self.THelper))
    assert(helper.Check(domain))
    self.h= helper

  def RandX(self,n):
    return RandB(self.BoundsX(n))

  def RandY(self,n):
    return RandB(self.BoundsY(n))

  def RandA(self,n):
    return RandB(self.d.ActionDefs[n].Bounds)


  #Optimize a set of reference vectors of {n, n+1, ..., N} stages.
  #references= [refs_0,...,refs_N].
  def Optimize(self, n, references):
    #self.UpdateCMA(max(1,n), references)  #TODO: flag to choose CMA/Grad
    self.ModifyRefs(max(1,n), references)
    for i in range(self.Options['grad_iter']):
      self.UpdateGrad(max(1,n), references)


  #Evaluate a reference xyar_n, an instance of TXYAR().
  def EvalRef(self, n, xyar_n, references):
    #i_next: which reference of the next stage we use.
    #i_next==i may not be good since if references[n+1][i_next] is bad, fobj at a bad references[n][i] becomes the best.
    #i_next= i
    i_next= 0
    if n==self.d.N:
      return self.Reward(n, xyar_n.y)[0]
    else:
      #Dx= self.Dx(n); Dy= self.Dy(n)
      #x_n= xya_n[:Dx]
      #y_n= xya_n[Dx:Dx+Dy]
      #a_n= xya_n[Dx+Dy:]
      r= self.Reward(n, xyar_n.y)[0]
      f= self.Options['reward_shape_f']
      x_next,var_x_next, y_next,var_y_next= self.Forward(n, x_n=xyar_n.x, a_n=xyar_n.a)
      return r - f * la.norm(MCVec(references[n+1][i_next].x) - x_next)**2 - f * la.norm(MCVec(references[n+1][i_next].y) - y_next)**2

  #Evaluate refs_n. refs_n: a list of TXYAR().
  def EvalRefs(self, n, refs_n, references):
    if len(refs_n)==0:  return
    for xyar_n in refs_n:
      xyar_n.r= self.EvalRef(n, xyar_n, references)

  #Objective function for a concatenated vector of (ref.x,ref.a).
  def FObjXA(self, n, xa_n, references):
    i_next= 0
    if n==self.d.N:
      return None
    else:
      Dx= self.Dx(n)
      x_n= xa_n[:Dx]
      a_n= xa_n[Dx:]
      x_next,var_x_next, y_next,var_y_next= self.Forward(n, x_n=x_n, a_n=a_n)
      return -la.norm(MCVec(references[n+1][i_next].x) - x_next)**2 - la.norm(MCVec(references[n+1][i_next].y) - y_next)**2


  #Modify and/or generate references.
  def ModifyRefs(self, n, references):
    if n<self.d.N:
      self.ModifyRefs(n+1, references)

    num_refs= self.Options['num_refs']
    num_bests= self.Options['num_rpl_best']
    num_random= self.Options['num_rpl_rand']

    bestidx= self.h.Database.SearchTopN(lambda eps: eps.R if eps.Len==self.d.N+1 else None, num_bests)
    if len(bestidx)<num_bests:  num_bests= len(bestidx)

    randxya= lambda: TXYAR(x=self.RandX(n), y=self.RandY(n), a=self.RandA(n))
    copyxya= lambda xya: TXYAR(x=copy.deepcopy(xya.x), y=copy.deepcopy(xya.y), a=copy.deepcopy(xya.a))

    if len(references[n])==0:
      references[n]= [randxya() for i in range(num_refs)]
    else:
      references[n][-num_random:]= [randxya() for i in range(num_random)]
    references[n][-num_random-num_bests:-num_random]= [copyxya(self.h.Database.GetEpisode(idx).Seq[n]) for idx in bestidx[:num_bests]]
    self.EvalRefs(n, references[n], references)
    references[n].sort(reverse=True,key=lambda x:x.r)

  '''Update reference states, outputs, and actions with gradient descent. '''
  def UpdateGrad(self, n, references):
    #Update succeeding references:
    if n<self.d.N:
      self.UpdateGrad(n+1, references)

    #Update references[n]:
    for i in range(len(references[n])):
      #We update references[n][i].y and references[n][i].{x,a} independently.

      bounds_y= self.BoundsY(n)
      y_n0= MCVec(references[n][i].y)
      R_n,var_R_n,dR_dy_n= self.Reward(n, y_n0, with_grad=True)
      grad= dR_dy_n
      NormalizeGrad(grad)  #TEST: normalization of gradient
      cnst= lambda y_n: ConstrainN(bounds_y, y_n)
      fobj= lambda y_n: self.Reward(n, y_n)[0]
      y_n= LineSearch(fobj,constrain=cnst,x0=y_n0,direc=grad,grad=grad,l0=0.05,rho=0.5,eps=0.2,f0=None)
      references[n][i].y= ToList(y_n)
      references[n][i].r= fobj(y_n)

      if n<self.d.N:
        bounds_xa= self.BoundsXA(n)
        #i_next: which reference of the next stage we use.
        #i_next==i may not be good since if references[n+1][i_next] is bad, fobj at a bad references[n][i] becomes the best.
        #i_next= i
        i_next= 0
        x_n0= MCVec(references[n][i].x)
        a_n0= MCVec(references[n][i].a)

        x_next,var_x_next, y_next,var_y_next, dFx_dxa_n, dFy_dxa_n= self.Forward(n, x_n=x_n0, a_n=a_n0, with_grad=True)
        dFx_dx_n= dFx_dxa_n[:Len(x_n0),:]
        dFx_da_n= dFx_dxa_n[Len(x_n0):,:]
        dFy_dx_n= dFy_dxa_n[:Len(x_n0),:]
        dFy_da_n= dFy_dxa_n[Len(x_n0):,:]

        err_x= MCVec(references[n+1][i_next].x) - x_next
        err_y= MCVec(references[n+1][i_next].y) - y_next
        grad_x= dFx_dx_n*err_x + dFy_dx_n*err_y
        grad_a= dFx_da_n*err_x + dFy_da_n*err_y
        grad= np.concatenate((grad_x, grad_a))
        NormalizeGrad(grad)  #TEST: normalization of gradient
        cnst= lambda xa_n: ConstrainN(bounds_xa, xa_n)
        fobj= lambda xa_n: self.FObjXA(n, xa_n, references)
        xa_n= LineSearch(fobj,constrain=cnst,x0=np.concatenate((x_n0,a_n0)),direc=grad,grad=grad,l0=0.05,rho=0.5,eps=0.2,f0=None)
        references[n][i].x= ToList(xa_n[:self.Dx(n)])
        references[n][i].a= ToList(xa_n[self.Dx(n):])
        references[n][i].r+= fobj(xa_n)
    references[n].sort(reverse=True,key=lambda x:x.r)

  '''Update reference states, outputs, and actions using CMA-ES. '''
  '''
  def UpdateCMA(self, n, references):
    f_scale0= 0.1
    bounds,Dx,Da= self.BoundsXA(n)
    if n==self.d.N:
      #Update references (terminal):
      s0= 0.5*min([bounds[1][d]-bounds[0][d] for d in range(len(bounds[0]))])
      if len(references[n])==0:
        references[n]= [self.TXAR()]
        x_n0= [0.5*(bounds[1][d]+bounds[0][d]) for d in range(len(bounds[0]))]
      else:
        x_n0= references[n][0].x
        s0*= f_scale0
      eval_ref= self.EvalRefFunc(n, Dx, criteria='R_xerr')
      fobj= lambda x_n: eval_ref(np.mat(x_n).T)
      x_n,value= OptCMA(x0=x_n0,s0=s0,f=fobj,bounds=bounds,fp=None,maxfevals=100)
      references[n][0].x= ToList(x_n)
      references[n][0].R= self.EvalRefFunc(n, Dx, criteria='R_xerr')(np.mat(x_n).T)
      return

    #Update succeeding references:
    self.UpdateCMA(n+1)

    #Update references[n]:
    s0= 0.5*min([bounds[1][d]-bounds[0][d] for d in range(len(bounds[0]))])
    if len(references[n])==0:
      references[n]= [self.TXAR()]
      xa_n0= [0.5*(bounds[1][d]+bounds[0][d]) for d in range(len(bounds[0]))]
    else:
      xa_n0= references[n][0].x + references[n][0].a
      s0*= f_scale0
    eval_ref= self.EvalRefFunc(n, Dx, criteria='R_xerr')
    fobj= lambda xa_n: eval_ref(np.mat(xa_n).T)
    xa_n,value= OptCMA(x0=xa_n0,s0=s0,f=fobj,bounds=bounds,fp=None,maxfevals=100)
    references[n][0].x= ToList(xa_n[:Dx])
    references[n][0].a= ToList(xa_n[Dx:])
    references[n][0].R= self.EvalRefFunc(n, Dx, criteria='R_xerr')(np.mat(xa_n).T)
  '''



'''Stochastic differential dynamic programming for a chain dynamical system.
See the system description of TChainDomainDP.

[Solver helper]
References= [refs_0,...,refs_N]
  List of reference vectors; refs_k should be a list of TXYAR or its subclass (refs_k[i].r is internally used).
  k-th references: refs_k= [xyarref_k_0,xyarref_k_1,...] (good-->bad order).
Database= database
  database is storing previously executed episodes:
    {episode,...}, episode is an instance of TChainEpisode or its subclass.
  database should provide following methods.
  These functions should be specialized for the current problem setup
  (note that the database may contain episodes of different dynamical systems).
  database.SearchIf(condition):
    returns indexes that episode matches condition(episode)={True,False},
    where episode should have an interface of TChainEpisode.
  database.GetEpisode(index):
    returns an episode specified by index.  index is one of SearchIf results.
LogFP= File pointer to a log file (can be None).
'''
#FIXME:TODO: Should be able to adjust a given action sequence (i.e. start from a given action sequence)
class TChainDDPSolver(TChainDPUtil):
  class THelper:
    def __init__(self):
      self.References= None  #List of N+1 references (list of (state,output,action))
      self.Database= None    #Reference to a database
      self.LogFP= None       #Pointer to a log file descriptor
    #Check the consistency.
    def Check(self,domain):
      if not len(self.References)==domain.N+1:  return False
      return True

  @staticmethod
  def DefaultOptions():
    #Options= {}
    Options= TChainDPUtil.DefaultOptions()
    Options['initguess_using_db']= True  #Use database in initial guess.
    Options['initguess_R_min']= 0.3  #In init guess, only consider samples whose R > this value.
    Options['initguess_num']= 10  #In init guess, how many samples do we generate.
    Options['initguess_criteria_db']= ('none','refs')  #In init guess, which criteria to evaluate a sample in DB. 'original': Original R obtained actually. Otherwise: evaluate actions with the criteria.
    Options['initguess_criteria_rand']= ('none','refs')  #In init guess, which criteria to evaluate a random guess. None: use self.Options['criteria']

    Options['criteria']= ('sum','none')  #Criteria to evaluate.
    '''
    First element:  reward computation.
      'none': 0.0 (using only the reward shaping term).
      'sum':  expected sum of reward.
      'ucb': expected UCB, i.e. expected sum of (reward + UCB_f*std-deviation).
      'lcb': expected LCB, i.e. expected sum of (reward - LCB_f*std-deviation).
    Second element: reward shaping.
      'none': no reward shaping.
      'refs': reward shaping with the best of h.References.
    '''
    Options['UCB_f']= 1.0  #Scale factor used in UCB.
    Options['LCB_f']= 1.0  #Scale factor used in LCB.
    Options['reward_shape_f']= 0.1  #Factor of reward shape, i.e. blend ratio of reward and state error.

    Options['plan_method']= 'grad'  #Planning method.
    '''
    'grad': gradient descent DP.
    'sCMA': single CMA over serialized actions.
    '''
    Options['plan_var_x']= None  #In Select at n, this value is used as variance of x_n, so that the robot behaves cautiously.  Set None to disable this.

    Options['plan_cautionary_ratio']= 1.0  #In forward propagation in plan, each covariance is multiplied by this value, which increases the cautiousness if this value>1.0.

    Options['grad_act_noise']= 0.2  #Search noise used in PlanGrad.
    Options['grad_criterias']= [('none','refs'),('none','refs'),None,None]  #Multi-stage optimization. In each stage, we can use different criteria. None uses default (==Options['criteria']).
    Options['grad_max_iter']= 40  #Max number of iterations of each gradient descent DP.
    Options['grad_max_stages']= 8  #If requested quality is not achieved with grad_criterias, grad_criterias[-1] is repeated until the total # of stages reaches grad_max_stages.
    Options['grad_term_quality_ratio']= 0.7  #Requested quality: evaluation with criteria is greater than this rate * rewards of references.
    #TODO:FIXME: 'grad_term_quality_ratio' might cause bad behaviors.
    Options['grad_refs']= 2  #Number of references used to compute gradient candidates.
    Options['grad_linesearch']= 'static'  #PlanGrad: Line search method.
    '''
      'static': Static line length.
      'backtrc': Backtracing.
    '''
    Options['grad_ls_alpha']= 0.03  #PlanGrad: Line length for grad_linesearch=='static'.

    Options['sCMA_initguess']= True  #If do init guess in plan sCMA.

    return Options
  #@staticmethod
  #def DefaultParams():
    #Params= {}
    #return Params


  def __init__(self):
    TChainDPUtil.__init__(self)

  #Initialize the planner.  We set a domain which should be an instance of TChainDomainDP.
  def Init(self,domain,helper):
    TChainDPUtil.Init(self,domain)
    assert(isinstance(helper,self.THelper))
    assert(helper.Check(domain))
    self.h= helper


  #Get action map {k:a_k} from an episode log xyar_seq, which should be a list of TXYAR or its subclass.
  def XYARSeqToAMap(self, xyar_seq):
    return {k:MCVec(xyar.a) if Len(xyar.a)>0 else None for k,xyar in enumerate(xyar_seq)}


  '''
  We modified Criteria function.
  Changed labels:
  cr0:
    'none'
    'sum': Expected sum, == previous 'Esum'
    - 'Esum' is removed
    'ucb': == previous 'EUCB'
    'lcb': == previous 'ELCB'
  cr1:
    'none'
    'refs': == previous 'ideal'
  Changed implementation:
    Replace
      sum(eval_n.R)+sum(eval_n.Rerr)
    by
      sum(eval_n.R)
  Reason:
    We are working with stochastic models, i.e. we always consider expectations.
    We assume models give expectation, so we can not obtain Rerr (R includes it).
    Use 'use_prob_in_pred' option if you want to disable the stochastic planning.
  '''

  #Get a single evaluation from TEvalRes according to the criteria.
  #TODO: Modify this to return a function (lambda) to reduce computation cost of each iteration.
  def Criteria(self, eval_n, cr=None):
    if cr==None:  cr= self.Options['criteria']
    cr0,cr1= cr
    if cr0=='none':  value= 0.0
    #Sum of reward criteria:
    elif cr0=='sum':  value= sum(eval_n.R)
    ##Expected sum of reward + std-deviation criteria (i.e. UCB):  #NOTE: we do not use this criteria since taking derivative becomes complicated
    #value= sum(eval_n.R)+math.sqrt(sum(eval_n.VarR))
    #Expected sum of (reward + std-deviation) criteria (i.e. UCB):
    elif cr0=='ucb':  value= sum(eval_n.R)+sum(map(math.sqrt,eval_n.VarR))*self.Options['UCB_f']
    #Expected sum of (reward - std-deviation) criteria (i.e. LCB or negative UCB):
    elif cr0=='lcb':  value= sum(eval_n.R)-sum(map(math.sqrt,eval_n.VarR))*self.Options['LCB_f']
    else:  raise Exception('Unknown criteria: %s' % repr(cr))
    #Reward shaping with References:
    if cr1=='none':  pass
    elif cr1=='refs':
      value+= -(sum(eval_n.XRefErr)+sum(eval_n.YRefErr))*(1.0 if cr0=='none' else self.Options['reward_shape_f'])
    else:  raise Exception('Unknown criteria: %s' % repr(cr))
    return value

  #Evaluation result class.
  class TEvalRes:
    def __init__(self, n):
      self.Start_n= n
      self.R= []     #Reward expectations: E[R_n+1], E[R_n+2], ...
      self.VarR= []  #Variances of R: var[R_n+1], var[R_n+2], ...
      self.XRefErr= []  #Norm between xref_n+1 and x_n+1, xref_n+2 and x_n+2, ...
      self.YRefErr= []  #Norm between yref_n+1 and y_n+1, yref_n+2 and y_n+2, ...
      self.XY= []  #TXYAR sequence (only x,y): xy[n], xy[n+1], ...

  #Evaluate an action map a_map = {k:a_k} for k=n,...,N-1 with a state distribution N(x_n, var_x_n).
  def EvalActMap(self, n, x_n, a_map, var_x_n=None, var_a_map={}, with_xy=False):
    x_k= x_n
    var_x_k= var_x_n
    eval_n= self.TEvalRes(n)
    if with_xy:  eval_n.XY.append(TXYAR(x=x_k))
    for k in range(n,self.d.N):
      a_k= a_map[k]
      var_a_k= var_a_map[k] if k in var_a_map else None
      '''Note: Following computation works even when dim(y_l)==0.
          This is possible thanks to NumPy's calculation.
          e.g.  la.norm(MCVec(None)-MCVec(None))==0.0 '''
      x_l, var_x_l, y_l, var_y_l= self.Forward(k, x_k, var_x_k, a_k, var_a_k)
      var_x_l= var_x_l*self.Options['plan_cautionary_ratio']
      var_y_l= var_y_l*self.Options['plan_cautionary_ratio']
      refs_l= self.h.References[k+1]
      xref_l_err= la.norm(MCVec(refs_l[0].x)-x_l)**2 if len(refs_l)>0 else 0.0
      yref_l_err= la.norm(MCVec(refs_l[0].y)-y_l)**2 if len(refs_l)>0 else 0.0
      R_l,var_R_l= self.Reward(k+1, y_l, var_y_l)
      eval_n.R.append(R_l)
      eval_n.VarR.append(var_R_l)
      eval_n.XRefErr.append(xref_l_err)
      eval_n.YRefErr.append(yref_l_err)
      if with_xy:  eval_n.XY.append(TXYAR(x=x_l,y=y_l))
      x_k= x_l
      var_x_k= var_x_l
    return eval_n

  def SerializeActMap(self, n, a_map):
    #size= sum([len(b[0]) for b in self.action_bounds[n:] if b!=None])
    size= sum([ad.D for ad in self.d.ActionDefs[n:]])
    a_list= [0.0]*size
    i= 0
    for k in range(n,self.d.N):
      if self.d.ActionDefs[k].D>0:
        ie= i+self.d.ActionDefs[k].D
        a_list[i:ie]= ToList(a_map[k])
        i= ie
    return a_list

  def DeserializeActList(self, n, a_list):
    a_map= {}
    i= 0
    for k in range(n,self.d.N):
      if self.d.ActionDefs[k].D==0:
        a_map[k]= None
      else:
        ie= i+self.d.ActionDefs[k].D
        a_map[k]= MCVec(a_list[i:ie])
        i= ie
    return a_map

  #Generate action map randomly.
  def RandActMap(self, n):
    if n>=self.d.N:
      raise Exception('RandActMap: invalid call for n:',n)
    return {k:MCVec(RandB(ad.Bounds)) if ad.D>0 else None for k,ad in enumerate(self.d.ActionDefs[n:],n)}

  #Generate Gaussian noise of action map.
  #sigma: diag([(var*(amax_d-amin_d))**2]) where amin_d,amax_d are action bound.
  def ActMapNoise(self, n, var):
    if n>=self.d.N:
      raise Exception('ActMapNoise: invalid call for n:',n)
    va_map= {}
    for k,ad in enumerate(self.d.ActionDefs[n:],n):
      if ad.D>0:
        sigma= np.diag([(var*(amax-amin))**2 for amin,amax in zip(ad.Min,ad.Max)])
        va_n= MCVec( np.random.multivariate_normal([0.0]*ad.D, sigma) )
        va_map[k]= va_n
      else:
        va_map[k]= None
    return va_map

  #Initial guess for gradient based search.
  def InitGuess(self, n, x_n, var_x_n=None):
    if n>=self.d.N:
      raise Exception('InitGuess: invalid call for n:',n)
    fa_data= []
    num= self.Options['initguess_num']
    if self.Options['initguess_using_db'] and self.h.Database is not None:
      #Initial guess from database (past log)
      R_min= self.Options['initguess_R_min']
      randidx= self.h.Database.SearchIf(lambda eps: eps.R is not None and eps.R>R_min)
      random.shuffle(randidx)
      for i in randidx[:num]:
        eps= self.h.Database.GetEpisode(i)
        a_map= self.XYARSeqToAMap(eps.Seq)
        if self.Options['initguess_criteria_db']=='original':
          value= eps.R
        else:
          value= self.Criteria(self.EvalActMap(n, x_n, a_map, var_x_n=var_x_n), cr=self.Options['initguess_criteria_db'])
        fa_data.append([value,a_map])
    for i in range(num-len(fa_data)):
      a_map= self.RandActMap(n)
      value= self.Criteria(self.EvalActMap(n, x_n, a_map, var_x_n=var_x_n), cr=self.Options['initguess_criteria_rand'])
      fa_data.append([value,a_map])
    fa_data.sort(reverse=True,key=lambda x:x[0])
    return fa_data[0][1]


  #Plan actions, return an action map {k:a_k} for k=n,...,N-1.
  def Plan(self, n, x_n, var_x_n=None):
    if var_x_n is None:  var_x_n= self.Options['plan_var_x']
    var_x_n, var_is_zero= RegularizeCov(var_x_n, self.Dx(n))

    if self.Options['plan_method']=='grad':
      a_map,eval_n= self.PlanGrad(n, x_n, var_x_n); a_n= a_map[n]
    elif self.Options['plan_method']=='sCMA':
      a_map,eval_n= self.PlanSCMA(n, x_n, var_x_n); a_n= a_map[n]

    return a_map,eval_n

  #CMA-ES planning (dynamic programming) where we consider all actions as a single action.
  def PlanSCMA(self, n, x_n, var_x_n=None):
    none_a_map= {k:None for k in range(n,self.d.N)}
    if n>=self.d.N:  return None, self.EvalActMap(n, x_n, none_a_map, var_x_n)
    bmin= self.SerializeActMap(n, {k:ad.Min for k,ad in enumerate(self.d.ActionDefs[n:],n)})
    bmax= self.SerializeActMap(n, {k:ad.Max for k,ad in enumerate(self.d.ActionDefs[n:],n)})
    bounds= [bmin,bmax]
    if len(bmin)==0:  return None, self.EvalActMap(n, x_n, none_a_map, var_x_n)
    if self.Options['sCMA_initguess']:
      a_list0= self.SerializeActMap(n, self.InitGuess(n, x_n) )
    else:
      a_list0= [0.5*(b1+b2) for b1,b2 in zip(bmin,bmax)]
    s0= 0.5*min([bmax[d]-bmin[d] for d in range(len(bmin))])
    feval= lambda a_list: self.EvalActMap(n, x_n, self.DeserializeActList(n,a_list), var_x_n)
    criteria= self.Criteria
    a_list,eval_n,value= OptCMA2(x0=a_list0,s0=s0,f_assess=feval,eval_criteria=criteria,bounds=bounds,fp=self.h.LogFP,maxfevals=300)
    return self.DeserializeActList(n,a_list),eval_n


  #Plan action with differential dynamic programming.
  def PlanGrad(self, n, x_n, var_x_n=None, tol=1.0e-4):
    #TODO: diff_value seems to be inefficient.
    diff_value= lambda a_map1,a_map2,cr: self.Criteria(self.EvalActMap(n, x_n, a_map1, var_x_n), cr=cr) - self.Criteria(self.EvalActMap(n, x_n, a_map2, var_x_n), cr=cr)
    a_map= self.InitGuess(n, x_n, var_x_n)
    a_map_best= None
    fp= self.h.LogFP
    Nj= len(self.Options['grad_criterias'])
    Ni= self.Options['grad_max_iter']
    R_refs= sum(self.Reward(k,self.h.References[k][0].y)[0] for k in range(n+1,self.d.N+1))  #TODO: seems unsophisticated
    for j in range(self.Options['grad_max_stages']):
      criteria= self.Options['grad_criterias'][min(j,Nj-1)]
      for i in range(Ni):
        a_map_new= {}
        grad_data= []  #For log
        self.StepGradOpt(n, x_n, var_x_k=var_x_n, a_map=a_map, a_map_new=a_map_new, grad_data=grad_data, criteria=criteria)
        if fp:  #Write log on file
          eval_n= self.EvalActMap(n, x_n, a_map, var_x_n=var_x_n)
          value= sum(eval_n.R)
          cvalue= self.Criteria(eval_n, cr=criteria)
          fp.write('%s # %f %f # %s\n'%(ToStr(ToList(a_map[n])), value, cvalue, ' '.join([ToStr(ToList(g)) for g in grad_data]) ))
        dvalue= diff_value(a_map_new, a_map, cr=criteria)
        if dvalue > 0.0: a_map= a_map_new
        #print dvalue
        if fp and (i==Ni-1 or dvalue < tol):
          eval_n= self.EvalActMap(n, x_n, a_map, var_x_n=var_x_n)
          value= sum(eval_n.R)
          cvalue= self.Criteria(eval_n, cr=criteria)
          fp.write('%s # %f %f # %s\n'%(ToStr(ToList(a_map[n])), value, cvalue, ' '.join([ToStr([0.0]*Len(g)) for g in grad_data]) ))
        if dvalue < tol:  break
      if a_map_best==None or diff_value(a_map, a_map_best, cr=criteria)>0.0:
        a_map_best= a_map
      if j>=Nj-1 and j<self.Options['grad_max_stages']-1:
        if self.Criteria(self.EvalActMap(n, x_n, a_map_best, var_x_n=var_x_n)) > self.Options['grad_term_quality_ratio']*R_refs:
          break
      if j<self.Options['grad_max_stages']-1:
        if criteria==self.Options['grad_criterias'][min(j+1,Nj-1)]:
          #Random jump for next optimization stage.
          a_map_noise= self.ActMapNoise(n,var=self.Options['grad_act_noise'])
          a_map= {}
          for k in range(n,self.d.N):
            if a_map_best[k]!=None:
              a_map[k]= ConstrainN(self.d.ActionDefs[k].Bounds, a_map_best[k] + a_map_noise[k])
            else:
              a_map[k]= None
        else:
          a_map= a_map_best
      if fp: fp.write('\n')
    a_map= a_map_best
    if fp:
      eval_n= self.EvalActMap(n, x_n, a_map, var_x_n=var_x_n)
      value= sum(eval_n.R)
      cvalue= self.Criteria(eval_n, cr=criteria)
      fp.write('%s # %f %f # %s\n'%(ToStr(ToList(a_map[n])), value, cvalue, ' '.join([ToStr([0.0]*Len(g)) for g in grad_data]) ))
    #return a_map[n], self.EvalActMap(n, x_n, a_map, var_x_n=var_x_n)
    return a_map, self.EvalActMap(n, x_n, a_map, var_x_n=var_x_n)

  #One step update of actions a_map using a gradient chain.
  #Updated actions are stored in a_map_new.
  def StepGradOpt(self, k, x_k, var_x_k, a_map, a_map_new, criteria, with_dJ=False, grad_data=None):
    num_refs= self.Options['grad_refs']
    if k>=self.d.N:
      raise Exception('StepGradOpt: invalid call for k:',k)

    #Compute one-step future (k-th --> l-th) using forward models:
    x_l, var_x_l, y_l, var_y_l, dFx_dxa_k, dFy_dxa_k= self.Forward(k, x_k, var_x_k, a_map[k], var_a_n=None, with_grad=True)
    var_x_l= var_x_l*self.Options['plan_cautionary_ratio']
    var_y_l= var_y_l*self.Options['plan_cautionary_ratio']
    dFx_dx_k= dFx_dxa_k[:Len(x_k),:]
    dFx_da_k= dFx_dxa_k[Len(x_k):,:]
    dFy_dx_k= dFy_dxa_k[:Len(x_k),:]
    dFy_da_k= dFy_dxa_k[Len(x_k):,:]
    R_l,var_R_l,dR_dy_l= self.Reward(k+1, y_l, var_y_l, with_grad=True)

    #Compute gradients:
    '''Note: Following computation works even when dim(y_l)==0, i.e. no reward is defined.
        This is possible thanks to NumPy's calculation.
        e.g.  np.zeros((3,0))*MCVec(np.zeros((0,1))) == np.mat([0,0,0]).T '''
    grads= [None]*(1+num_refs)
    dJ_dx_k= None  #For future use
    if k==self.d.N-1:
      grads[0]= dFy_da_k * dR_dy_l
      if with_dJ:
        dJ_dx_k= dFy_dx_k * dR_dy_l
    else:
      dJ_dx_l= self.StepGradOpt(k+1, x_l, var_x_l, a_map, a_map_new, with_dJ=True, criteria=criteria)
      grads[0]= dFy_da_k * dR_dy_l + dFx_da_k * dJ_dx_l
      if with_dJ:
        dJ_dx_k= dFy_dx_k * dR_dy_l + dFx_dx_k * dJ_dx_l
    #Gradients with references:
    for i in range(num_refs):
      ref_l= self.h.References[k+1][i]
      grads[1+i]= dFy_da_k * (MCVec(ref_l.y) - y_l) + dFx_da_k * (MCVec(ref_l.x) - x_l)

    #Update action[k]
    if a_map[k]==None:
      a_map_new[k]= None
    else:
      va_data= []
      for grad in grads:
        f= lambda a: ( a_map_new.__setitem__(k,a) , self.Criteria(self.EvalActMap(k, x_k, a_map_new, var_x_n=var_x_k), cr=criteria) )[1]

        if self.Options['grad_linesearch']=='static':
          NormalizeGrad(grad)  #TEST: normalization of gradient
          a_new= a_map[k] + self.Options['grad_ls_alpha'] * grad
          a_new= ConstrainN(self.d.ActionDefs[k].Bounds, a_new)
        elif self.Options['grad_linesearch']=='backtrc':
          NormalizeGrad(grad)  #TEST: normalization of gradient
          cnst= lambda a: ConstrainN(self.d.ActionDefs[k].Bounds, a)
          a_new= LineSearch(f,constrain=cnst,x0=a_map[k],direc=grad,grad=grad,l0=0.2,rho=0.5,eps=0.2,f0=None)
          a_new= ConstrainN(self.d.ActionDefs[k].Bounds, a_new)
        else:
          raise Exception('Unknown grad_linesearch:',self.Options['grad_linesearch'])

        va_data.append([f(a_new), a_new])
        if grad_data!=None:  grad_data.append(grad)
      #print 'va_data',va_data
      va_data.sort(reverse=True,key=lambda x:x[0])
      a_map_new[k]= va_data[0][1]

    return dJ_dx_k


#Storing an I/O of a single dynamical system (For TChainDDPSolver2).
class TChainDynIO(object):
  def __init__(self):
    #Input and output:
    self.x0= None  #Current state
    self.a0= None  #Action
    self.x1= None  #Next state
    self.y1= None  #Output
    self.r1= None  #Reward
    #Covariances:
    self.cov_x0= None
    self.cov_a0= None
    self.cov_x1= None
    self.cov_y1= None
    self.var_r1= None
    #Gradients:
    self.dFx_dx0= None
    self.dFx_da0= None
    self.dFy_dx0= None
    self.dFy_da0= None
    self.dR_dy1= None
    self.dJ_dx0= None
    self.dJ_da0= None  #NOTE: This will be a list when we use multiple gradients.
    #Other stuff:
    self.data= None  #Any data can be stored (e.g. internal status of optimizer).

#Storing a sequence of TChainDynIO (For TChainDDPSolver2).
class TChainTraj:
  def __init__(self, n_start=None, N=None, x_n=None, var_x_n=None, a_map=None):
    if n_start is not None and N is not None:
      self.Seq= {k:TChainDynIO() for k in range(n_start,N)}
      self.Start= n_start
      if x_n is not None:  self.Seq[n_start].x0= x_n
      if var_x_n is not None:  self.Seq[n_start].cov_x0= var_x_n
      if a_map is not None:
        for k,a in a_map.iteritems():
          if k in self.Seq:  self.Seq[k].a0= a
          else:  print 'Warning in TChainTraj.__init__: ignoring action in a_map:',k,a
    else:
      self.Seq= {}
      self.Start= None
    self.FlagFwd= 0  #Forward is 0:Not computed, 1:Computed without gradients, 2:Computed with gradients.
    self.FlagBwd= 0  #Backward is 0:Not computed, 1:Computed.

  #Return TChainTraj with the same structure and the same initial state.
  def Blank(self):
    return TChainTraj(self.Start, max(self.Seq.keys())+1, x_n=self.Seq[self.Start].x0, var_x_n=self.Seq[self.Start].cov_x0)


'''Stochastic differential dynamic programming for a chain dynamical system.
See the system description of TChainDomainDP.
In this version 2, we implement more systematic gradient descent solver.

[Solver helper]
References= [refs_0,...,refs_N]
  List of reference vectors; refs_k should be a list of TXYAR or its subclass (refs_k[i].r is internally used).
  k-th references: refs_k= [xyarref_k_0,xyarref_k_1,...] (good-->bad order).
Database= database
  database is storing previously executed episodes:
    {episode,...}, episode is an instance of TChainEpisode or its subclass.
  database should provide following methods.
  These functions should be specialized for the current problem setup
  (note that the database may contain episodes of different dynamical systems).
  database.SearchIf(condition):
    returns indexes that episode matches condition(episode)={True,False},
    where episode should have an interface of TChainEpisode.
  database.GetEpisode(index):
    returns an episode specified by index.  index is one of SearchIf results.
LogFP= File pointer to a log file (can be None).
'''
#FIXME:TODO: Should be able to adjust a given action sequence (i.e. start from a given action sequence)
class TChainDDPSolver2(TChainDPUtil):
  class THelper:
    def __init__(self):
      self.References= None  #List of N+1 references (list of (state,output,action))
      self.Database= None    #Reference to a database
      self.LogFP= None       #Pointer to a log file descriptor
    #Check the consistency.
    def Check(self,domain):
      if not (self.References is None or len(self.References)==domain.N+1):  return False
      return True

  @staticmethod
  def DefaultOptions():
    #Options= {}
    Options= TChainDPUtil.DefaultOptions()
    Options['initguess_using_db']= True  #Use database in initial guess.
    Options['initguess_R_min']= 0.3  #In init guess, only consider samples whose R > this value.
    Options['initguess_num']= 10  #In init guess, how many samples do we generate.
    Options['initguess_criteria_db']= ('none','refs')  #In init guess, which criteria to evaluate a sample in DB. 'original': Original R obtained actually. Otherwise: evaluate actions with the criteria.
    Options['initguess_criteria_rand']= ('none','refs')  #In init guess, which criteria to evaluate a random guess. None: use self.Options['criteria']

    Options['criteria']= ('sum','none')  #Criteria to evaluate.
    '''
    First element:  reward computation.
      'none': 0.0 (using only the reward shaping term).
      'sum':  expected sum of reward.
      'ucb': expected UCB, i.e. expected sum of (reward + UCB_f*std-deviation).
      'lcb': expected LCB, i.e. expected sum of (reward - LCB_f*std-deviation).
    Second element: reward shaping.
      'none': no reward shaping.
      'refs': reward shaping with the best of h.References.
    '''
    Options['UCB_f']= 1.0  #Scale factor used in UCB.
    Options['LCB_f']= 1.0  #Scale factor used in LCB.
    Options['reward_shape_f']= 0.1  #Factor of reward shape, i.e. blend ratio of reward and state error.

    Options['plan_var_x']= None  #In Select at n, this value is used as variance of x_n, so that the robot behaves cautiously.  Set None to disable this.

    Options['plan_cautionary_ratio']= 1.0  #In forward propagation in plan, each covariance is multiplied by this value, which increases the cautiousness if this value>1.0.

    Options['grad_act_noise']= 0.2  #Search noise used in PlanGrad.
    Options['grad_criterias']= [('none','refs'),('none','refs'),None,None]  #Multi-stage optimization. In each stage, we can use different criteria. None uses default (==Options['criteria']).
    Options['grad_max_iter']= 40  #Max number of iterations of each gradient descent DP.
    Options['grad_min_iter']= 3   #Min number of iterations of each gradient descent DP.
    Options['grad_max_stages']= 8  #If requested quality is not achieved with grad_criterias, grad_criterias[-1] is repeated until the total # of stages reaches grad_max_stages.
    Options['grad_term_quality_ratio']= 0.7  #Requested quality: evaluation with criteria is greater than this rate * rewards of references.
    #TODO:FIXME: 'grad_term_quality_ratio' might cause bad behaviors.
    Options['grad_refs']= 2  #Number of references used to compute gradient candidates.

    Options['optimizer']= 'gd'  #Gradient descent algorithm.
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
    TChainDPUtil.__init__(self)

  #Initialize the planner.  We set a domain which should be an instance of TChainDomainDP.
  def Init(self,domain,helper):
    TChainDPUtil.Init(self,domain)
    assert(isinstance(helper,self.THelper))
    assert(helper.Check(domain))
    self.h= helper


  '''Do forward computation of traj.
    We start from traj.Start stage (traj.Seq[traj.Start].x0 should be given) and
    propagate the states with given actions (traj.Seq[:].a0 should be given). '''
  def ForwardN(self, traj, with_grad=False):
    for k in range(traj.Start, self.d.N):
      io0= traj.Seq[k]
      if with_grad:
        io0.x1, io0.cov_x1, io0.y1, io0.cov_y1, dFx_dxa_k, dFy_dxa_k= self.Forward(k, io0.x0, io0.cov_x0, io0.a0, io0.cov_a0, with_grad=True)
        Dx0= self.Dx(k)
        io0.dFx_dx0= dFx_dxa_k[:Dx0,:]
        io0.dFx_da0= dFx_dxa_k[Dx0:,:]
        io0.dFy_dx0= dFy_dxa_k[:Dx0,:]
        io0.dFy_da0= dFy_dxa_k[Dx0:,:]
      else:
        io0.x1, io0.cov_x1, io0.y1, io0.cov_y1= self.Forward(k, io0.x0, io0.cov_x0, io0.a0, io0.cov_a0, with_grad=False)
      io0.cov_x1= io0.cov_x1*self.Options['plan_cautionary_ratio']
      io0.cov_y1= io0.cov_y1*self.Options['plan_cautionary_ratio']
      if with_grad:
        io0.r1,io0.var_r1,io0.dR_dy1= self.Reward(k+1, io0.y1, io0.cov_y1, with_grad=True)
      else:
        io0.r1,io0.var_r1= self.Reward(k+1, io0.y1, io0.cov_y1, with_grad=False)

      if k<self.d.N-1:
        io1= traj.Seq[k+1]
        io1.x0= io0.x1
        io1.cov_x0= io0.cov_x1
    traj.FlagFwd= 2 if with_grad else 1
    traj.FlagBwd= 0
    return traj  #Return traj for convenience (input traj is modified).

  '''Do backward computation of traj.
    We compute gradients of J w.r.t. action.'''
  def BackwardN(self, traj):
    '''Note: Following computation works even when dim(y1)==0, i.e. no reward is defined.
        This is possible thanks to NumPy's calculation.
        e.g.  np.zeros((3,0))*MCVec(np.zeros((0,1))) == np.mat([0,0,0]).T '''
    if traj.FlagFwd in (0,1):
      self.ForwardN(traj, with_grad=True)
    num_refs= self.Options['grad_refs'] if self.h.References is not None else 0
    for k in range(self.d.N-1, traj.Start-1, -1):
      io0= traj.Seq[k]
      io0.dJ_da0= [None]*(1+num_refs)
      io0.dJ_dx0= None
      if k==self.d.N-1:
        io0.dJ_da0[0]= io0.dFy_da0 * io0.dR_dy1
        io0.dJ_dx0   = io0.dFy_dx0 * io0.dR_dy1
      else:
        dJ_dx1= traj.Seq[k+1].dJ_dx0
        io0.dJ_da0[0]= io0.dFy_da0 * io0.dR_dy1 + io0.dFx_da0 * dJ_dx1
        io0.dJ_dx0   = io0.dFy_dx0 * io0.dR_dy1 + io0.dFx_dx0 * dJ_dx1
      #Gradients with references:
      for i in range(num_refs):
        ref1= self.h.References[k+1][i]
        io0.dJ_da0[1+i]= io0.dFy_da0 * (MCVec(ref1.y) - io0.y1) + io0.dFx_da0 * (MCVec(ref1.x) - io0.x1)
    traj.FlagBwd= 1

  #Get an evaluation from TChainTraj according to the criteria.
  #TODO: Modify this to return a function (lambda) to reduce computation cost of each iteration.
  def Criteria(self, traj, cr=None):
    if traj.FlagFwd==0:  self.ForwardN(traj, with_grad=False)
    if cr==None:  cr= self.Options['criteria']
    cr0,cr1= cr
    if cr0=='none':  value= 0.0
    #Sum of reward criteria:
    elif cr0=='sum':  value= sum([io0.r1 for k,io0 in traj.Seq.iteritems()])
    #Expected sum of (reward + std-deviation) criteria (i.e. UCB):
    elif cr0=='ucb':
      f= self.Options['UCB_f']
      value= sum([io0.r1 + math.sqrt(io0.var_r1)*f for k,io0 in traj.Seq.iteritems()])
    #Expected sum of (reward - std-deviation) criteria (i.e. LCB or negative UCB):
    elif cr0=='lcb':
      f= self.Options['LCB_f']
      value= sum([io0.r1 - math.sqrt(io0.var_r1)*f for k,io0 in traj.Seq.iteritems()])
    else:  raise Exception('Unknown criteria: %s' % repr(cr))
    #Reward shaping with References:
    if cr1=='none':  pass
    elif cr1=='refs':
      refs= self.h.References
      if refs is None:  raise Exception('helper.References is not given but the criteria is used: %s' % repr(cr))
      f= 1.0 if cr0=='none' else self.Options['reward_shape_f']
      value+= -f*sum([
        (la.norm(MCVec(refs[k+1][0].x)-io0.x1)**2 + la.norm(MCVec(refs[k+1][0].y)-io0.y1)**2) if len(refs[k+1])>0 else 0.0
        for k,io0 in traj.Seq.iteritems()])
    else:  raise Exception('Unknown criteria: %s' % repr(cr))
    return value

  #Generate TChainTraj (only actions are filled) from an episode log xyar_seq, which should be a list of TXYAR or its subclass.
  def XYARSeqToActTraj(self, n, xyar_seq):
    traj= TChainTraj(n_start=n, N=self.d.N, x_n=xyar_seq[n].x)
    for k,xyar in enumerate(xyar_seq[n:-1],n):  traj.Seq[k].a0= MCVec(xyar.a) if Len(xyar.a)>0 else None
    return traj

  #Generate randomly the actions of TChainTraj.
  def RandActTraj(self, n):
    traj= TChainTraj(n_start=n, N=self.d.N)
    for k,ad in enumerate(self.d.ActionDefs[n:-1],n):  traj.Seq[k].a0= MCVec(RandB(ad.Bounds)) if ad.D>0 else None
    return traj

  #Generate Gaussian noise of action map.
  #sigma: diag([(var*(amax_d-amin_d))**2]) where amin_d,amax_d are action bound.
  def ActMapNoise(self, n, var):
    if n>=self.d.N:
      raise Exception('ActMapNoise: invalid call for n:',n)
    va_map= {}
    for k,ad in enumerate(self.d.ActionDefs[n:],n):
      if ad.D>0:
        sigma= np.diag([(var*(amax-amin))**2 for amin,amax in zip(ad.Min,ad.Max)])
        va_n= MCVec( np.random.multivariate_normal([0.0]*ad.D, sigma) )
        va_map[k]= va_n
      else:
        va_map[k]= None
    return va_map

  #Initial guess for gradient based search.
  def InitGuess(self, n, x_n, var_x_n=None):
    if n>=self.d.N:
      raise Exception('InitGuess: invalid call for n:',n)
    fa_data= []
    num= self.Options['initguess_num']
    if self.Options['initguess_using_db'] and self.h.Database is not None:
      #Initial guess from database (past log)
      R_min= self.Options['initguess_R_min']
      randidx= self.h.Database.SearchIf(lambda eps: eps.R is not None and eps.R>R_min)
      random.shuffle(randidx)
      for i in randidx[:num]:
        eps= self.h.Database.GetEpisode(i)
        traj= self.XYARSeqToActTraj(n, eps.Seq)
        traj.Seq[n].x0= x_n
        traj.Seq[n].cov_x0= var_x_n
        if self.Options['initguess_criteria_db']=='original':
          value= eps.R
        else:
          value= self.Criteria(traj, cr=self.Options['initguess_criteria_db'])
        fa_data.append([value,traj])
    for i in range(num-len(fa_data)):
      traj= self.RandActTraj(n)
      traj.Seq[n].x0= x_n
      traj.Seq[n].cov_x0= var_x_n
      value= self.Criteria(traj, cr=self.Options['initguess_criteria_rand'])
      fa_data.append([value,traj])
    fa_data.sort(reverse=True,key=lambda x:x[0])
    return fa_data[0][1]


  #Plan actions, return TChainTraj for k=n,...,N-1.
  def Plan(self, n, x_n, var_x_n=None):
    if var_x_n is None:  var_x_n= self.Options['plan_var_x']
    var_x_n, var_is_zero= RegularizeCov(var_x_n, self.Dx(n))

    traj= self.PlanGrad(n, x_n, var_x_n)

    return traj

  #Plan actions with differential dynamic programming.
  def PlanGrad(self, n, x_n, var_x_n=None, tol=1.0e-4):
    diff_value= lambda traj1,traj2,cr: self.Criteria(traj1, cr=cr) - self.Criteria(traj2, cr=cr)
    traj= self.InitGuess(n, x_n, var_x_n)
    traj_best= None
    fp= self.h.LogFP
    if self.h.LogFP is not None:
      grads_or= lambda grads: grads if grads is not None else []
      log= lambda traj: self.h.LogFP.write('%s # %f %f # %s\n'%(
            ToStr(ToList(traj.Seq[n].a0)),
            self.Criteria(traj, cr=('sum','none')),
            self.Criteria(traj, cr=criteria),
            ' '.join([ToStr(ToList(g)) for g in grads_or(traj.Seq[n].dJ_da0)]) ))
      log_br= lambda: self.h.LogFP.write('\n')
    else:
      log= lambda traj: None
      log_br= lambda: None
    Nj= len(self.Options['grad_criterias'])
    Ni= self.Options['grad_max_iter']
    Ni_min= self.Options['grad_min_iter']
    if self.h.References is not None:
      R_refs= sum(self.Reward(k,self.h.References[k][0].y)[0] for k in range(n+1,self.d.N+1))  #TODO: seems unsophisticated
    else:
      R_refs= None
    for j in range(self.Options['grad_max_stages']):
      criteria= self.Options['grad_criterias'][min(j,Nj-1)]
      opt= self.InitOpt(traj)
      for i in range(Ni):
        log(traj)
        traj_new= self.StepOpt(opt, traj, criteria)
        dvalue= diff_value(traj_new, traj, cr=criteria)
        if dvalue > 0.0: traj= traj_new
        #print dvalue
        if i >= Ni_min-1 and dvalue < tol:  break
      log(traj)
      if traj_best==None or diff_value(traj, traj_best, cr=criteria)>0.0:
        traj_best= traj
      if j>=Nj-1 and j<self.Options['grad_max_stages']-1 and R_refs is not None:
        if self.Criteria(traj_best) > self.Options['grad_term_quality_ratio']*R_refs:
          break
      if j<self.Options['grad_max_stages']-1:
        if criteria==self.Options['grad_criterias'][min(j+1,Nj-1)]:
          #Random jump for next optimization stage.
          a_map_noise= self.ActMapNoise(n,var=self.Options['grad_act_noise'])
          traj= traj_best.Blank()
          for k in range(n,self.d.N):
            if self.Da(k)>0:
              traj.Seq[k].a0= ConstrainN(self.d.ActionDefs[k].Bounds, traj_best.Seq[k].a0 + a_map_noise[k])
        else:
          traj= traj_best
      log_br()
    traj= traj_best
    log(traj)
    return traj

  #Initialize an optimizer for actions in TChainTraj traj.
  #Internal state of optimizer is also initialized and stored in traj.
  def InitOpt(self, traj):
    if self.Options['optimizer']=='gd':
      opt= TGradientAscent(alpha=self.Options['gd_alpha'], normalize_grad=self.Options['gd_nz_grad'])
    elif self.Options['optimizer']=='adadelta':
      opt= TAdaDeltaMax(rho=self.Options['ad_rho'], eps=self.Options['ad_eps'], normalize_grad=self.Options['ad_nz_grad'])
    for k in range(traj.Start, self.d.N):
      if traj.Seq[k].a0 is None:
        pass
        #traj.Seq[k].data= None
      else:
        traj.Seq[k].data= opt.Init(traj.Seq[k].a0)
    return opt

  #One step update of actions in TChainTraj using a gradient descent.
  #Updated actions are stored in returned TChainTraj.
  def StepOpt(self, opt, traj, criteria):
    if traj.FlagBwd==0: self.BackwardN(traj)
    va_data= []
    num_grads= 1+self.Options['grad_refs']
    for gidx in range(num_grads):
      traj_test= traj.Blank()
      for k in range(traj.Start, self.d.N):
        if traj.Seq[k].a0 is None:
          traj_test.Seq[k].a0= None
        else:
          grad= traj.Seq[k].dJ_da0[gidx]
          traj_test.Seq[k].a0,traj_test.Seq[k].data= opt.Step(traj.Seq[k].a0,grad,traj.Seq[k].data)
          traj_test.Seq[k].a0= ConstrainN(self.d.ActionDefs[k].Bounds, traj_test.Seq[k].a0)
      va_data.append([self.Criteria(self.ForwardN(traj_test,with_grad=True), cr=criteria), traj_test])
    va_data.sort(reverse=True,key=lambda x:x[0])
    return va_data[0][1]



'''New version of TDynPlanLearn2 where some solvers/optimizers are separated.'''
'''Dynamic programming and model learning for chain dynamical systems.
A new version of TDynPlanLearn2 where some solvers are separated from the class.
See the system description of TChainDomainRL.
'''
class TChainDynPlanLearn(TChainRLUtil):
  @staticmethod
  def DefaultOptions():
    #Options= TChainRLUtil.DefaultOptions()
    Options= {}
    Options['ddp_ver']= 1
    Options['ddp_sol']= TChainDDPSolver.DefaultOptions()  #Options of DDP solver (ver.1).
    Options['ddp_sol2']= TChainDDPSolver2.DefaultOptions()  #Options of DDP solver (ver.2).
    Options['ref_opt']= TChainRefOpt.DefaultOptions()  #Options of reference optimizer.
    Options['using_refs']= True  #If we use references.

    Options['x_expand']= 1.1  #References are searched over state space bounds expanded with this ratio.
    Options['y_expand']= 1.1  #References are searched over output space bounds expanded with this ratio.

    Options['model_type']= 'dnn'  #Dynamics model type.
    '''
      'lwr': Locally weighted regression (LWR).
      'dnn': Deep neural net (DNN).
    '''

    #Parameters for LWR:
    Options['lwr_options']= {'kernel':'l2g', 'c_min':0.01, 'f_reg':0.0001}  #Options for LWR.

    #Parameters for DNN:
    Options['dnn_hidden_units']= [200,200,200]  #Number of hidden units.
    Options['dnn_options']= {}  #Options for DNN. 'n_units' option is ignored. e.g. Options['dnn_options']['dropout_ratio']= 0.01

    Options['num_min_predictable']= 3  #Minimum number of random samples necessary for dynamics prediction.

    Options['explore_noise']= 'none'  #Type of exploration noise.
    '''
      'none': No noise.
      'gauss': Gaussian noise with explore_noise_gain*Var[sum(R)].
    '''
    Options['explore_noise_gain']= 0.005  #Exploration noise parameter for 'gauss'.
    Options['explore_var_max']= 1.0  #Max variance of exploration noise of 'gauss'.

    Options['base_dir']= '/tmp/dpl/'  #Base directory.  Last '/' matters.
    '''Format of log file name for each optimization.
        i: i-th run, n: n-th stage,
        base: Options['base_dir'].'''
    Options['opt_log_name']= '{base}seq/opt-{i:04d}-{n:03d}.dat'
    return Options

  #@staticmethod
  #def DefaultParams():
    #Params= {}
    #return Params

  @property
  def DB(self):
    return self.database

  #Generate an instance; domain should be an instance of TChainDomainRL.
  def __init__(self, domain):
    TChainRLUtil.__init__(self)
    TChainRLUtil.Init(self,domain)
    self.models_x= None
    self.models_y= None
    self.state_bounds= [TBoundingBox(self.Dx(n)) for n in range(self.d.N+1)]
    self.out_bounds= [TBoundingBox(self.Dy(n)) for n in range(self.d.N+1)]
    self.references= [[] for n in range(self.d.N+1)]  #Reference vectors.

    self.database= TChainEpisodeDB()
    #TODO:consider to log 'plan_time'

  #Save into data (dict):  {'options':{options}, 'params':{parameters}}
  #base_dir: used to store data into external data file(s); None for a default value.
  #  Note: returned dict may contain file path(s) containing data.
  #        Such path(s) my contain {base} which is actually base_dir.
  #        Those {base} will be replaced by base_dir when using Load().
  #        This is useful to move the data files and load them.
  def Save(self, base_dir=None):
    if base_dir==None:  base_dir= self.Options['base_dir']
    L= lambda f: f.format(base=base_dir)
    ST= ToStdType
    data= {}
    data['options']= self.Options
    data['params']= {}
    data['params']['state_bounds']= ST(self.state_bounds)
    data['params']['out_bounds']= ST(self.out_bounds)
    data['params']['references']= ST(self.references)
    data['params']['database']= '{base}database.yaml'
    SaveYAML(self.database.Save(), L(data['params']['database']))

    if self.Options['model_type']=='lwr':    file_suffix= 'lwr_model.yaml'
    elif self.Options['model_type']=='dnn':  file_suffix= 'nn_model.yaml'
    if self.Options['model_type'] in ('lwr', 'dnn'):
      data['params']['models_x']= [{} for n in range(self.d.N)]
      data['params']['models_y']= [{} for n in range(self.d.N)]
      for n in range(self.d.N):
        prefix= '{base}mx_{n}_'.format(base='{base}', n=n)
        data['params']['models_x'][n]['prefix']= prefix
        data['params']['models_x'][n]['model']= prefix+file_suffix
        SaveYAML(self.models_x[n].Save(L(data['params']['models_x'][n]['prefix'])), L(data['params']['models_x'][n]['model']))
        prefix= '{base}my_{n}_'.format(base='{base}', n=n)
        data['params']['models_y'][n]['prefix']= prefix
        data['params']['models_y'][n]['model']= prefix+file_suffix
        SaveYAML(self.models_y[n].Save(L(data['params']['models_y'][n]['prefix'])), L(data['params']['models_y'][n]['model']))
    return data

  #Load from data (dict):  {'options':{options}, 'params':{parameters}}
  #base_dir: where external data file(s) are stored; None for a default value.
  #  Note: data may contain a filename like '{base}/func.dat'
  #        where {base} is supposed be replaced by base_dir.
  #        Use self.Locate to get the actual path (e.g. self.Locate('{base}/func.dat')).
  def Load(self, data, base_dir=None):
    TChainRLUtil.Load(self, data)
    if 'params' not in data:  return
    if base_dir!=None:  self.load_base_dir= base_dir
    L= self.Locate

    for n,bb in enumerate(data['params']['state_bounds']):
      self.state_bounds[n]= TBoundingBox(bb['D'])
      self.state_bounds[n].Min= bb['Min']
      self.state_bounds[n].Max= bb['Max']
    for n,bb in enumerate(data['params']['out_bounds']):
      self.out_bounds[n]= TBoundingBox(bb['D'])
      self.out_bounds[n].Min= bb['Min']
      self.out_bounds[n].Max= bb['Max']
    for n,refs in enumerate(data['params']['references']):
      self.references[n]= [None]*len(refs)
      for i,xyar in enumerate(refs):
        self.references[n][i]= TXYAR(x=xyar['x'],y=xyar['y'],a=xyar['a'],r=xyar['r'])
    self.database.Load(LoadYAML(L(data['params']['database'])))

    self.CreateModels()

    if self.Options['model_type'] in ('lwr', 'dnn'):
      for n in range(self.d.N):
        self.models_x[n].Load(LoadYAML(L(data['params']['models_x'][n]['model'])), L(data['params']['models_x'][n]['prefix']))
        self.models_y[n].Load(LoadYAML(L(data['params']['models_y'][n]['model'])), L(data['params']['models_y'][n]['prefix']))

  def Locate(self, filename):
    if filename.find('{base}')>=0 and self.load_base_dir==None:
      raise Exception('Use Load with specifying base_dir argument. Otherwise Locate() can not return the correct location for the filename: %s'%filename)
    return filename.format(base=self.load_base_dir)

  #Initialize planner/learner.  Should be executed before execution.
  def Init(self):
    if self.models_x==None or self.models_y==None:  self.CreateModels()
    if self.Options['model_type'] in ('lwr', 'dnn'):
      for n in range(self.d.N):
        if self.models_x[n] is not None:
          self.models_x[n].Options['base_dir']= self.Options['base_dir']
          self.models_x[n].Init()
        if self.models_y[n] is not None:
          self.models_y[n].Options['base_dir']= self.Options['base_dir']
          self.models_y[n].Init()

  def CreateModels(self):
    self.models_x= [None]*self.d.N
    self.models_y= [None]*self.d.N
    if self.Options['model_type']=='lwr':
      options= copy.deepcopy(self.Options['lwr_options'])
      options['base_dir']= self.Options['base_dir']
      for k in range(self.d.N):
        Dxa= self.d.StateDefs[k].D + self.d.ActionDefs[k].D
        if self.d.StateDefs[k+1].D>0:
          model_x= TLWR()
          model_x.Load(data={'options':options})
          #model_x.Importance= self.sample_importance  #Share importance in every model
          self.models_x[k]= model_x
        else:
          self.models_x[k]= TZeroFunc(Dxa)
        if self.d.OutDefs[k+1].D>0:
          model_y= TLWR()
          model_y.Load(data={'options':options})
          #model_y.Importance= self.sample_importance  #Share importance in every model
          self.models_y[k]= model_y
        else:
          self.models_y[k]= TZeroFunc(Dxa)
    elif self.Options['model_type']=='dnn':
      options= copy.deepcopy(self.Options['dnn_options'])
      options['base_dir']= self.Options['base_dir']
      for k in range(self.d.N):
        Dxa= self.d.StateDefs[k].D + self.d.ActionDefs[k].D
        if self.d.StateDefs[k+1].D>0:
          options['n_units']= [Dxa] + list(self.Options['dnn_hidden_units']) + [self.d.StateDefs[k+1].D]
          options['name']= str(k)+'x'
          model_x= TNNRegression()
          model_x.Load(data={'options':options})
          self.models_x[k]= model_x
        else:
          self.models_x[k]= TZeroFunc(Dxa)
        if self.d.OutDefs[k+1].D>0:
          options['n_units']= [Dxa] + list(self.Options['dnn_hidden_units']) + [self.d.OutDefs[k+1].D]
          options['name']= str(k)+'y'
          model_y= TNNRegression()
          model_y.Load(data={'options':options})
          self.models_y[k]= model_y
        else:
          self.models_y[k]= TZeroFunc(Dxa)

  #If have trained models to predict future states.
  def IsPredictable(self, n):
    if self.models_x==None or self.models_y==None or self.d.N==0:  return False
    predictable= True
    for k in range(n,self.d.N):
      if self.models_x[k].NSamples<self.Options['num_min_predictable'] or not self.models_x[k].IsPredictable():
        predictable= False
        break
      if self.models_y[k].NSamples<self.Options['num_min_predictable'] or not self.models_y[k].IsPredictable():
        predictable= False
        break
    return predictable

  #Get domain definition for DP problem.
  def GetDPDomain(self, with_expand_xy=False):
    domain= TChainDomainDP()
    domain.N= self.d.N
    if not with_expand_xy:
      domain.StateDefs= self.state_bounds
      domain.OutDefs= self.out_bounds
    else:
      domain.StateDefs= [ExpandBounds(space,self.Options['x_expand'],len_min=1.0e-6) for space in self.state_bounds]
      domain.OutDefs= [ExpandBounds(space,self.Options['y_expand'],len_min=1.0e-6) for space in self.out_bounds]
    domain.ActionDefs= self.d.ActionDefs
    domain.Rewards= self.d.Rewards
    domain.ModelsX= self.models_x
    domain.ModelsY= self.models_y
    return domain

  #Get a reference optimizer.
  def GetRefOpt(self):
    domain= self.GetDPDomain(with_expand_xy=True)
    ref_opt= TChainRefOpt()
    helper= TChainRefOpt.THelper()
    helper.Database= self.database
    ref_opt.Load({'options':self.Options['ref_opt']})
    ref_opt.Init(domain, helper)
    return ref_opt

  #Get a DDP solver
  def GetDDPSol(self, logfp=None):
    domain= self.GetDPDomain()
    if self.Options['ddp_ver']==1:
      ddp_sol= TChainDDPSolver()
      helper= TChainDDPSolver.THelper()
      opt= 'ddp_sol'
    elif self.Options['ddp_ver']==2:
      ddp_sol= TChainDDPSolver2()
      helper= TChainDDPSolver2.THelper()
      opt= 'ddp_sol2'
    helper.References= self.references if self.Options['using_refs'] else None
    helper.Database= self.database
    helper.LogFP= logfp
    ddp_sol.Load({'options':self.Options[opt]})
    ddp_sol.Init(domain, helper)
    return ddp_sol


  #Start a new episode.
  def NewEpisode(self):
    self.database.NewEntry(self.d.N+1)

  #End the current episode.
  def EndEpisode(self):
    if self.database.Entry[-1].Len!=self.d.N+1:
      print 'Warning: broken database entry:',self.database.CurrentId
      return
    self.database.UpdateR()

  #n: Index of stage.
  #x_n: State observation.
  #y_n: Output observation.
  #prev: Values of previous stage, should be an instance of TIndexedXYAR.
  #  Usually internally given.
  #NOTE: Execute even for n==0.
  def Update(self, n, x_n, y_n=None, prev=None):
    r_n= self.Reward(n, y_n)[0] if n>0 else None
    self.database.ModifySeq(n=n, x=x_n, y=y_n, r=r_n)

    #Update forward dynamics model:
    if n>0:
      if prev is None:  xa_prev= ToList(self.prev.x)+ToList(self.prev.a)
      else:             xa_prev= ToList(prev.x)+ToList(prev.a)
      self.models_x[n-1].Update(xa_prev, ToList(x_n))
      self.models_y[n-1].Update(xa_prev, ToList(y_n))
    #Update state/output bounds:
    self.state_bounds[n].Add(ToList(x_n))
    self.out_bounds[n].Add(ToList(y_n))

  def Select(self, n, x_n):
    if self.d.ActionDefs[n].D==0:
      a_n= None
    elif not self.IsPredictable(n):
      #If some models are not predictable yet, we generate action randomly.
      a_n= RandB(self.d.ActionDefs[n].Bounds)
    else:
      #Optimize references
      if self.Options['using_refs']:
        ref_opt= self.GetRefOpt()
        ref_opt.Optimize(n, self.references)
        #print 'references:'
        #for k in range(max(1,n),self.d.N+1):  print '  %i: %s'%(k,[(xyar.x,xyar.y,xyar.r) for xyar in self.references[k]])

      t_start= time.time()

      #DDP
      logfp= open(self.Options['opt_log_name'].format(i=self.database.CurrentId,n=n,base=self.Options['base_dir']),'w')
      ddp_sol= self.GetDDPSol(logfp)
      if self.Options['ddp_ver']==1:
        a_map,eval_n= ddp_sol.Plan(n, x_n)
        a_n= a_map[n]
        sum_var_r= sum(eval_n.VarR)
      elif self.Options['ddp_ver']==2:
        traj= ddp_sol.Plan(n, x_n)
        a_n= traj.Seq[n].a0
        sum_var_r= sum([io.var_r1 for k,io in traj.Seq.iteritems()])
      print 'Select: logged to:',self.Options['opt_log_name'].format(i=self.database.CurrentId,n=n,base=self.Options['base_dir'])
      logfp.close()

      #TODO:Save plan_time into a database.
      plan_time= time.time() - t_start

      #Add exploration noise
      if self.Options['explore_noise']=='none':
        pass
      elif self.Options['explore_noise']=='gauss':
        var= self.Options['explore_noise_gain']*math.sqrt(sum_var_r)
        if var>self.Options['explore_var_max']:  var= self.Options['explore_var_max']
        a_noise= np.mat([random.gauss(0.0,var) for d in range(Len(a_n))]).T
        print 'var,a_noise=',var,a_noise.T
        ##TEST---
        #for ngain in (2.0, 1.0, 0.5, 0.1, 0.05, 0.01, 0.005):
          #print '####action noise effect (%f):'%ngain,
          #for ia in range(Len(a_n)):
            ##var= ngain*math.sqrt(sum(eval_n.VarR))
            #var= ngain*math.sqrt(self.action_bounds[n][1][ia]-self.action_bounds[n][0][ia])
            #var_a_map={n:np.diag([0.0]*Len(a_n))}
            #var_a_map[n][ia,ia]= var
            #eval_n_noise= self.EvalActMap(n, x_n, a_map, var_x_n=var_x_n, var_a_map=var_a_map)
            #print self.Criteria(eval_n_noise,cr=('EUCB','none'))-self.Criteria(eval_n,cr=('EUCB','none')),
          #print
        ##TEST---
        a_n+= a_noise
        a_n= ConstrainN(self.d.ActionDefs[n].Bounds, a_n)

    self.prev= TIndexedXYAR(n=n, x=copy.deepcopy(x_n), a=copy.deepcopy(a_n))
    self.database.ModifySeq(n=n, x=x_n, a=a_n)
    return a_n

  #Dump data for plot into files.
  #file_prefix: prefix of the file names; {n} is replaced by a model index.
  #  {file_prefix}_x_est.dat, {file_prefix}_x_smp.dat, {file_prefix}_x_ref.dat,
  #  {file_prefix}_y_est.dat, {file_prefix}_y_smp.dat, {file_prefix}_y_ref.dat are generated.
  def PlotModel(self,n,f_reduce,f_repair,file_prefix='/tmp/f{n}'):
    if n<self.d.N:
      bounds_xa= CatSpaces(self.state_bounds[n], self.d.ActionDefs[n]).Bounds
      if self.Dx(n+1)>0: DumpPlot(self.models_x[n], f_reduce=f_reduce, f_repair=f_repair, x_var=0.0, bounds=bounds_xa, file_prefix='%s_x'%(file_prefix.format(n=n)))
      if self.Dy(n+1)>0: DumpPlot(self.models_y[n], f_reduce=f_reduce, f_repair=f_repair, x_var=0.0, bounds=bounds_xa, file_prefix='%s_y'%(file_prefix.format(n=n)))

    xafp= open('%s_xa_ref.dat'%(file_prefix.format(n=n)),'w') if self.Dx(n)+self.Da(n)>0 else None
    yfp= open('%s_y_ref.dat'%(file_prefix.format(n=n)),'w') if self.Dy(n)>0 else None
    for xyar in self.references[n]:
      xa= ToList(xyar.x)+ToList(xyar.a)
      if xafp: xafp.write('%s\n' % ToStr(f_reduce(xa),xyar.x,xyar.a,[xyar.r]))
      if yfp:  yfp.write('%s\n' % ToStr(xyar.y,[xyar.r]))
    if xafp: xafp.close()
    if yfp:  yfp.close()






