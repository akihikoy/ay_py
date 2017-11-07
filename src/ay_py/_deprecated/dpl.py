#!/usr/bin/python
#Dynamic programming and learning.
from base_util import *
from base_ml import *
from base_ml_lwr import *

#Simple wrapper of TContOptNoGrad.
def OptCMA(x0,s0,f,bounds,fp,maxfevals=20):
  D= len(x0)
  ss= [abs(ma-mi) for ma,mi in zip(bounds[0],bounds[1])]
  #zeros= filter(lambda i:ss[i]==0, range(D))
  #zeros= [i if ss[i]>0 else -1 for i in range(D)]
  mapper= [-1]*D
  j= 0
  for i in range(D):
    if ss[i]>0.0:
      mapper[i]= j
      j+= 1
  shrink= lambda x: [x[i] for i in range(D) if mapper[i]>=0]
  expand= lambda xs: [xs[mapper[i]] if mapper[i]>=0 else x0[i] for i in range(D)]
  opt= TContOptNoGrad()
  options= {}
  options['bounds']= [shrink(bounds[0]),shrink(bounds[1])]
  options['tolfun']= 1.0e-4
  options['scale0']= s0
  #print '===',x0,shrink(x0)
  options['parameters0']= shrink(x0)
  options['maxfevals']= maxfevals
  options['verb_disp']= False
  options['scaling_of_variables']= shrink([s/max(ss) for s in ss])
  opt.Init({'options':options})

  count= 0
  while not opt.Stopped():
    x= expand(opt.Select())
    y= f(x)
    opt.Update(y)
    count+= 1
    if fp:  fp.write('%s\n' % ToStr(x,[y]))
    if fp:  print x,'#',y
  print 'Result=',opt.Result(),'obtained in',count
  x,score= opt.Result()
  return expand(x),score

#Simple wrapper of TContOptNoGrad.
def OptCMA2(x0,s0,f_assess,eval_criteria,bounds,fp,maxfevals=20):
  D= len(x0)
  ss= [abs(ma-mi) for ma,mi in zip(bounds[0],bounds[1])]
  #zeros= filter(lambda i:ss[i]==0, range(D))
  #zeros= [i if ss[i]>0 else -1 for i in range(D)]
  mapper= [-1]*D
  j= 0
  for i in range(D):
    if ss[i]>0.0:
      mapper[i]= j
      j+= 1
  shrink= lambda x: [x[i] for i in range(D) if mapper[i]>=0]
  expand= lambda xs: [xs[mapper[i]] if mapper[i]>=0 else x0[i] for i in range(D)]
  opt= TContOptNoGrad()
  options= {}
  options['bounds']= [shrink(bounds[0]),shrink(bounds[1])]
  options['tolfun']= 1.0e-4
  options['scale0']= s0
  #print '===',x0,shrink(x0)
  options['parameters0']= shrink(x0)
  options['maxfevals']= maxfevals
  options['verb_disp']= False
  options['scaling_of_variables']= shrink([s/max(ss) for s in ss])
  opt.Init({'options':options})

  count= 0
  best= (None,None,None)
  while not opt.Stopped():
    x= expand(opt.Select())
    y= f_assess(x)
    e= eval_criteria(y)
    opt.Update(e)
    if best[2]==None or e>best[2]:  best= (x,y,e)
    count+= 1
    if fp:  fp.write('%s\n' % ToStr(x,[e]))
    #if fp:  print x,'#',e
  print 'Result=',best,'obtained in',count
  return best

#Normalize a gradient.
def NormalizeGrad(grad,effective_g_norm_min=1.0e-6):
  g_norm= la.norm(grad)
  if g_norm>effective_g_norm_min:
    grad/= g_norm

'''
Dynamic programming and learning implementation.
Assume a system x_0,a_0 -> F_0 -> x_1,a_1 -> F_1 -> ... -> x_N-1,a_N-1 -> F_N-1 -> x_N.
x_n: state, a_n: action at n-th stage.
action_bounds[n] is bound [min,max] of a_n. action_bounds[n]==None means no action.
R_n(x_n), n=1,...,N: assessment (reward) of the actions.
assessments[n]: A function: R_n(x_n)= assessments[n](x_n)
ideals[n]: A function: I_n(x_n)= x_ideal_n
'''
class TDynPlanLearn:
  @staticmethod
  def DefaultOptions():
    Options= {}
    Options['lwr_kernel']= 'maxg'  #LWR kernel type.
    '''
      'l2g': Standard Gaussian with L2 norm.
      'maxg': Gaussian with max norm.
    '''
    Options['lwr_c_min']= 0.01  #LWR parameter (minimum Gauss kernel width).
    Options['lwr_f_reg']= 0.0001  #LWR parameter (inverse regularization).
    Options['lwr_importance_gain']= 1.0  #Used to compute importance of samples for LWR; importance= sum(R) * this-value if sum(R)>lwr_importance_R_min else None (i.e. 1.0).
    Options['lwr_importance_R_min']= 1.0e+10  #If sum(R)>this value, put importance on that sample.
    Options['initguess_using_db']= True  #Use database in initial guess.
    Options['initguess_R_min']= 0.3  #In init guess, only consider samples whose R > this value.
    Options['initguess_num']= 10  #In init guess, how many samples do we generate.
    Options['initguess_criteria_db']= ('none','ideal')  #In init guess, which criteria to evaluate a sample in DB. 'original': Original R obtained actually. Otherwise: evaluate actions with the criteria.
    Options['initguess_criteria_rand']= ('none','ideal')  #In init guess, which criteria to evaluate a random guess. None: use self.Options['criteria']

    Options['num_bests']= 10  #Number of best samples to be logged.
    Options['best_R_min']= -10.0  #A sample of R < this value can not be in the best samples.

    Options['num_min_predictable']= 3  #Minimum number of random samples necessary for dynamics prediction.

    Options['criteria']= ('EUCB','none')  #Criteria to evaluate.
    '''
    First element:  reward estimation mode.
      'none': 0.0 (may work with reward shaping)
      'sum':  sum of rewards
      'Esum': expected sum of reward
      'EUCB': expected UCB, i.e. expected sum of (reward + UCB_f*std-deviation)
      'ELCB': expected LCB, i.e. expected sum of (reward - LCB_f*std-deviation)
    Second element: reward shaping mode.
      'none': no reward shaping.
      'ideal': reward shaping with the best of ideals.
      'ideal2': reward shaping with the best of ideals only if whose sum_n[R_n(ideal_n)]>reward_shape_R_min.
    '''
    Options['UCB_f']= 1.0  #Scale factor used in UCB.
    Options['LCB_f']= 1.0  #Scale factor used in LCB.
    Options['reward_shape_f']= 0.1  #Factor of reward shape, i.e. blend ratio of reward and state error.
    Options['reward_shape_R_min']= 0.5  #Threshold for criteria[1]=='ideal2'.

    Options['use_xa_var_in_pred']= True  #TEST: Using a variance of x,a to in forward model to predict.

    Options['explore_noise']= 'none'  #Type of exploration noise.
    '''
      'none': No noise.
      'gauss': Gaussian noise with explore_noise_gain*Var[sum(R)].
    '''
    Options['explore_noise_gain']= 0.005  #Exploration noise parameter for 'gauss'.
    Options['explore_var_max']= 1.0  #Max variance of exploration noise of 'gauss'.

    Options['plan_method']= 'grad'  #Planning method.
    '''
    'grad': gradient descent DP.
    'sCMA': single CMA over serialized actions.
    'bfCMA': brute force DP with CMA.
    '''
    Options['grad_act_noise']= 0.2  #Search noise used in PlanGrad.
    Options['grad_criterias']= [('none','ideal'),('none','ideal'),None,None]  #Multi-stage optimization. In each stage, we can use different criteria. None uses default (==Options['criteria']).
    Options['grad_max_iter']= 40  #Max number of iterations of each gradient descent DP.
    Options['grad_max_stages']= 8  #If requested quality is not achieved with grad_criterias, grad_criterias[-1] is repeated until the total # of stages reaches grad_max_stages.
    Options['grad_term_quality_ratio']= 0.7  #Requested quality: evaluation with criteria is greater than this rate * rewards for ideals.
    Options['grad_ideals']= 2  #Number of ideals used to compute gradient candidates.
    Options['grad_using_rsg']= False  #Using gradient of normal + reward shaping.
    Options['grad_linesearch']= 'static'  #PlanGrad: Line search method.
    '''
      'static': Static line length.
      'backtrc': Backtracing.
    '''
    Options['grad_ls_alpha']= 0.03  #PlanGrad: Line length for grad_linesearch=='static'.

    Options['sCMA_initguess']= True  #If do init guess in plan sCMA.

    Options['ideals_grad_iter']= 3  #Updating ideals: number of gradient descent iterations.
    Options['ideals_num']= 6  #Number of ideals to be searched.
    Options['ideals_rpl_best']= 3  #Number of ideals to be replaced by xar_log_best in ModifyIdeals.
    Options['ideals_rpl_rand']= 1  #Number of ideals to be replaced by random value in ModifyIdeals.
    Options['ideals_x_expand']= 1.1  #Ideals' are searched from state space expanded with this ratio.

    Options['opt_log_name']= '/tmp/seq/opt-{i:04d}-{n:03d}.dat'  #Format of log file name of each optimization, i-th run, n-th stage

    return Options
  #@staticmethod
  #def DefaultParams():
    #Params= {}
    #return Params

  #Initialize learner.  data: set to continue from previous state, generated by Save.
  def InitConfig(self, data=None):
    if data<>None and 'options' in data: InsertDict(self.Options, data['options'])
    #if data<>None and 'params' in data: InsertDict(self.Params, data['params'])

  def __init__(self, N, action_bounds, assessments, ideal_funcs, config=None):
    assert(len(action_bounds)==N)
    assert(len(assessments)==N+1)
    assert(len(ideal_funcs)==N+1)
    self.action_bounds= action_bounds
    self.assessments= assessments
    self.ideal_funcs= ideal_funcs
    self.state_bounds= [None]*(N+1)
    self.ideals= [[] for n in range(N+1)]  # [some TXAR]*(N+1)
    self.models= []
    self.sample_importance= {} #importance of samples for models

    self.Options= {}
    #self.Params= {}
    #self.InitConfig(data={'options':self.DefaultOptions(), 'params':self.DefaultParams()})
    self.InitConfig(data={'options':self.DefaultOptions()})
    self.InitConfig(data=config)

    for i in range(N):
      model= TLWR()
      #model.Init(c_min=0.3, f_reg=0.001)
      #model.Init(c_min=0.01, f_reg=0.001)
      #model.Init(c_min=0.002, f_reg=0.001)
      model.Init(c_min=self.Options['lwr_c_min'], f_reg=self.Options['lwr_f_reg'])
      model.Importance= self.sample_importance  #Share importance in every model
      self.models.append(model)
    self.logid= -1
    #self.fp_ideals= open('/tmp/ideals.dat','w')
    self.xar_log= []  #All execution log: Num-data x TEpisodeLog
    self.xar_log_best= []  #Top TEpisodeLog

  #Save into data (dict):  {'options':{options}, 'params':{parameters}}
  def Save(self):
    ST= ToStdType
    data= {}
    data['options']= self.Options
    data['params']= {}
    data['params']['action_bounds']= ST(self.action_bounds)
    data['params']['state_bounds']= [[ST(sb.Min),ST(sb.Max)] for sb in self.state_bounds] #[TBoundingBox]*(N+1)
    data['params']['ideals']= [[[ST(xar.x),ST(xar.a),ST(xar.R)] for xar in ideals_n] for ideals_n in self.ideals] #[[TXAR]*K]*(N+1)
    data['params']['sample_importance']= ST(self.sample_importance)
    data['params']['logid']= ST(self.logid)
    data['params']['xar_log']= [{'seq':[[ST(xar.x),ST(xar.a),ST(xar.R)] for xar in eps.seq],'R':ST(eps.R),'plan_time':ST(eps.plan_time)} for eps in self.xar_log]
    #[TEpisodeLog]*K; TEpisodeLog={'seq':[TXAR]*(N+1),'R':[float]*N,'plan_time':[float]*N}
    data['params']['xar_log_best']= [{'seq':[[ST(xar.x),ST(xar.a),ST(xar.R)] for xar in eps.seq],'R':ST(eps.R),'plan_time':ST(eps.plan_time)} for eps in self.xar_log_best] #[TEpisodeLog]*K
    data['params']['models']= [{} for n in range(len(self.models))]
    for n in range(len(self.models)):
      data['params']['models'][n]['DataX']= ST(self.models[n].DataX)
      data['params']['models'][n]['DataY']= ST(self.models[n].DataY)
    return data

  #Load from data (dict):  {'options':{options}, 'params':{parameters}}
  def Load(self, data):
    self.InitConfig(data={'options':data['options']})
    self.action_bounds= data['params']['action_bounds']
    for n in range(len(data['params']['state_bounds'])):
      self.state_bounds[n]= TBoundingBox(len(data['params']['state_bounds'][n][0]))
      self.state_bounds[n].Min= data['params']['state_bounds'][n][0]
      self.state_bounds[n].Max= data['params']['state_bounds'][n][1]
    for n in range(len(self.ideals)):
      for x,a,R in data['params']['ideals'][n]:  self.ideals[n].append(self.TXAR(x,a,R))
    self.sample_importance= data['params']['sample_importance']
    self.logid= data['params']['logid']
    for eps in data['params']['xar_log']:
      eps2= self.TEpisodeLog(len(eps['seq'])-1)
      eps2.R= eps['R']
      eps2.plan_time= eps['plan_time']
      for n in range(len(eps2.seq)):  eps2.seq[n]= self.TXAR(*eps['seq'][n])
      self.xar_log.append(eps2)
    for eps in data['params']['xar_log_best']:
      eps2= self.TEpisodeLog(len(eps['seq'])-1)
      eps2.R= eps['R']
      eps2.plan_time= eps['plan_time']
      for n in range(len(eps2.seq)):  eps2.seq[n]= self.TXAR(*eps['seq'][n])
      self.xar_log_best.append(eps2)
    for n in range(len(self.models)):
      for x,y in zip(data['params']['models'][n]['DataX'], data['params']['models'][n]['DataY']):
        self.models[n].Update(x,y)

  class TXAR:
    def __init__(self,x=None,a=None,R=None):
      self.x= x  #State
      self.a= a  #Action
      self.R= R  #Reward

  class TEpisodeLog:
    def __init__(self, N):  #N: index of final stage; 0,..,N
      self.seq= [TDynPlanLearn.TXAR() for n in range(N+1)]
      self.R= None  #Sum of rewards
      self.plan_time= [0.0]*N

  def XARLog(self):
    return self.xar_log

  #xar_data should be self.xar_log[i] (i is arbitrary)
  def XARSeqToAMap(self, xar_data):
    return {k:np.mat(xar_data[k].a).T if Len(xar_data[k].a)>0 else None for k in range(len(xar_data))}

  def DumpXARLogLine(self, idx=-1):
    if len(self.xar_log)==0:  return ''
    log= self.xar_log[idx].seq
    R= self.xar_log[idx].R
    N= len(self.models)
    i_cols= 1
    for n in range(N+1):
      if log[n].R==None:  return '#BROKEN_DATA'
      if n==0:
        line= '%i #x %s #a %s'%(n, ToStr(log[n].x), ToStr(log[n].a) )
        i_cols+= 3 + Len(log[n].x) + Len(log[n].a)
      elif n==N:
        line+= ' #%i# %i #x %s #r %f'%(i_cols, n, ToStr(log[n].x), log[n].R )
        i_cols+= 5 + Len(log[n].x)
        line+= ' #%i# #R %f'%(i_cols, R )
        i_cols+= 3
      else:
        line+= ' #%i# %i #x %s #a %s #r %f'%(i_cols, n, ToStr(log[n].x),
                                          ToStr(log[n].a), log[n].R )
        i_cols+= 6 + Len(log[n].x) + Len(log[n].a)
    line+= ' #%i# #Tplan %s '%(i_cols, ToStr(self.xar_log[idx].plan_time) )
    return line

  #Get bounds of state+action at stage n.
  #f_x_expand: expansion of state space.
  #return bounds=[xa_min, xa_max], len(x), len(a)
  def XABounds(self,n,f_x_expand=1.0):
    bb= self.state_bounds[n].GetExpanded(f_x_expand, len_min=0.1)  #FIXME: should be parameter
    Dx= len(bb.Min)
    if n==len(self.models):
      bounds= [bb.Min, bb.Max]
      return bounds, Dx, 0
    if self.action_bounds[n]==None:
      bounds= [bb.Min, bb.Max]
      Da= 0
    else:
      bounds= [bb.Min+self.action_bounds[n][0], bb.Max+self.action_bounds[n][1]]
      Da= len(self.action_bounds[n][0])
    return bounds, Dx, Da

  #If have trained models to predict future states.
  def Predictable(self, n):
    predictable= True
    for k in range(n,len(self.models)):
      if len(self.models[k].DataX)<self.Options['num_min_predictable'] or not self.models[k].Available() or self.state_bounds[k]==None:
        predictable= False
        break
    return predictable

  #Get variance from var_x_n for models[n].Predict which consider a single dimensional variance.
  #Da: number of dimensions of action.
  def XAVarForPred(self, n, var_x_n, var_a_n=None):
    if not self.Options['use_xa_var_in_pred']:  return 0.0
    if var_x_n!=None:
      assert(var_x_n.shape[0]==var_x_n.shape[1])
      var_x= [var_x_n[d,d] for d in range(var_x_n.shape[0])]
    else:
      var_x= [0.0]*Len(self.state_bounds[n].Min)
    if var_a_n!=None:
      var_a= [var_a_n[d,d] for d in range(var_a_n.shape[0])]
    elif self.action_bounds[n]==None:
      var_a= []
    else:
      var_a= [0.0]*Len(self.action_bounds[n][0])
    #return max([var_x_n[d,d] for d in range(var_x_n.shape[0])])
    return np.array(var_x+var_a)


  #n: Index of stage.
  #x_n: State observation.
  def Update(self, n, x_n, xa_prev=None):
    self.xar_log[-1].seq[n].R= self.assessments[n](x_n)
    if n==len(self.models):
      self.xar_log[-1].seq[n].x= ToList(x_n)
      self.xar_log[-1].R= sum([xar.R for xar in self.xar_log[-1].seq])
      if self.xar_log[-1].R >= self.Options['best_R_min']:
        N_best= self.Options['num_bests']
        if len(self.xar_log_best)<N_best or self.xar_log_best[-1].R<self.xar_log[-1].R:
          self.xar_log_best.append(self.xar_log[-1])
          self.xar_log_best.sort(reverse=True, key=lambda x:x.R)
        if len(self.xar_log_best)>N_best:
          self.xar_log_best.pop()
      if self.xar_log[-1].R >= self.Options['lwr_importance_R_min']:
        self.sample_importance[len(self.models[n-1].DataX)]= self.Options['lwr_importance_gain'] * self.xar_log[-1].R

    if xa_prev==None:
      xa_prev= ToList(self.x_prev[1]) + ToList(self.a_prev[1])
    #Update forward dynamics model:
    self.models[n-1].Update(xa_prev, ToList(x_n))
    #Update state bounds:
    if self.state_bounds[n]==None:
      self.state_bounds[n]= TBoundingBox(Len(x_n))
    self.state_bounds[n].Add(ToList(x_n))
    if n==1:
      if self.state_bounds[n-1]==None:
        self.state_bounds[n-1]= TBoundingBox(Len(self.x_prev[1]))
      self.state_bounds[n-1].Add(ToList(self.x_prev[1]))

    #Update ideal states and actions
    '''
    if self.Predictable(1):
      #self.UpdateIdealsCMA(1)
      self.ModifyIdeals(1)
      for i in range(self.Options['ideals_grad_iter']):
        self.UpdateIdealsGrad(1)
      print 'self.ideals(x):'
      for k in range(1,len(self.models)+1):  print '  %i: %s'%(k,[(xar.x,xar.R) for xar in self.ideals[k]])
    #'''

  '''Update ideal states and actions using CMA-ES.
  For a terminal stage n==N, we solve:
    bound= self.state_bounds[n] * f_expand
    x_ideal_n= argmax_{x_n}{ R_n(x_n) }
  For the other stages, we solve:
    bound= (self.state_bounds[n] * f_expand + self.action_bounds[n])
    x_ideal_n,a_ideal_n= argmax_{x_n,a_n}{ R_n(x_n) - norm(x_ideal_{n+1} - F_n(x_n,a_n)) }
  '''
  def UpdateIdealsCMA(self, n):
    f_scale0= 0.1
    bounds,Dx,Da= self.XABounds(n,f_x_expand=self.Options['ideals_x_expand'])
    if n==len(self.models):
      #Get ideals (terminal):
      s0= 0.5*min([bounds[1][d]-bounds[0][d] for d in range(len(bounds[0]))])
      if len(self.ideals[n])==0:
        self.ideals[n]= [self.TXAR()]
        x_n0= [0.5*(bounds[1][d]+bounds[0][d]) for d in range(len(bounds[0]))]
      else:
        x_n0= self.ideals[n][0].x
        s0*= f_scale0
      eval_ideal= self.EvalIdealFunc(n, Dx, criteria='R_xerr')
      fobj= lambda x_n: eval_ideal(np.mat(x_n).T)
      x_n,value= OptCMA(x0=x_n0,s0=s0,f=fobj,bounds=bounds,fp=None,maxfevals=100)
      self.ideals[n][0].x= ToList(x_n)
      self.ideals[n][0].R= self.EvalIdealFunc(n, Dx, criteria='R_xerr')(np.mat(x_n).T)
      return

    #Get succeeding ideals:
    self.UpdateIdealsCMA(n+1)

    #Get ideals:
    s0= 0.5*min([bounds[1][d]-bounds[0][d] for d in range(len(bounds[0]))])
    if len(self.ideals[n])==0:
      self.ideals[n]= [self.TXAR()]
      xa_n0= [0.5*(bounds[1][d]+bounds[0][d]) for d in range(len(bounds[0]))]
    else:
      xa_n0= self.ideals[n][0].x + self.ideals[n][0].a
      s0*= f_scale0
    eval_ideal= self.EvalIdealFunc(n, Dx, criteria='R_xerr')
    fobj= lambda xa_n: eval_ideal(np.mat(xa_n).T)
    xa_n,value= OptCMA(x0=xa_n0,s0=s0,f=fobj,bounds=bounds,fp=None,maxfevals=100)
    self.ideals[n][0].x= ToList(xa_n[:Dx])
    self.ideals[n][0].a= ToList(xa_n[Dx:])
    self.ideals[n][0].R= self.EvalIdealFunc(n, Dx, criteria='R_xerr')(np.mat(xa_n).T)

  #Modify and/or generate ideals
  def ModifyIdeals(self, n):
    num_ideals= self.Options['ideals_num']
    num_bests= self.Options['ideals_rpl_best']
    num_random= self.Options['ideals_rpl_rand']
    bounds,Dx,Da= self.XABounds(n, f_x_expand=self.Options['ideals_x_expand'])

    #Get ideals with human specified ideals function:
    if self.ideal_funcs[n]!=None:
      raise Exception('FIXME')
      self.ideals[n].insert(0, self.TXAR(x=self.ideal_funcs[n](x_n)))

    if len(self.xar_log_best)<num_bests:  num_bests= len(self.xar_log_best)

    if n==len(self.models):
      #Get ideals (terminal):
      if len(self.ideals[n])==0:
        self.ideals[n]= [self.TXAR(x=RandB(bounds)) for i in range(num_ideals)]
      else:
        self.ideals[n][-num_random:]= [self.TXAR(x=RandB(bounds)) for i in range(num_random)]
      self.ideals[n][-num_random-num_bests:-num_random]= [self.TXAR(x=eps.seq[n].x) for eps in self.xar_log_best[:num_bests]]
      self.EvalIdeals(n, self.ideals[n], criteria='R_xerr')
      self.ideals[n].sort(reverse=True,key=lambda x:x.R)
      return

    self.ModifyIdeals(n+1)

    if len(self.ideals[n])==0:
      self.ideals[n]= [self.TXAR(x=RandN(bounds[0][:Dx],bounds[1][:Dx]),
                                 a=RandN(bounds[0][Dx:],bounds[1][Dx:])) for i in range(num_ideals)]
    else:
      self.ideals[n][-num_random:]= [self.TXAR(x=RandN(bounds[0][:Dx],bounds[1][:Dx]),
                                               a=RandN(bounds[0][Dx:],bounds[1][Dx:])) for i in range(num_random)]
    self.ideals[n][-num_random-num_bests:-num_random]= [self.TXAR(x=eps.seq[n].x, a=eps.seq[n].a) for eps in self.xar_log_best[:num_bests]]
    self.EvalIdeals(n, self.ideals[n], criteria='R_xerr')
    self.ideals[n].sort(reverse=True,key=lambda x:x.R)

  '''Update ideal states and actions with gradient descent.
  For a terminal stage n==N, we solve:
    bound= self.state_bounds[n] * f_expand
    x_ideal_n= argmax_{x_n}{ R_n(x_n) }
  For the other stages, we solve:
    bound= (self.state_bounds[n] * f_expand + self.action_bounds[n])
    x_ideal_n,a_ideal_n= argmax_{x_n,a_n}{ R_n(x_n) - norm(x_ideal_{n+1} - F_n(x_n,a_n)) }
  '''
  def UpdateIdealsGrad(self, n):
    bounds,Dx,Da= self.XABounds(n, f_x_expand=self.Options['ideals_x_expand'])
    if n==len(self.models):
      #Get ideals (terminal):
      for i in range(len(self.ideals[n])):
        x_n0= self.ideals[n][i].x
        x_n0= np.mat(x_n0).T
        R_n,b_n,A_n= TaylorExp2(self.assessments[n], x_n0)
        grad= b_n
        NormalizeGrad(grad)  #TEST: normalization of gradient
        cnst= lambda x_n: ConstrainN(bounds, x_n)
        fobj= self.EvalIdealFunc(n, Dx, criteria='R_xerr')
        x_n= LineSearch(fobj,constrain=cnst,x0=x_n0,direc=grad,grad=grad,l0=0.05,rho=0.5,eps=0.2,f0=None)
        #x_n= LineSearchG(fobj,constrain=cnst,x0=x_n0,direc=grad,l_max=0.1,n_max=5)
        self.ideals[n][i].x= ToList(x_n)
        self.ideals[n][i].R= self.EvalIdealFunc(n, Dx, criteria='R_xerr')(x_n)
      self.ideals[n].sort(reverse=True,key=lambda x:x.R)
      return

    #Get succeeding ideals:
    self.UpdateIdealsGrad(n+1)

    #Get ideals:
    for i in range(len(self.ideals[n])):
      #i_next: which ideal of the next stage do we use.
      #i_next==i may be not good since if x_ideals[n+1][i_next] is bad, fobj at a bad x_ideals[n][i] may be the best.
      #i_next= i
      i_next= 0
      xa_n0= self.ideals[n][i].x + self.ideals[n][i].a

      pred= self.models[n].Predict(xa_n0, with_grad=True)
      x_next= pred.Y
      F_xa_n= pred.Grad.T  #Gradient of model
      F_x_n= F_xa_n[:,:Dx]
      F_a_n= F_xa_n[:,Dx:]

      xa_n0= np.mat(xa_n0).T
      R_n,b_n,A_n= TaylorExp2(self.assessments[n], xa_n0[:Dx])

      err= np.mat(self.ideals[n+1][i_next].x).T - x_next
      grad= np.concatenate((b_n + F_x_n.T*err, F_a_n.T*err))  #FIXME# put self.Options['reward_shape_f']
      NormalizeGrad(grad)  #TEST: normalization of gradient
      cnst= lambda xa_n: ConstrainN(bounds, xa_n)
      fobj= self.EvalIdealFunc(n, Dx, criteria='R_xerr')
      #print 'ideal xa_n search:',fobj(xa_n0),fobj(xa_n0+0.01*grad)
      xa_n= LineSearch(fobj,constrain=cnst,x0=xa_n0,direc=grad,grad=grad,l0=0.05,rho=0.5,eps=0.2,f0=None)
      #xa_n= LineSearchG(fobj,constrain=cnst,x0=xa_n0,direc=grad,l_max=0.1,n_max=5)
      self.ideals[n][i].x= ToList(xa_n[:Dx])
      self.ideals[n][i].a= ToList(xa_n[Dx:])
      self.ideals[n][i].R= self.EvalIdealFunc(n, Dx, criteria='R_xerr')(xa_n)
    self.ideals[n].sort(reverse=True,key=lambda x:x.R)


  def Select(self, n, x_n):
    if n==0:
      self.xar_log.append(self.TEpisodeLog(len(self.models)))
      self.logid+= 1

    self.x_prev= (n, copy.deepcopy(x_n))
    if self.action_bounds[n]==None:
      self.a_prev= (n, None)
    else:
      if not self.Predictable(n):
        self.a_prev= (n, RandB(self.action_bounds[n]))
      else:
        #Update ideal states and actions
        #'''
        #self.UpdateIdealsCMA(max(1,n))  #FIXME: flag to choose CMA/Grad
        self.ModifyIdeals(max(1,n))
        for i in range(self.Options['ideals_grad_iter']):
          self.UpdateIdealsGrad(max(1,n))
        print 'self.ideals(x):'
        for k in range(max(1,n),len(self.models)+1):  print '  %i: %s'%(k,[(xar.x,xar.R) for xar in self.ideals[k]])
        #'''

        t_start= time.time()
        #if self.Options['plan_method']=='grad':  a_n,eval_n= self.PlanGrad(n, x_n)
        #elif self.Options['plan_method']=='sCMA':  a_n,eval_n= self.PlanSCMA(n, x_n)
        if self.Options['plan_method']=='grad':  a_map,eval_n= self.PlanGrad(n, x_n); a_n= a_map[n]
        elif self.Options['plan_method']=='sCMA':  a_map,eval_n= self.PlanSCMA(n, x_n); a_n= a_map[n]
        elif self.Options['plan_method']=='bfCMA':  a_n,eval_n= self.PlanBFCMA(n, x_n, var_x_n=None)
        self.xar_log[-1].plan_time[n]= time.time() - t_start

        if self.Options['explore_noise']=='none':
          pass
        elif self.Options['explore_noise']=='gauss':
          var= self.Options['explore_noise_gain']*math.sqrt(sum(eval_n.VarR))
          if var>self.Options['explore_var_max']:  var= self.Options['explore_var_max']
          a_noise= np.mat([random.gauss(0.0,var) for d in range(Len(a_n))]).T
          print 'var,a_noise=',var,a_noise.T
          #TEST---
          for ngain in (2.0, 1.0, 0.5, 0.1, 0.05, 0.01, 0.005):
            print '####action noise effect (%f):'%ngain,
            for ia in range(Len(a_n)):
              #var= ngain*math.sqrt(sum(eval_n.VarR))
              var= ngain*math.sqrt(self.action_bounds[n][1][ia]-self.action_bounds[n][0][ia])
              var_a_map={n:np.diag([0.0]*Len(a_n))}
              var_a_map[n][ia,ia]= var
              eval_n_noise= self.EvalActMap(n, x_n, a_map, var_x_n=None, var_a_map=var_a_map)
              print self.Criteria(eval_n_noise,cr=('EUCB','none'))-self.Criteria(eval_n,cr=('EUCB','none')),
            print
          #TEST---
          a_n+= a_noise
          a_n= ConstrainN(self.action_bounds[n], a_n)
        self.a_prev= (n, a_n)

    self.xar_log[-1].seq[n].x= ToList(x_n)
    self.xar_log[-1].seq[n].a= ToList(self.a_prev[1])
    if n==0:  self.xar_log[-1].seq[n].R= 0.0
    return self.a_prev[1]

  #Brute force planning (dynamic programming) with CMA-ES.
  def PlanBFCMA(self, n, x_n, var_x_n=None):
    if n>=len(self.models):  return None, self.Eval(n, x_n, None, var_x_n)
    bounds= self.action_bounds[n]
    if bounds==None:  return None, self.Eval(n, x_n, None, var_x_n)
    a_n0= [0.5*(bounds[0][d]+bounds[1][d]) for d in range(len(bounds[0]))]
    s0= 0.5*min([bounds[1][d]-bounds[0][d] for d in range(len(bounds[0]))])
    feval= lambda a_n: self.Eval(n, x_n, a_n, var_x_n)
    criteria= self.Criteria
    fp= open(self.Options['opt_log_name'].format(i=self.logid,n=n),'w')
    a_n,eval_n,value= OptCMA2(x0=a_n0,s0=s0,f_assess=feval,eval_criteria=criteria,bounds=bounds,fp=fp,maxfevals=300)
    print 'PlanBFCMA: logged to:',self.Options['opt_log_name'].format(i=self.logid,n=n)
    fp.close()
    return a_n,eval_n

  #Get a single evaluation from TEvalRes according to the criteria.
  def Criteria(self, eval_n, cr=None):
    if cr==None:  cr= self.Options['criteria']
    cr0,cr1= cr
    if cr0=='none':  value= 0.0
    #Sum of reward criteria:
    elif cr0=='sum':  value= sum(eval_n.R)
    #Expected sum of reward criteria:
    elif cr0=='Esum':  value= sum(eval_n.R)+sum(eval_n.Rerr)
    ##Expected sum of reward + std-deviation criteria (i.e. UCB):  #NOTE: we do not use this criteria since taking derivative becomes complicated
    #value= sum(eval_n.R)+sum(eval_n.Rerr)+math.sqrt(sum(eval_n.VarR))
    #Expected sum of (reward + std-deviation) criteria (i.e. UCB2):
    elif cr0=='EUCB':  value= sum(eval_n.R)+sum(eval_n.Rerr)+sum(map(math.sqrt,eval_n.VarR))*self.Options['UCB_f']
    #Expected sum of (reward - std-deviation) criteria (i.e. LCB or negative UCB):
    elif cr0=='ELCB':  value= sum(eval_n.R)+sum(eval_n.Rerr)-sum(map(math.sqrt,eval_n.VarR))*self.Options['LCB_f']
    else:  raise Exception('Unknown criteria: %s' % repr(cr))
    #Reward shaping with x_ideal:
    if cr1=='none':  pass
    elif cr1=='ideal':
      value+= -sum(eval_n.IdealXerr)*(1.0 if cr0=='none' else self.Options['reward_shape_f'])
    elif cr1=='ideal2':
      Ri= sum([self.assessments[n](np.mat(self.ideals[n][0].x).T) for n in range(1,len(self.ideals))])
      if Ri>self.Options['reward_shape_R_min']:
        value+= -sum(eval_n.IdealXerr)*(1.0 if cr0=='none' else self.Options['reward_shape_f'])
    else:  raise Exception('Unknown criteria: %s' % repr(cr))
    return value

  #Evaluation result class.
  class TEvalRes:
    def __init__(self, n):
      self.Start_n= n
      self.X= []  #States: x_n, x_n+1, ...
      self.A= []  #Actions: a_n, a_n+1, ...
      self.R= []  #Assessments (rewards): R_n+1, R_n+2, ...
      self.Rerr= []  #Residual of reward expectation: E[R_n+1]-R_n+1, E[R_n+2]-R_n+2, ...
      self.VarR= []  #Variances of R: var[R_n+1], var[R_n+2], ...
      self.IdealXerr= []  #Norm between x_ideal_n and x_n

  #Estimating evaluation (return) without a value function with a brute force planner.
  #Evaluation of taking an action a_n at a brief state N(x_n, var_x_n).
  def Eval(self, n, x_n, a_n, var_x_n=None):
    if n>=len(self.models):
      eval_n= self.TEvalRes(n)
      eval_n.X= [x_n]
      return eval_n
    pred= self.models[n].Predict(ToList(x_n)+ToList(a_n), self.XAVarForPred(n,var_x_n), with_var=True, with_grad=True)
    x_next= pred.Y
    x_ideal_next_err= la.norm(np.mat(self.ideals[n+1][0].x).T-x_next)**2 if len(self.ideals[n+1])>0 else 0.0
    F_xa_n= pred.Grad.T  #Gradient of model
    F_x_n= F_xa_n[:,:Len(x_n)]
    #F_a_n= F_xa_n[:,Len(x_n):]
    var_x_next= pred.Var
    if var_x_n!=None:  var_x_next+= F_x_n * var_x_n * F_x_n.T
    R_next,Rerr_next,var_R_next= self.Reward(n+1, x_next, var_x_next)
    a_next,eval_next= self.PlanBFCMA(n+1, x_next, var_x_next)
    #eval_next= self.Eval(n+1, x_next, self.PlanBFCMA(n+1, x_next, var_x_next), var_x_next)
    eval_n= eval_next
    eval_n.Start_n= n
    eval_n.X.insert(0, x_n)
    eval_n.A.insert(0, a_n)
    eval_n.R.insert(0, R_next)
    eval_n.Rerr.insert(0, Rerr_next)
    eval_n.VarR.insert(0, var_R_next)
    eval_n.IdealXerr.insert(0, x_ideal_next_err)
    return eval_n

  #Evaluate action map a_map[k]=action_k starting at a brief state N(x_n, var_x_n).
  def EvalActMap(self, n, x_n, a_map, var_x_n=None, var_a_map={}):
    eval_n= self.TEvalRes(n)
    for k in range(n,len(self.models)):
      a_n= a_map[k]
      if k in var_a_map:  var_xa_n= self.XAVarForPred(k,var_x_n,var_a_map[k])
      else:               var_xa_n= self.XAVarForPred(k,var_x_n)
      pred= self.models[k].Predict(ToList(x_n)+ToList(a_n), var_xa_n, with_var=True, with_grad=True)
      x_next= pred.Y
      x_ideal_next_err= la.norm(np.mat(self.ideals[k+1][0].x).T-x_next)**2  if len(self.ideals[k+1])>0 else 0.0
      F_xa_n= pred.Grad.T  #Gradient of model
      F_x_n= F_xa_n[:,:Len(x_n)]
      F_a_n= F_xa_n[:,Len(x_n):]
      var_x_next= pred.Var
      if var_x_n!=None:  var_x_next+= F_x_n * var_x_n * F_x_n.T
      if k in var_a_map: var_x_next+= F_a_n * var_a_map[k] * F_a_n.T
      R_next,Rerr_next,var_R_next= self.Reward(k+1, x_next, var_x_next)
      eval_n.X.append(x_n)
      eval_n.A.append(a_n)
      eval_n.R.append(R_next)
      eval_n.Rerr.append(Rerr_next)
      eval_n.VarR.append(var_R_next)
      eval_n.IdealXerr.append(x_ideal_next_err)
      x_n= x_next
      var_x_n= var_x_next
    return eval_n

  #Compute an immediate reward R_n(x_n) at a brief state N(x_n, var_x_n)
  #  return R_n, Rerr_n, var_R_n
  #  where Rerr_n: residual of reward expectation (i.e. E[R_n]= R_n+Rerr_n),
  #  var_R_n: variance of reward (i.e. var[R_n]).
  #  n>=1
  def Reward(self, n, x_n, var_x_n):
    #Get a quadratic form of R_n around x_n:
    #  b_n, A_n: 1st and 2nd order derivative of R_n around x_n
    R_n,b_n,A_n= TaylorExp2(self.assessments[n], x_n)
    if var_x_n.size>0:
      #Residual of reward expectation:
      Rerr_n= (A_n*var_x_n).trace()[0,0]
      #Variance of reward:
      var_R_n= 2.0*(A_n*var_x_n*A_n*var_x_n).trace() + b_n.T*var_x_n*b_n
      var_R_n= var_R_n[0,0]
    else:
      Rerr_n= 0.0
      var_R_n= 0.0
    return R_n, Rerr_n, var_R_n

  #Return function to evaluate an ideal x+a.
  #criteria: 'sumR': R(x_n)+ideals[n+1].R, 'R_xerr': R(x_n)-|error of ideals[n+1].x|**2
  def EvalIdealFunc(self, n, Dx, criteria):
    #i_next: which ideal of the next stage do we use.
    #i_next==i may not be good since if x_ideals[n+1][i_next] is bad, fobj at a bad x_ideals[n][i] may be the best.
    #i_next= i
    i_next= 0
    if n==len(self.models):
      if criteria=='sumR' or criteria=='R_xerr':
        fobj= lambda x_n: self.assessments[n](x_n)
    else:
      if criteria=='sumR':
        fobj= lambda xa_n: self.assessments[n](xa_n[:Dx]) + self.ideals[n+1][i_next].R
      elif criteria=='R_xerr':
        fobj= lambda xa_n: self.assessments[n](xa_n[:Dx]) - self.Options['reward_shape_f'] * la.norm(np.mat(self.ideals[n+1][i_next].x).T - self.models[n].Predict(ToList(xa_n)).Y)**2
    return fobj

  #Evaluate ideals. ideals: list of TXAR().
  def EvalIdeals(self, n, ideals, criteria):
    if len(ideals)==0:  return
    fobj= self.EvalIdealFunc(n, Dx=Len(ideals[0].x), criteria=criteria)
    for xar in ideals:
      xar.R= fobj(np.mat(ToList(xar.x)+ToList(xar.a)).T)


  def SerializeActMap(self, n, a_map):
    size= sum([len(b[0]) for b in self.action_bounds[n:] if b!=None])
    a_list= [0.0]*size
    i= 0
    for k in range(n,len(self.models)):
      if a_map[k]!=None:
        ie= i+Len(a_map[k])
        a_list[i:ie]= ToList(a_map[k])
        i= ie
    return a_list

  def DeserializeActList(self, n, a_list):
    a_map= {}
    i= 0
    for k in range(n,len(self.models)):
      if self.action_bounds[k]==None:
        a_map[k]= None
      else:
        ie= i+len(self.action_bounds[k][0])
        a_map[k]= np.mat(a_list[i:ie]).T
        i= ie
    return a_map

  #CMA-ES planning (dynamic programming) where we consider all actions as a single action.
  def PlanSCMA(self, n, x_n, var_x_n=None):
    N= len(self.models)
    none_a_map= {k:None for k in range(n,N)}
    if n>=len(self.models):  return None, self.EvalActMap(n, x_n, none_a_map, var_x_n)
    bmin= self.SerializeActMap(n, {k:np.mat(self.action_bounds[k][0]).T if self.action_bounds[k]!=None else None for k in range(n,N)})
    bmax= self.SerializeActMap(n, {k:np.mat(self.action_bounds[k][1]).T if self.action_bounds[k]!=None else None for k in range(n,N)})
    bounds= [bmin,bmax]
    if len(bmin)==0:  return None, self.EvalActMap(n, x_n, none_a_map, var_x_n)
    if self.Options['sCMA_initguess']:
      a_list0= self.SerializeActMap(n, self.InitGuess(n, x_n) )
    else:
      a_list0= [0.5*(b1+b2) for b1,b2 in zip(bmin,bmax)]
    s0= 0.5*min([bmax[d]-bmin[d] for d in range(len(bmin))])
    feval= lambda a_list: self.EvalActMap(n, x_n, self.DeserializeActList(n,a_list), var_x_n)
    criteria= self.Criteria
    fp= open(self.Options['opt_log_name'].format(i=self.logid,n=n),'w')
    a_list,eval_n,value= OptCMA2(x0=a_list0,s0=s0,f_assess=feval,eval_criteria=criteria,bounds=bounds,fp=fp,maxfevals=300)
    print 'PlanSCMA: logged to:',self.Options['opt_log_name'].format(i=self.logid,n=n)
    fp.close()
    #return self.DeserializeActList(n,a_list)[n],eval_n
    return self.DeserializeActList(n,a_list),eval_n


  #Plan action with gradient chain.
  def PlanGrad(self, n, x_n, tol=1.0e-4):
    diff_value= lambda a_map1,a_map2,cr: self.Criteria(self.EvalActMap(n, x_n, a_map1, var_x_n=None), cr=cr) - self.Criteria(self.EvalActMap(n, x_n, a_map2, var_x_n=None), cr=cr)
    a_map= self.InitGuess(n, x_n)
    a_map_best= None
    fp= open(self.Options['opt_log_name'].format(i=self.logid,n=n),'w')
    Nj= len(self.Options['grad_criterias'])
    Ni= self.Options['grad_max_iter']
    R_ideals= sum(self.assessments[k](np.mat(self.ideals[k][0].x).T) for k in range(n+1,len(self.assessments)))
    for j in range(self.Options['grad_max_stages']):
      criteria= self.Options['grad_criterias'][min(j,Nj-1)]
      for i in range(Ni):
        a_map_new= {}
        grad_data= []  #For log
        self.StepGradOpt(n, x_n, var_x_n=None, a_map=a_map, a_map_new=a_map_new, grad_data=grad_data, criteria=criteria)
        if fp:  #Write log on file
          eval_n= self.EvalActMap(n, x_n, a_map, var_x_n=None)
          value= sum(eval_n.R)
          cvalue= self.Criteria(eval_n, cr=criteria)
          fp.write('%s # %f %f # %s\n'%(ToStr(ToList(a_map[n])), value, cvalue, ' '.join([ToStr(ToList(g)) for g in grad_data]) ))
        dvalue= diff_value(a_map_new, a_map, cr=criteria)
        if dvalue > 0.0: a_map= a_map_new
        #print dvalue
        if fp and (i==Ni-1 or dvalue < tol):
          eval_n= self.EvalActMap(n, x_n, a_map, var_x_n=None)
          value= sum(eval_n.R)
          cvalue= self.Criteria(eval_n, cr=criteria)
          fp.write('%s # %f %f # %s\n'%(ToStr(ToList(a_map[n])), value, cvalue, ' '.join([ToStr([0.0]*Len(g)) for g in grad_data]) ))
        if dvalue < tol:  break
      if a_map_best==None or diff_value(a_map, a_map_best, cr=criteria)>0.0:
        a_map_best= a_map
      if j>=Nj-1 and j<self.Options['grad_max_stages']-1:
        if self.Criteria(self.EvalActMap(n, x_n, a_map_best, var_x_n=None)) > self.Options['grad_term_quality_ratio']*R_ideals:
          break
      if j<self.Options['grad_max_stages']-1:
        if criteria==self.Options['grad_criterias'][min(j+1,Nj-1)]:
          #Random jump for next optimization stage.
          a_map_noise= self.ActMapNoise(n,var=self.Options['grad_act_noise'])
          a_map= {}
          for k in range(n,len(self.models)):
            if a_map_best[k]!=None:
              a_map[k]= ConstrainN(self.action_bounds[k], a_map_best[k] + a_map_noise[k])
            else:
              a_map[k]= None
        else:
          a_map= a_map_best
      if fp: fp.write('\n')
    a_map= a_map_best
    if fp:
      eval_n= self.EvalActMap(n, x_n, a_map, var_x_n=None)
      value= sum(eval_n.R)
      cvalue= self.Criteria(eval_n, cr=criteria)
      fp.write('%s # %f %f # %s\n'%(ToStr(ToList(a_map[n])), value, cvalue, ' '.join([ToStr([0.0]*Len(g)) for g in grad_data]) ))
      print 'PlanGrad: logged to:',self.Options['opt_log_name'].format(i=self.logid,n=n)
      fp.close()
    #return a_map[n], self.EvalActMap(n, x_n, a_map, var_x_n=None)
    return a_map, self.EvalActMap(n, x_n, a_map, var_x_n=None)

  #Initial guess for gradient based search.
  def InitGuess(self, n, x_n):
    N= len(self.models)
    if n>=len(self.models):
      raise Exception('InitGuess: invalid call for n:',n)
    #if self.action_bounds[n]!=None:
      #a_n= np.mat(RandB(self.action_bounds[n])).T
    #else:
      #a_n= None
    #if n==N-1:
      #return {n:a_n}
    #else:
      #x_next= self.models[n].Predict(ToList(x_n)+ToList(a_n)).Y
      #a_map= self.InitGuess(n+1, x_next)
      #a_map[n]= a_n
      #return a_map
    fa_data= []
    num= self.Options['initguess_num']
    if self.Options['initguess_using_db']:
      #Initial guess from database (past log)
      R_min= self.Options['initguess_R_min']
      randidx= [i for i in range(len(self.xar_log)) if self.xar_log[i].R!=None and self.xar_log[i].R>R_min]
      random.shuffle(randidx)
      for i in randidx[:num]:
        a_map= self.XARSeqToAMap(self.xar_log[i].seq)
        if self.Options['initguess_criteria_db']=='original':
          value= self.xar_log[i].R
        else:
          value= self.Criteria(self.EvalActMap(n, x_n, a_map, var_x_n=None), cr=self.Options['initguess_criteria_db'])
        fa_data.append([value,a_map])
    for i in range(num-len(fa_data)):
      a_map= self.RandActMap(n)
      value= self.Criteria(self.EvalActMap(n, x_n, a_map, var_x_n=None), cr=self.Options['initguess_criteria_rand'])
      fa_data.append([value,a_map])
    fa_data.sort(reverse=True,key=lambda x:x[0])
    return fa_data[0][1]

  #Generate action map randomly.
  def RandActMap(self, n):
    if n>=len(self.models):
      raise Exception('RandActMap: invalid call for n:',n)
    if self.action_bounds[n]!=None:
      a_n= np.mat(RandB(self.action_bounds[n])).T
    else:
      a_n= None
    if n==len(self.models)-1:
      return {n:a_n}
    else:
      a_map= self.RandActMap(n+1)
      a_map[n]= a_n
      return a_map

  #Generate Gaussian noise of action map.
  #sigma: diag([(var*(amax_d-amin_d))**2]) where amin_d,amax_d are action bound.
  def ActMapNoise(self, n, var):
    if n>=len(self.models):
      raise Exception('ActMapNoise: invalid call for n:',n)
    if self.action_bounds[n]!=None:
      sigma= np.diag([(var*(amax-amin))**2 for amin,amax in zip(*self.action_bounds[n])])
      va_n= np.mat( np.random.multivariate_normal([0.0]*len(self.action_bounds[n][0]), sigma) ).T
    else:
      va_n= None
    if n==len(self.models)-1:
      return {n:va_n}
    else:
      va_map= self.ActMapNoise(n+1, var)
      va_map[n]= va_n
      return va_map

  #One step update of actions a_map using a gradient chain.
  #Updated actions are stored in a_map_new.
  def StepGradOpt(self, n, x_n, var_x_n, a_map, a_map_new, criteria, with_DR=False, grad_data=None):
    num_ideals= self.Options['grad_ideals']
    if n>=len(self.models):
      raise Exception('StepGradOpt: invalid call for n:',n)
    pred= self.models[n].Predict(ToList(x_n)+ToList(a_map[n]), self.XAVarForPred(n,var_x_n), with_var=True, with_grad=True)
    x_next= pred.Y
    F_xa_n= pred.Grad.T  #Gradient of model
    F_x_n= F_xa_n[:,:Len(x_n)]
    F_a_n= F_xa_n[:,Len(x_n):]
    if var_x_n!=None:  var_x_next= F_x_n * var_x_n * F_x_n.T + pred.Var
    else:              var_x_next= pred.Var
    if n==len(self.models)-1:
      #Get a quadratic form of R_next around x_next:
      #  b_next, A_next: 1st and 2nd order derivative of R_next around x_next
      R_next,b_next,A_next= TaylorExp2(self.assessments[n+1], x_next)
      d_R_next= b_next  #2.0*A_next*x_next +
      d_R_next2= [(np.mat(xari.x).T - x_next) for xari in self.ideals[n+1][:num_ideals]]  #TEST
      #Get a gradient chain:
      #DR_n= d_R_next
      #(MULTIPLE GRADIENTS TEST)
      DR_n= [d_R_next] + d_R_next2
      if self.Options['grad_using_rsg']:
        DR_n+= [DR_n[0] + self.Options['reward_shape_f']*d_R for d_R in d_R_next2]  #TEST:Reward shape-C
    else:
      DR_n= self.StepGradOpt(n+1, x_next, var_x_next, a_map, a_map_new, with_DR=True, criteria=criteria)

    #Update action[n]
    if a_map[n]==None:
      a_map_new[n]= None
    else:
      va_data= []
      for i in range(len(DR_n)):
      #for i in range(1):
        f= lambda a: ( a_map_new.__setitem__(n,a) , self.Criteria(self.EvalActMap(n, x_n, a_map_new, var_x_n=None), cr=criteria) )[1]

        if self.Options['grad_linesearch']=='static':
          grad= F_a_n.T * DR_n[i]
          NormalizeGrad(grad)  #TEST: normalization of gradient
          a_new= a_map[n] + self.Options['grad_ls_alpha'] * grad
          a_new= ConstrainN(self.action_bounds[n], a_new)
        elif self.Options['grad_linesearch']=='backtrc':
          grad= F_a_n.T * DR_n[i]
          NormalizeGrad(grad)  #TEST: normalization of gradient
          #is_in= lambda a: IsInN(self.action_bounds[n], a)
          cnst= lambda a: ConstrainN(self.action_bounds[n], a)
          a_new= LineSearch(f,constrain=cnst,x0=a_map[n],direc=grad,grad=grad,l0=0.2,rho=0.5,eps=0.2,f0=None)
          #a_new= LineSearchG(f,constrain=cnst,x0=a_map[n],direc=grad,l_max=0.2,n_max=5)
          a_new= ConstrainN(self.action_bounds[n], a_new)
        else:
          raise Exception('Unknown grad_linesearch:',self.Options['grad_linesearch'])

        va_data.append([f(a_new), a_new])
        if grad_data!=None:  grad_data.append(grad)
      #print 'va_data',va_data
      va_data.sort(reverse=True,key=lambda x:x[0])
      a_map_new[n]= va_data[0][1]

      #if fp:  #Write log on file
        #eval_n= self.EvalActMap(n, x_n, a_map_new, var_x_n=None)
        #value= sum(eval_n.R)
        #fp.write('%s %f %s %s\n'%(ToStr(ToList(a_map_new[n])), value, ToStr(ToList(grad_data[0])), ToStr(ToList(grad_data[1])) ))

    if n>=1 and with_DR:
      #Get a quadratic form of R_n around x_n:
      #  b_n, A_n: 1st and 2nd order derivative of R_n around x_n
      R_n,b_n,A_n= TaylorExp2(self.assessments[n], x_n)
      d_R_n= b_n  #2.0*A_n*x_n +
      d_R_n2= [(np.mat(xari.x).T - x_n) for xari in self.ideals[n][:num_ideals]]  #TEST
      if Len(x_n)>0:
        #DR_prev= [F_x_n.T * DR_n[0] + d_R_n] + F_x_n.T * DR_n[1+i] + d_R_n2[i] for i=0,..,???  #TEST:Reward shape-A
        DR_prev= [F_x_n.T * DR_n[0] + d_R_n] + d_R_n2  #TEST:Reward shape-B
      else:
        DR_prev= [d_R_n] + d_R_n2
      if self.Options['grad_using_rsg']:
        DR_prev+= [DR_prev[0] + self.Options['reward_shape_f']*d_R for d_R in d_R_n2]  #TEST:Reward shape-C
    else:
      DR_prev= [None]
    return DR_prev

  #f_x_expand: expansion ration of state space to be plotted.
  def PlotModel(self,n,f_reduce,f_repair,file_prefix='/tmp/f{n}',f_x_expand=None):
    if f_x_expand==None:  f_x_expand= self.Options['ideals_x_expand']
    bounds,Dx,Da= self.XABounds(n,f_x_expand=f_x_expand)
    self.models[n].DumpPlot(bounds,f_reduce,f_repair,file_prefix.format(n=n))

    fp= open('%s_ideals.dat'%(file_prefix.format(n=n)),'w')
    for xar1 in self.ideals[n]:
      xa1= ToList(xar1.x)+ToList(xar1.a)
      fp.write('%s\n' % ToStr(f_reduce(xa1),xar1.x,xar1.a,[xar1.R]))
    fp.close()


