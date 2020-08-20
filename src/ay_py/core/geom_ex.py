#! /usr/bin/env python
#Basic tools (extended geometry).
import numpy as np
import numpy.linalg as la
import math
import random
from scipy.spatial import ConvexHull as scipy_ConvexHull
from scipy.optimize import minimize as scipy_minimize
from util import *
from geom import *

#Return an intersection between (p1,p2) and (pA,pB).
#Return None if there is no intersection.
#Based on: https://www.cs.hmc.edu/ACM/lectures/intersections.html
def LineLineIntersection(p1, p2, pA, pB, tol=1e-8):
  x1, y1 = p1;   x2, y2 = p2
  dx1 = x2 - x1;  dy1 = y2 - y1
  xA, yA = pA;   xB, yB = pB;
  dxA = xB - xA;  dyA = yB - yA;

  DET = (-dx1 * dyA + dy1 * dxA)
  if math.fabs(DET) < tol: return None

  DETinv = 1.0/DET
  r = DETinv * (-dyA * (xA-x1) + dxA * (yA-y1))
  s = DETinv * (-dy1 * (xA-x1) + dx1 * (yA-y1))
  if r<0.0 or s<0.0 or r>1.0 or s>1.0:  return None

  xi = (x1 + r*dx1 + xA + s*dxA)/2.0
  yi = (y1 + r*dy1 + yA + s*dyA)/2.0
  return [xi,yi]

def LinePolygonIntersection(p1, p2, points):
  pIs= [LineLineIntersection(p1,p2,pA,pB) for pA,pB in zip(points, points[1:]+[points[0]])]
  return filter(None,pIs)

#Closest point on a line (p1,p2) from a reference point
def LineClosestPoint(p1, p2, point_ref):
  a= Vec(p2)-Vec(p1)
  t_max= la.norm(a)
  a= a/t_max
  t= np.dot(Vec(point_ref)-p1, a)
  if t>=t_max:  return p2
  if t<=0:      return p1
  return Vec(p1) + t*a

#Closest point on a polygon from a reference point
def PolygonClosestPoint(points, point_ref):
  if len(points)==0:  return None
  if len(points)==1:  return points[0]
  p_closest= None
  d_closest= 1.0e20
  N= len(points)
  for i in range(N):
    p= LineClosestPoint(points[i], points[i+1 if i+1!=N else 0], point_ref)
    d= la.norm(Vec(point_ref)-Vec(p))
    if p_closest is None or d<d_closest:
      p_closest= p
      d_closest= d
  return p_closest

#ref. http://stackoverflow.com/questions/11716268/point-in-polygon-algorithm
#Ray-casting algorithm (http://en.wikipedia.org/wiki/Point_in_polygon)
def PointInPolygon2D(points, point):
  c= False
  j= len(points)-1
  for i in range(len(points)):
    if ((points[i][1]>point[1]) != (points[j][1]>point[1])) and (point[0] < (points[j][0]-points[i][0]) * (point[1]-points[i][1]) / (points[j][1]-points[i][1]) + points[i][0]) :
      c= not c
    j= i
  return c

#Calculate area of a polygon in 2D.
# http://mathworld.wolfram.com/PolygonArea.html
# http://stackoverflow.com/questions/451426/how-do-i-calculate-the-area-of-a-2d-polygon
def PolygonArea(points):
  if len(points)<3:  return 0.0
  return 0.5*abs(sum(x0*y1-x1*y0
                     for ((x0,y0), (x1,y1)) in zip(points, points[1:]+[points[0]])))

#Shrink a polygon with scale (e.g. 0.8) with respect to the center.
def ShrinkPolygon(points, scale):
  center= np.mean(points,axis=0)
  shrunken= [center+scale*(point-center) for point in points]
  return shrunken

def ConvexHull(points):
  return [points[v] for v in scipy_ConvexHull(points).vertices]

'''Move a polygon points along with axes so that it matches with points_ref.
  More specifically, find r such that intersection-area between
  points+r*axes and points_ref is maximized.
  NOTE: For simplicity of computation, points_ref is converted to a convex hull.
  axes: [ax1] or [ax1,ax2] where ax=[x,y].
    In case of [ax1], r is 1-dimensional, and r is 2-d for [ax1,ax2].
  bounds: bounds of r.
'''
def MatchPolygons(points, points_ref, axes, bounds, maxeval=1000):
  points_ref= ConvexHull(points_ref)
  points= np.array(points)
  axes= np.array(axes)
  def f_obj(r):
    move= np.dot(r,axes)
    points_mv= points+move
    intersection= ClipPolygon(points_mv.tolist(), points_ref)
    return -PolygonArea(intersection)
  r= [0.0]*len(axes)
  while f_obj(r)==0.0 and maxeval>0:
    r= RandB(bounds)
    maxeval-= 1
    print r,f_obj(r)
  if f_obj(r)==0.0:  return None, points
  bounds2= [[xmin,xmax] for xmin,xmax in zip(bounds[0],bounds[1])]
  res= scipy_minimize(f_obj, r, bounds=bounds2, options={'maxiter':maxeval})
  print res
  r= res['x']
  points_mv= points+np.dot(r,axes)
  return r, points_mv.tolist()

class TPCA:
  def __init__(self,points,calc_projected=True):
    self.Mean= np.mean(points,axis=0)
    data= points-self.Mean
    cov= np.cov(data.T)
    evals, evecs= la.eig(cov)
    idx= evals.argsort()[::-1]  #Sort by eigenvalue in decreasing order
    self.EVecs= evecs[:,idx]
    self.EVals= evals[idx]
    self.Projected= None
    if calc_projected:
      self.Projected= np.dot(data, self.EVecs)

  def Project(self,points):
    return np.dot(points-self.Mean, self.EVecs)

  def Reconstruct(self,proj,idx=None):
    if idx is None:  idx= range(len(self.EVecs))
    return np.dot(proj, self.EVecs[:,idx].T) + self.Mean

class TPCA_SVD(TPCA):
  def __init__(self,points,calc_projected=True):
    self.Mean= np.mean(points,axis=0)
    data= points-self.Mean
    cov= np.cov(data.T)
    U,S,V= la.svd(cov,0)
    #if S[-1]/S[0]<1.0e-12:  raise Exception('TPCA_SVD: data is singular')
    self.EVecs= V.T
    self.EVals= S
    self.Projected= None
    if calc_projected:
      self.Projected= np.dot(data, self.EVecs)

#Centroid of a polygon
#ref. http://en.wikipedia.org/wiki/Centroid
def PolygonCentroid(points, pca_default=None, only_centroid=True):
  if len(points)==0:  return None
  if len(points)==1:  return points[0]
  assert(len(points[0])==3)
  if pca_default is None:
    pca= TPCA(points)
  else:
    pca= pca_default
  xy= pca.Projected[:,[0,1]]
  N= len(xy)
  xy= np.vstack((xy,[xy[0]]))  #Extend so that xy[N]==xy[0]
  A= 0.5*sum([xy[n,0]*xy[n+1,1] - xy[n+1,0]*xy[n,1] for n in range(N)])
  Cx= sum([(xy[n,0]+xy[n+1,0])*(xy[n,0]*xy[n+1,1]-xy[n+1,0]*xy[n,1]) for n in range(N)]) / (6.0*A)
  Cy= sum([(xy[n,1]+xy[n+1,1])*(xy[n,0]*xy[n+1,1]-xy[n+1,0]*xy[n,1]) for n in range(N)]) / (6.0*A)
  centroid= pca.Reconstruct([Cx,Cy],[0,1])
  if only_centroid:  return centroid
  else:  return centroid, [Cx,Cy]

#Get a parameterized polygon
#  closed: Closed polygon or not.  Can take True,False,'auto' (automatically decided)
#  center_modifier: Function (centroid of polygon --> new center) to modify the center.
class TParameterizedPolygon:
  def __init__(self, points, center_modifier=None, closed='auto'):
    assert(len(points)>=3);
    pca= TPCA(points)
    self.Center, self.Center2d= PolygonCentroid(points, pca, only_centroid=False)
    if center_modifier is not None:
      self.Center= np.array(center_modifier(self.Center))
      self.Center2d= pca.Project(self.Center)[:2]
    self.PCAAxes= pca.EVecs
    self.PCAValues= pca.EVals
    self.PCAMean= pca.Mean
    angles= [0]*len(pca.Projected)
    dirc= 0
    for i in range(len(points)):
      diff= pca.Projected[i,[0,1]] - np.array(self.Center2d)
      angles[i]= math.atan2(diff[1],diff[0])
      if angles[i]>math.pi:  angles[i]-= 2.0*math.pi
      if i>0:
        if AngleDisplacement(angles[i-1],angles[i])>0: dirc+=1
        else: dirc-=1
    #print dirc
    dirc= Sign(dirc)
    self.Direction= dirc
    self.Angles= []
    self.Points= []
    self.Points2D= []
    aprev= angles[0]
    for i in range(len(points)):
      if i==0 or Sign(AngleDisplacement(aprev,angles[i]))==dirc:
        self.Angles.append(angles[i])
        self.Points.append(points[i])
        self.Points2D.append(pca.Projected[i,[0,1]])
        aprev= angles[i]
    #print dirc,self.Angles[0],self.Angles[-1]
    if   dirc>0 and self.Angles[-1]>self.Angles[0]:  self.Offset= -math.pi-self.Angles[0]
    elif dirc<0 and self.Angles[-1]<self.Angles[0]:  self.Offset= -math.pi-self.Angles[-1]
    elif dirc>0 and self.Angles[-1]<self.Angles[0]:  self.Offset= math.pi-self.Angles[0]+1.0e-12
    elif dirc<0 and self.Angles[-1]>self.Angles[0]:  self.Offset= math.pi-self.Angles[-1]+1.0e-12
    self.Angles= [AngleMod1(a+self.Offset) for a in self.Angles]
    if closed=='auto':
      avr_a_diff= sum([abs(AngleDisplacement(self.Angles[i-1],self.Angles[i])) for i in range(1,len(self.Angles))])/float(len(self.Angles)-1)
      self.IsClosed= (abs(AngleDisplacement(self.Angles[-1],self.Angles[0])) < 2.0*avr_a_diff)
      #print dirc,self.Angles[0],self.Angles[-1]
      #print 'IsClosed:', self.IsClosed,avr_a_diff, abs(AngleDisplacement(self.Angles[-1],self.Angles[0]))
    else:
      self.IsClosed= closed
    if self.IsClosed:
      self.Angles.append(self.Angles[0])
      self.Points.append(points[0])
      self.Points2D.append(pca.Projected[0,[0,1]])
    self.IdxAngleMin= min(range(len(self.Angles)), key=lambda i: self.Angles[i])
    self.IdxAngleMax= max(range(len(self.Angles)), key=lambda i: self.Angles[i])
    #print 'angles:',angles
    #print 'self.Angles:',self.Angles
    self.Bounds= self.ComputeBounds()
    self.EstimateAxes(dtheta=2.0*math.pi/50.0)

  #Return a boundary of possible angle for AngleToPoint.
  #NOTE: AngleToPoint(angle) will return None even if angle is in this bound.
  def ComputeBounds(self):
    if self.IsClosed:
      return [-math.pi, math.pi]
    if Sign(self.Angles[-1]-self.Angles[0])==self.Direction:
      if self.Angles[0]<self.Angles[-1]:  return [self.Angles[0], self.Angles[-1]]
      else:                               return [self.Angles[-1], self.Angles[0]]
    raise Exception('Bug in TParameterizedPolygon::ComputeBounds')
    #if self.Direction>0:
      #return [[self.Angles[0],math.pi],[-math.pi,self.Angles[-1]]]
    #else:
      #return [[self.Angles[-1],math.pi],[-math.pi,self.Angles[0]]]

  def AngleToPoint(self, angle):
    angle= AngleMod1(angle)
    if not IsIn(angle, self.Bounds):  return None
    if angle<=self.Angles[self.IdxAngleMin]:
      i_closest= self.IdxAngleMin
      i_closest2= self.IdxAngleMax
      alpha= abs(angle-self.Angles[i_closest])
      alpha2= 2.0*math.pi-self.Angles[i_closest2]+angle
    elif angle>=self.Angles[self.IdxAngleMax]:
      i_closest= self.IdxAngleMax
      i_closest2= self.IdxAngleMin
      alpha= abs(angle-self.Angles[i_closest])
      alpha2= 2.0*math.pi+self.Angles[i_closest2]-angle
    else:
      i_closest= filter(lambda i: IsAngleIn(angle,[self.Angles[i],self.Angles[i+1]]), range(len(self.Angles)-1))
      if len(i_closest)==0:  return None
      i_closest= i_closest[0]
      i_closest2= i_closest+1
      alpha= abs(angle-self.Angles[i_closest])
      alpha2= abs(self.Angles[i_closest2]-self.Angles[i_closest]) - alpha
    if abs(alpha)<1.0e-6:  return np.array(self.Points[i_closest])
    if abs(alpha2)<1.0e-6:  return np.array(self.Points[i_closest2])
    pi= np.array(self.Points2D[i_closest])
    pi2= np.array(self.Points2D[i_closest2])
    c= self.Center2d
    ratio= la.norm(pi-c)/la.norm(pi2-c) * math.sin(alpha)/math.sin(alpha2)
    t= ratio/(1.0+ratio)
    if t<0.0 or t>1.0:  return None
    assert(t>=0.0 and t<=1.0)
    return t*np.array(self.Points[i_closest2]) + (1.0-t)*np.array(self.Points[i_closest])

  def PointToAngle(self, point):
    diff= np.dot(point-self.PCAMean, self.PCAAxes)[[0,1]] - np.array(self.Center2d)
    return AngleMod1(math.atan2(diff[1],diff[0])+self.Offset)

  def EstimateAxes(self,dtheta):
    theta= 0.0
    points= []
    while theta<2.0*math.pi:
      p= self.AngleToPoint(theta)
      if p is not None: points.append(p)
      theta+= dtheta
    pca= TPCA(points)
    self.Axes= pca.EVecs
    self.AxValues= pca.EVals

#Clip subject_polygon by clip_polygon with Sutherland-Hodgman-Algorithm.
#subject_polygon is an arbitrary polygon, clip_polygon is a convex polygon.
#Output polygon is counterclockwise.
#Ref: https://rosettacode.org/wiki/Sutherland-Hodgman_polygon_clipping
def ClipPolygon(subject_polygon, clip_polygon):
  def is_left_of(edge_1, edge_2, test):
    tmp1 = [edge_2[0] - edge_1[0], edge_2[1] - edge_1[1]]
    tmp2 = [test[0] - edge_2[0], test[1] - edge_2[1]]
    x = (tmp1[0] * tmp2[1]) - (tmp1[1] * tmp2[0])
    if x < 0:  return False
    elif x > 0:  return True
    else:  return None  # Colinear points;

  def is_clockwise(polygon):
    for p in polygon:
      is_left = is_left_of(polygon[0], polygon[1], p);
      if is_left != None:  #some of the points may be colinear.  That's ok as long as the overall is a polygon
        return not is_left
    return None #All the points in the polygon are colinear

  def inside(p):
    return(cp2[0]-cp1[0])*(p[1]-cp1[1]) > (cp2[1]-cp1[1])*(p[0]-cp1[0])

  def compute_intersection():
    dc = [ cp1[0] - cp2[0], cp1[1] - cp2[1] ]
    dp = [ s[0] - e[0], s[1] - e[1] ]
    n1 = cp1[0] * cp2[1] - cp1[1] * cp2[0]
    n2 = s[0] * e[1] - s[1] * e[0]
    n3 = 1.0 / (dc[0] * dp[1] - dc[1] * dp[0])
    return [(n1*dp[0] - n2*dc[0]) * n3, (n1*dp[1] - n2*dc[1]) * n3]

  ic = is_clockwise(subject_polygon)
  if ic is None:  return []
  if ic:  subject_polygon.reverse()
  ic = is_clockwise(clip_polygon)
  #print 'is_clockwise(clip_polygon)=',ic
  if ic is None:  return []
  if ic:  clip_polygon.reverse()

  output_list = subject_polygon
  cp1 = clip_polygon[-1]
  #print 'clip_polygon=',clip_polygon

  for clip_vertex in clip_polygon:
    cp2 = clip_vertex
    input_list = output_list
    output_list = []
    #print 'input_list =',input_list
    if len(input_list)==0:
      return []
    s = input_list[-1]
    #print '  s=',s
    #print '  cp1=',cp1
    #print '  cp2=',cp2

    for subject_vertex in input_list:
      e = subject_vertex
      #print '  e=',e,inside(e),inside(s)
      if inside(e):
        if not inside(s):
          output_list.append(compute_intersection())
        output_list.append(e)
      elif inside(s):
        output_list.append(compute_intersection())
      s = e
    cp1 = cp2
  return(output_list)

#Fitting a circle to the data XY, return the center [x,y] and the radius
#based on: http://people.cas.uab.edu/~mosya/cl/HyperSVD.m
#cf. http://people.cas.uab.edu/~mosya/cl/MATLABcircle.html
def CircleFit2D(XY):
  centroid= np.average(XY,0) # the centroid of the data set

  X= [XY[d][0]-centroid[0] for d in range(len(XY))] # centering data
  Y= [XY[d][1]-centroid[1] for d in range(len(XY))] # centering data
  Z= [X[d]**2 + Y[d]**2 for d in range(len(XY))]
  ZXY1= np.matrix([Z, X, Y, [1.0]*len(Z)]).transpose()
  U,S,V= la.svd(ZXY1,0)
  if S[3]/S[0]<1.0e-12:  # singular case
    print 'CircleFit2D: SINGULAR'
    A= (V.transpose())[:,3]
  else:  # regular case
    R= np.average(np.array(ZXY1),0)
    N= np.matrix([[8.0*R[0], 4.0*R[1], 4.0*R[2], 2.0],
                  [4.0*R[1], 1.0, 0.0, 0.0],
                  [4.0*R[2], 0.0, 1.0, 0.0],
                  [2.0,      0.0, 0.0, 0.0]])
    W= V.transpose()*np.diag(S)*V
    D,E= la.eig(W*la.inv(N)*W)  # values, vectors
    idx= D.argsort()
    Astar= E[:,idx[1]]
    A= la.solve(W, Astar)

  A= np.array(A)[:,0]
  center= -A[1:3].transpose()/A[0]/2.0+centroid
  radius= math.sqrt(A[1]**2+A[2]**2-4.0*A[0]*A[3])/abs(A[0])/2.0
  return center, radius

#Fitting a circle to the data XYZ, return the center [x,y,z], the radius, and the rotation matrix.
#Note: ez[2] of the rotation matrix will be always positive.
def CircleFit3D(XYZ):
  pca= TPCA(XYZ)
  XY= pca.Projected[:,[0,1]]
  c,r= CircleFit2D(XY)
  ex= pca.EVecs[:,0]
  ey= pca.EVecs[:,1]
  ez= np.cross(ex,ey)
  if ez[2]<0.0:
    ex=-ex; ey=-ey; ez=-ez
  Rot= np.zeros((3,3))
  Rot[:,0],Rot[:,1],Rot[:,2]= ex,ey,ez
  return pca.Reconstruct(c,[0,1]), r, Rot


#Fitting a circle to the data marker_data, return the center [x,y,z] and the radius
#  marker_data: a sequence of pose vector [0-2]: position x,y,z, [3-6]: orientation qx,qy,qz,qw
def CircleFitX(marker_data):
  #Compute the normal vector
  p_mean= np.array([0.0,0.0,0.0])
  n_z= np.array([0.0,0.0,0.0])
  for x in marker_data:
    p,R= XToPosRot(x)
    n_z+= R[:,2]
    p_mean+= p
  n_z= n_z/float(len(marker_data))
  n_z= n_z/la.norm(n_z)
  p_mean= p_mean/float(len(marker_data))
  n_x= np.array([-n_z[1],n_z[0],0.0])
  n_x= n_x/la.norm(n_x)
  n_y= np.cross(n_z,n_x)

  print 'p_mean= ',p_mean
  print 'n_x= ',n_x
  print 'n_y= ',n_y
  print 'n_z= ',n_z

  #Project the data onto the plane n_x, n_y
  p_data= []
  for x in marker_data:
    p= np.array([x[0],x[1],x[2]])
    lx= np.dot(p-p_mean,n_x)
    ly= np.dot(p-p_mean,n_y)
    p_data.append([lx,ly])
    #print 'lx,ly= ',lx,ly

  #print p_data
  p_file= file('/tmp/original.dat','w')
  for x in marker_data:
    p_file.write('%f %f %f\n' % (x[0],x[1],x[2]))
  p_file.close()

  #Compute the circle parameters
  lcx, radius= CircleFit2D(p_data)
  x_center= lcx[0]*n_x + lcx[1]*n_y + p_mean
  #print x_center, radius

  p_file= file('/tmp/circle.dat','w')
  for ith in range(1000):
    th= (2.0*math.pi)/1000.0*float(ith)
    lx= lcx[0]+radius*math.cos(th)
    ly= lcx[1]+radius*math.sin(th)
    x= lx*n_x + ly*n_y + p_mean
    p_file.write('%f %f %f\n' % (x[0],x[1],x[2]))
  p_file.close()

  return x_center, radius
