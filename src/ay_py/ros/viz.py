#! /usr/bin/env python
#ROS tools (visualization).
import roslib; roslib.load_manifest('rospy')
import rospy
import visualization_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
from base import *
from const import *
from ..core.geom import *

class TSimpleVisualizer(object):
  def __init__(self, viz_dt=rospy.Duration(), name_space='visualizer', frame=None, queue_size=1):
    self.viz_pub= rospy.Publisher('visualization_marker', visualization_msgs.msg.Marker, queue_size=queue_size)
    self.curr_id= 0
    self.added_ids= set()
    self.viz_frame= 'base' if frame is None else frame
    self.viz_ns= name_space
    self.viz_dt= viz_dt
    #self.viz_dt= rospy.Duration()
    #ICol:r,g,b,  y,p,sb, w
    self.indexed_colors= [[1,0,0],[0,1,0],[0,0,1],[1,1,0],[1,0,1],[0,1,1],[1,1,1]]

  def __del__(self):
    if self.viz_dt in (None, rospy.Duration()):
      self.DeleteAllMarkers()
    self.Reset()
    #self.viz_pub.publish()
    self.viz_pub.unregister()

  def Reset(self, viz_dt=None):
    self.curr_id= 0
    if viz_dt!=None:
      self.viz_dt= viz_dt

  def DeleteMarker(self, mid):
    marker= visualization_msgs.msg.Marker()
    marker.header.frame_id= self.viz_frame
    marker.ns= self.viz_ns
    marker.id= mid
    marker.action= visualization_msgs.msg.Marker.DELETE
    self.viz_pub.publish(marker)
    #marker.header.frame_id= self.viz_frame
    #marker.ns= self.viz_ns
    #marker.action= visualization_msgs.msg.Marker.DELETEALL
    #self.viz_pub.publish(marker)
    if mid in self.added_ids:  self.added_ids.remove(mid)

  def DeleteAllMarkers(self):
    #print '[Viz]Deleting all markers:',self.added_ids
    marker= visualization_msgs.msg.Marker()
    #for mid in self.added_ids:
      #marker.header.frame_id= self.viz_frame
      #marker.ns= self.viz_ns
      #marker.id= mid
      #marker.action= visualization_msgs.msg.Marker.DELETE
      #self.viz_pub.publish(marker)
    marker.header.frame_id= self.viz_frame
    marker.ns= self.viz_ns
    marker.action= visualization_msgs.msg.Marker.DELETEALL
    self.viz_pub.publish(marker)
    self.added_ids= set()

  def ICol(self, i):
    return self.indexed_colors[i%len(self.indexed_colors)]

  def GenMarker(self, x, scale, rgb, alpha):
    marker= visualization_msgs.msg.Marker()
    marker.header.frame_id= self.viz_frame
    marker.header.stamp= rospy.Time.now()
    marker.ns= self.viz_ns
    #marker.id= self.curr_id
    marker.action= visualization_msgs.msg.Marker.ADD  # or DELETE
    marker.lifetime= self.viz_dt
    marker.scale= geometry_msgs.msg.Vector3(*scale[:3])
    if isinstance(rgb[0],(int,float)):
      marker.color= std_msgs.msg.ColorRGBA(rgb[0],rgb[1],rgb[2],alpha)
    else:
      marker.colors= [std_msgs.msg.ColorRGBA(r,g,b,alpha) for r,g,b in rgb]
    marker.pose= XToGPose(x)
    #self.curr_id+= 1
    return marker

  def SetID(self, marker, mid):
    if mid is None:
      marker.id= self.curr_id
      self.curr_id+= 1
    else:
      marker.id= mid
      if marker.id>=self.curr_id:
        self.curr_id= marker.id+1
    self.added_ids= self.added_ids.union([marker.id])
    return marker.id+1

  #Visualize a marker at x.  If mid is None, the id is automatically assigned
  def AddMarker(self, x, scale=[0.02,0.02,0.004], rgb=[1,1,1], alpha=1.0, mid=None):
    marker= self.GenMarker(x, scale, rgb, alpha)
    mid2= self.SetID(marker,mid)
    marker.type= visualization_msgs.msg.Marker.CUBE  # or CUBE, SPHERE, ARROW, CYLINDER
    self.viz_pub.publish(marker)
    return mid2

  #Visualize an arrow at x.  If mid is None, the id is automatically assigned
  def AddArrow(self, x, scale=[0.05,0.002,0.002], rgb=[1,1,1], alpha=1.0, mid=None):
    marker= self.GenMarker(x, scale, rgb, alpha)
    mid2= self.SetID(marker,mid)
    marker.type= visualization_msgs.msg.Marker.ARROW  # or CUBE, SPHERE, ARROW, CYLINDER
    self.viz_pub.publish(marker)
    return mid2

  #Visualize a list of arrows.  If mid is None, the id is automatically assigned
  def AddArrowList(self, x_list, axis='x', scale=[0.05,0.002], rgb=[1,1,1], alpha=1.0, mid=None):
    iex,iey= {'x':(0,1),'y':(1,2),'z':(2,0)}[axis]
    def point_on_arrow(x,l):
      exyz= RotToExyz(QToRot(x[3:]))
      ex,ey= exyz[iex],exyz[iey]
      pt= x[:3]+l*ex
      return [x[:3], pt, pt, x[:3]+0.7*l*ex+0.15*l*ey, pt, x[:3]+0.7*l*ex-0.15*l*ey]
    x= [0,0,0, 0,0,0,1]
    if not isinstance(rgb[0],(int,float)):  rgb= np.repeat(rgb,6,axis=0)
    marker= self.GenMarker(x, [scale[1],0.0,0.0], rgb, alpha)
    mid2= self.SetID(marker,mid)
    marker.type= visualization_msgs.msg.Marker.LINE_LIST
    marker.points= [geometry_msgs.msg.Point(*p) for x in x_list for p in point_on_arrow(x,scale[0])]
    self.viz_pub.publish(marker)
    return mid2

  #Visualize a cube at x.  If mid is None, the id is automatically assigned
  def AddCube(self, x, scale=[0.05,0.03,0.03], rgb=[1,1,1], alpha=1.0, mid=None):
    marker= self.GenMarker(x, scale, rgb, alpha)
    mid2= self.SetID(marker,mid)
    marker.type= visualization_msgs.msg.Marker.CUBE  # or CUBE, SPHERE, ARROW, CYLINDER
    self.viz_pub.publish(marker)
    return mid2

  #Visualize a list of cubes [[x,y,z]*N].  If mid is None, the id is automatically assigned
  def AddCubeList(self, points, scale=[0.05,0.03,0.03], rgb=[1,1,1], alpha=1.0, mid=None):
    x= [0,0,0, 0,0,0,1]
    marker= self.GenMarker(x, scale, rgb, alpha)
    mid2= self.SetID(marker,mid)
    marker.type= visualization_msgs.msg.Marker.CUBE_LIST
    marker.points= [geometry_msgs.msg.Point(*p[:3]) for p in points]
    self.viz_pub.publish(marker)
    return mid2

  #Visualize a sphere at p=[x,y,z].  If mid is None, the id is automatically assigned
  def AddSphere(self, p, scale=[0.05,0.05,0.05], rgb=[1,1,1], alpha=1.0, mid=None):
    if len(p)==3:
      x= list(p)+[0,0,0,1]
    else:
      x= p
    marker= self.GenMarker(x, scale, rgb, alpha)
    mid2= self.SetID(marker,mid)
    marker.type= visualization_msgs.msg.Marker.SPHERE  # or CUBE, SPHERE, ARROW, CYLINDER
    self.viz_pub.publish(marker)
    return mid2

  #Visualize a list of spheres [[x,y,z]*N].  If mid is None, the id is automatically assigned
  def AddSphereList(self, points, scale=[0.05,0.05,0.05], rgb=[1,1,1], alpha=1.0, mid=None):
    x= [0,0,0, 0,0,0,1]
    marker= self.GenMarker(x, scale, rgb, alpha)
    mid2= self.SetID(marker,mid)
    marker.type= visualization_msgs.msg.Marker.SPHERE_LIST
    marker.points= [geometry_msgs.msg.Point(*p[:3]) for p in points]
    self.viz_pub.publish(marker)
    return mid2

  #Visualize a cylinder whose end points are p1 and p2.  If mid is None, the id is automatically assigned
  def AddCylinder(self, p1, p2, diameter, rgb=[1,1,1], alpha=1.0, mid=None):
    x= XFromP1P2(p1, p2, ax='z', r=0.5)
    length= la.norm(Vec(p2)-Vec(p1))

    scale= [diameter,diameter,length]
    marker= self.GenMarker(x, scale, rgb, alpha)
    mid2= self.SetID(marker,mid)
    marker.type= visualization_msgs.msg.Marker.CYLINDER  # or CUBE, SPHERE, ARROW, CYLINDER
    self.viz_pub.publish(marker)
    return mid2

  #Visualize a cylinder at x along its axis ('x','y','z').  If mid is None, the id is automatically assigned
  def AddCylinderX(self, x, axis, diameter, l1, l2, rgb=[1,1,1], alpha=1.0, mid=None):
    e= RotToExyz(QToRot(x[3:]))[{'x':0,'y':1,'z':2}[axis]]
    p1= x[:3]+l1*e
    p2= x[:3]+l2*e
    return self.AddCylinder(p1,p2, diameter, rgb=rgb, alpha=alpha, mid=mid)

  #Visualize a points [[x,y,z]*N].  If mid is None, the id is automatically assigned
  def AddPoints(self, points, scale=[0.03,0.03], rgb=[1,1,1], alpha=1.0, mid=None):
    x= [0,0,0, 0,0,0,1]
    marker= self.GenMarker(x, list(scale)+[0.0], rgb, alpha)
    mid2= self.SetID(marker,mid)
    marker.type= visualization_msgs.msg.Marker.POINTS
    marker.points= [geometry_msgs.msg.Point(*p[:3]) for p in points]
    self.viz_pub.publish(marker)
    return mid2

  #Visualize a coordinate system at x with arrows.  If mid is None, the id is automatically assigned
  def AddCoord(self, x, scale=[0.05,0.002], alpha=1.0, mid=None):
    scale= [scale[0],scale[1],scale[1]]
    p,R= XToPosRot(x)
    Ry= np.array([R[:,1],R[:,2],R[:,0]]).T
    Rz= np.array([R[:,2],R[:,0],R[:,1]]).T
    mid= self.AddArrow(x, scale=scale, rgb=self.ICol(0), alpha=alpha, mid=mid)
    mid= self.AddArrow(PosRotToX(p,Ry), scale=scale, rgb=self.ICol(1), alpha=alpha, mid=mid)
    mid= self.AddArrow(PosRotToX(p,Rz), scale=scale, rgb=self.ICol(2), alpha=alpha, mid=mid)
    return mid

  #Visualize a coordinate system at x with cylinders.  If mid is None, the id is automatically assigned
  def AddCoordC(self, x, scale=[0.05,0.002], alpha=1.0, mid=None):
    scale= [scale[0],scale[1],scale[1]]
    p,R= XToPosRot(x)
    Ry= np.array([R[:,1],R[:,2],R[:,0]]).T
    Rz= np.array([R[:,2],R[:,0],R[:,1]]).T
    mid= self.AddCylinderX(x, 'x', scale[1], 0, scale[0], rgb=self.ICol(0), alpha=alpha, mid=mid)
    mid= self.AddCylinderX(x, 'y', scale[1], 0, scale[0], rgb=self.ICol(1), alpha=alpha, mid=mid)
    mid= self.AddCylinderX(x, 'z', scale[1], 0, scale[0], rgb=self.ICol(2), alpha=alpha, mid=mid)
    return mid

  #Visualize a polygon [[x,y,z]*N].  If mid is None, the id is automatically assigned
  def AddPolygon(self, points, scale=[0.02], rgb=[1,1,1], alpha=1.0, mid=None):
    x= [0,0,0, 0,0,0,1]
    marker= self.GenMarker(x, list(scale)+[0.0,0.0], rgb, alpha)
    mid2= self.SetID(marker,mid)
    marker.type= visualization_msgs.msg.Marker.LINE_STRIP
    marker.points= [geometry_msgs.msg.Point(*p[:3]) for p in points]
    self.viz_pub.publish(marker)
    return mid2

  #Visualize a list of lines [[x,y,z]*N] (2i-th and (2i+1)-th points are pair).  If mid is None, the id is automatically assigned
  def AddLineList(self, points, scale=[0.02], rgb=[1,1,1], alpha=1.0, mid=None):
    x= [0,0,0, 0,0,0,1]
    marker= self.GenMarker(x, list(scale)+[0.0,0.0], rgb, alpha)
    mid2= self.SetID(marker,mid)
    marker.type= visualization_msgs.msg.Marker.LINE_LIST
    marker.points= [geometry_msgs.msg.Point(*p[:3]) for p in points]
    self.viz_pub.publish(marker)
    return mid2

  #Visualize a text.  If mid is None, the id is automatically assigned
  def AddText(self, p, text, scale=[0.02], rgb=[1,1,1], alpha=1.0, mid=None):
    if len(p)==3:
      x= list(p)+[0,0,0,1]
    else:
      x= p
    marker= self.GenMarker(x, [0.0,0.0]+list(scale), rgb, alpha)
    mid2= self.SetID(marker,mid)
    marker.type= visualization_msgs.msg.Marker.TEXT_VIEW_FACING
    marker.text= text
    self.viz_pub.publish(marker)
    return mid2

  #Visualize contacts which should be an moveit_msgs/ContactInformation[] [DEPRECATED:arm_navigation_msgs/ContactInformation[]]
  def AddContacts(self, contacts, with_normal=False, scale=[0.01], rgb=[1,1,0], alpha=0.7, mid=None):
    if len(contacts)==0:  return self.curr_id
    cscale= scale*3
    self.viz_frame= contacts[0].header.frame_id
    for c in contacts:
      p= [c.position.x, c.position.y, c.position.z]
      mid= self.AddSphere(p+[0.,0.,0.,1.], scale=cscale, rgb=rgb, alpha=alpha, mid=mid)
      if with_normal:
        x= XFromP1P2(p, Vec(p)+Vec([c.normal.x, c.normal.y, c.normal.z]), ax='x', r=0.0)
        ascale= [c.depth,0.2*scale[0],0.2*scale[0]]
        mid= self.AddArrow(x, scale=ascale, rgb=rgb, alpha=alpha, mid=mid)
    return mid

#Visualize contacts which should be an moveit_msgs/ContactInformation[] [DEPRECATED:arm_navigation_msgs/ContactInformation[]]
#NOTE: not efficient, for debug
def VisualizeContacts(contacts, with_normal=False, pt_size=0.01, ns='visualizer_contacts', dt=rospy.Duration(5.0)):
  viz= TSimpleVisualizer(dt, name_space=ns)
  viz.AddContacts(contacts, with_normal=with_normal, scale=[pt_size])
