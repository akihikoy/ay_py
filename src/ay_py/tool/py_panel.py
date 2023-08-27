#!/usr/bin/python
# -*- coding: utf-8 -*-
#\file    simple_panel1.py
#\brief   Qt-based simple panel designer.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Apr.14, 2021

import os, sys, copy, math

if 'PYQT_VERSION' not in os.environ:
  os.environ['PYQT_VERSION']= '5'
if str(os.environ['PYQT_VERSION'])=='4':
  from PyQt4 import QtCore,QtGui
elif str(os.environ['PYQT_VERSION'])=='5':
  from PyQt5 import QtCore,QtWidgets
  import PyQt5.QtGui as PyQt5QtGui
  QtGui= QtWidgets
  for component in ('QFont', 'QFontMetrics', 'QIntValidator', 'QDoubleValidator', 'QPalette', 'QColor', 'QLinearGradient', 'QPainter'):
    setattr(QtGui,component, getattr(PyQt5QtGui,component))
else:
  raise Exception('Failed to import the requested version of PyQt:',os.environ['PYQT_VERSION'])

try:
  import roslib
  roslib.load_manifest('rviz')
  import rviz
except:
  print 'Failed to import rviz'


def MergeDict(d_base, d_new):
  for k_new,v_new in d_new.iteritems():
    if k_new in d_base and (type(v_new)==dict and type(d_base[k_new])==dict):
      MergeDict(d_base[k_new], v_new)
    else:
      d_base[k_new]= v_new
  return d_base  #NOTE: d_base is overwritten. Returning it is for the convenience.

def MergeDict2(d_base, *d_new):
  for d in d_new:
    MergeDict(d_base, d)
  return d_base  #NOTE: d_base is overwritten. Returning it is for the convenience.

def AskYesNoDialog(parent, message, title='Inquiry'):
  return QtGui.QMessageBox.question(parent, title, message, QtGui.QMessageBox.Yes, QtGui.QMessageBox.No) == QtGui.QMessageBox.Yes

class TRadioBox(QtGui.QWidget):
  def __init__(self, *args, **kwargs):
    super(TRadioBox, self).__init__(*args, **kwargs)

  #layout: 'h'(horizontal), 'v'(vertical), 'grid'
  def Construct(self, layout, options, index, onclick):
    self.layout= None
    if layout=='h':  self.layout= QtGui.QHBoxLayout()
    elif layout=='v':  self.layout= QtGui.QVBoxLayout()
    elif layout=='grid':  raise Exception('Not implemented yet')
    else:  raise Exception('Invalid layout option:',layout)

    self.group= QtGui.QButtonGroup()
    self.radbtns= []
    for idx,option in enumerate(options):
      radbtn= QtGui.QRadioButton(option, self)
      radbtn.setCheckable(True)
      if idx==index:  radbtn.setChecked(True)
      radbtn.setFocusPolicy(QtCore.Qt.NoFocus)
      #radbtn.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Fixed)
      radbtn.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
      #radbtn.move(10, 60)
      if onclick:  radbtn.clicked.connect(onclick)
      self.layout.addWidget(radbtn)
      self.group.addButton(radbtn)
      self.radbtns.append(radbtn)
    self.setLayout(self.layout)

  def setFont(self, f):
    for radbtn in self.radbtns:
      radbtn.setFont(f)
    h= f.pointSize()*1.5
    self.setStyleSheet('QRadioButton::indicator {{width:{0}px;height:{0}px;}};'.format(h))

class TSlider(QtGui.QWidget):
  def __init__(self, *args, **kwargs):
    super(TSlider, self).__init__(*args, **kwargs)

  def convert_from(self, slider_value):
    return min(self.range_step[1], self.range_step[0] + self.range_step[2]*slider_value)

  def convert_to(self, value):
    return max(0,min(self.slider_max,(value-self.range_step[0])/self.range_step[2]))

  def value(self):
    return self.convert_from(self.slider.value())

  def setValue(self, value):
    slider_value= self.convert_to(value)
    self.slider.setValue(slider_value)
    self.setLabel(value)

  def setLabel(self, value):
    self.label.setText(str(value).rjust(len(str(self.range_step[1]))))

  #style: 0:Default, 1:Variable handle size.
  def Construct(self, range_step, n_labels, slider_style, onvaluechange):
    self.range_step= range_step
    self.slider_max= (self.range_step[1]-self.range_step[0])/self.range_step[2]
    self.slider_style= slider_style

    self.layout= QtGui.QGridLayout()

    vspacer1= QtGui.QSpacerItem(1, 1, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
    self.layout.addItem(vspacer1, 0, 0, 1, n_labels+1)

    self.slider= QtGui.QSlider(QtCore.Qt.Horizontal, self)
    #self.slider.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
    self.slider.setTickPosition(QtGui.QSlider.TicksBothSides)
    self.slider.setRange(0, self.slider_max)
    self.slider.setTickInterval(1)
    self.slider.setSingleStep(1)
    self.slider.setPageStep(1)
    #self.slider.move(10, 60)
    #self.slider.resize(100, 20)
    self.slider.valueChanged.connect(lambda *args,**kwargs:(self.setLabel(self.value()), onvaluechange(*args,**kwargs) if onvaluechange else None)[-1])

    self.layout.addWidget(self.slider, 1, 0, 1, n_labels)

    self.label= QtGui.QLabel('0',self)
    self.layout.addWidget(self.label, 1, n_labels, 1, 1, QtCore.Qt.AlignLeft)

    #hspacer1= QtGui.QSpacerItem(1, 1, QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.MinimumExpanding)
    #self.layout.addItem(hspacer1, 1, n_labels+1)

    self.tick_labels= []
    if n_labels>1:
      #tick_font= QtGui.QFont(self.label.font().family(), self.label.font().pointSize()*0.6)
      label_step= (range_step[1]-range_step[0])/(n_labels-1)
      for i_label in range(n_labels):
        label= str(range_step[0]+i_label*label_step)
        tick_label= QtGui.QLabel(label,self)
        #tick_label.setFont(tick_font)
        if i_label<(n_labels-1)/2:  align= QtCore.Qt.AlignLeft
        elif i_label==(n_labels-1)/2:  align= QtCore.Qt.AlignCenter
        else:  align= QtCore.Qt.AlignRight
        self.layout.addWidget(tick_label, 2, i_label, 1, 1, align)
        self.tick_labels.append(tick_label)

    vspacer2= QtGui.QSpacerItem(1, 1, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
    self.layout.addItem(vspacer2, 3, 0, 1, n_labels+1)
    self.setValue(range_step[0])
    self.setLayout(self.layout)
    self.setStyleForFont(self.label.font())

  def setStyleForFont(self, f):
    tick_f= QtGui.QFont(f.family(), f.pointSize()*0.6)
    for tick_label in self.tick_labels:
      tick_label.setFont(tick_f)
    if self.slider_style==0:
      self.slider.setStyleSheet('')
    elif self.slider_style==1:
      h0= f.pointSize()*2
      h1= h0+8
      self.slider.setStyleSheet('''
        QSlider {{
            height: {1}px;
        }}
        QSlider::groove:horizontal {{
            background: transparent;
            border: 2px solid #aaa;
            height: {0}px;
            margin: 0 0;
        }}
        QSlider::handle:horizontal {{
            background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #b4b4b4, stop:1 #8f8f8f);
            border: 1px solid #5c5c5c;
            width: {0}px;
            margin: 0 0;
            border-radius: 3px;
        }}
        '''.format(h0,h1))

  def setFont(self, f):
    self.label.setFont(f)
    self.setStyleForFont(f)

class TPrimitivePainter(QtGui.QWidget):
  def __init__(self, shape, margin, color, parent=None):
    super(TPrimitivePainter, self).__init__(parent)

    self.shape= shape
    self.margin= margin  #(horizontal_margin(ratio),vertical_margin(ratio))
    self.color= color
    self.min_size= 100
    self.max_size= 400
    self.width_height_ratio= 1.2
    self.draw_bevel= True
    self.setBackgroundRole(QtGui.QPalette.Base)
    #self.setAutoFillBackground(True)

    size_policy= QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Preferred)
    size_policy.setHeightForWidth(True)
    self.setSizePolicy(size_policy)

  def setPaintColor(self, rgb):
    self.color= rgb
    self.update()

  def setRandomColor(self):
    self.setPaintColor([255*random.random(),255*random.random(),255*random.random()])

  def setShape(self, shape):
    self.update()
    self.shape= shape

  def setMargin(self, margin):
    self.update()
    self.margin= margin

  def minimumSizeHint(self):
    return QtCore.QSize(self.min_size, self.heightForWidth(self.min_size))

  def sizeHint(self):
    return QtCore.QSize(self.max_size, self.heightForWidth(self.max_size))

  def heightForWidth(self, width):
    return width*self.width_height_ratio

  def paintEvent(self, event):
    col1= QtGui.QColor(*self.color)
    col2= QtGui.QColor(0.6*self.color[0], 0.6*self.color[1], 0.6*self.color[2])
    linear_gradient= QtGui.QLinearGradient(0, 0, self.width(), self.height())
    linear_gradient.setColorAt(0.0, QtCore.Qt.white)
    linear_gradient.setColorAt(0.2, col1)
    linear_gradient.setColorAt(0.8, col2)
    linear_gradient.setColorAt(1.0, QtCore.Qt.black)

    painter= QtGui.QPainter(self)
    painter.setPen(QtCore.Qt.SolidLine)
    painter.setBrush(linear_gradient)
    painter.setRenderHint(QtGui.QPainter.Antialiasing)

    painter.save()
    painter.translate(0, 0)

    if self.shape in ('ellipse','rect'):
      rect= QtCore.QRect(self.width()*self.margin[0], self.height()*self.margin[1], self.width()*(1.0-2.0*self.margin[0]), self.height()*(1.0-2.0*self.margin[1]))
    elif self.shape in ('circle','square'):
      l= min(self.width()*(1.0-2.0*self.margin[0]), self.height()*(1.0-2.0*self.margin[1]))
      rect= QtCore.QRect((self.width()-l)/2, (self.height()-l)/2, l, l)

    if self.shape in ('ellipse','circle'):
      painter.drawEllipse(rect)
    elif self.shape in ('rect','square'):
      painter.drawRect(rect)

    painter.restore()

    if self.draw_bevel:
      painter.setPen(self.palette().dark().color())
      painter.setBrush(QtCore.Qt.NoBrush)
      painter.drawRect(QtCore.QRect(0, 0, self.width() - 1, self.height() - 1))

class TStatusGrid(QtGui.QWidget):
  def __init__(self, *args, **kwargs):
    super(TStatusGrid, self).__init__(*args, **kwargs)
    #self.setMinimumSize(100, 100)
    self.setBackgroundRole(QtGui.QPalette.Base)

  '''Construct the status grid widget.
    list_status: List of status items (dictionaries) (this class also modified the content).
    direction: Direction to list in the grid ('vertical' or 'horizontal').
    shape: Shape of 'color' item ('circle', 'square').
    rows,columns: Number of rows and columns. At least one of them should be specified.
  '''
  def Construct(self, list_status, direction='vertical', shape='circle', margin=(0.05,0.05), rows=None, columns=3):
    # Grid layout
    self.grid= QtGui.QGridLayout()
    self.grid.setContentsMargins(1, 1, 1, 1)

    # Define a list of status items.
    self.list_status= list_status
    self.dict_status= {item['label']: item for item in self.list_status}

    if rows is None and columns is None:  raise Exception('TStatusGrid: one of rows or columns should be None.')
    if rows is not None and columns is not None: assert(len(self.list_status)<rows*columns)
    if rows is None:  rows= int(math.ceil(float(len(self.list_status))/columns))
    if columns is None:  columns= int(math.ceil(float(len(self.list_status))/rows))
    self.rows,self.columns= rows,columns

    self.direction= direction
    self.shape= shape
    self.margin= margin

    #self.default_font_size= 10
    if self.direction=='vertical':
      for c in range(self.columns):
        for r in range(self.rows):
          idx= c*self.rows + r
          if idx>=len(self.list_status):  break
          self.AddWidgetsForItem(self.list_status[idx], r, c)
    elif self.direction=='horizontal':
      for r in range(self.rows):
        for c in range(self.columns):
          idx= r*self.columns + c
          if idx>=len(self.list_status):  break
          self.AddWidgetsForItem(self.list_status[idx], r, c)
    else:
      raise Exception('TStatusGrid: invalid direction:',self.direction)

    self.setLayout(self.grid)

  def AddWidgetsForItem(self, item, r, c):
    if item['type']=='color':
      color1= TPrimitivePainter(self.shape, self.margin, self.StateColor(item), self)
      color1.setSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Minimum)
      color1.min_size= 20
      color1.width_height_ratio= 1.0
      color1.draw_bevel= False
      #color1.draw_bevel= True  #DEBUG
      item['w_color']= color1

      label1= QtGui.QLabel(item['label'], self)
      label1.setSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Minimum)
      label1.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
      #label1.font_size= (8,28)
      #label1.setFont(QtGui.QFont('', self.default_font_size))
      #label1.resizeEvent= lambda event,obj=label1: self.ResizeText(obj,event)
      item['w_label']= label1
      #label1.setStyleSheet("background-color: lightgreen")  #DEBUG

      rowsize,colsize= 1,1
      self.grid.addWidget(color1, r, 2*c, rowsize, colsize)
      self.grid.addWidget(label1, r, 2*c+1, rowsize, colsize)

    elif item['type']=='text':
      label1= QtGui.QLabel(self.StateText(item), self)
      label1.setSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Minimum)
      label1.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
      #label1.font_size= (8,28)
      #label1.setFont(QtGui.QFont('', self.default_font_size))
      #label1.resizeEvent= lambda event,obj=label1: self.ResizeText(obj,event)
      item['w_label']= label1
      #label1.setStyleSheet("background-color: lightgreen")  #DEBUG

      rowsize,colsize= 1,2
      self.grid.addWidget(label1, r, 2*c, rowsize, colsize)

  def StateColor(self, item):
    state= item['state']
    col_map= {'green': (0,255,0), 'yellow': (255,255,0), 'red': (255,0,0)}
    if state in col_map:  return col_map[state]
    return (128,128,128)

  def StateText(self, item):
    state= item['state']
    return '{}: {}'.format(item['label'],item['state'])

  def UpdateStatus(self, label, state):
    item= self.dict_status[label]
    item['state']= state
    if item['type']=='color':
      item['w_color'].color= self.StateColor(item)
      item['w_color'].update()
    elif item['type']=='text':
      item['w_label'].setText(self.StateText(item))

  def setFont(self, f):
    for item in self.list_status:
      if 'w_label' in item:  item['w_label'].setFont(f)
    text_height= QtGui.QFontMetrics(f).boundingRect('H').height()
    for item in self.list_status:
      if 'w_color' in item:
        item['w_color'].min_size= text_height
        item['w_color'].max_size= text_height
        item['w_color'].update()

class TVirtualJoyStick(QtGui.QWidget):
  onstickmoved= QtCore.pyqtSignal(list)
  def __init__(self, parent=None):
    super(TVirtualJoyStick, self).__init__(parent)
    #self.setMinimumSize(100, 100)
    self.setBackgroundRole(QtGui.QPalette.Base)

    self.kind= 'circle'  #circle, ellipse, hbox, vbox
    self.bg_color= [200]*3  #Color of movable range.
    self.bg_height= 0.3  #Display height of movable range (effective with kind==hbox,vbox).
    self.resetStickPos()
    self.stick_grabbed= False
    self.stick_size= 0.6  #Stick size per width/height of movable range.
    self.stick_color= [128, 128, 255]

  #kind: 2d-joy: circle, ellipse, 1d-joy: hbox, vbox
  def setKind(self, kind):
    self.kind= kind

  #Set color of movable range.
  def setBGColor(self, bg_color):
    self.bg_color= bg_color

  #Set display height of movable range (effective with kind==hbox,vbox).
  def setBGHeight(self, bg_height):
    self.bg_height= bg_height

  #Set stick size per width/height of movable range.
  def setStickSize(self, stick_size):
    self.stick_size= stick_size

  def setStickColor(self, stick_color):
    self.stick_color= stick_color

  def resetStickPos(self):
    #Normalized stick position:
    self.stick_pos= QtCore.QLineF(QtCore.QPointF(0,0),QtCore.QPointF(0,0))

  def center(self):
    return QtCore.QPointF(self.width()/2, self.height()/2)

  def minimumSizeHint(self):
    #return QtCore.QSize(100, self.heightForWidth(100))
    return QtCore.QSize(20, 20)

  #def sizeHint(self):
    #return QtCore.QSize(400, self.heightForWidth(400))

  #def heightForWidth(self, width):
    #return width*1.2

  #Return the current joystick position.
  def position(self):
    #return self.stick_pos.p2(), self.stick_pos.length(), self.stick_pos.angle()
    if self.kind in ('ellipse','circle'):
      return [self.stick_pos.p2().x(), self.stick_pos.p2().y()]
    elif self.kind=='hbox':
      return [self.stick_pos.p2().x()]
    elif self.kind=='vbox':
      return [self.stick_pos.p2().y()]

  def getGradient(self, color, bounds, reverse=False):
    col0= QtGui.QColor(min(255,1.5*color[0]), min(255,1.5*color[1]), min(255,1.5*color[2]))
    col1= QtGui.QColor(*color)
    col2= QtGui.QColor(0.6*color[0], 0.6*color[1], 0.6*color[2])
    col3= QtGui.QColor(0.2*color[0], 0.2*color[1], 0.2*color[2])
    positions= [0.0,0.2,0.8,1.0]
    #colors= [QtCore.Qt.white,col1,col2,QtCore.Qt.black]
    colors= [col0,col1,col2,col3]
    linear_gradient= QtGui.QLinearGradient(bounds.topLeft(), bounds.bottomRight())
    for pos,col in zip(positions, colors if not reverse else reversed(colors)):
      linear_gradient.setColorAt(pos, col)
    return linear_gradient

  def getMovableRange(self):
    if self.kind=='ellipse':
      return self.width(), self.height()
    elif self.kind=='circle':
      l= min(self.width(), self.height())
      return l,l
    elif self.kind=='hbox':
      return self.width(), self.width()*self.bg_height
    elif self.kind=='vbox':
      return self.height()*self.bg_height, self.height()

  def getStickSize(self):
    mw,mh= self.getMovableRange()
    if self.kind in ('ellipse','circle'):
      return mw*self.stick_size, mw*self.stick_size
    elif self.kind=='hbox':
      return mw*self.stick_size, mh
    elif self.kind=='vbox':
      return mw, mh*self.stick_size

  def getStickRange(self):
    mw,mh= self.getMovableRange()
    sw,sh= self.getStickSize()
    if self.kind in ('ellipse','circle'):
      return mw/2.0-sw/2.0, mh/2.0-sh/2.0
    elif self.kind=='hbox':
      return mw/2.0-sw/2.0, 0
    elif self.kind=='vbox':
      return 0, mh/2.0-sh/2.0

  def paintEvent(self, event):
    painter= QtGui.QPainter(self)
    painter.setPen(QtCore.Qt.SolidLine)
    painter.setRenderHint(QtGui.QPainter.Antialiasing)

    #Drawing movable range.
    mw,mh= self.getMovableRange()
    bounds= QtCore.QRectF(-mw/2+1, -mh/2+1, mw-2, mh-2).translated(self.center())
    painter.setBrush(self.getGradient(self.bg_color, bounds, reverse=True))
    if self.kind in ('ellipse','circle'):
      painter.drawEllipse(bounds)
    elif self.kind in ('hbox','vbox'):
      painter.drawRect(bounds)

    #Drawing stick.
    sx,sy= self.getStickRange()
    stick_displacement= self.center()+QtCore.QPointF(self.stick_pos.p2().x()*sx,-self.stick_pos.p2().y()*sy)
    sw,sh= self.getStickSize()
    bounds= QtCore.QRectF(-sw/2.0+1, -sh/2.0+1, sw-2, sh-2).translated(stick_displacement)
    painter.setBrush(self.getGradient(self.stick_color, bounds))
    #painter.setBrush(QtCore.Qt.black)
    if self.kind in ('ellipse','circle'):
      painter.drawEllipse(bounds)
    elif self.kind in ('hbox','vbox'):
      painter.drawRect(bounds)

  def mousePressEvent(self, event):
    dp= QtCore.QLineF(self.center(), event.pos())
    sw,sh= self.getStickSize()
    if self.kind in ('ellipse','circle'):
      self.stick_grabbed= dp.length()<sw
    elif self.kind in ('hbox','vbox'):
      self.stick_grabbed= abs(dp.dx())<sw/2.0 and abs(dp.dy())<sh/2.0
    if self.stick_grabbed:  self.pos_grabbed= event.pos()
    self.onstickmoved.emit(self.position())
    return super(TVirtualJoyStick, self).mousePressEvent(event)

  def mouseReleaseEvent(self, event):
    self.stick_grabbed= False
    self.resetStickPos()
    self.update()
    self.onstickmoved.emit(self.position())

  def mouseMoveEvent(self, event):
    if self.stick_grabbed:
      dp= QtCore.QPointF(event.pos()-self.pos_grabbed)
      sx,sy= self.getStickRange()
      self.stick_pos= QtCore.QLineF(QtCore.QPointF(0,0), QtCore.QPointF(dp.x()/sx if sx>0 else 0,-dp.y()/sy if sy>0 else 0))
      if self.stick_pos.length()>1.0:  self.stick_pos.setLength(1.0)
      self.update()
      self.onstickmoved.emit(self.position())
    #print self.position()

if 'rviz' in sys.modules:
  class TRVizUtil(rviz.VisualizationFrame):
    def __init__(self):
      super(TRVizUtil, self).__init__()

      self.reader= rviz.YamlConfigReader()
      self.config= rviz.Config()
      self.setSplashPath('')
      self.setMenuBar(None)
      self.setStatusBar(None)
      self.config_file= os.environ['HOME']+'/.rviz/default.rviz'
      self.is_initialized= False

    def setConfigFile(self, config_file):
      self.config_file= config_file

    def setup(self):
      if not self.is_initialized:
        self.initialize()
        self.is_initialized= True
        self.reader.readFile(self.config, self.config_file)  #NOTE: Reloading causes abort.
        self.load(self.config)
      self.setMenuBar(None)
      self.setStatusBar(None)
      self.setHideButtonVisibility(False)


class TSimplePanel(QtGui.QWidget):
  def __init__(self, title, size=(800,400), font_height_scale=100.0):
    QtGui.QWidget.__init__(self)
    self.close_callback= None
    self.font_height_scale= font_height_scale
    self.alignments= {
      '': QtCore.Qt.Alignment(),
      'left': QtCore.Qt.AlignLeft,
      'center': QtCore.Qt.AlignCenter,
      'right': QtCore.Qt.AlignRight,
      }
    self.size_policies= {
      'fixed': QtGui.QSizePolicy.Fixed,
      'minimum': QtGui.QSizePolicy.Minimum,
      'maximum': QtGui.QSizePolicy.Maximum,
      'preferred': QtGui.QSizePolicy.Preferred,
      'expanding': QtGui.QSizePolicy.Expanding,
      'minimum_expanding': QtGui.QSizePolicy.MinimumExpanding,
      'ignored': QtGui.QSizePolicy.Ignored,
      }
    self.param_common={
      'enabled': True,
      'font_size_range': (10,30),
      'minimum_size': None,  #None, or (horizontal_minimum_size, vertical_minimum_size)
      'maximum_size': None,  #None, or (horizontal_maximum_size, vertical_maximum_size)
      'size_policy': ('expanding', 'expanding'),  #(horizontal_size_policy, vertical_size_policy), or size_policy
      }
    self.widget_generator= {
      #'duplicate': self.DuplicateWidget,
      'button': self.AddButton,
      'buttonchk': self.AddButtonCheckable,
      'checkbox': self.AddCheckBox,
      'combobox': self.AddComboBox,
      'lineedit': self.AddLineEdit,
      'radiobox': self.AddRadioBox,
      'sliderh': self.AddSliderH,
      'spacer': self.AddSpacer,
      'label': self.AddLabel,
      'textedit': self.AddTextEdit,
      'primitive_painer': self.AddPrimitivePainter,
      'status_grid': self.AddStatusGrid,
      'virtual_joystick': self.AddVirtualJoyStick,
      'rviz': self.AddRViz,
      }
    self.resize(*size)  #window size
    self.setWindowTitle(title)
    self.widgets_in= {}
    self.widgets= {}
    self.layout_in= None
    self.layouts= {}

  #Add widgets from widget description dict.
  def AddWidgets(self, widgets):
    duplicate_req= []
    for name,(w_type, w_param) in widgets.iteritems():
      if name in self.widgets_in:
        raise Exception('TSimplePanel.AddWidgets: widget already exists: {0}'.format(name))
      if w_type=='duplicate':
        duplicate_req.append((name, w_param))
      else:
        self.widgets_in[name]= (w_type, w_param)
    for name,w_param in duplicate_req:
      self.widgets_in[name]= self.widgets_in[w_param]
    for name in widgets.iterkeys():
      w_type, w_param= self.widgets_in[name]
      self.widgets[name]= self.widget_generator[w_type](w_param)

  def Construct(self, layout):
    self.layout_in= layout
    self.setLayout(self.AddLayouts(self.layout_in))

    self.ResizeText(None)
    self.resizeEvent= self.ResizeText

    self.show()

  def ApplyCommonWidgetConfig(self, widget, param):
    if hasattr(widget,'setFont') and param['font_size_range'] is not None:
      widget.font_size_range= param['font_size_range']
      widget.setFont(QtGui.QFont('', widget.font_size_range[0]))
    if param['enabled'] is not None:  widget.setEnabled(param['enabled'])
    if param['minimum_size'] is not None:
      if param['minimum_size'][0]:  widget.setMinimumWidth(param['minimum_size'][0])
      if param['minimum_size'][1]:  widget.setMinimumHeight(param['minimum_size'][1])
    if param['maximum_size'] is not None:
      if param['maximum_size'][0]:  widget.setMaximumWidth(param['maximum_size'][0])
      if param['maximum_size'][1]:  widget.setMaximumHeight(param['maximum_size'][1])
    if param['size_policy'] is not None:
      if isinstance(param['size_policy'],str):  widget.setSizePolicy(self.size_policies[param['size_policy']])
      else:  widget.setSizePolicy(self.size_policies[param['size_policy'][0]], self.size_policies[param['size_policy'][1]])

  def ResizeTextOfObj(self, obj, font_size_range, size):
    font_size= min(font_size_range[1],max(font_size_range[0],int(size*font_size_range[0])))
    f= QtGui.QFont('', font_size)
    if isinstance(obj,QtGui.QLineEdit):
      #obj.resize(obj.sizeHint().width(),obj.height())
      text= obj.text()
      rect= obj.fontMetrics().boundingRect(text if text!='' else '0')
      obj.setMinimumWidth(rect.width()+15)
      obj.setMinimumHeight(rect.height()+15)
    #elif isinstance(obj,QtGui.QComboBox):
      #obj.resize(obj.sizeHint().width(),obj.height())
    if hasattr(obj,'setFont'):  obj.setFont(f)

  def ResizeText(self, event):
    s= self.rect().height()/self.font_height_scale
    for name,obj in self.widgets.iteritems():
      if not hasattr(obj,'font_size_range'):  continue
      self.ResizeTextOfObj(obj, obj.font_size_range, s)

  ##Duplicate a widget (regenerate a widget with the same parameter).
  ##NOTE: w_param is a widget name to be copied.
  #def DuplicateWidget(self, w_param):
    #src_widget_name= w_param
    #w_type, w_param= self.widgets_in[src_widget_name]
    #widget= self.widget_generator[w_type](w_param)
    #return widget

  def AddButton(self, w_param):
    param= MergeDict2(copy.deepcopy(self.param_common), {
      'text': 'button',
      'onclick': None,
      }, w_param)
    btn= QtGui.QPushButton(param['text'], self)
    btn.setFocusPolicy(QtCore.Qt.NoFocus)
    #btn.setFlat(True)
    #btn.setToolTip('Click to make something happen')
    if param['onclick']:  btn.clicked.connect(lambda checked=False,btn=btn: param['onclick'](self,btn))
    #btn.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
    btn.resize(btn.sizeHint())
    #btn.move(100, 150)
    self.ApplyCommonWidgetConfig(btn, param)
    return btn

  def AddButtonCheckable(self, w_param):
    param= MergeDict2(copy.deepcopy(self.param_common), {
      'text': ('button','button'),
      'checked': False,
      'onclick': None,
      'ontoggled': None,
      }, w_param)
    btn= QtGui.QPushButton(param['text'][0], self)
    btn.setFocusPolicy(QtCore.Qt.NoFocus)
    btn.setCheckable(True)
    btn.setChecked(param['checked'])
    if param['onclick']:  btn.clicked.connect(lambda checked=False,bnt=btn: (param['onclick'][0](self,btn) if param['onclick'][0] else None) if btn.isChecked() else (param['onclick'][1](self,btn) if param['onclick'][1] else None) )
    if param['ontoggled']:  btn.toggled.connect(lambda checked=False,bnt=btn: (param['ontoggled'][0](self,btn) if param['ontoggled'][0] else None, btn.setText(param['text'][1])) if btn.isChecked() else (param['ontoggled'][1](self,btn) if param['ontoggled'][1] else None, btn.setText(param['text'][0])) )
    else:  btn.toggled.connect(lambda checked=False,bnt=btn: btn.setText(param['text'][1]) if btn.isChecked() else btn.setText(param['text'][0]) )
    #btn.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
    btn.resize(btn.sizeHint())
    #btn.move(220, 100)
    self.ApplyCommonWidgetConfig(btn, param)
    return btn

  def AddCheckBox(self, w_param):
    param= MergeDict2(copy.deepcopy(self.param_common), {
      'text': 'checkbox',
      'checked': False,
      'onclick': None,
      }, w_param)
    chkbx= QtGui.QCheckBox(param['text'], self)
    chkbx.setChecked(param['checked'])
    chkbx.setFocusPolicy(QtCore.Qt.NoFocus)
    if param['onclick']:  chkbx.clicked.connect(lambda checked=False,chkbx=chkbx: param['onclick'](self,chkbx))
    self.ApplyCommonWidgetConfig(chkbx, param)
    return chkbx

  def AddComboBox(self, w_param):
    param= MergeDict2(copy.deepcopy(self.param_common), {
      'options': [],
      'index': 0,
      'size_adjust_policy': None,  #'all_contents','first_content','min_content'
      'onactivated': None,
      }, w_param)
    cmbbx= QtGui.QComboBox(self)
    cmbbx.setFocusPolicy(QtCore.Qt.NoFocus)
    for option in param['options']:
      cmbbx.addItem(option)
    if param['index'] is not None:  cmbbx.setCurrentIndex(param['index'])
    if param['size_adjust_policy'] is not None:
      policy= {'all_contents':  QtGui.QComboBox.AdjustToContents,
               'first_content': QtGui.QComboBox.AdjustToContentsOnFirstShow,
               'min_content':   QtGui.QComboBox.AdjustToMinimumContentsLengthWithIcon}[param['size_adjust_policy']]
      cmbbx.setSizeAdjustPolicy(policy)
    if param['onactivated']:  cmbbx.activated[str].connect(lambda _,cmbbx=cmbbx:param['onactivated'](self,cmbbx))
    #cmbbx.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
    cmbbx.resize(cmbbx.sizeHint())
    #cmbbx.move(10, 60)
    self.ApplyCommonWidgetConfig(cmbbx, param)
    return cmbbx

  def AddLineEdit(self, w_param):
    param= MergeDict2(copy.deepcopy(self.param_common), {
      'validator': None,  #'int'
      'size_policy': ('expanding', 'fixed'),
      }, w_param)
    edit= QtGui.QLineEdit(self)
    if param['validator']=='int':    edit.setValidator(QtGui.QIntValidator())
    if param['validator']=='float':  edit.setValidator(QtGui.QDoubleValidator())
    edit.setMinimumHeight(10)
    edit.setMinimumWidth(10)
    #edit.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Fixed)
    edit.resize(edit.sizeHint())
    #edit.move(10, 60)
    self.ApplyCommonWidgetConfig(edit, param)
    return edit

  def AddRadioBox(self, w_param):
    param= MergeDict2(copy.deepcopy(self.param_common), {
      'options': [],
      'index': None,
      'layout': 'h',  #h(horizontal),v(vertical),grid
      'onclick': None,
      }, w_param)
    radiobox= TRadioBox(self)
    if param['onclick']:  clicked= lambda _,radiobox=radiobox:param['onclick'](self,radiobox)
    else:  clicked= None
    radiobox.Construct(param['layout'], param['options'], index=param['index'], onclick=clicked)
    self.ApplyCommonWidgetConfig(radiobox, param)
    return radiobox

  def AddSliderH(self, w_param):
    param= MergeDict2(copy.deepcopy(self.param_common), {
      'range': (0,10,1),
      'value': 0,
      'n_labels': 3,
      'slider_style': 0,
      'onvaluechange': None,
      }, w_param)
    slider= TSlider(self)
    if param['onvaluechange']:  onvaluechange= lambda _,slider=slider:param['onvaluechange'](self,slider)
    else:  onvaluechange= None
    slider.Construct(param['range'], n_labels=param['n_labels'], slider_style=param['slider_style'], onvaluechange=onvaluechange)
    if param['value'] is not None:  slider.setValue(param['value'])
    self.ApplyCommonWidgetConfig(slider, param)
    return slider

  def AddSpacer(self, w_param):
    param= MergeDict2(copy.deepcopy(self.param_common), {
      'w': 1,
      'h': 1,
      'size_policy': ('expanding', 'expanding'),
      }, w_param)
    if isinstance(param['size_policy'],str):
      spacer= QtGui.QSpacerItem(param['w'], param['h'], self.size_policies[param['size_policy']], self.size_policies[param['size_policy']])
    else:
      spacer= QtGui.QSpacerItem(param['w'], param['h'], self.size_policies[param['size_policy'][0]], self.size_policies[param['size_policy'][1]])
    #self.ApplyCommonWidgetConfig(spacer, param)
    return spacer

  def AddLabel(self, w_param):
    param= MergeDict2(copy.deepcopy(self.param_common), {
      'text':'',
      'selectable_by_mouse':False,  #Text is selectable by mouse.
      }, w_param)
    label= QtGui.QLabel(param['text'], self)
    #label.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
    if param['selectable_by_mouse']:
      label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
    self.ApplyCommonWidgetConfig(label, param)
    return label

  def AddTextEdit(self, w_param):
    param= MergeDict2(copy.deepcopy(self.param_common), {
      'text':'',
      'read_only':False,
      }, w_param)
    text= QtGui.QTextEdit(self)
    text.setText(param['text'])
    text.setReadOnly(param['read_only'])
    #text.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Preferred)
    self.ApplyCommonWidgetConfig(text, param)
    return text

  def AddPrimitivePainter(self, w_param):
    param= MergeDict2(copy.deepcopy(self.param_common), {
      'shape':'rect',
      'margin':(0.05,0.05),
      'color':(0,0,255),
      'min_size': 100,
      'max_size': 400,
      'draw_bevel': True,
      }, w_param)
    primitive= TPrimitivePainter(param['shape'],param['margin'],param['color'],self)
    primitive.min_size= param['min_size']
    primitive.max_size= param['max_size']
    primitive.draw_bevel= param['draw_bevel']
    self.ApplyCommonWidgetConfig(primitive, param)
    return primitive

  def AddStatusGrid(self, w_param):
    param= MergeDict2(copy.deepcopy(self.param_common), {
      'list_status':[],  #List of status items (dictionaries) (this class also modified the content).
      'direction':'vertical',  #Direction to list in the grid ('vertical' or 'horizontal').
      'shape':'circle',  #Shape of 'color' item ('circle', 'square').
      'margin':(0.05,0.05),
      'rows':None,  #Number of rows and columns. At least one of them should be specified.
      'columns':3,
      }, w_param)
    statusgrid= TStatusGrid(self)
    statusgrid.Construct(**{key: param[key] for key in ('list_status', 'direction', 'shape', 'margin', 'rows', 'columns')})
    self.ApplyCommonWidgetConfig(statusgrid, param)
    return statusgrid

  def AddVirtualJoyStick(self, w_param):
    param= MergeDict2(copy.deepcopy(self.param_common), {
      'kind':'circle',  #kind: 2d-joy: circle, ellipse, 1d-joy: hbox, vbox
      'bg_color':[200,200,200],
      'bg_height':0.3,  #Display height of movable range (effective with kind==hbox,vbox).
      'stick_size':0.6,  #Stick size per width/height of movable range.
      'stick_color':[128,128,255],
      'onstickmoved':None,
      }, w_param)
    joy= TVirtualJoyStick(self)
    joy.setKind(param['kind'])
    joy.setBGColor(param['bg_color'])
    joy.setBGHeight(param['bg_height'])
    joy.setStickSize(param['stick_size'])
    joy.setStickColor(param['stick_color'])
    if param['onstickmoved']:  joy.onstickmoved.connect(lambda pos=None: param['onstickmoved'](self,joy))
    self.ApplyCommonWidgetConfig(joy, param)
    return joy

  def AddRViz(self, w_param):
    param= MergeDict2(copy.deepcopy(self.param_common), {
      'config':None,
      'setup':False,  #Setup rviz at the beginning.  roscore should be running.
      }, w_param)
    rviz= TRVizUtil()
    if param['config'] is not None:  rviz.setConfigFile(param['config'])
    if param['setup']:  rviz.setup()
    self.ApplyCommonWidgetConfig(rviz, param)
    return rviz

  def AddLayouts(self, layout):
    l_type,name,items= layout
    if name is None:
      for i in range(10000):
        name= l_type+str(i)
        if name not in self.layouts:  break
    self.layouts[name]= None

    layout= None
    if l_type in ('boxv','boxh'):
      layout= QtGui.QVBoxLayout() if l_type=='boxv' else QtGui.QHBoxLayout()
      for item in items:
        if isinstance(item,str):
          widget= self.widgets[item]
          if isinstance(widget,QtGui.QSpacerItem):
            layout.addSpacerItem(widget)
          else:
            layout.addWidget(widget)
        else:
          sublayout= self.AddLayouts(item)
          layout.addLayout(sublayout)
    elif l_type=='grid':
      layout= QtGui.QGridLayout()
      for item_loc in items:
        if len(item_loc)==3:
          (item,r,c),rs,cs,align= item_loc,1,1,self.alignments['']
        elif len(item_loc)==4:
          (item,r,c),rs,cs,align= item_loc[:3],1,1,self.alignments[item_loc[3]]
        elif len(item_loc)==5:
          (item,r,c,rs,cs),align= item_loc,self.alignments['']
        elif len(item_loc)==6:
          item,r,c,rs,cs,align= item_loc
        else:
          raise Exception('Invalid grid item:',item_loc)
        if isinstance(item,str):
          widget= self.widgets[item]
          if isinstance(widget,QtGui.QSpacerItem):
            layout.addItem(widget,r,c,rs,cs,align)
          else:
            layout.addWidget(widget,r,c,rs,cs,align)
        else:
          sublayout= self.AddLayouts(item)
          layout.addLayout(sublayout,r,c,rs,cs,align)
    elif l_type=='tab':
      #As a tab is a widget in Qt, we make an outer layout with zero margin.
      #NOTE: The tab widget is stored into layout.tabs member variable.
      layout= QtGui.QVBoxLayout()
      layout.setContentsMargins(0, 0, 0, 0)
      layout.tabs= QtGui.QTabWidget()
      layout.addWidget(layout.tabs)
      layout.tab= []
      layout.tab_name_to_index= {}
      for tab_name,tab_layout in items:
        tab= QtGui.QWidget()
        layout.tab.append(tab)
        layout.tab_name_to_index[tab_name]= len(layout.tab)-1
        layout.tabs.addTab(tab,tab_name)
        sublayout= self.AddLayouts(tab_layout)
        tab.setLayout(sublayout)
      #For convenience, we define a setCurrentTab method to show a tab by name.
      layout.setCurrentTab= lambda tab_name:layout.tabs.setCurrentIndex(layout.tab_name_to_index[tab_name])
      #For convenience, we define a setCurrentTab method to show a tab by name.
      layout.setTabEnabled= lambda tab_name,enabled:layout.tabs.setTabEnabled(layout.tab_name_to_index[tab_name],enabled)

    self.layouts[name]= layout
    return layout

  # Override closing event
  def closeEvent(self, event):
    if self.close_callback is not None:
      res= self.close_callback(event)
      if res in (None,True):
        event.accept()
      else:
        event.ignore()
    else:
      event.accept()


app= None

def InitPanelApp():
  global app
  app= QtGui.QApplication(sys.argv)
  return app

def RunPanelApp():
  global app
  sys.exit(app.exec_())


if __name__=='__main__':
  def Print(*s):
    for ss in s:  print ss,
    print ''

  widgets= {
    'btn1': (
      'button',{
        'text':'Close',
        'onclick':lambda w,obj:w.close() if w.widgets['btn2'].isChecked() and AskYesNoDialog(w,'Are you sure to quit?') else w.widgets['btn2'].setChecked(True)}),
    'btn2': (
      'buttonchk',{
        'text':('TurnOn','TurnOff'),
        'onclick': (lambda w,obj:Print('ON!'),
                    lambda w,obj:Print('OFF!'))}),
    'btn_totab10': (
      'button',{
        'text':'To tab1',
        'onclick':lambda w,obj:w.layouts['maintab'].setCurrentTab('tab1')}),
    'btn_totab11': ('duplicate', 'btn_totab10'),
    'btn_totab12': ('duplicate', 'btn_totab10'),
    'btn_totab20': (
      'button',{
        'text':'To tab2',
        'onclick':lambda w,obj:w.layouts['maintab'].setCurrentTab('tab2')}),
    'btn_totab21': ('duplicate', 'btn_totab20'),
    'btn_totab22': ('duplicate', 'btn_totab20'),
    'btn_totab30': (
      'button',{
        'text':'To tab3',
        'onclick':lambda w,obj:w.layouts['maintab'].setCurrentTab('tab3')}),
    'btn_totab31': ('duplicate', 'btn_totab30'),
    'btn_totab32': ('duplicate', 'btn_totab30'),
    'cmb1': (
      'combobox',{
        'options':('Option-0','Option-1','Option-2','Other'),
        'index':1,
        'onactivated': lambda w,obj:(Print('Selected',obj.currentText()),
                                     w.widgets['edit_cmb1other'].setEnabled(obj.currentText()=='Other'))}),
    'edit_cmb1other': (
      'lineedit',{
        'validator':'int',
        'enabled':False}),
    'radbox1': (
      'radiobox',{
        'options':('Option-0','Option-1','Option-2','Other'),
        'layout': 'h',
        'index': 0,
        'onclick': lambda w,obj:(Print('Selected',obj.group.checkedButton().text()),
                                 w.widgets['edit_radbox1other'].setEnabled(obj.group.checkedButton().text()=='Other'))}),
    'edit_radbox1other': (
      'lineedit',{
        'validator':'int',
        'enabled':False}),
    'radbox2': (
      'radiobox',{
        'options':('Option-A','Option-B','Option-C','Other'),
        'layout': 'v',
        'index': None,
        'onclick': lambda w,obj:(Print('Selected',obj.group.checkedButton().text()),
                                 w.widgets['slider_radbox2other'].setEnabled(obj.group.checkedButton().text()=='Other'))}),
    'slider_radbox2other': (
      'sliderh',{
        'range': (1000,1800,100),
        'value': 1600,
        'n_labels': 3,
        'slider_style':1,
        'enabled':False,
        'onvaluechange': lambda w,obj:Print('Value:',obj.value())}),
    'label_tab2': (
      'label',{
        'text': 'Select a tab to go',
        'size_policy': ('expanding', 'minimum')}),
    'textedit1': (
      'textedit',{
        'text': 'Example\nOf\nTextEdit',}),
    'primitive_painer1': (
      'primitive_painer',{
        'color': (0,0,255),
        'margin': (0,0),
        'minimum_size': (None,20),
        'maximum_size': (None,20),
        'size_policy': ('expanding', 'fixed')}),
    'status_grid1': (
      'status_grid',{
        'list_status':[
            dict(label='Safety1', type='color', state='green'),
            dict(label='Safety2', type='color', state='green'),
            dict(label='Safety3', type='color', state='green'),
            dict(label='Safety4', type='color', state='yellow'),
            dict(label='Safety5', type='color', state='red'),
            dict(label='Arm1', type='color', state='green'),
            dict(label='Arm2', type='color', state='green'),
            dict(label='Gripper1', type='color', state='green'),
            dict(label='Gripper2', type='color', state='green'),
            dict(label='Sensor1', type='color', state='green'),
            dict(label='Sensor2', type='color', state='red'),
            dict(label='Sensor3', type='color', state='red'),
            dict(label='Type', type='text', state='A'),
            dict(label='Counter', type='text', state='20'),
          ],
        'direction':'vertical',
        'shape':'square',
        'margin':(0.05,0.05),
        'rows':None,
        'columns':3 }),
    'checkbox1': (
      'checkbox',{
        'text': 'Please check!'}),
    #'rviz1': (
      #'rviz',{
        #'setup': True,  }),
    'joy_xy1': (
      'virtual_joystick',{
        'kind':'circle',
        'stick_color':[128,128,255],
        'onstickmoved': lambda w,obj:Print('Pos:',obj.position()), }),
    'joy_xy2': (
      'virtual_joystick',{
        'kind':'circle',
        'stick_color':[255,128,128],
        'onstickmoved': lambda w,obj:Print('Pos:',obj.position()), }),
    'joy_h1': (
      'virtual_joystick',{
        'kind':'hbox',
        'stick_color':[128,255,128],
        'size_policy': ('expanding', 'minimum'),
        'onstickmoved': lambda w,obj:Print('Pos:',obj.position()), }),
    'joy_v1': (
      'virtual_joystick',{
        'kind':'vbox',
        'stick_color':[128,255,128],
        'size_policy': ('minimum', 'expanding'),
        'onstickmoved': lambda w,obj:Print('Pos:',obj.position()), }),
    'spacer1': ('spacer', {}),
    }
  #Layout option: vbox, hbox, grid, tab
  #layout= ('boxv',None,(
             #('grid',None, (('btn1',0,0),('btn2',0,1,1,2),
                            #('spacer1',1,0),('cmb1',1,1),('edit_cmb1other',1,2)) ),
             #('boxh',None, ('radbox1','edit_radbox1other') ),
             #('boxv',None, ('radbox2','slider_radbox2other') ),
             #'spacer1',
           #))
  layout= ('tab','maintab',(
           ('tab1',
              ('boxv',None,
                (
                  ('grid',None, (('btn1',0,0),('btn2',0,1,1,2),
                                  ('spacer1',1,0),('cmb1',1,1),('edit_cmb1other',1,2)) ),
                  ('boxh',None, ('radbox1','edit_radbox1other') ),
                  ('boxv',None, ('radbox2','slider_radbox2other') ),
                  ('boxh',None, ('btn_totab20', 'btn_totab30') ),
                  'spacer1',
                )) ),
            ('tab2', ('boxv',None, ('label_tab2', 'btn_totab10', 'btn_totab31', 'checkbox1') ) ),
            ('tab3', ('boxv',None, ('primitive_painer1', 'btn_totab11', 'btn_totab21', 'textedit1') ) ),
            #('tab4', ('boxv',None, ('rviz1',) ) ),
            ('tab5', ('boxh',None, ('status_grid1', 'btn_totab12', 'btn_totab22', 'btn_totab32' ) ) ),
            ('tab6', ('boxv',None,
                        (
                          ('boxh',None, ( ('boxh',None, ('joy_xy1','joy_xy2')),'joy_v1')),
                          'joy_h1',
                        ) ) ),
            ))

  InitPanelApp()
  panel= TSimplePanel('Simple Panel Example', size=(600,400), font_height_scale=300.0)
  panel.AddWidgets(widgets)
  panel.Construct(layout)
  RunPanelApp()

