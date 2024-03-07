#!/usr/bin/python
#\file    terminal_tab8lib.py
#\brief   Simple Tab-Terminal GUI command launcher (library).
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Sep.10, 2017
#\version 0.2
#\date    Sep.3, 2018
#         Option (radio button) interface is added.
#\date    May.30, 2019
#         Option (combobox) interface is added.
#\date    Mar.7, 2024
#         Added a method to save and load options (:radio and :cmd) into/from a file.
#Requirements: tmux rxvt-unicode-256color

import sys, os
import signal
import subprocess
import yaml
from PyQt4 import QtCore,QtGui,QtTest

class TTerminalTab(QtGui.QWidget):
  def __init__(self,title,widgets,exit_command,size=(800,400),horizontal=True,no_focus=True,term_width=400):
    QtGui.QWidget.__init__(self)
    self.pid= str(os.getpid())+'-'
    self.InitUI(title,widgets,exit_command,size,horizontal,no_focus,term_width)

  # Get a dict of option name: option content
  def ExpandOpt(self):
    opt= {name:str(rbgroup.checkedButton().text()) for name,rbgroup in self.RBOptions.iteritems()}
    opt.update({name:str(cmbbx.currentText()) for name,cmbbx in self.CBOptions.iteritems()})
    return opt

  # Save the option dict as a yaml file
  def SaveOpts(self, file_name):
    with open(file_name,'w') as fp:
      fp.write(yaml.dump(self.ExpandOpt()))
    print 'Options are saved into {}'.format(file_name)

  # Load option dict from a yaml file and update the UI.
  def LoadOpts(self, file_name):
    with open(file_name) as fp:
      opt= yaml.load(fp.read())
    #print 'debug',opt
    for name,value in opt.iteritems():
      if name in self.RBOptions:
        for radbtn in self.RBOptions[name].buttons():
          if radbtn.text()==value:
            radbtn.setChecked(True)
            break
      elif name in self.CBOptions:
        idx= self.CBOptions[name].findText(value, QtCore.Qt.MatchFixedString)
        self.CBOptions[name].setCurrentIndex(idx)
    print 'Loaded options from {}'.format(file_name)

  def CmdToLambda(self,term,cmd):
    if cmd==':close':  return self.close
    if len(cmd)==0:  return lambda:None
    if cmd[0]==':all':  return lambda:self.SendCmdToAll([c.format(**self.ExpandOpt()) for c in cmd[1:]])
    if cmd[0]==':saveopts':  return lambda:self.SaveOpts(cmd[1].format(**self.ExpandOpt()))
    return lambda:self.SendCmd(term,[c.format(**self.ExpandOpt()) for c in cmd])

  def InitUI(self,title,widgets,exit_command,size,horizontal,no_focus,term_width,grid_type='vhbox'):
    # Set window size.
    self.resize(*size)
    self.setMinimumSize(term_width, size[1])
    self.Processes= []
    self.TermProcesses= []

    # Set window title
    self.WinTitle= title
    self.setWindowTitle(self.WinTitle)

    self.ExitCommand= exit_command
    self.Widgets= widgets  #Widget definitions.
    self.Terminals= [line for line in self.Widgets if isinstance(line[1],(tuple,list))]
    self.RBOptions= {}  #Option name: Option radio button group
    self.CBOptions= {}  #Option name: Option combobox
    self.term_to_idx= {term:r for r,(term,row) in enumerate(self.Terminals)}
    self.Objects= {}  #Actual widget objects (Qt objects).

    # Horizontal box layout
    if horizontal:  boxlayout= QtGui.QHBoxLayout()
    else:           boxlayout= QtGui.QVBoxLayout()
    self.setLayout(boxlayout)

    self.qttabs= self.MakeTabs()
    #self.qttabs.setSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Expanding)
    self.qttabs.setFixedWidth(term_width)
    boxlayout.addWidget(self.qttabs)

    # Grid layout

    wg= QtGui.QWidget()
    grid= QtGui.QGridLayout()
    #if grid_type=='grid':  grid= QtGui.QGridLayout()
    #elif grid_type=='vhbox':  grid= QtGui.QVBoxLayout()
    wg.setLayout(grid)
    #boxlayout.addWidget(wg)

    #Scroll Area Properties
    vbox2= QtGui.QVBoxLayout()
    vbox2.addWidget(wg)
    widget= QtGui.QWidget()
    widget.setLayout(vbox2)
    scroll= QtGui.QScrollArea()  # Scroll Area which contains the widgets, set as the centralWidget
    scroll.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
    scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
    #scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
    scroll.setWidgetResizable(True)
    scroll.setWidget(widget)

    vbox3= QtGui.QVBoxLayout()
    vbox3.addWidget(scroll)

    boxlayout.addLayout(vbox3)

    # Add widgets on grid
    for r,line in enumerate(self.Widgets):
      gcol= [0]
      if grid_type=='grid':
        def add_widget(w):
          grid.addWidget(w, r, gcol[0])
          gcol[0]+= 1
      elif grid_type=='vhbox':
        gline= QtGui.QHBoxLayout()
        def add_widget(w):
          if gcol[0]==0:
            grid.addWidget(w, r, 0)
            grid.addLayout(gline, r, 1)
            gcol[0]+= 1
          else:
            w.setSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
            #w.setMaximumWidth(5)
            #w.resize(w.sizeHint().width(), w.sizeHint().height()*2)
            gline.addWidget(w)
      if isinstance(line,(tuple,list)) and len(line)>1 and line[1]==':radio':
        name,_,options= line
        label= QtGui.QLabel()
        label.setText(name)
        label.setAlignment(QtCore.Qt.AlignCenter)
        add_widget(label)
        group= QtGui.QButtonGroup()
        self.Objects[name]= {}
        self.Objects[name]['label']= label
        self.Objects[name]['group']= group
        self.Objects[name]['radbtns']= []
        for i,opt in enumerate(options):
          radbtn= QtGui.QRadioButton(opt)
          radbtn.setCheckable(True)
          if no_focus:  radbtn.setFocusPolicy(QtCore.Qt.NoFocus)
          if i==0:  radbtn.setChecked(True)
          group.addButton(radbtn,1)
          add_widget(radbtn)
          self.Objects[name]['radbtns'].append(radbtn)
        self.RBOptions[name]= group
      elif isinstance(line,(tuple,list)) and len(line)>1 and line[1]==':cmb':
        name,_,options= line
        label= QtGui.QLabel()
        label.setText(name)
        label.setAlignment(QtCore.Qt.AlignCenter)
        add_widget(label)
        cmbbx= QtGui.QComboBox(self)
        self.Objects[name]= {}
        self.Objects[name]['label']= label
        for opt in options:
          cmbbx.addItem(opt)
        cmbbx.setCurrentIndex(0)
        add_widget(cmbbx)
        self.Objects[name]['cmbbx']= cmbbx
        self.CBOptions[name]= cmbbx
      elif isinstance(line,(tuple,list)) and len(line)>1 and isinstance(line[1],(tuple,list)):
        term,row= line
        btn0= QtGui.QPushButton('({term})'.format(term=term))
        btn0.setFlat(True)
        if no_focus:  btn0.setFocusPolicy(QtCore.Qt.NoFocus)
        btn0.clicked.connect(lambda clicked,term=term:self.ShowTermTab(term))
        add_widget(btn0)
        self.Objects[term]= {}
        self.Objects[term]['label']= btn0
        self.Objects[term]['buttons']= {}
        for commands in row:
          if commands[0]==':pair':
            name1,f1= commands[1][0],self.CmdToLambda(term,commands[1][1])
            name2,f2= commands[2][0],self.CmdToLambda(term,commands[2][1])
            btn= QtGui.QPushButton(name1)
            btn.setStyleSheet('padding:5px 10px 5px 10px')
            btn.setCheckable(True)
            if no_focus:  btn.setFocusPolicy(QtCore.Qt.NoFocus)
            btn.clicked.connect(lambda b,btn=btn,name1=name1,f1=f1,name2=name2,f2=f2:
                                  (f1(),btn.setText(name2)) if btn.isChecked() else (f2(),btn.setText(name1)))
            add_widget(btn)
            self.Objects[term]['buttons'][name1]= btn
          else:
            name1,f= commands[0],self.CmdToLambda(term,commands[1])
            btn= QtGui.QPushButton(name1)
            btn.setStyleSheet('padding:5px 10px 5px 10px')
            if no_focus:  btn.setFocusPolicy(QtCore.Qt.NoFocus)
            btn.clicked.connect(f)
            add_widget(btn)
            self.Objects[term]['buttons'][name1]= btn
      else:
        raise Exception('Unknown syntax:',line)
      if grid_type=='vhbox':
        gline.addSpacerItem(QtGui.QSpacerItem(1, 1, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding))

    # Show window
    self.show()
    self.CreateTerminals()

    ## Adjust the alignment of the terminal widget.
    #for idx in range(self.qttabs.count()):
      #self.qttabs.widget(idx).layout().setAlignment(QtCore.Qt.AlignTop)

  def MakeTabs(self):
    tabs= QtGui.QTabWidget()

    self.qtterm= {}
    for r,(term,row) in enumerate(self.Terminals):
      tab= QtGui.QWidget()
      tabs.addTab(tab, term)

      terminal= QtGui.QWidget(self)
      hBoxlayout= QtGui.QHBoxLayout()
      tab.setLayout(hBoxlayout)
      hBoxlayout.addWidget(terminal)
      self.qtterm[term]= terminal

    return tabs

  def ShowTermTab(self, term):
    self.qttabs.setCurrentIndex(self.term_to_idx[term])

  def StartProc(self, prog, args):
    child= QtCore.QProcess()
    self.Processes.append(child)
    child.start(prog, args)
    return child

  def SendCmd(self, term, cmd):
    self.ShowTermTab(term)
    self.StartProc('tmux', ['send-keys', '-t', self.pid+term+':0'] + list(cmd))

  def SendCmdToAll(self, cmd):
    for r,(term,row) in enumerate(self.Terminals):
      self.ShowTermTab(term)
      self.StartProc('tmux', ['send-keys', '-t', self.pid+term+':0'] + list(cmd))

  def CreateTerminals(self):
    #QtTest.QTest.qWait(1000)
    for r,(term,row) in enumerate(self.Terminals):
      self.qttabs.setCurrentIndex(r)
      #self.qttabs.widget(r).update()
      #print 'debug',self.pid+term,str(self.qtterm[term].winId())
      child= self.StartProc('urxvt',
                            ['-embed', str(self.qtterm[term].winId()),
                              '-e', 'tmux', 'new', '-s', self.pid+term])
      self.TermProcesses.append(child)
      #print '  --',child,child.pid()
      #self.SendCmd(term, ['ls','Enter'])
      #child= self.StartProc('tmux', ['capture-pane', '-t', self.pid+term+':0'] + list(cmd))
      QtTest.QTest.qWait(50)
      #res= subprocess.check_output(['tmux', 'capture-pane', '-t', self.pid+term+':0', 'echo ok', 'Enter'])
      try:
        res= subprocess.check_output(['tmux', 'send-keys', '-t', self.pid+term+':0', 'echo ok', 'Enter'])
        #print '  --',res
      except subprocess.CalledProcessError:
        #print '  --CalledProcessError'
        print 'CalledProcessError',self.pid+term,str(self.qtterm[term].winId())
      #self.qttabs.widget(r).update()
      #print 'new terminal proc:',self.pid+term,self.TermProcesses[-1].pid()
      #QtTest.QTest.qWait(100)
    #QtTest.QTest.qWait(200)
    #QtTest.QTest.qWait(200*len(self.Terminals))
    #for r,(term,row) in enumerate(self.Terminals):
      #self.StartProc('tmux', ['send-keys', '-t', self.pid+term+':0'] + self.InitCommand)
    self.qttabs.setCurrentIndex(0)

  def Exit(self):
    for r,(term,row) in enumerate(self.Terminals):
      self.qttabs.setCurrentIndex(r)
      self.StartProc('tmux', ['send-keys', '-t', self.pid+term+':0'] + self.ExitCommand)
    #QtTest.QTest.qWait(200)
    for r,(term,row) in enumerate(self.Terminals):
      self.qttabs.setCurrentIndex(r)
      self.StartProc('tmux', ['send-keys', '-t', self.pid+term+':0', 'exit', 'Enter'])
    #QtTest.QTest.qWait(200)
    for proc in self.TermProcesses:
      os.kill(proc.pid(), signal.SIGTERM)

  # Override closing event
  def closeEvent(self, event):
    quit_msg= 'Really exit {title}?'.format(title=self.WinTitle)
    reply= QtGui.QMessageBox.question(self, 'Message',
                     quit_msg, QtGui.QMessageBox.Yes, QtGui.QMessageBox.No)
    if reply==QtGui.QMessageBox.Yes:
      self.Exit()
      event.accept()
    else:
      event.ignore()

def RunTerminalTab(title, widgets, exit_command, 
                   size=(800,400), horizontal=True, no_focus=True, start=True,
                   init_option_file=None):
  app= QtGui.QApplication(sys.argv)
  win= TTerminalTab(title,widgets,exit_command,size=size,horizontal=horizontal,no_focus=no_focus)
  signal.signal(signal.SIGINT, lambda signum,frame,win=win: (win.Exit(),QtGui.QApplication.quit()) )
  timer= QtCore.QTimer()
  timer.start(500)  # You may change this if you wish.
  timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.
  if init_option_file is not None:  win.LoadOpts(init_option_file)
  if start:  StartApp(app)
  return app,win

def StartApp(app):
  sys.exit(app.exec_())

if __name__=='__main__':
  E= 'Enter'
  option_file= '/tmp/opts.yaml'
  widgets= [
    ('main1',[
      ('Init',[':all','ros',E,'norobot',E]),
      ('KillAll',[':all',E,'C-c']),
      ('SaveOpts',[':saveopts',option_file]),
      ('Exit',':close') ]),
    ('s1',[
      (':pair', ('rviz',['rviz',E]),
                ('kill',['C-c']) )  ]),
    ('LS_Dir',':radio',['./','/tmp/','/home/']),
    ('LS_Opt',':cmb',['','-a','-rtl']),
    ('s2',[
      ('ls',('ls {LS_Opt} {LS_Dir}',E)),
      ('nodes',['rostopic list',E]),
      ('topics',['rosnode list',E]) ]),
    ]
  exit_command= [E,'C-c']
  RunTerminalTab('Sample Launcher',widgets,exit_command,init_option_file=option_file)
