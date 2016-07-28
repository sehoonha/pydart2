#!/usr/bin/env python

# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group

import sys
import os
import signal
import numpy as np
import math
import time
import inspect
import logging

from OpenGL.GLUT import *
from PyQt4 import QtGui
from PyQt4 import QtCore
from glwidget import GLWidget
from trackball import Trackball


def signal_handler(signal, frame):
    print('You pressed Ctrl+C! Bye.')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


def S(x):
    ret = "["
    tokens = ["%.6f" % v for v in x]
    ret += ", ".join(tokens)
    ret += "]"
    return ret


class MyWindow(QtGui.QMainWindow):
    def __init__(self, sim=None, callbacks=None):
        super(MyWindow, self).__init__()
        # setup_logger()
        self.logger = logging.getLogger(__name__)
        self.sim = sim
        self.callbacks = callbacks if callbacks is not None else dict()
        self.logger.info('sim = [%s]' % str(self.sim))

        # Check and create captures directory
        self.capture_dir = 'data/captures'
        if not os.path.isdir('data/captures'):
            os.makedirs('data/captures')

        self.initUI()
        self.initActions()
        self.initToolbar()
        self.initMenu()

        self.idleTimer = QtCore.QTimer()
        self.idleTimer.timeout.connect(self.idleTimerEvent)
        self.idleTimer.start(0)

        self.renderTimer = QtCore.QTimer()
        self.renderTimer.timeout.connect(self.renderTimerEvent)
        self.renderTimer.start(80)

        self.cam0Event()

        self.after_reset = True

        self.input_keys = list()
        self.onInit()
        self.topLeft()

    def topLeft(self):
        frameGm = self.frameGeometry()
        desktop = QtGui.QApplication.desktop()
        screen = desktop.screenNumber(desktop.cursor().pos())
        # centerPoint = desktop.screenGeometry(screen).center()
        # frameGm.moveCenter(centerPoint)
        topLeftPoint = desktop.screenGeometry(screen).topLeft()
        frameGm.moveTopLeft(topLeftPoint)
        self.move(frameGm.topLeft())

    def get_simulation(self):
        return self.sim

    def set_simulation(self, sim):
        self.sim = sim
        self.glwidget.sim = sim

    def initUI(self):
        TOOLBOX_HEIGHT = 30
        STATUS_HEIGHT = 30
        PANEL_WIDTH = 0
        self.setGeometry(0, 0,
                         1280 + PANEL_WIDTH,
                         720 + TOOLBOX_HEIGHT + STATUS_HEIGHT)
        # self.setWindowTitle('Toolbar')

        self.ui = QtGui.QWidget(self)
        self.ui.setGeometry(0, TOOLBOX_HEIGHT, 1280 + PANEL_WIDTH, 720)

        self.hbox = QtGui.QHBoxLayout()

        self.glwidget = GLWidget()
        self.glwidget.sim  = self.sim
        self.glwidget.viewer = self
        self.glwidget.setGeometry(0, 0, 1280, 720)

        self.hbox.addWidget(self.glwidget)
        self.ui.setLayout(self.hbox)

    def initActions(self):
        # Create actions
        self.resetAction = QtGui.QAction('Reset', self)
        self.resetAction.triggered.connect(self.resetEvent)

        self.playAction = QtGui.QAction('Play', self)
        self.playAction.setCheckable(True)
        self.playAction.setShortcut('Space')

        self.animAction = QtGui.QAction('Anim', self)
        self.animAction.setCheckable(True)

        self.optAction = QtGui.QAction('Opt', self)
        self.optAction.triggered.connect(self.optEvent)

        self.captureAction = QtGui.QAction('Capture', self)
        self.captureAction.setCheckable(True)

        self.emptyAction = QtGui.QAction('Empty', self)
        self.emptyAction.triggered.connect(self.emptyEvent)

        self.movieAction = QtGui.QAction('Movie', self)
        self.movieAction.triggered.connect(self.movieEvent)

        self.screenshotAction = QtGui.QAction('Screenshot', self)
        self.screenshotAction.triggered.connect(self.screenshotEvent)

        self.exportAction = QtGui.QAction('Export BVH', self)
        self.exportAction.triggered.connect(self.exportEvent)

        # Camera Menu
        self.cam0Action = QtGui.QAction('Camera0', self)
        self.cam0Action.triggered.connect(self.cam0Event)

        self.cam1Action = QtGui.QAction('Camera1', self)
        self.cam1Action.triggered.connect(self.cam1Event)

        self.cam2Action = QtGui.QAction('Camera2', self)
        self.cam2Action.triggered.connect(self.cam2Event)

        self.cam3Action = QtGui.QAction('Camera3', self)
        self.cam3Action.triggered.connect(self.cam3Event)

        self.cam4Action = QtGui.QAction('Camera4', self)
        self.cam4Action.triggered.connect(self.cam4Event)

        self.cam5Action = QtGui.QAction('Camera5', self)
        self.cam5Action.triggered.connect(self.cam5Event)

        self.cam6Action = QtGui.QAction('Camera6', self)
        self.cam6Action.triggered.connect(self.cam6Event)

        self.printCamAction = QtGui.QAction('Print Camera', self)
        self.printCamAction.triggered.connect(self.printCamEvent)

    def initToolbar(self):
        # Create a toolbar
        self.toolbar = self.addToolBar('Control')
        self.toolbar.addAction(self.resetAction)
        self.toolbar.addAction(self.playAction)
        self.toolbar.addAction(self.animAction)
        self.toolbar.addAction(self.optAction)
        self.toolbar.addSeparator()
        self.toolbar.addAction(self.screenshotAction)
        self.toolbar.addAction(self.captureAction)
        self.toolbar.addAction(self.emptyAction)
        self.toolbar.addAction(self.movieAction)
        self.toolbar.addSeparator()
        self.toolbar.addAction(self.exportAction)

        self.rangeSlider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.rangeSlider.valueChanged[int].connect(self.rangeSliderEvent)
        self.toolbar.addWidget(self.rangeSlider)

    def initMenu(self):
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        fileMenu.addSeparator()

        # Camera menu
        cameraMenu = menubar.addMenu('&Camera')
        cameraMenu.addAction(self.cam0Action)
        cameraMenu.addAction(self.cam1Action)
        cameraMenu.addAction(self.cam2Action)
        cameraMenu.addAction(self.cam3Action)
        cameraMenu.addAction(self.cam4Action)
        cameraMenu.addAction(self.cam5Action)
        cameraMenu.addAction(self.cam6Action)
        cameraMenu.addSeparator()
        cameraMenu.addAction(self.printCamAction)

        # Recording menu
        recordingMenu = menubar.addMenu('&Recording')
        recordingMenu.addAction(self.screenshotAction)
        recordingMenu.addSeparator()
        recordingMenu.addAction(self.captureAction)
        recordingMenu.addAction(self.movieAction)
        recordingMenu.addSeparator()
        recordingMenu.addAction(self.exportAction)

    def onInit(self):
        funcname = inspect.stack()[0][3]
        print('[%s] check the callbacak' % funcname)
        if funcname in self.callbacks:
            print('[%s] detect the function in callback' % funcname)
            self.callbacks[funcname](self)

    def idleTimerEvent(self):
        doCapture = True

        # Do animation
        if self.animAction.isChecked():
            v = self.rangeSlider.value()
            v += 1
            if v <= self.rangeSlider.maximum():
                self.rangeSlider.setValue(v)
            else:
                self.animAction.setChecked(False)
            doCapture = True

        # Do play
        elif self.playAction.isChecked():
            if hasattr(self.sim, 'step'):
                self.sim.step()
            # capture_rate = 10
            # doCapture = (self.world.frame() % capture_rate == 1)
            doCapture = False

        if self.captureAction.isChecked() and doCapture:
            self.glwidget.capture(self.prob_name())

    def renderTimerEvent(self):
        self.glwidget.updateGL()

        if hasattr(self.sim, 'status'):
            self.statusBar().showMessage(self.sim.status())
        else:
            self.statusBar().showMessage(str(self.sim))

        if hasattr(self.sim, 'get_num_frames'):
            n = self.sim.get_num_frames()
            self.rangeSlider.setRange(0, n - 1)

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Escape:
            print('Escape key pressed! Bye.')
            self.close()
        if 0 <= event.key() and event.key() < 256:  # If key is ascii
            key = chr(event.key())
            print('key = %s' % key)
            self.input_keys.append(key)

    def clear_key_events(self):
        self.input_keys = list()

    def pop_key_event(self):
        if len(self.input_keys) > 0:
            return self.input_keys.pop(0)
        else:
            return None

    def rangeSliderEvent(self, value):
        if hasattr(self.sim, 'set_frame'):
            self.sim.set_frame(value)

    def screenshotEvent(self):
        self.glwidget.capture("%s_frame" % self.prob_name())

    def exportEvent(self):
        self.sim.export()

    def autoEvent(self):
        print('auto start!')
        self.auto_counter = 0

    def prob_name(self):
        return "robot"

    def emptyEvent(self):
        cmd = 'rm %s/*.png' % self.capture_dir
        print('cmd = %s' % cmd)
        os.system(cmd)

    def movieEvent(self):
        name = self.prob_name()
        # yuv420p for compatibility for outdated codecs
        cmt_fmt = 'avconv -r 60'
        cmt_fmt += ' -i %s/%s.%%04d.png'
        cmt_fmt += ' -pix_fmt yuv420p'
        cmt_fmt += ' %s.mp4'
        print(self.capture_dir)
        cmd = cmt_fmt % (self.capture_dir, name, name)
        print('cmd = %s' % cmd)
        os.system(cmd)

    def resetEvent(self):
        self.after_reset = True
        self.rangeSlider.setValue(0)
        if hasattr(self.sim, 'reset'):
            self.sim.reset()

    def optEvent(self):
        funcname = inspect.stack()[0][3]
        print('[%s] check the callbacak' % funcname)
        if funcname in self.callbacks:
            print('[%s] detect the function in callback' % funcname)
            self.callbacks[funcname](self)

    def cam0Event(self):
        print('cam0Event')
        self.glwidget.tb = Trackball(rot=[-0.152, 0.045, -0.002, 0.987],
                                     trans=[0.050, 0.210, -0.910])

    def cam1Event(self):
        print('cam1Event')
        self.glwidget.tb = Trackball(phi=-0.1, theta=-18.9, zoom=1.0,
                                     rot=[-0.00, -0.99, -0.16, -0.00],
                                     trans=[0.01, 0.57, -1.59])

    def cam2Event(self):
        print('cam2Event')
        self.glwidget.tb = Trackball(phi=1.5, theta=-22.2, zoom=1.0,
                                     rot=[0.19, -0.19, -0.05, -0.96],
                                     trans=[0.03, -0.03, -0.26])

    def cam3Event(self):
        print('cam3Event')
        self.glwidget.tb = Trackball(phi=1.2, theta=-22.4, zoom=1.0,
                                     rot=[0.19, -0.19, -0.05, -0.96],
                                     trans=[-0.01, -0.02, -0.26])

    def cam4Event(self):
        print('cam4Event')
        self.glwidget.tb = Trackball(phi=2.6, theta=-27.6, zoom=1.0,
                                     rot=[0.14, -0.83, -0.22, -0.48],
                                     trans=[0.02, -0.06, -0.61])

    def cam5Event(self):
        print('cam5Event')
        self.glwidget.tb = Trackball(phi=-2.7, theta=-21.9, zoom=1.0,
                                     rot=[0.14, -0.55, -0.08, -0.81],
                                     trans=[0.02, -0.06, -0.69])

    def cam6Event(self):
        print('cam6Event')
        self.glwidget.tb = Trackball(phi=-0.3, theta=-11.2, zoom=1.0,
                                     rot=[0.10, -0.05, -0.00, -0.99],
                                     trans=[0.02, -0.06, -0.72])

    def printCamEvent(self):
        print('printCamEvent')
        print('----')
        print(repr(self.glwidget.tb))
        print('----')


def launch(sim=None, title=None, callbacks=None):
    glutInit(sys.argv)
    if title is None:
        title = "Simulation Viewer"
    print("title = %s" % title)
    app = QtGui.QApplication([title])
    w = MyWindow(sim, callbacks)
    w.show()
    app.exec_()
