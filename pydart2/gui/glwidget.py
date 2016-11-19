from __future__ import division
from __future__ import print_function
from __future__ import absolute_import
from past.utils import old_div
# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
import OpenGL.GL as GL
import OpenGL.GLU as GLU
# import OpenGL.GLUT as GLUT
from PyQt4 import QtGui
from PyQt4 import QtCore
from PyQt4.QtOpenGL import QGLWidget, QGLFormat
from . import trackball
# import time
from .renderer import Renderer
import numpy as np
# from numpy.linalg import norm


class GLWidget(QGLWidget):

    def __init__(self, parent=None):
        super(GLWidget, self).__init__(parent)
        fmt = QGLFormat()
        fmt.setSampleBuffers(True)
        fmt.setSamples(4)
        # print fmt.samples()
        # exit(0)

        self.setFormat(fmt)  # Anti-aliasing
        self.width = 1280
        self.height = 720

        self.tb = trackball.Trackball()
        self.lastPos = None
        self.zoom = -1.2

        self.sim = None
        self.viewer = None
        self.captureIndex = 0
        self.renderer = Renderer()

        self.lock_camera = False

    def sizeHint(self):
        return QtCore.QSize(self.width, self.height)

    def paintGL(self):
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glClearColor(0.98, 0.98, 0.98, 0.0)
        GL.glClearColor(1.0, 1.0, 1.0, 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)

        GL.glLoadIdentity()
        # glTranslate(0.0, -0.2, self.zoom)  # Camera
        GL.glTranslate(*self.tb.trans)
        GL.glMultMatrixf(self.tb.matrix)

        if self.sim is not None and hasattr(self.sim, "render"):
            self.sim.render()

        self.renderer.enable("COLOR_MATERIAL")
        if self.sim is not None and hasattr(self.sim, "render_with_ri"):
            self.sim.render_with_ri(self.renderer)

        self.enable2D()
        if self.sim is not None and hasattr(self.sim, "draw_with_ri"):
            self.sim.draw_with_ri(self.renderer)
            self.renderer.draw_text([-100, -100], "")
        self.disable2D()

    def resizeGL(self, w, h):
        (self.width, self.height) = (w, h)
        GL.glViewport(0, 0, w, h)
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()

        GLU.gluPerspective(45.0, old_div(float(w), float(h)), 0.01, 100.0)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()

    def initializeGL(self):
        # return self.initializeGL.GL_Stelian()
        GL.glDisable(GL.GL_CULL_FACE)
        GL.glEnable(GL.GL_DEPTH_TEST)

        GL.glDepthFunc(GL.GL_LEQUAL)
        GL.glHint(GL.GL_PERSPECTIVE_CORRECTION_HINT, GL.GL_NICEST)

        GL.glEnable(GL.GL_LINE_SMOOTH)
        GL.glHint(GL.GL_LINE_SMOOTH_HINT, GL.GL_NICEST)
        # GlEnable(GL.GL_POLYGON_SMOOTH)
        GL.glHint(GL.GL_POLYGON_SMOOTH_HINT, GL.GL_NICEST)

        GL.glEnable(GL.GL_DITHER)
        GL.glShadeModel(GL.GL_SMOOTH)
        GL.glHint(GL.GL_PERSPECTIVE_CORRECTION_HINT, GL.GL_NICEST)

        GL.glClearColor(1.0, 1.0, 1.0, 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT)

        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glDepthFunc(GL.GL_LEQUAL)
        GL.glDisable(GL.GL_CULL_FACE)
        GL.glEnable(GL.GL_NORMALIZE)

        GL.glColorMaterial(GL.GL_FRONT_AND_BACK, GL.GL_AMBIENT_AND_DIFFUSE)
        GL.glEnable(GL.GL_COLOR_MATERIAL)

        GL.glEnable(GL.GL_BLEND)
        GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
        GL.glEnable(GL.GL_MULTISAMPLE)
        # glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA)

        ambient = [0.2, 0.2, 0.2, 1.0]
        diffuse = [0.6, 0.6, 0.6, 1.0]
        front_mat_shininess = [60.0]
        front_mat_specular = [0.2, 0.2, 0.2, 1.0]
        front_mat_diffuse = [0.5, 0.28, 0.38, 1.0]
        lmodel_ambient = [0.2, 0.2, 0.2, 1.0]
        lmodel_twoside = [GL.GL_FALSE]

        # position = [1.0, 1.0, 1.0, 0.0]
        # position1 = [-1.0, 1.0, 0.0, 0.0]

        position = [1.0, 1.0, 0.0, 0.0]
        position1 = [-1.0, 0.0, 0.0, 0.0]

        GL.glEnable(GL.GL_LIGHT0)
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_AMBIENT, ambient)
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_DIFFUSE, diffuse)
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_POSITION, position)

        GL.glLightModelfv(GL.GL_LIGHT_MODEL_AMBIENT, lmodel_ambient)
        GL.glLightModelfv(GL.GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside)

        GL.glEnable(GL.GL_LIGHT1)
        # glLightfv(GL.GL_LIGHT1, GL.GL_AMBIENT, ambient)
        GL.glLightfv(GL.GL_LIGHT1, GL.GL_DIFFUSE, diffuse)
        GL.glLightfv(GL.GL_LIGHT1, GL.GL_POSITION, position1)
        GL.glEnable(GL.GL_LIGHTING)

        GL.glEnable(GL.GL_COLOR_MATERIAL)
        GL.glMaterialfv(GL.GL_FRONT_AND_BACK, GL.GL_SHININESS,
                        front_mat_shininess)
        GL.glMaterialfv(GL.GL_FRONT_AND_BACK, GL.GL_SPECULAR,
                        front_mat_specular)
        GL.glMaterialfv(GL.GL_FRONT_AND_BACK, GL.GL_DIFFUSE,
                        front_mat_diffuse)

        # glBlendFunc(GL.GL_ONE, GL.GL_ONE)
        # glCullFace(GL.GL_BACK)

    def enable2D(self):
        w, h = self.width, self.height
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        GL.glDisable(GL.GL_LIGHTING | GL.GL_DEPTH_TEST)
        GL.glDepthMask(0)
        GL.glOrtho(0, w, h, 0, -1, 1)
        GL.glViewport(0, 0, w, h)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()

    def disable2D(self):
        w, h = self.width, self.height
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()

        GL.glEnable(GL.GL_DEPTH_TEST | GL.GL_LIGHTING)
        GL.glDepthMask(1)
        GLU.gluPerspective(45.0, old_div(float(w), float(h)), 0.01, 100.0)

        GL.glViewport(0, 0, w, h)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()

    def set_lock_camera(self, lock=True):
        self.lock_camera = lock

    def mousePressEvent(self, event):
        self.lastPos = event.pos()

        pos = np.array([event.pos().x(), event.pos().y()],
                       dtype=np.float64)
        self.viewer.safe_call_callback("on_mouse_press", pos)

    def mouseReleaseEvent(self, event):
        self.lastPos = None
        self.viewer.safe_call_callback("on_mouse_release")

    def mouseMoveEvent(self, event):
        # (w, h) = (self.width, self.height)
        x = event.x()
        y = event.y()
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()

        if not self.lock_camera:
            modifiers = QtGui.QApplication.keyboardModifiers()
            if modifiers == QtCore.Qt.ShiftModifier:
                self.tb.zoom_to(dx, -dy)
            elif modifiers == QtCore.Qt.ControlModifier:
                self.tb.trans_to(dx, -dy)
            else:
                self.tb.drag_to(x, y, dx, -dy)

        p0 = np.array([self.lastPos.x(), self.lastPos.y()],
                      dtype=np.float64)
        p1 = np.array([event.pos().x(), event.pos().y()],
                      dtype=np.float64)
        self.viewer.safe_call_callback("on_mouse_move", p0, p1)

        self.lastPos = event.pos()
        self.updateGL()

    def capture(self, name=None):
        img = self.grabFrameBuffer()
        if name is None:
            name = 'frame'
        dir = self.viewer.capture_dir()
        filename = '%s/%s.%04d.png' % (dir, name, self.captureIndex)
        img.save(filename)
        print(('Capture to %s' % filename))
        self.captureIndex += 1
