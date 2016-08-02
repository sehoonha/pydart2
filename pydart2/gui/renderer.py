# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import math
import numpy as np


def norm(v):
    return np.linalg.norm(v)


def normalize(v):
    return v / np.linalg.norm(v)


def rot2d(v, theta):
    cth = math.cos(theta)
    sth = math.sin(theta)
    R = np.array([[cth, -sth], [sth, cth]])
    return R.dot(v)


def rad_to_deg(th):
    return (180.0 / math.pi) * th


class Renderer(object):
    def __init__(self, ):
        self.quadric = gluNewQuadric()
        self.textures = list()
	# gluQuadricNormals(self.quadric, GLU_SMOOTH)
        # gluQuadricTexture(self.quadric, GL_TRUE)

    def color(self, r, g, b, a=1.0):
        glColor4d(r, g, b, a)

    def set_color(self, r, g, b, a=1.0):
        glColor4d(r, g, b, a)

    def push(self):
        glPushMatrix()

    def pop(self):
        glPopMatrix()

    def parse_flag(self, x):
        if x == "COLOR_MATERIAL":
            return GL_COLOR_MATERIAL
        else:
            return x

    def enable(self, x):
        glEnable(self.parse_flag(x))

    def disable(self, x):
        glDisable(self.parse_flag(x))

    def translate(self, x, y, z=0.0):
        glTranslate(x, y, z)

    def scale(self, x, y, z=1.0):
        glScale(x, y, z)

    def rotate2d(self, angle_in_rad):
        glRotate(rad_to_deg(angle_in_rad), 0.0, 0.0, 1.0)

    def mult_matrix(self, m):
        mat = [m[0, 0], m[1, 0], m[2, 0], m[3, 0],
               m[0, 1], m[1, 1], m[2, 1], m[3, 1],
               m[0, 2], m[1, 2], m[2, 2], m[3, 2],
               m[0, 3], m[1, 3], m[2, 3], m[3, 3], ]
        glMultMatrixf(mat)

    def mult_matrix_raw(self, m):
        glMultMatrixf(m)

    def line_width(self, w):
        glLineWidth(w)

    def set_line_width(self, w):
        glLineWidth(w)

    def draw_line(self, p0, p1):
        self.push()
        glBegin(GL_LINES)
        glVertex2d(*p0)
        glVertex2d(*p1)
        glEnd()
        self.pop()

    def draw_arrow(self, p0, p1, head_len=20.0, head_angle=0.25):
        self.push()
        self.draw_line(p0, p1)
        u = normalize(np.array(p0) - np.array(p1))
        v1 = rot2d(u, head_angle * math.pi)
        v2 = rot2d(u, -head_angle * math.pi)
        self.draw_line(p1, p1 + v1 * head_len)
        self.draw_line(p1, p1 + v2 * head_len)
        self.pop()

    def draw_box(self, sx, sy, center=None, angle=None, fill=False):
        self.push()
        if center is not None:
            self.translate(center[0], center[1])
        if angle is not None:
            self.rotate2d(angle)
        glBegin(GL_LINE_LOOP if not fill else GL_POLYGON)
        glVertex2d(0.5 * sx, 0.5 * sy)
        glVertex2d(0.5 * sx, -0.5 * sy)
        glVertex2d(-0.5 * sx, -0.5 * sy)
        glVertex2d(-0.5 * sx, 0.5 * sy)
        glEnd()
        self.pop()

    def draw_circle(self, r, num_segments=20, center=None, fill=False):
        self.push()
        if center is not None:
            self.translate(center[0], center[1])
        glBegin(GL_LINE_LOOP if not fill else GL_POLYGON)
        for i in range(num_segments):
            th = float(i) / float(num_segments) * 2.0 * math.pi
            cth = math.cos(th)
            sth = math.sin(th)
            x = r * cth
            y = r * sth
            glVertex2d(x, y)
        glEnd()
        self.pop()

    def draw_ellipse(self, p0, p1, b, num_segments=20, fill=False):
        self.draw_circle(5.0, center=p0)
        self.draw_circle(5.0, center=p1)
        self.push()
        p0 = np.array(p0).astype(float)
        p1 = np.array(p1).astype(float)
        a = 0.5 * norm(p1 - p0)
        center = 0.5 * (p0 + p1)
        u = p1 - p0
        angle = np.arctan2(u[1], u[0])
        self.translate(center[0], center[1])
        self.rotate2d(angle)

        glBegin(GL_LINE_LOOP if not fill else GL_POLYGON)
        for i in range(num_segments):
            th = float(i) / float(num_segments) * 2.0 * math.pi
            cth = math.cos(th)
            sth = math.sin(th)
            x = a * cth
            y = b * sth
            glVertex2d(x, y)
        glEnd()
        self.pop()

    def draw_ellipse_at(self, a, b, center=None, angle=None,
                        num_segments=20, fill=False):
        self.push()
        if center is not None:
            self.translate(center[0], center[1])
        if angle is not None:
            self.rotate2d(angle)

        glBegin(GL_LINE_LOOP if not fill else GL_POLYGON)
        for i in range(num_segments):
            th = float(i) / float(num_segments) * 2.0 * math.pi
            cth = math.cos(th)
            sth = math.sin(th)
            x = a * cth
            y = b * sth
            glVertex2d(x, y)
        glEnd()
        self.pop()

    def draw_text(self, pos, text, font=GLUT_BITMAP_HELVETICA_18):
        self.push()
        glRasterPos(*pos)
        for c in text:
            glutBitmapCharacter(font, ord(c))
        self.pop()

    def gen_textures(self, num_textures=1):
        im = glGenTextures(num_textures)
        self.textures.append(im)
        return im

    def delete_textures(self, textures):
        glDeleteTextures(textures)

    def bind_texture(self, texture):
        glBindTexture(GL_TEXTURE_2D, texture)
        # print("bind_texture: %d" % texture)

    def set_texture_as_image(self, img, texture=-1):
        if texture == -1:
            texture = self.textures[-1]
        self.bind_texture(texture)
        textureData = np.fromstring(img.tostring(), np.uint8)
        width, height = img.size
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height,
                     0, GL_RGBA, GL_UNSIGNED_BYTE, textureData)
        # print("set_texture_data: %d x %d" % (width, height))

    def draw_image(self, sx, sy, texture=-1,
                   center=None, angle=None):
        self.set_color(1.0, 1.0, 1.0)
        glEnable(GL_TEXTURE_2D)
        if texture == -1:
            texture = self.textures[-1]
        self.bind_texture(texture)
        self.push()
        if center is not None:
            self.translate(center[0], center[1])
        if angle is not None:
            self.rotate2d(angle)
        glBegin(GL_QUADS)
        glTexCoord2f(0, 0)
        glVertex2d(-0.5 * sx, -0.5 * sy)
        glTexCoord2f(0, 1)
        glVertex2d(-0.5 * sx, 0.5 * sy)
        glTexCoord2f(1, 1)
        glVertex2d(0.5 * sx, 0.5 * sy)
        glTexCoord2f(1, 0)
        glVertex2d(0.5 * sx, -0.5 * sy)
        glEnd()
        self.pop()
        glDisable(GL_TEXTURE_2D)

    def render_line(self, p0, p1):
        glBegin(GL_LINES)
        glVertex(*p0)
        glVertex(*p1)
        glEnd()

    def render_sphere(self, pos, r, num_seg1=20, num_seg2=10):
        glPushMatrix()
        glTranslated(*pos)
        glutSolidSphere(r, num_seg1, num_seg2)
        glPopMatrix()

    def render_box(self, pos, size):
        glPushMatrix()
        glTranslated(*pos)
        glScaled(*size)
        glutSolidCube(1.0)
        glPopMatrix()

    def render_chessboard(self, sz, n=10, color1=None, color2=None):
        glDisable(GL_LIGHTING)
        step = sz / float(n)
        if color1 is None:
            color1 = np.array([0.95, 0.95, 0.95])
        if color2 is None:
            color2 = np.array([0.7, 0.7, 0.7])
        sz2 = 0.5 * sz
        for i, x in enumerate(np.linspace(-sz2, sz2, n, endpoint=False)):
            for j, z in enumerate(np.linspace(-sz2, sz2, n, endpoint=False)):
                x2 = x + step
                z2 = z + step
                if (i + j) % 2 == 0:
                    self.set_color(*color1)
                else:
                    self.set_color(*color2)

                glBegin(GL_POLYGON)
                glVertex([x, 0, z])
                glVertex([x, 0, z2])
                glVertex([x2, 0, z2])
                glVertex([x2, 0, z])
                glEnd()
        glEnable(GL_LIGHTING)

    def render_cylinder(self, pos, r, h, num_seg1=20, num_seg2=10):
        glPushMatrix()
        glTranslated(*pos)
        gluCylinder(self.quadric, r, r, h, num_seg1, num_seg2)
        glRotatef(180, 1, 0, 0)
        gluDisk(self.quadric, 0.0, r, num_seg1, 1)
        glRotatef(180, 1, 0, 0)
        glTranslatef(0.0, 0.0, h)
        gluDisk(self.quadric, 0.0, r, num_seg1, 1)
        glPopMatrix()

    def render_arrow(self, p, q, r_base=0.01, head_width=0.015, head_len=0.01):
        # glDisable(GL_LIGHTING)
        m_quadric = self.quadric
        gluQuadricNormals(m_quadric, GLU_SMOOTH)
        p = np.array(p)
        q = np.array(q)
        u = (q - p)
        u /= norm(u)
        P = q - head_len * u
        z = np.array([0, 0.0, 1.0])
        z /= norm(z)
        if norm(z - u) < 1e-8:
            axis = np.array([0, 0, 1])
            angle = 0.0
        else:
            axis = np.cross(z, u)
            axis /= norm(axis)
            angle = math.acos(u.dot(z))
        M = R_axis_angle(axis, angle)
        m = [M[0, 0], M[1, 0], M[2, 0], 0.0, M[0, 1], M[1, 1], M[2, 1], 0.0,
             M[0, 2], M[1, 2], M[2, 2], 0.0, P[0], P[1], P[2], 1.0]
        m2 = [M[0, 0], M[1, 0], M[2, 0], 0.0, M[0, 1], M[1, 1], M[2, 1], 0.0,
              M[0, 2], M[1, 2], M[2, 2], 0.0, p[0], p[1], p[2], 1.0]

        glPushMatrix()
        glMultMatrixd(m2)
        arrow_len = norm(q - p) - head_len
        # glColor(0.9, 0.2, 0.2)
        gluCylinder(m_quadric, r_base, r_base, arrow_len, 10, 10)
        glPopMatrix()

        glColor(1.0, 0.0, 0.0)
        glPushMatrix()
        glMultMatrixd(m)
        glutSolidCone(head_width, head_len, 10, 3)
        glPopMatrix()
        # gluDeleteQuadric(m_quadric)
        # glEnable(GL_LIGHTING)

    def render_axes(self, pos, r, rgb=True):
        o = np.array(pos)
        x = np.array([1.0, 0.0, 0.0])
        y = np.array([0.0, 1.0, 0.0])
        z = np.array([0.0, 0.0, 1.0])

        if rgb:
            self.color(1.0, 0.0, 0.0, 1.0)
        self.render_line(o - 0.5 * r * x, o + 0.5 * r * x)
        if rgb:
            self.color(0.0, 1.0, 0.0, 1.0)
        self.render_line(o - 0.5 * r * y, o + 0.5 * r * y)
        if rgb:
            self.color(0.0, 0.0, 1.0, 1.0)
        self.render_line(o - 0.5 * r * z, o + 0.5 * r * z)


def R_axis_angle(axis, angle):
    """Generate the rotation matrix from the axis-angle notation.
    Conversion equations
    ====================
    From Wikipedia (http://en.wikipedia.org/wiki/Rotation_matrix)::
        c = cos(angle); s = sin(angle); C = 1-c
        xs = x*s;   ys = y*s;   zs = z*s
        xC = x*C;   yC = y*C;   zC = z*C
        xyC = x*yC; yzC = y*zC; zxC = z*xC
        [ x*xC+c   xyC-zs   zxC+ys ]
        [ xyC+zs   y*yC+c   yzC-xs ]
        [ zxC-ys   yzC+xs   z*zC+c ]
    @param matrix:  The 3x3 rotation matrix to update.
    @type matrix:   3x3 numpy array
    @param axis:    The 3D rotation axis.
    @type axis:     numpy array, len 3
    @param angle:   The rotation angle.
    @type angle:    float
    """

    # Trig factors.
    ca = math.cos(angle)
    sa = math.sin(angle)
    C = 1 - ca

    # Depack the axis.
    x, y, z = axis

    # Multiplications (to remove duplicate calculations).
    xs = x * sa
    ys = y * sa
    zs = z * sa
    xC = x * C
    yC = y * C
    zC = z * C
    xyC = x * yC
    yzC = y * zC
    zxC = z * xC

    # Update the rotation matrix.
    matrix = np.zeros((3, 3))
    matrix[0, 0] = x * xC + ca
    matrix[0, 1] = xyC - zs
    matrix[0, 2] = zxC + ys
    matrix[1, 0] = xyC + zs
    matrix[1, 1] = y * yC + ca
    matrix[1, 2] = yzC - xs
    matrix[2, 0] = zxC - ys
    matrix[2, 1] = yzC + xs
    matrix[2, 2] = z * zC + ca
    return matrix
