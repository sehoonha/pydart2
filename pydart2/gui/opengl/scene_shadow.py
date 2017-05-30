import numpy as np
from pydart2.gui.opengl.scene import OpenGLScene
from pydart2.gui.opengl.shader import Shader
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *


class OpenGLShadowScene(OpenGLScene):
    def __init__(self, width, height, window=None):
        super(OpenGLShadowScene, self).__init__(width, height, window)

        self.SHADOWMAP_SIZE = 1024
        # self.light_pos = np.array((60, 90, 50))
        self.light_pos = (1.0, 2.0, 1.0)
        self.light_dir = np.negative(self.light_pos)
        self.light_color = (1.0, 1.0, 1.0)
        self.light_inner_angle = 15  # default: 20
        self.light_outer_angle = 25  # default: 30

    def init(self, ):
        super(OpenGLShadowScene, self).init()
        self.init_shaders()
        self.init_depth_buffers()
        glClearColor(0.1, 0.1, 0.1, 1.0)
        glClearDepth(1.0)
        glEnable(GL_CULL_FACE)
        glEnable(GL_TEXTURE_2D)
        glEnable(GL_DEPTH_TEST)

    def init_shaders(self, ):
        from pydart2.gui.opengl.shader_programs import SHADER_DISPLAY_VERT
        from pydart2.gui.opengl.shader_programs import SHADER_DISPLAY_FRAG
        self.display_shader = Shader(SHADER_DISPLAY_VERT,
                                     SHADER_DISPLAY_FRAG)
        self.display_shader.compile()

        from pydart2.gui.opengl.shader_programs import SHADER_SHADOWMAP_VERT
        from pydart2.gui.opengl.shader_programs import SHADER_SHADOWMAP_FRAG
        self.shadowmap_shader = Shader(SHADER_SHADOWMAP_VERT,
                                       SHADER_SHADOWMAP_FRAG)
        self.shadowmap_shader.compile()
        print("[pydart2] init_shaders OK")

    def init_depth_buffers(self, ):
        # This sets up the depth texture and the framebuffer that captures the
        # depth component.
        SHADOWMAP_SIZE = self.SHADOWMAP_SIZE
        render_target = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, render_target)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SHADOWMAP_SIZE,
                     SHADOWMAP_SIZE, 0, GL_DEPTH_COMPONENT, GL_FLOAT, None)
        fbo = glGenFramebuffers(1)
        glBindFramebuffer(GL_FRAMEBUFFER, fbo)
        glFramebufferTexture2D(
            GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D,
            render_target, 0)
        glBindFramebuffer(GL_FRAMEBUFFER, 0)

        self.render_target = render_target
        self.fbo = fbo
        print("[pydart2] init_depth_buffers OK")

    def render(self, sim=None):
        self.render_shadowmap(sim)
        self.render_scene(sim)
        self.render_shadowmap_texture()
        # super(OpenGLShadowScene, self).render(sim)

    def render_shadowmap(self, sim=None):
        width, height = self.width, self.height
        light_pos = self.light_pos
        SHADOWMAP_SIZE = self.SHADOWMAP_SIZE

        self.shadowmap_shader.enable()
        glCullFace(GL_FRONT)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(60, float(width) / float(height), 0.01, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        params = list(light_pos) + [0, 0, 0] + [0, 1, 0]
        gluLookAt(*params)
        # We do not do the rotation here as the scene is not rotating RELATIVE
        # to the light
        light_proj = glGetFloatv(GL_PROJECTION_MATRIX).flatten()
        light_view = glGetFloatv(GL_MODELVIEW_MATRIX).flatten()
        glBindFramebuffer(GL_FRAMEBUFFER, self.fbo)
        glViewport(0, 0, SHADOWMAP_SIZE, SHADOWMAP_SIZE)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # glEnableClientState(GL_VERTEX_ARRAY)
        # glBindBuffer(GL_ARRAY_BUFFER, vbo)
        # glVertexPointer(3, GL_FLOAT, 0, None)
        # glBindBuffer(GL_ARRAY_BUFFER, 0)
        # glDrawArrays(GL_TRIANGLES, 0, int(len(v) / 3))
        # glutSolidCube(1.0)
        # glDisableClientState(GL_VERTEX_ARRAY)

        sim.render()
        self.test_render()

        self.shadowmap_shader.disable()

        self.light_proj = light_proj
        self.light_view = light_view
        # glBindFramebuffer(GL_FRAMEBUFFER, 0)

    def render_scene(self, sim=None):
        width, height = self.width, self.height
        light_pos = self.light_pos
        light_color = self.light_color
        light_dir = self.light_dir
        light_inner_angle = self.light_inner_angle
        light_outer_angle = self.light_outer_angle

        glCullFace(GL_BACK)
        bias = [0.5, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
                0.0, 0.0, 0.5, 0.0, 0.5, 0.5, 0.5, 1.0]
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glMultMatrixf(bias)
        glMultMatrixf(self.light_proj)
        glMultMatrixf(self.light_view)
        biasMVPMatrix = glGetFloatv(GL_MODELVIEW_MATRIX).flatten()
        glViewport(0, 0, width, height)
        glBindFramebuffer(GL_FRAMEBUFFER, 0)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, float(width) / float(height), 0.01, 100.0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslate(*self.tb.trans)
        glMultMatrixf(self.tb.matrix)

        # cameraPos = (0, 3, 4)
        # params = list(cameraPos) + [0, 0, 0] + [0, 1, 0]
        # gluLookAt(*params)
        # glRotate(rotation, 0, 1, 0)
        # cameraProj = glGetFloatv(GL_PROJECTION_MATRIX).flatten()
        # cameraView = glGetFloatv(GL_MODELVIEW_MATRIX).flatten()

        display_shader = self.display_shader
        display_shader.enable()
        glActiveTexture(GL_TEXTURE1)
        glBindTexture(GL_TEXTURE_2D, self.render_target)
        # glActiveTexture(GL_TEXTURE0)
        # glBindTexture(GL_TEXTURE_2D, modelTex)
        display_shader.setUniform("u_modelTexture", "sampler2D", 0)
        display_shader.setUniform("u_shadowMap", "sampler2D", 1)
        display_shader.setUniform("u_biasMVPMatrix", "mat4", biasMVPMatrix)
        display_shader.setUniform("u_light.color", "vec3", light_color)
        display_shader.setUniform("u_light.direction", "vec3", light_dir)
        display_shader.setUniform("u_light.position", "vec3", light_pos)
        display_shader.setUniform("u_light.innerAngle",
                                  "float", light_inner_angle)
        display_shader.setUniform("u_light.outerAngle",
                                  "float", light_outer_angle)

        sim.render()
        self.test_render()

        glPushMatrix()
        glTranslated(*self.light_pos)
        glColor4d(1.0, 1.0, 0.0, 1.0)
        # glutSolidCube(0.1)
        glutSolidSphere(0.2, 50, 20)
        glPopMatrix()

        # glEnableClientState(GL_VERTEX_ARRAY)
        # glEnableClientState(GL_TEXTURE_COORD_ARRAY)
        # glEnableClientState(GL_NORMAL_ARRAY)
        # glBindBuffer(GL_ARRAY_BUFFER, vbo)
        # glVertexPointer(3, GL_FLOAT, 0, None)
        # glBindBuffer(GL_ARRAY_BUFFER, tbo)
        # glTexCoordPointer(2, GL_FLOAT, 0, None)
        # glBindBuffer(GL_ARRAY_BUFFER, nbo)
        # glNormalPointer(GL_FLOAT, 0, None)
        # glDrawArrays(GL_TRIANGLES, 0, int(len(v) / 3))
        # glutSolidCube(40.0)
        # glBindBuffer(GL_ARRAY_BUFFER, 0)
        # glDisableClientState(GL_VERTEX_ARRAY)
        # glDisableClientState(GL_TEXTURE_COORD_ARRAY)
        # glDisableClientState(GL_NORMAL_ARRAY)

        display_shader.disable()

    def render_shadowmap_texture(self, ):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(-1, 1, -1, 1, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glActiveTexture(GL_TEXTURE1)
        glBindTexture(GL_TEXTURE_2D, self.render_target)
        glBegin(GL_QUADS)
        glColor3f(1, 1, 1)
        glTexCoord2f(0, 0)
        glVertex3f(0.5, 0.5, 0)
        glTexCoord2f(1, 0)
        glVertex3f(1, 0.5, 0)
        glTexCoord2f(1, 1)
        glVertex3f(1, 1, 0)
        glTexCoord2f(0, 1)
        glVertex3f(0.5, 1, 0)
        glEnd()

    def test_render(self, ):
        SCALE = 0.01
        glPushMatrix()
        glColor4d(0.1, 0.1, 0.5, 1.0)
        glTranslated(0.0, -0.89, 0.0)
        for i in range(1):
            for j in range(1):
                size = SCALE * 500.0 / 2.0
                normal = (0.0, 1.0, 0.0)
                glPushMatrix()
                glTranslated(i * 5.0, 0.0, j * 5.0)
                glBegin(GL_QUADS)
                glNormal3f(*normal)
                glVertex3f(-size, 0.0, -size)
                glNormal3f(*normal)
                glVertex3f(-size, 0.0, size)
                glNormal3f(*normal)
                glVertex3f(size, 0.0, size)
                glNormal3f(*normal)
                glVertex3f(size, 0.0, -size)
                glEnd()
                glPopMatrix()
        glPopMatrix()
        return

        glPushMatrix()
        glTranslated(0.0, SCALE * 5.0, 0.0)
        glColor4d(1.0, 1.0, 0.0, 1.0)
        # glutSolidSphere(20.0, 100, 50)
        glutSolidCube(SCALE * 20.0)
        glPopMatrix()

        glPushMatrix()
        glTranslated(SCALE * 30.0, SCALE * 30.0, SCALE * 10.0)
        glColor4d(0.8, 0.0, 0.3, 1.0)
        glutSolidSphere(SCALE * 10.0, 40, 20)
        glPopMatrix()
