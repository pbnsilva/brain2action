from OpenGL.GL import *
from OpenGL.GLU import *


class Scene:

    def __init__(self):
        self._grid_size = 20
        self._models = []

    def init(self):
        glClearColor(0.16, 0.16, 0.16, 1.0)
        glClearDepth(1.0)
        glEnable(GL_LIGHTING)
        glEnable(GL_CULL_FACE)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glShadeModel(GL_SMOOTH)
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glEnable(GL_COLOR_MATERIAL)

        ambient = [.2, .2, .2, 1.0]
        diffuse = [.7, .7, .7, 1.0]
        specular = [1, 1, 1, 1]
        glLightfv(GL_LIGHT0, GL_AMBIENT, ambient)
        glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse)
        glLightfv(GL_LIGHT0, GL_SPECULAR, specular)

        pos = [-4, 2.5, 2.5, 1]
        glLightfv(GL_LIGHT0, GL_POSITION, pos)

        glEnable(GL_LIGHT0)

    def resize(self, width, height):
        ratio = width / float(height)
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, ratio, 0.1, 100.0)

    def draw(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        self.draw_grid()
        self.draw_axes(-2, 0, 0)

        for model in self._models:
            # glColor3f(*model.color)
            # glTranslatef(*model.position)
            # self.draw_model(model)

            for animation in model.animations:
                model.update_animation(animation, do_transform_vertices=False)

            glTranslatef(0, 1.13, 0)
            glScalef(model._scale_factor, model._scale_factor, model._scale_factor)
            self.draw_joints(model)

    def draw_model(self, model):
        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointer(3, GL_DOUBLE, 0, model.vertex_array)
        glEnableClientState(GL_NORMAL_ARRAY)
        glNormalPointer(GL_DOUBLE, 0, model.normal_array)

        glCullFace(GL_BACK)

        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, [0.8, 0.8, 1.0])
        glDrawArrays(GL_TRIANGLES, 0, len(model.vertex_indices))

        glDisableClientState(GL_VERTEX_ARRAY)
        glDisableClientState(GL_NORMAL_ARRAY)

    def draw_joints(self, model):
        glPushMatrix()
        glPointSize(5.0)
        glLineWidth(4.0)
        glBegin(GL_POINTS)
        for k in model.skeleton.joints:
            p = model.skeleton.joints[k].absolute_translation
            glVertex3f(p[0], p[1], p[2])
        glEnd()
        glLineWidth(1.0)
        glPopMatrix()

    def draw_grid(self):
        grid_size = self._grid_size
        glDisable(GL_LIGHTING)
        for i in xrange(grid_size * 2):
            glBegin(GL_LINES)
            glColor3f(0.0, 0.0, 0.0)
            glVertex3f(-grid_size, 0.0, i - grid_size)
            glVertex3f(grid_size, 0.0, i - grid_size)
            glVertex3f(i - grid_size, 0.0, -grid_size)
            glVertex3f(i - grid_size, 0.0, grid_size)
            glEnd()
        glEnable(GL_LIGHTING)

    def draw_axes(self, x, y, z):
        glDisable(GL_LIGHTING)
        glPushMatrix()
        glTranslatef(x, y + 0.1, z)
        glBegin(GL_LINE_STRIP)
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.5, 0.0, 0.0)
        glEnd()
        glBegin(GL_LINE_STRIP)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 0.5, 0.0)
        glEnd()
        glBegin(GL_LINE_STRIP)
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 0.5)
        glEnd()
        glPopMatrix()
        glEnable(GL_LIGHTING)

    def add_model(self, model):
        self._models.append(model)
