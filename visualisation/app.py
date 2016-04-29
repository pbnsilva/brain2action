import glfw
from OpenGL.GL import *
from OpenGL.GLU import *
import math
from utils import Quaternion
from model import Model, Animation


WINDOW_WIDTH = 1440
WINDOW_HEIGHT = 900
old_x = 0
theta = 0
is_mouse_down = False
GRID_SIZE = 20


def error_callback(error, description):
    print description


def key_callback(window, key, scancode, action, mods):
    global K
    if key == glfw.KEY_ESCAPE and action == glfw.PRESS:
        glfw.set_window_should_close(window, GL_TRUE)


def mouse_button_callback(window, button, action, mods):
    global is_mouse_down
    if button == glfw.MOUSE_BUTTON_LEFT and action == glfw.PRESS:
        old_x, _ = glfw.get_cursor_pos(window)
        is_mouse_down = True
    elif button == glfw.MOUSE_BUTTON_LEFT and action == glfw.RELEASE:
        is_mouse_down = False


def cursor_position_callback(window, xpos, ypos):
    global theta
    global old_x
    if is_mouse_down:
        theta += (old_x - xpos) * 0.01
    old_x = xpos


def draw_grid():
    glDisable(GL_LIGHTING)
    for i in xrange(GRID_SIZE * 2):
        glBegin(GL_LINES)
        glColor3f(0.0, 0.0, 0.0)
        glVertex3f(-GRID_SIZE, 0.0, i - GRID_SIZE)
        glVertex3f(GRID_SIZE, 0.0, i - GRID_SIZE)
        glVertex3f(i - GRID_SIZE, 0.0, -GRID_SIZE)
        glVertex3f(i - GRID_SIZE, 0.0, GRID_SIZE)
        glEnd()
    glEnable(GL_LIGHTING)


def draw_axes(x, y, z, xpos=0.5, ypos=0.5, zpos=0.5):
    glDisable(GL_LIGHTING)
    glPushMatrix()
    glTranslatef(x, y + 0.1, z)
    glBegin(GL_LINE_STRIP)
    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(xpos, 0.0, 0.0)
    glEnd()
    glBegin(GL_LINE_STRIP)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, ypos, 0.0)
    glEnd()
    glBegin(GL_LINE_STRIP)
    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, 0.0, zpos)
    glEnd()
    glPopMatrix()
    glEnable(GL_LIGHTING)


def init_lights():
    ambient = [.2, .2, .2, 1.0]
    diffuse = [.7, .7, .7, 1.0]
    specular = [1, 1, 1, 1]
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse)
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular)

    pos = [-4, 2.5, 2.5, 1]
    glLightfv(GL_LIGHT0, GL_POSITION, pos)

    glEnable(GL_LIGHT0)


def draw_model(model):
    glEnableClientState(GL_VERTEX_ARRAY)
    glVertexPointer(3, GL_DOUBLE, 0, model.vertex_array)
    glEnableClientState(GL_NORMAL_ARRAY)
    glNormalPointer(GL_DOUBLE, 0, model.normal_array)

    glCullFace(GL_BACK)

    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, [0.8, 0.8, 1.0])
    glDrawArrays(GL_TRIANGLES, 0, len(model.vertex_indices))

    glDisableClientState(GL_VERTEX_ARRAY)
    glDisableClientState(GL_NORMAL_ARRAY)


def draw_joints(model):
    glPushMatrix()
    glPointSize(5.0)
    glBegin(GL_POINTS)
    for k in model.skeleton.joints:
        p = model.skeleton.joints[k].absolute_translation
        glVertex3f(p[0], p[1], p[2])
    glEnd()
    glPopMatrix()


def main():

    model = Model('model.ram')

    model.skeleton.joints[15].relative_rotation = Quaternion(0.5, 0.5, 0.5, 1)
    for k in model.skeleton.joints:
        model.skeleton.joints[k].update_absolutes()
    model.transform_vertices()

    animation = Animation('walk.raa')
    model.add_animation('walk', animation)

    if not glfw.init():
        return

    window = glfw.create_window(WINDOW_WIDTH, WINDOW_HEIGHT, "Avatar", None, None)
    if not window:
        glfw.terminate()
        return

    glfw.make_context_current(window)
    glfw.swap_interval(1)

    glfw.set_key_callback(window, key_callback)
    glfw.set_mouse_button_callback(window, mouse_button_callback)
    glfw.set_cursor_pos_callback(window, cursor_position_callback)

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

    init_lights()

    while not glfw.window_should_close(window):
        width, height = glfw.get_framebuffer_size(window)
        ratio = width / float(height)

        glViewport(0, 0, width, height)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, ratio, 0.1, 100.0)

        eye_x = 5 * math.sin(theta)
        eye_z = 5 * math.cos(theta)
        gluLookAt(eye_x, 2, eye_z, 0.0, 0.9, 0.0, 0.0, 1.0, 0.0)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)

        glLoadIdentity()

        draw_grid()
        draw_axes(-2, 0, 0)

        glColor3f(1.0, 1.0, 1.0)
        glTranslatef(0, 1.13, 0)
        draw_model(model)

        # glTranslatef(0, 1.13, 0)
        # glScalef(model._scale_factor, model._scale_factor, model._scale_factor)
        # draw_joints(model)

        # model.update_animation('walk', do_transform_vertices=False)

        glfw.swap_buffers(window)

        glfw.poll_events()

    glfw.terminate()


if __name__ == "__main__":
    main()
