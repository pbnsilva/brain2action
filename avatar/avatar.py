import glfw
from OpenGL.GL import *
from OpenGL.GLU import *
import math
import time
import numpy as np


# Graphical parameters
theta = 0
is_mouse_down = False
GRID_SIZE = 20


def error_callback(error, description):
    print description


def key_callback(window, key, scancode, action, mods):
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
    for i in xrange(GRID_SIZE * 2):
        glBegin(GL_LINES)
        glColor3f(0.0, 0.0, 0.0)
        glVertex3f(-GRID_SIZE, 0.0, i - GRID_SIZE)
        glVertex3f(GRID_SIZE, 0.0, i - GRID_SIZE)
        glVertex3f(i - GRID_SIZE, 0.0, -GRID_SIZE)
        glVertex3f(i - GRID_SIZE, 0.0, GRID_SIZE)
        glEnd()


def draw_axes(x, y, z, xpos=1, ypos=1, zpos=1):
    glPushMatrix()
    glTranslatef(x, y + 0.1, z)
    glBegin(GL_LINES)
    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(xpos, 0.0, 0.0)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, ypos, 0.0)
    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, 0.0, zpos)
    glEnd()
    glPopMatrix()


def joint_draw(root):
    glPushMatrix()
    glTranslatef(root[1], root[2], root[3])
    for child in root[4]:
        glBegin(GL_LINES)
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(child[1], child[2], child[3])
        glEnd()
        joint_draw(child)
    glPopMatrix()


def create_joint(ix, x, y, z):
    return [ix, x, y, z, []]


def joint_add_child(parent, child):
    parent[4].append(child)


def main():
    root = create_joint(0, 0.0, 3.25, 0.0)
    shoulder_right = create_joint(1, -0.5, 0.0, 0.0)
    shoulder_left = create_joint(2, 0.5, 0.0, 0.0)
    back_base = create_joint(3, 0.0, -1.25, 0.0)
    elbow_right = create_joint(4, 0, -0.5, 0.0)
    elbow_left = create_joint(5, 0, -0.5, 0.0)
    wrist_right = create_joint(6, 0, -0.5, 0.0)
    wrist_left = create_joint(7, 0, -0.5, 0.0)
    hip_right = create_joint(8, -0.25, 0.0, 0.0)
    hip_left = create_joint(9, 0.25, 0.0, 0.0)
    knee_right = create_joint(10, 0.0, -1.0, 0.0)
    knee_left = create_joint(11, 0.0, -1.0, 0.0)
    ankle_right = create_joint(12, 0.0, -1.0, 0.0)
    ankle_left = create_joint(13, 0.0, -1.0, 0.0)
    joint_add_child(root, shoulder_right)
    joint_add_child(root, shoulder_left)
    joint_add_child(root, back_base)
    joint_add_child(shoulder_right, elbow_right)
    joint_add_child(shoulder_left, elbow_left)
    joint_add_child(elbow_right, wrist_right)
    joint_add_child(elbow_left, wrist_left)
    joint_add_child(back_base, hip_right)
    joint_add_child(back_base, hip_left)
    joint_add_child(hip_right, knee_right)
    joint_add_child(hip_left, knee_left)
    joint_add_child(knee_right, ankle_right)
    joint_add_child(knee_left, ankle_left)

    if not glfw.init():
        return

    window = glfw.create_window(1024, 768, "Avatar", None, None)
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
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glShadeModel(GL_SMOOTH)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

    while not glfw.window_should_close(window):
        width, height = glfw.get_framebuffer_size(window)
        ratio = width / float(height)

        glViewport(0, 0, width, height)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, ratio, 0.1, 100.0)

        eye_x = 10 * math.sin(theta)
        eye_z = 10 * math.cos(theta)
        gluLookAt(eye_x, 5.5, eye_z, 0.0, 1.5, 0.0, 0.0, 1.0, 0.0)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)

        glLoadIdentity()

        draw_grid()
        draw_axes(-3, 0, 0)
        joint_draw(root)

        glfw.swap_buffers(window)

        glfw.poll_events()

    glfw.terminate()


if __name__ == "__main__":
    main()
