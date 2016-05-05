import glfw
from OpenGL.GL import *
from OpenGL.GLU import *
import math
from rendering.model import Model
from rendering.animation import Animation
from rendering.scene import Scene
from rendering.utils import Quaternion


WINDOW_WIDTH = 1440
WINDOW_HEIGHT = 900
old_x = 0
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


def main():

    scene = Scene()

    model = Model('data/model.ram')
    model.color = [1, 1, 1]
    model.position = [0, 1.13, 0]

    animation = Animation('data/walk.raa')
    model.add_animation('walk', animation)

    scene.add_model(model)

    if not glfw.init():
        return

    window = glfw.create_window(WINDOW_WIDTH, WINDOW_HEIGHT, "brain2action", None, None)
    if not window:
        glfw.terminate()
        return

    glfw.make_context_current(window)
    glfw.swap_interval(1)

    glfw.set_key_callback(window, key_callback)
    glfw.set_mouse_button_callback(window, mouse_button_callback)
    glfw.set_cursor_pos_callback(window, cursor_position_callback)

    scene.init()

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

        scene.draw()

        glfw.swap_buffers(window)
        glfw.poll_events()

    glfw.terminate()


if __name__ == "__main__":
    main()
