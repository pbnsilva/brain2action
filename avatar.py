import glfw
from OpenGL.GL import *
from OpenGL.GLU import *
import math
import serial
import time
import numpy as np

# Notes
# When initialising have the imus facing outside and not forward!!!
READ_SENSOR_DATA = True

# Arduino parameters
port = "/dev/cu.usbserial-A4007TOu"
baudrate = 115200

# Graphical parameters
theta = 0
is_mouse_down = False
GRID_SIZE = 20

# Geometry parameters
w8_to_stabilize = 5 # seconds
rr = [0.0, 0.0, 0.0, 0.0]
quat = None
collect_4_offset = 10
offset_q = None

# Other parameters
talk = False


def error_callback(error, description):
    print description


def key_callback(window, key, scancode, action, mods):
    if key == glfw.KEY_ESCAPE and action == glfw.PRESS:
        glfw.set_window_should_close(window, GL_TRUE)


def mouse_button_callback(window, button, action, mods):
    global is_mouse_down
    if button == glfw.MOUSE_BUTTON_LEFT and action == glfw.PRESS:
        old_x, _ = glfw.get_cursor_pos(window)
        print "Add y axis rotation"
        is_mouse_down = True
    elif button == glfw.MOUSE_BUTTON_LEFT and action == glfw.RELEASE:
        is_mouse_down = False


def cursor_position_callback(window, xpos, ypos):
    global theta
    global old_x
    if is_mouse_down:
        theta += (old_x - xpos) * 0.01
        print "Add y axis rotation"
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
    glTranslatef(x, y, z)
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


def joint_draw(root, q):
    # glPushMatrix()
    glLoadIdentity()
    glTranslatef(root[1], root[2], root[3])
    for child in root[4]:
        idx = child[0]
        if idx in (4, 6):
            a = 2.0 * np.rad2deg(np.arccos(q[idx][0]))
            if abs(a) > 0.5:
                # 0 degree means no rotation
                n = [qi / np.sqrt(1.0-q[idx][0]**2) for qi in q[idx][1:]]
                print a, n
                # Figure out the right angles
                # glRotatef(a, n[0], n[1], n[2])
                # glRotatef(a, n[0], n[2], n[1])
                # glRotatef(a, n[1], n[0], n[2])
                # glRotatef(a, n[1], n[2], n[0])
                # glRotatef(a, n[2], n[0], n[1])
                # glRotatef(a, n[2], n[1], n[0])
                
                # Figure out the right direction
                glRotatef(a, -n[2], n[0], -n[1])
                
                # Version on the hand
                # The version on the box
                # glRotatef(a, -n[1], n[2], -n[0])
                                

        glBegin(GL_LINES)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(0, 0, 0)
        # glColor3f(1.0, 0.0, 0.0)
        glVertex3f(child[1], child[2], child[3])
        glVertex3f(child[1], child[2], child[3])
        glVertex3f(child[1], child[2], child[3]+0.25)
        glEnd()
        # if child[0] == 4:
        #     break
        joint_draw(child, q)
    # glPopMatrix()


def create_joint(ix, x, y, z):
    return [ix, x, y, z, []]


def joint_add_child(parent, child):
    parent[4].append(child)


def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z

def q_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

def qv_mult(q1, v1):
    q2 = (0.0,) + v1
    return q_mult(q_mult(q1, q2), q_conjugate(q1))[1:]

def announce(msg):
    if talk:
        subprocess.call(['spd-say', msg])
    print msg

def normalize(v):
    v = np.array(v)
    v = v / np.linalg.norm(v)
    return v.tolist()

def get_quarterions(ser):
    ser.reset_input_buffer()

    qs = [None,None]
    while qs[0] is None or qs[1] is None:
        line = ser.readline()

        if 'IMU #' not in line:
            # print 'Not parsed:', line
            continue

        try:
            candidates = line.split('IMU #')[1:]
            assert len(candidates) == 2
            for idx, candidate in enumerate(candidates):
                candidate = candidate.strip() # The first one is empty
                candidate = map(float, candidate.split('\t')[2:])
                assert len(candidate) == 4 
                q = normalize(candidate)
                qs[idx] = q
        except:
            pass
            # print "WARNING: Could not parse the following to quaterion: [{}]".format(line)

    return {6:qs[1], 4:qs[0]}

def get_quarterion_offset(ser):
    announce('Waiting for the IMU to stabilize. Do not move.')
    start = time.time()
    last_check = w8_to_stabilize
    while time.time() - start < w8_to_stabilize: 
        q = get_quarterions(ser)
        check = np.round(w8_to_stabilize - (time.time()-start))
        if check < last_check:
            print check, 
            for key in q:
                print key, np.round(q[key],2),
            print
            last_check = check
        time.sleep(0.1)

    announce('Measuring offset. Do not move.')
    global offset_q
    offset_q = {}
    for i in xrange(collect_4_offset):      
        q = get_quarterions(ser)
        for key in q:
            values = offset_q.get(key,[]) 
            values.append(q[key])
            offset_q[key] = values
        
    for key in offset_q:
        offset_q[key] = np.array(offset_q[key]).mean(0).tolist()

def main():
    global rr

    if READ_SENSOR_DATA:
        ser = serial.Serial(port, baudrate)
        ser.timeout = 10
        get_quarterion_offset(ser)

    root = create_joint(0, 0.0, 0.0, 0.0)
    shoulder_right = create_joint(1, -0.5, 0.0, 0.0)
    shoulder_left = create_joint(2, 0.5, 0.0, 0.0)
    back_base = create_joint(3, 0.0, -1.25, 0.0)
    elbow_right = create_joint(4, 0, -0.5, 0.0)
    elbow_left = create_joint(5, 0, -0.5, 0.0)
    wrist_right = create_joint(6, 0, -0.5, 0.0)
    wrist_left = create_joint(7, 0, -0.5, 0.0)
    # hip_right = create_joint(8, -0.25, 0.0, 0.0)
    # hip_left = create_joint(9, 0.25, 0.0, 0.0)
    # knee_right = create_joint(10, 0.0, -1.0, 0.0)
    # knee_left = create_joint(11, 0.0, -1.0, 0.0)
    # ankle_right = create_joint(12, 0.0, -1.0, 0.0)
    # ankle_left = create_joint(13, 0.0, -1.0, 0.0)
    joint_add_child(root, shoulder_right)
    joint_add_child(root, shoulder_left)
    joint_add_child(root, back_base)
    joint_add_child(shoulder_right, elbow_right)
    joint_add_child(shoulder_left, elbow_left)
    joint_add_child(elbow_right, wrist_right)
    joint_add_child(elbow_left, wrist_left)
    # joint_add_child(back_base, hip_right)
    # joint_add_child(back_base, hip_left)
    # joint_add_child(hip_right, knee_right)
    # joint_add_child(hip_left, knee_left)
    # joint_add_child(knee_right, ankle_right)
    # joint_add_child(knee_left, ankle_left)

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
        if READ_SENSOR_DATA:
            q = get_quarterions(ser)
            for key in q:
                q[key] = q_mult(q_conjugate(offset_q[key]), q[key])
                q[key] = normalize(q[key])
            print q

        width, height = glfw.get_framebuffer_size(window)
        ratio = width / float(height)

        glViewport(0, 0, width, height)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, ratio, 0.1, 100.0)

        eye_x = 10 * math.sin(theta)
        print "Add y axis rotation"
        eye_z = 10 * math.cos(theta)
        gluLookAt(eye_x, 5.5, eye_z, 0.0, 1.5, 0.0, 0.0, 1.0, 0.0)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)

        glLoadIdentity()

        draw_grid()

        draw_axes(-3, 0, 0)

        glTranslatef(0.0, 3.25, 0.0)
        joint_draw(root, q)

        glfw.swap_buffers(window)

        glfw.poll_events()

    if READ_SENSOR_DATA:
        ser.close()

    glfw.terminate()

if __name__ == "__main__":
    main()
