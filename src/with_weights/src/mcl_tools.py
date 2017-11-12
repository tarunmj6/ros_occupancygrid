
# mcl_tools
#
# Some utility and visualization functions to help with
# planar Monte Carlo Localization.

import sys
import math
import time
import random
import roslib

# You'll need to go into src/raycaster and type "make"
# to build the C extension.
try:
    from raycaster import *
except:
    print "Error importing raycaster extension, do"
    print
    print "    sudo apt-get install swig"
    print "    (roscd sample_hw7/src/raycaster && make)"
    print
    sys.exit(1)

MAP_WIDTH  = 53.9
MAP_HEIGHT = 10.6
MAP_PPM    = 10.0 
LASER_MAX  = 5.0

parset    = []
map_img   = None
raycaster = None

real_pose = (0,0,0,0)
best_pose = (0,0,0,0)

try:
    import Image
    import numpy
    from OpenGL.GLUT import *
    from OpenGL.GL import *
    from OpenGL.GLU import *
except:
    print "Error importing libraries, try"
    print
    print "    apt-get install python-opengl python-numpy"
    print
    sys.exit(1)

# Generate and sample particles.

import mcl_debug

def random_particle():
    x = random.uniform(0, MAP_WIDTH)
    y = random.uniform(0, MAP_HEIGHT)
    t = random.uniform(0, 2*math.pi)
    w = random.uniform(0, 1)
    return (x, y, t, w)

def random_sample(xs, size, p):
    p = numpy.array(p, dtype=numpy.double, ndmin=1, copy=0)
    cdf = p.cumsum()
    cdf /= cdf[-1]
    uniform_samples = numpy.random.random(size)
    idx = cdf.searchsorted(uniform_samples, side='right')
    return map(lambda ii: xs[ii], idx)

# Operations to convert world <-> image pixel coords.

def clamp(xx, x0, x1):
    return int(min(max(xx, x0), x1))

def wx_to_px(wx):
    global map_img
    (ww, hh) = map_img.size
    return clamp((wx / MAP_WIDTH) * ww, 0, ww - 1)

def wy_to_py(wy):
    global map_img
    (ww, hh) = map_img.size
    return hh - clamp((wy / MAP_HEIGHT) * hh, 0, hh - 1)

def px_to_wx(px):
    global map_img
    (ww, hh) = map_img.size
    return (float(px)/ww) * MAP_WIDTH

def py_to_wy(py):
    global map_img
    (ww, hh) = map_img.size
    return MAP_HEIGHT - ((float(py)/hh) * MAP_HEIGHT)

def wt_to_pt(pt):
    x = math.cos(pt)
    y = math.sin(pt)
    return math.atan2(-y, x)

# Operations to perform raycasting on the map image.

def map_hit(wx, wy):
    global raycaster
    global map_img
    (ww, hh) = map_img.size

    px = wx_to_px(wx)
    py = wy_to_py(wy)

    wall = raycaster_test_pixel(raycaster, px, py)
    return wall == 1

def map_range(particle, phi):
    global raycaster
    global map_img
    (ww, hh) = map_img.size

    (wx, wy, theta, weight) = particle
    px = wx_to_px(wx)
    py = wy_to_py(wy)
    pt = wt_to_pt(theta+phi)

    return raycaster_cast(raycaster, px, py, pt)

# OpenGL visualization

def show_real_pose(particle):
    global real_pose
    real_pose = particle
    time.sleep(0.01)

def show_best_pose(particle):
    global best_pose
    best_pose = particle
    time.sleep(0.01)

def show_particles(parset1):
    global parset
    parset = parset1
    time.sleep(0.01)

def gl_display():
    global parset
    global real_pose
    global best_pose

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    # Draw background.
    glEnable( GL_TEXTURE_2D )
    glColor3f(0.5, 0.5, 0.5)
    glBegin(GL_QUADS)
    glTexCoord2f(0, 1)
    glVertex2f(0, 1)
    glTexCoord2f(0, 0)
    glVertex2f(0, 0)
    glTexCoord2f(1, 0)
    glVertex2f(1, 0)
    glTexCoord2f(1, 1)
    glVertex2f(1, 1)
    glEnd()

    # Draw particles.
    glDisable( GL_TEXTURE_2D )
    glColor3f(0.0, 1.0, 0.0)
    glPointSize(1.0)
    glBegin(GL_POINTS)
    for pt in parset:
        (x, y, theta, w) = pt
        x = x / MAP_WIDTH
        y = y / MAP_HEIGHT
        glVertex2f(x, y)
    glEnd()

    # Draw poses
    glPointSize(5.0)
    glBegin(GL_POINTS)
    glColor3f(1.0, 0.0, 0.0)
    glVertex2f(real_pose[0] / MAP_WIDTH, real_pose[1] / MAP_HEIGHT)
    glColor3f(0.0, 0.3, 1.0)    
    glVertex2f(best_pose[0] / MAP_WIDTH, best_pose[1] / MAP_HEIGHT)
    glEnd()

    glutSwapBuffers()

def gl_idle():
    glutPostRedisplay()
    time.sleep(0.05)

def gl_click(button, state, px, py):
    if button != 0 or state != 1:
        return

    wx = px_to_wx(px)
    wy = py_to_wy(py)

    print "Click at world =", wx, wy, "; image =", px, py, "; hit =", map_hit(wx, wy)

def mcl_init(pkg_name):
    global map_img
    global raycaster

    pkg_dir = roslib.packages.get_pkg_dir(pkg_name)
    map_img = Image.open(pkg_dir + "/hallway.png")
    (ww, hh) = map_img.size

    print "pixels per meter:", ww / MAP_WIDTH

    raycaster = raycaster_init(str(pkg_dir + "/hallway.png"))

    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB)
    glutInitWindowSize(ww, hh)
    glutInitWindowPosition(100, 100)
    glutCreateWindow(sys.argv[0])

    gluOrtho2D(0.0, 1.0, 0.0, 1.0)

    glEnable( GL_POINT_SMOOTH );
    glEnable( GL_BLEND );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    glPointSize( 1.0 );

    glClearColor ( 0, 0, 0, 0 )
    glShadeModel( GL_SMOOTH )
    glDisable( GL_LIGHTING )
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR )
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR )
    glTexImage2D( GL_TEXTURE_2D, 0, 3, ww, hh, 0, GL_RGB, GL_UNSIGNED_BYTE, 
                  map_img.tostring("raw", "RGB", 0, -1))
    glEnable( GL_TEXTURE_2D )

    glutDisplayFunc(gl_display)
    glutIdleFunc(gl_idle)
    glutMouseFunc(gl_click)

def mcl_run_viz():
    glutMainLoop()
