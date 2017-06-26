#!/usr/bin/python
# -*- raytracer -*-

from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
import numpy as np
from math import cos, radians, sin


import odak

__author__ = ("Kishore Rathinavel")

window = 0
width, height = 1000, 720

# Variables for mouse iteractions
v_mouse = [0.0, 0.0]
mouseLeft, mouseMiddle, mouseRight = False, False, False
eyeVectorAxisAngles = [ 90.0, 45.0 ]
eyeDistFromOrigin = 100.0
upVec = [0.0, 0.0, 1.0]

def callGLVertex3f(pt):
    glVertex3f(pt[0], pt[1], pt[2])

def callGLNormal3f(pt):
    glNormal3f(pt[0], pt[1], pt[2])

def drawRay(vector, distance):
    loc = [vector[0,0,0], vector[0,1,0], vector[0,2,0]]
    dir = [vector[1,0,0], vector[1,1,0], vector[1,2,0]]
    des = loc + distance*dir
    # print des
    # input ('Enter key to continue')
    glBegin(GL_LINES)
    callGLVertex3f(loc)
    callGLVertex3f(des)
    glEnd()

def drawMeshAsPoints(mesh):
    dims = mesh.shape
    # print dims
    # input ('Enter key to continue')
    glBegin(GL_POINTS)
    for i in xrange(0, dims[1] - 1):
        callGLVertex3f(mesh[:,i])
    glEnd()

def drawMeshAsLines(mesh):
    sampleno = mesh.shape[0]
    glBegin(GL_LINES)
    for i in xrange(0, sampleno - 1):
        for j in xrange(0, sampleno - 1):
            callGLVertex3f(mesh[i,j])
            callGLVertex3f(mesh[i+1,j])
            callGLVertex3f(mesh[i,j+1])
    glEnd()

def drawMeshAsPolygons(mesh):
    sampleno = mesh.shape[0]
    glBegin(GL_POLYGON)
    for i in xrange(0, sampleno - 1):
        for j in xrange(0, sampleno - 1):
            callGLVertex3f(mesh[i,j])
            callGLVertex3f(mesh[i+1,j])
            callGLVertex3f(mesh[i,j+1])
    glEnd()

def drawMeshAsTriangles(mesh):
    sampleno = mesh.shape[0]
    for i in xrange(0, sampleno - 1):
        for j in xrange(0, sampleno - 1):
            glBegin(GL_TRIANGLES)
            callGLNormal3f(mesh[i,j])
            callGLVertex3f(mesh[i,j])
            callGLNormal3f(mesh[i+1,j])
            callGLVertex3f(mesh[i+1,j])
            callGLNormal3f(mesh[i,j+1])
            callGLVertex3f(mesh[i,j+1])
            glEnd()


def setupCamera(width, height):
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    # glOrtho(-20,20,-20,20,-0.1,20000)
    glFrustum(-0.1, 0.1, -0.1, 0.1, 0.1, 1000)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    eyePos = [0.0, 0.0, 0.0]
    eyePos[0] = eyeDistFromOrigin * cos(radians(eyeVectorAxisAngles[1])) * cos(radians(eyeVectorAxisAngles[0]))
    eyePos[1] = eyeDistFromOrigin * cos(radians(eyeVectorAxisAngles[1])) * sin(radians(eyeVectorAxisAngles[0]))
    eyePos[2] = eyeDistFromOrigin * sin(radians(eyeVectorAxisAngles[1]))
    gluLookAt(eyePos[0], eyePos[1], eyePos[2], 0.0, 0.0, 0.0, upVec[0], upVec[1], upVec[2])

def mouseFunc(button, state, x, y):
    global v_mouse, mouseLeft, mouseMiddle, mouseRight
    v_mouse[0] = x
    v_mouse[1] = y

    if (state == GLUT_DOWN):
        if (button == GLUT_LEFT_BUTTON): mouseLeft = True 
        elif (button == GLUT_MIDDLE_BUTTON): mouseMiddle = True 
        elif (button == GLUT_RIGHT_BUTTON): mouseRight = True 
    else:
        if (button == GLUT_LEFT_BUTTON): mouseLeft = False 
        elif (button == GLUT_MIDDLE_BUTTON): mouseMiddle = False 
        elif (button == GLUT_RIGHT_BUTTON): mouseRight = False 

def motionFunc(x, y):
    global eyeVectorAxisAngles, eyeDistFromOrigin
    dx = x - v_mouse[0]
    dy = y - v_mouse[1]
    v_mouse[0] = x;
    v_mouse[1] = y;
    if (mouseLeft):
        eyeVectorAxisAngles[1] = (eyeVectorAxisAngles[1] + (0.05*dy))
        if(sin(radians(eyeVectorAxisAngles[1])) > 0):
            if(y > height/2):
                eyeVectorAxisAngles[0] = (eyeVectorAxisAngles[0] - (0.1*dx))
            else:
                eyeVectorAxisAngles[0] = (eyeVectorAxisAngles[0] + (0.1*dx))
        else:
            if(y > height/2):
                eyeVectorAxisAngles[0] = (eyeVectorAxisAngles[0] + (0.1*dx))
            else:
                eyeVectorAxisAngles[0] = (eyeVectorAxisAngles[0] - (0.1*dx))
        if(eyeVectorAxisAngles[1] < -89):
            eyeVectorAxisAngles[1] = -89
        if(eyeVectorAxisAngles[1] > 89):
            eyeVectorAxisAngles[1] = 89
    else:
        eyeDistFromOrigin += 0.05*dx
        if(eyeDistFromOrigin < 0.5):
            eyeDistFromOrigin = 0.5

def rotateRayWorldZ(ray, angle):
    loc = ray[:1,:]
    dir = ray[1:,:]
    mat = np.array([[cos(radians(angle)), -sin(radians(angle)), 0], [sin(radians(angle)), cos(radians(angle)), 0], [0, 0, 1]])
    new_dir = mat.dot(dir)
    new_dir = np.reshape(new_dir, dir.shape)
    new_ray = np.concatenate((loc, new_dir), axis=0)
    return new_ray

def example_of_ray_tracing():
    n         = 1
    raytracer       = odak.raytracing()

    sphere = raytracer.plotsphericallens(0,0,0,10, PlotFlag=False)
    mesh = raytracer.CalculateSpherMesh(sphere)

    glColor4f(0.0, 0.0, 1.0, 0.9)
    # drawMeshAsPolygons(mesh)
    # drawMeshAsTriangles(mesh)
    drawMeshAsLines(mesh)

    glColor3f(1.0, 1.0, 0.0)
    for angle in xrange(-20, 20):
        # ray            = raytracer.createvector((-20,0, 20),(angle, 90 - angle, 135))
        ray            = raytracer.createvector((-20,0, 20),(45, 90, 135))
        ray = rotateRayWorldZ(ray, angle)
        # print ray
        glColor3f(1.0, 1.0, 0.0)
        distance,normvec  = raytracer.findinterspher(ray,sphere)
        if distance != 0:
            drawRay(ray,distance)
            refractedRay = raytracer.snell(ray,normvec,1.0,1.51)
            distance,normvec  = raytracer.findinterspher(refractedRay,sphere)
        if distance != 0:
            drawRay(refractedRay,distance)
            refractedRay2 = raytracer.snell(refractedRay,normvec,1.51,1.)
            drawRay(refractedRay2, np.array([40]))
    return


def drawWorldCoordinateAxes():
    inf = 10000;
    glBegin(GL_LINES)
    glColor3f(0.5, 0, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(inf, 0, 0)
    glColor3f(0, 0.5, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(0, inf, 0)
    glColor3f(0, 0, 0.5)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, inf)
    glEnd()
    inf = -10000;
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(20, 0xAAAA)
    glBegin(GL_LINES)
    glColor3f(0.5, 0, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(inf, 0, 0)
    glColor3f(0, 0.5, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(0, inf, 0)
    glColor3f(0, 0, 0.5)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, inf)
    glEnd()
    glDisable(GL_LINE_STIPPLE);

def draw():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    setupCamera(width, height)
    drawWorldCoordinateAxes()
    example_of_ray_tracing()
    glutSwapBuffers()

def keyboard(key, x, y):
    if(key == chr(27)):
        exit(0)
    return

glutInit()
glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
glutInitWindowSize(width, height)
glutInitWindowPosition(0,0)
window = glutCreateWindow("trial")
glutDisplayFunc(draw)
glutKeyboardFunc(keyboard)
glutMotionFunc(motionFunc)
glutMouseFunc(mouseFunc)
glutIdleFunc(draw)
glEnable(GL_DEPTH_TEST)
glEnable(GL_LINE_SMOOTH)
glEnable(GL_POLYGON_SMOOTH)
glEnable(GL_BLEND)
glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
glutMainLoop()
