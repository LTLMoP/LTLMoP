#!/usr/bin/env python

import ode, xode.parser
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import math, time, copy, sys

from loadModules import *
from parseTextFiles import *
from matrixFunctions import *
from CKBotSimHelper import *

# needs to add the path of ltlmop_root to sys path
sys.path.append('../../../..')
import lib.regions

info = """CKBotSim

CKBot Simulator for LTLMoP
[Sebastian Castro - Cornell University]
[Autonomous Systems Laboratory - 2010]

"""

class CKBotSim:
    """
    CKBot Simulator Class
    """

    fps = 15.0
    cameraDistance = 40.0
    vel = 0.5
    turn = 0.5
    turn2 = 0.5
    clip = 1000.0
    res = (800, 600)

    def __init__(self, robotfile, standalone=0,obstaclefile=None,regionfile=None,region_calib=None,startingpose=None,heightmap=None):
        """
        Initialize the simulator.
        """

        # Simulation world parameters.
        self._initOpenGL()
        self.world = ode.World()
        self.world.setGravity((0, -9.81, 0))
        self.world.setERP(0.1)
        self.space = ode.Space()
        self.ground = ode.GeomPlane(space=self.space, normal=(0,1,0), dist=0)

        # CKBot module parameters.
        self.cubesize = 6.0
        self.cubemass = 10.0
        self.hingemaxangle = 1.6
        self.hingeminangle = -1.6
        self.hingemaxforce = 1000000

        # Control parameters.
        self.gait = 0
        self.gain = 1.5
        self.counter = 0
        
        # Setting standalone to 1 allows for manual key input.
        # Setting standalone to 0 (LTLMoP mode) causes the camera to automatically follow the spawned robot)
        if standalone==1:
            self.standalone = 1
        else:
            self.standalone = 0   
        self.clock = pygame.time.Clock()

        # If regionfile=0, render the ground as default solid green terrain.
        # Otherwise, load the regions file specified and draw the region colors on the ground.
        # Initialize the region file interface
        self.rfi = lib.regions.RegionFileInterface()

        # Load a region file if it has been specified on instantiation.
        if (regionfile!=None):
            self.rfi.readFile(regionfile)
            self.region_calib = region_calib
        
        # Make obstacles if they exist.
        if (obstaclefile!=None):
            loadObstacles(self, obstaclefile)

        # Create region heights if they are specified.
        self.heightmap = heightmap
        if (self.heightmap!=None):
            loadRegionHeights(self, self.heightmap)

        # Define Modular Arrays
        self.body = []
        self.lowerjoint = []
        self.upperjoint = []
        self.hinge = []
        self.fixed = []
        self.M = []

        # Configuration/Gait Data
        self.config = "none"
        self.connM = []    
        self.basepos = []
        self.baserot = genmatrix(0,1)          # Identity Matrix for now.
        self.gaits = []
        loadRobotData(self, robotfile)
        
        # Obstacle Data
        self.obstacle = []
        self.obstaclepiece = []
        self.obstacleM = []

        # Account for the "startingpose" argument which translates all the modules
        self.startingpose = startingpose
        if self.startingpose != None:
            tempx = self.basepos[0] + self.startingpose[0]
            tempy = self.basepos[1] + self.startingpose[1]
            tempz = self.basepos[2] + self.startingpose[2]     
            self.basepos = (tempx, tempy, tempz)

        # Load the objects.
        loadModuleObjects(self)
        self._cjoints = ode.JointGroup()

        self._xRot = 0.0
        self._yRot = 0.0

        self._xCoeff = 360.0 / 480.0
        self._yCoeff = 360.0 / 640.0

        self._vel = 0.0
        self._turn = 0.0
        self._turn2 = 0.0

        # Camera alignment parameters.
        self._ax = 0.0
        self._ay = 0.0
        self._az = 0.0
        self._align = 1.0
        self.clicking = False


    def _initOpenGL(self):
        """
        Initialise the scene.
        """

        # Create a window
        pygame.init()
        screen = pygame.display.set_mode(self.res, pygame.OPENGL | pygame.DOUBLEBUF)
        pygame.display.set_caption('CKBot Simulation')
        pygame.mouse.set_visible(False)

        glViewport(0, 0, self.res[0], self.res[1])
        glClearColor(0.8, 0.8, 0.9, 0)

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_NORMALIZE)
        glShadeModel(GL_FLAT)
        

    def _extractMatrix(self, geom):
        """
        Return a 4x4 matrix (represented by a 16-element tuple) created by
        combining the geom's rotation matrix and position.
        """

        x, y, z = geom.getPosition()
        rot = geom.getRotation()
        return (rot[0], rot[3], rot[6], 0.0,
                rot[1], rot[4], rot[7], 0.0,
                rot[2], rot[5], rot[8], 0.0,
                x, y, z, 1.0)


    def _renderGeom(self, geom, color):
        """
        Render either a ode.GeomBox or ode.GeomSphere object.
        """

        allowed = [ode.GeomBox, ode.GeomSphere]
        ok = False
        for klass in allowed:
            ok = ok or isinstance(geom, klass)
            if (not ok):
                return

        glPushMatrix()
        glMultMatrixd(self._extractMatrix(geom))

        glMaterialfv(GL_FRONT, GL_SPECULAR, color)

        if (isinstance(geom, ode.GeomBox)):
            sx, sy, sz = geom.getLengths()
            glScale(sx, sy, sz)
            glutInit()
            glutSolidCube(1)
        elif (isinstance(geom, ode.GeomSphere)):
            r = geom.getRadius()
            glutSolidSphere(r, 20, 20)

        glPopMatrix()
        

    def _renderGround(self):
        """
        Renders the ground plane.
        """

        normal, d = self.ground.getParams()
        x, y, z = self.lowerjoint[0].getPosition()

        # Draw a quad at the position of the vehicle that extends to the
        # clipping planes.

        if (self.rfi.regions==[]):

            glPushMatrix()
            glTranslate(x, 0.0, z)

            glMaterialfv(GL_FRONT, GL_SPECULAR, (0.0, 0.5, 0.0))

            glBegin(GL_QUADS)
            glNormal3f(*normal)
            glVertex3f(-self.clip, d, -self.clip)
            glNormal3f(*normal)
            glVertex3f(self.clip, d, -self.clip)
            glNormal3f(*normal)
            glVertex3f(self.clip, d, self.clip)
            glNormal3f(*normal)
            glVertex3f(-self.clip, d, self.clip)
            glEnd()

            glPopMatrix()
            
            # Render squares on the ground in order to observe motion of the robot.
            for i in range(-10,11,2):
                for j in range(-10,11,2):
                    glPushMatrix()
                    color = (0, 0, 0.5)
                    glMaterialfv(GL_FRONT, GL_SPECULAR, color)
                    glBegin(GL_POLYGON)
                    glNormal3f(*normal)
                    glVertex3f(-10 + 20*i, 0.1, -10 + 20*j)
                    glNormal3f(*normal)
                    glVertex3f(-10 + 20*i, 0.1, 10 + 20*j)
                    glNormal3f(*normal)
                    glVertex3f(10 + 20*i, 0.1, 10 + 20*j)
                    glNormal3f(*normal)
                    glVertex3f(10 + 20*i, 0.1, -10 + 20*j)
                    glEnd()
                    glPopMatrix()
                    
            # Render a triangle in the x direction
            glPushMatrix()
            color = (0.5, 0, 0)
            glMaterialfv(GL_FRONT, GL_SPECULAR, color)
            glBegin(GL_POLYGON)            
            glNormal3f(*normal)
            glVertex3f(10, 0.1, 10)
            glNormal3f(*normal)
            glVertex3f(10, 0.1, -10)
            glNormal3f(*normal)
            glVertex3f(25, 0.1, 0)
            glEnd()
            glPopMatrix()

        # If we have region data, draw the individual regions and color them accordingly.
        else:
            # Render the boundary.
            glPushMatrix()
            color = (1, 1, 1)
            glMaterialfv(GL_FRONT, GL_SPECULAR, color)

            glBegin(GL_POLYGON)
            for idx in range(len(self.boundary_data)):
                glNormal3f(*normal)
                glVertex3f(self.boundary_data[idx][0]*self.region_calib[0], d, -self.boundary_data[idx][1]*self.region_calib[1])
            glEnd()    
            glPopMatrix()    
    
            # Render the remaining regions.
            for region in self.rfi.regions:
                glPushMatrix()

                glMaterialfv(GL_FRONT, GL_SPECULAR, [x for x in region.color])

                glBegin(GL_POLYGON)
                for pt in region.getPoints():
                    glNormal3f(*normal)
                    glVertex3f(pt[0]*self.region_calib[0], d, -pt[1]*self.region_calib[1])
                glEnd()

                glPopMatrix()

    def _setCamera(self):
        """
        Position the camera to C{self.cameraDistance} units behind the
        vehicle's current position and rotated depending on the mouse position.
        """

        aspect = float(self.res[0]) / float(self.res[1])

        x, y = pygame.mouse.get_pos()
        if self.clicking:
            self._xRot = (y - self.res[1]/2) * self._xCoeff
            self._yRot = (x - self.res[0]/2) * self._yCoeff
        if (self._xRot < 0):
            self._xRot = 0

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glFrustum(-aspect, aspect, -1.0, 1.0, 1.5, self.clip)

        glLightfv(GL_LIGHT0, GL_POSITION, (-5.0, 10.0, 0, 0))
        glLightfv(GL_LIGHT0,GL_DIFFUSE, (1.0, 1.0, 1.0, 1.0))
        glLightfv(GL_LIGHT0,GL_SPECULAR, (1.0, 1.0, 1.0, 1.0))
        glEnable(GL_LIGHT0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # Set the camera angle to view the vehicle
        glTranslate(0.0, 0.0, -self.cameraDistance)
        glRotate(self._xRot, 1, 0, 0)
        glRotate(self._yRot, 0, 1, 0)

        # Set the camera so that the vehicle is drawn in the correct place.
        if (self._align == 1 or self.standalone == 0):
            x, y, z = self.lowerjoint[0].getPosition()
            self._ax = x
            self._ay = y
            self._az = z
            glTranslate(-self._ax, -self._ay, -self._az)


    def render(self):
        """
        Render the current simulation state.
        """

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self._renderGround()

        self._setCamera()

        for geom in self._geoms:
            self._renderGeom(geom,(0,0,0))

        if (self.heightmap != None):
            i = 0
            for geom in self.heightObstacles:
                self._renderGeom(geom,self.heightColors[i])
                i = i + 1

        glFlush()
        pygame.display.flip()


    def _keyDown(self, key):
        if (key == pygame.K_w):
            self._vel = self.vel
        elif (key == pygame.K_a):
            self._turn = -self.turn
        elif (key == pygame.K_d):
            self._turn = self.turn
        elif (key == pygame.K_s):
            self._vel = -self.vel
        elif (key == pygame.K_q):
            self._turn2 = self.turn2
        elif (key == pygame.K_e):
            self._turn2 = -self.turn2
        elif (key == pygame.K_ESCAPE):
            self._running = False
        elif (key == pygame.K_l):
            if self._align == 1:
                self._align = 0
            else:
                self._align = 1

        # If running in standalone mode, can press the keys 1-5 to set gaits 1-5.
        # Yes, only 5 gaits are supported but if you need to add more simply extend the code below!
        elif self.standalone == 1:
            if (key == pygame.K_0):
                self.gait = 0
            elif (key == pygame.K_1):
                if len(self.gaits) >= 1:
                    self.gait = 1
            elif (key == pygame.K_2):
                if len(self.gaits) >= 2:
                    self.gait = 2
            elif (key == pygame.K_3):
                if len(self.gaits) >= 3:
                    self.gait = 3
            elif (key == pygame.K_4):
                if len(self.gaits) >= 4:
                    self.gait = 4
            elif (key == pygame.K_5):
                if len(self.gaits) >= 5:
                    self.gait = 5
                    
            # Reconfiguration.
            elif (key == pygame.K_r):
                config = raw_input("Enter Configuration Name: ")
                reconfigure(self, config)


    def _keyUp(self, key):
        if (key == pygame.K_w):
            self._vel = 0.0
        elif (key == pygame.K_a):
            self._turn = 0.0
        elif (key == pygame.K_d):
            self._turn = 0.0
        elif (key == pygame.K_s):
            self._vel = 0.0
        elif (key == pygame.K_q):
            self._turn2 = 0.0
        elif (key == pygame.K_e):
            self._turn2 = 0.0
    
    
    def doEvents(self):
        """
        Process any input events.
        """

        events = pygame.event.get()

        for e in events:
            if (e.type == pygame.QUIT):
                self._running = False
            elif (e.type == pygame.KEYDOWN):
                self._keyDown(e.key)
            elif (e.type == pygame.KEYUP):
                self._keyUp(e.key)
            elif (e.type == pygame.MOUSEBUTTONDOWN):
                if e.button == 1:
                    self.clicking = True
                elif e.button == 4:
                    self.cameraDistance -= 5
                elif e.button == 5:
                    self.cameraDistance += 5
            elif (e.type == pygame.MOUSEBUTTONUP):
                if e.button == 1:
                    self.clicking = False


    def _nearcb(self, args, geom1, geom2):
        """
        Create contact joints between colliding geoms.
        """

        body1, body2 = geom1.getBody(), geom2.getBody()
        if (body1 is None):
            body1 = ode.environment
        if (body2 is None):
            body2 = ode.environment

        if (ode.areConnected(body1, body2)):
            return

        contacts = ode.collide(geom1, geom2)

        for c in contacts:
            c.setBounce(0.1)
            c.setMu(10000)
            j = ode.ContactJoint(self.world, self._cjoints, c)
            j.attach(body1, body2)


    def run(self):
        """
        Start the demo. This method will block until the demo exits.
        This method is used if the simulator is run stand-alone.
        """

        self.clock = pygame.time.Clock()
        self._running = True
        self.doEvents()

        # Receive Locomotion commands for all the hinges from LTLMoP.
        # Use these commands as reference angles for simple P-controlled servos.    
        while self._running:

            self.doEvents()

            counter = self.counter

            rungait(self)
            
            # Simulation Step
            self.space.collide((), self._nearcb)
            self.world.step(1/self.fps)
            self._cjoints.empty()
            self.render()

            # Limit the FPS.
            #self.clock.tick(self.fps)
            self.counter = self.counter + 1
            

    def run_once(self):
        """
        Run one simulation step -- used for LTLMoP integration.
        """

        self._running = True
        self.doEvents()

        # Receive Locomotion commands for all the hinges from LTLMoP.
        # Use these commands as reference angles for simple P-controlled servos.
        rungait(self)

        # Simulation Step
        self.space.collide((), self._nearcb)
        self.world.step(1/self.fps)
        self._cjoints.empty()
        self.render()

        # Limit the FPS.
        self.clock.tick(self.fps)
        self.counter = self.counter + 1


# Main method for standalone mode.
if (__name__ == '__main__'):
    """
    Instantiates a simulator and runs it in stand-alone mode with all default arguments.
    """

    print info

    curdir = os.getcwd()
    
    obstaclefile = None
    if len(sys.argv)==3:
            obstaclefile = "../obstacles/" + sys.argv[2] + ".obstacle"

    # Arguments to fill out
    #regionfile = "../../../../examples/looptest/looptest.regions"
    #heightmap = [["r1",3.0,3.0,"none"],["r3",0.0,3.0,"+x"],["r5",0.0,3.0,"-x"],["r4",0.0,3.0,"+y"],["r2",0.0,3.0,"-y"]]
    regioncalib = [1.0,-1.0]
    #startingpose = [regioncalib[0]*(250), 20, -regioncalib[1]*(250)]
    startingpose = [0, 0, 0]
    regionfile = None
    heightmap = None

    # Running from ckbot directory.
    if ("simulator" in curdir) and ("ode" in curdir) and ("ckbot" in curdir):
        robotfile = "config/" + sys.argv[1] + ".ckbot"
        sim = CKBotSim(robotfile, standalone=1, obstaclefile=obstaclefile,regionfile=regionfile,region_calib=regioncalib,startingpose=startingpose,heightmap = heightmap)
    
    # Running from src.
    else:
        robotfile = "lib/simulator/ode/ckbot/config/" + sys.argv[1] + ".ckbot"
        sim = CKBotSim(robotfile, standalone=0)
        
    #Ramp Test: Use for revamping obstacles. i.e. use no bodies/masses
    #geom = ode.GeomBox(space=sim.space, lengths=[10,10,10] )
    #geom.setPosition([50, 0, 0])
    #geom.setRotation(genmatrix(math.pi/4.0,1))
    #sim._geoms.append(geom)
        
    sim.run()
