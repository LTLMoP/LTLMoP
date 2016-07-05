#!/usr/bin/env python
"""
=================================================
rosDrive.py - Differential Drive Handler
=================================================

Converts a desired global velocity vector into translational and rotational rates for a differential-drive robot,
using feedback linearization.
"""

from math import sin, cos
import sys

import lib.handlers.handlerTemplates as handlerTemplates

class RosDriveHandler(handlerTemplates.DriveHandler):
    def __init__(self, executor, shared_data,d=0.6):
        """
        Initialization method of differential drive handler.

        d (float): Distance from front axle to point we are abstracting to [m] (default=0.6,max=0.8,min=0.2)
        """

        try:
            self.loco = executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler)
            self.coordmap = executor.hsub.coordmap_lab2map
        except NameError:
            print >>sys.__stdout__, "(DRIVE) Locomotion Command Handler not found."
            exit(-1)

        self.d = d

    def setVelocity(self, x, y, theta=0, z = 0):
        #print "VEL:%f,%f" % tuple(self.coordmap([x, y]))

        # Feedback linearization code:
        #d = 0.125 # Distance from front axle to point we are abstracting to [m]
        #d = 0.6 # Distance from front axle to point we are abstracting to [m]
        #vx = 0.09*X[0,0]
        #vy = 0.09*X[1,0]
        # ^^ Changed the scaling because it was getting stuck - too high of a velocity ? - Hadas 20/12/07
        vx = 0.29*x
        vy = 0.29*y
        vz = 0.29*z
        w = (1/self.d)*(-sin(theta)*vx + cos(theta)*vy)
        v = cos(theta)*vx + sin(theta)*vy

        self.loco.sendCommand([v,w,vz])

