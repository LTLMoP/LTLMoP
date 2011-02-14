#!/usr/bin/env python
"""
==================================================
skeletonController.py - Skeleton Motion Controller
==================================================
"""

import vectorControllerHelper
from numpy import *
from is_inside import *

class motionControlHandler:
    def __init__(self, proj, shared_data):
        # Get references to handlers we'll need to communicate with
        self.drive_handler = proj.drive_handler
        self.pose_handler = proj.pose_handler
        
        # Get information about regions
        self.rfi = proj.rfi
        self.coordmap_map2lab = proj.coordmap_map2lab

    def gotoRegion(self, current_reg, next_reg, last=False):
        """
        If ``last`` is True, we will move to the center of the destination region.

        Returns ``True`` if we've reached the destination region.
        """

        if current_reg == next_reg and not last:
            # No need to move!
            self.drive_handler.setVelocity(0, 0)  # So let's stop
            return False

        # Find our current configuration
        pose = self.pose_handler.getPose()

        # TODO: Calculate a velocity vector in the *GLOBAL REFERENCE FRAME* 
        # that will get us on our way to the next region

        # NOTE: Information about region geometry can be found in self.rfi.regions:
        pointArray = [x for x in self.rfi.regions[current_reg].getPoints()]
        pointArray = map(self.coordmap_map2lab, pointArray)
	vertices = mat(pointArray).T 

	if last:
            transFace = None
        else:
            # Find a face to go through
            # TODO: Account for non-determinacy?
            pt1, pt2 = self.rfi.transitions[current_reg][next_reg][0]
            
            # Find the index of this face
            # TODO: Why don't we just store this as the index?
            for i, face in enumerate([x for x in self.rfi.regions[current_reg].getFaces()]):
                # Account for both face orientations...
                if (pt1 == face[0] and pt2 == face[1]) or (pt1 == face[1] and pt2 == face[0]):
                    transFace = i
                    break

	V = vectorControllerHelper.getController([pose[0], pose[1]], vertices, transFace)

        [vx, vy, w] = [0, 0, 0]

        # Pass this desired velocity on to the drive handler
        self.drive_handler.setVelocity(V[0], V[1], pose[2])
        
        # TODO: Figure out whether we've reached the destination region
	
	if is_inside([pose[0], pose[1]], vertices):
	        arrived = False
	else:
		arrived = True

        return arrived
