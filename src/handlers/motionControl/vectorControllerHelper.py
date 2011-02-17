#!/usr/bin/env python
""" 
    =============================================================
    vectorControllerHelper.py - Convex Polygon Point Controller
    =============================================================
    
    A Python implementation of the Stephen R. Lindemann algorithm.
"""

from numpy import *
from scipy.linalg import norm
from numpy.matlib import zeros
import is_inside

def getController(p, vert, exit, last=False):
	"""
	This function implements an algorithm for smooth feedback plans over convex
	cells, as described by Stephen R. Lindemann in his paper: 'Smoothly Blending
	Vector Fields for Global Robot Navigation.' The algorithm calculates the
	value of a velocity vector field at a given point.  This vector field is
	defined such that it will direct the robot from any point in the region to
	the exit face or to the map referebce point (in the last region).
	The inputs are (given in order):
		p = the current x-y position of the robot
		vert = vertices of the region, specified in clockwise order (2 x No. of vertices)
		exit = index of exit face (i.e. if the exit face is the face defined by 
			vertices 3 and 4 then exit=3). If last = 1, any exit face will do...
		last = True, if the current region is the last region
			 = False, if the current region is NOT the last region
	"""
	
	[d, min_d, ROI] = getRegion(p, vert)
	s = getSParam(d, ROI)
	b = getBump(s)
	if size(array(ROI))==1:
		Vf = getFaceVF(vert, ROI, exit)
	else:
		Vf = [0, 0]
	Vc = getCellVF(p, vert, exit)
	Vel = getGlobalVF(b, Vf, Vc)
	return Vel

	
def getRegion(p, v):
	"""
	This function finds the distances from the robot's current position to each
	of the cell's faces, as well as the index of the face in whose region of
	influence the point lies.
	The inputs are (given in order):
		p = the current x-y position of the robot
		v = vertices of the region, specified in clockwise order (2 x No. of vertices)
	"""
	
	V = hstack((v, v[:, : 1]))
	d = zeros([v.shape[1], 1])
	for i in range(0, v.shape[1]):
		num = V[1, i+1] - V[1, i]
		den = V[0, i+1] - V[0, i]
		if den == 0:
			d[i, 0] = abs(p[0] - V[0, i])
		elif num == 0:
			d[i, 0] = abs(p[1] - V[1, i])
		else:
			d[i, 0] = abs((V[0, i+1] - V[0, i]) * (V[1, i] - p[1]) - \
				(V[0, i] - p[0]) * (V[1, i+1] - V[1, i])) / \
				sqrt(square(V[0, i+1] - V[0, i]) + square(V[1, i+1] - V[1, i]))
	min_d = min(d)
	ROI = d.argmin()
	return [d, min_d, ROI]


def getSParam(d, ROI):
	"""
	This function finds the s-parameter as defined by Lindemann.  The s-parameter
	is, essentially, the product of all of the fractional distances to the faces.
	The inputs are (given in order):
		d = the distance to the each of the faces of the cell
		ROI = the index of the face to which the current point is closest
	"""
	
	P = 1
	for i in range(0, d.shape[0]):
		if ROI != i:
			P = P * ((d[i] - d[ROI]) / d[i])
	s = 1-P
	return s


def getBump(s):
	"""
	This function performs a 'bump' function on the input parameter.  The result
	is a value that is normalized between 0 and 1, and is smooth over its entire
	domain.
	The input is:
		s = s-parameter, as defined in the preceding function
	"""
	
	if s <= 0:
		b = 1
	elif s >= 1:
		b = 0
	else:
		Ls = (1 / s) * exp(-1 / s)
		Ls_2 = (1 / (1-s)) * exp(-1 / (1-s))
		b = 1 - Ls / (Ls + Ls_2)
	return float(b)


def getFaceVF(vert, ROI, EF):
	"""
	This function find the value of the Face Vector Field for a given point in the
	cell.  The vector field is defined in the region of influence of a given face of
	the cell, as the unit normal vector of that face (outward-normal for the exit
	face, inward-normal for all other faces).
	The inputs are (given in order):
		vert = vertices of the region, specified in clockwise order (2 x No. of vertices)
		ROI = the index of the face to which the current point is closest
		EF = the index of the exit face
	"""
	
	Vert = hstack((vert, vert))
	dx = Vert[0, ROI+1] - Vert[0, ROI]
	dy = Vert[1, ROI+1] - Vert[1, ROI]
	Vf = [dy, -dx]/norm([dy, -dx])
	dx1 = Vert[0, ROI+2] - Vert[0, ROI+1]
	dy1 = Vert[1, ROI+2] - Vert[1, ROI+1]
	V1 = [dx1, dy1]/norm([dx1, dy1])
	ind = 1
	while dot(Vf, V1) == 0:
		dx1 = Vert[0, ROI+2+ind] - Vert[0, ROI+1+ind]
		dy1 = Vert[1, ROI+2+ind] - Vert[1, ROI+1+ind]
		V1 = [dx1, dy1]/norm([dx1, dy1])
		ind = ind + 1
	if dot(Vf, V1) < 0:
		Vf = -1*Vf
	if ROI == EF:
		Vf = -1*Vf
	return Vf


def getCellVF(p,vert,EF):
	"""
	This function takes calculates a vector field for the 'attractive' force in the
	cell. The attractive force seeks to drive the robot out of the cell in question.
	More specifically, this implementation of the Cell Vector Field drives the robot
	towards the midpoint of the exit face.
	The inputs are (given in order):
		p = the current x-y position of the robot
		vert = vertices of the region, specified in clockwise order (2 x No. of vertices)
		EF = the index of the exit face
	"""
	
	Vert = hstack((vert, vert[:, : 1]))
	dx = Vert[0, EF+1] - Vert[0, EF]
	dy = Vert[1, EF+1] - Vert[1, EF]
	mpx = Vert[0, EF] + dx/float(2)
	mpy = Vert[1, EF] + dy/float(2)
	Pa = array([mpx, mpy])
	Vc = (Pa-p[0:2]) / norm(Pa-p[0:2])
	return Vc
	
	
def getGlobalVF(bp,Vf,Vc):
	"""
	This function finds the overall vector field for the robot's location in the
	cell, incorporating the face and attractor vector fields.
	The inputs are (given in order):
		bp = the value of the bump function at the given point
		Vf = the value of the Face Vector Field
		Vc = the value of the Cell Vector Field
	"""
	
	V = bp*Vf + (1 - bp)*Vc
	V = V / norm(V)
	return V
