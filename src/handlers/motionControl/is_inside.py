#!/usr/bin/env python
""" 
=================================================
is_inside.py - Polygon/Point Test Python Function
=================================================
    
A Python implementation of the ray to infinity even-odd test to determin if a point is inside the specified polygon.
"""

from numpy import *

def is_inside(p, vert):
    """
    This function tests whether the point p is inside the specified shape.
    Arguments:
    	p - the 2d point
    	vert - (2,N) array of points difining the polygon

    Returns:
    - True/False based on result of in/out test.

    Uses the 'ray to infinity' even-odd test.
    Let the ray be the horizontal ray starting at p and going to +inf in x.
    """
    
    V = hstack((vert, [[vert[0,0]],[vert[1,0]]]))

    inside = False
    x,y=p[0],p[1]
    for i in range(V.shape[1]-1):
        v0 = V[:, i]
        v1 = V[:, i+1]
        # Check if both verts to the left of ray
        if v0[0]<x and v1[0]<x:
            continue
        # check if both on the same side of ray
        if (v0[1]<y and v1[1]<y) or (v0[1]>y and v1[1]>y):
            continue
        #check for horizontal line - another horz line can't intersect it
        if (v0[1]==v1[1]):
            continue
        # compute x intersection value
        xisect = v0[0] + (v1[0]-v0[0])*((y-v0[1])/(v1[1]-v0[1]))
        if xisect >= x:
            inside = not inside
    return inside
