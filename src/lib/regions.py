#!/usr/bin/env python

""" ===========================
    regions.py - Regions Module
    ===========================

    A simple module that defines a class for describing and manipulating
    rectangular and polygonal regions.

    This is completely free software; please feel free to adapt or use this in
    any way you like.

    Some parts extracted from pySketch by Erik Westra (ewestra@wave.co.nz)
"""

# TODO: store regions with absolute coordinates, not x/y-pos + relative!!

import os, sys
import fileMethods
import re, random, math
import Polygon, Polygon.Utils, os
import json

Polygon.setTolerance(0.01)

if 'wx' in sys.modules:
    print "wx is loaded, using wx objects for regions"

    import wx

    Point = wx.Point
    Size = wx.Size
    Color = wx.Colour
else:
    print "wx is NOT loaded, using ltlmop objects for regions"
    from numbers import Number

    class Point(object):
        def __init__(self, x, y):
            self.x = float(x)
            self.y = float(y)

        def __str__(self):
            return "(%f, %f)" % (self.x, self.y)

        def __add__(self, other):
            if isinstance(other, Point):
                return Point(self.x+other.x, self.y+other.y)
            else:
                raise TypeError("Points can only be added to other points")

        def __mul__(self, other):
            if isinstance(other, Number):
                return Point(self.x*other, self.y*other)
            else:
                raise TypeError("Points can only multiplied by numbers")

        def __rmul__(self, *args, **kwds):
            return self.__mul__(*args, **kwds)

        def __eq__(self, other):
            EPS = 1e-6

            return abs(self.x - other[0]) < EPS \
               and abs(self.y - other[1]) < EPS

        def __len__(self):
            return 2
    
        def __getitem__(self, idx):
            if idx == 0:
                return self.x
            elif idx == 1:
                return self.y
            else:
                raise IndexError()

        def __sub__(self, other):
            if isinstance(other, Point):
                return Point(self.x-other.x, self.y-other.y)
            else:
                raise TypeError("Points can only be subtracted from other points")

    class Size(Point):
        def __init__(self, w, h):
            super(Size, self).__init__(w, h)
        
        def GetWidth(self):  return self.x
        def GetHeight(self): return self.y

        def __getattr__(self, name):
            if name == "width": return self.x
            elif name == "height": return self.y
            else:
                raise AttributeError()

    class Color():
        def __init__(self, red=0, green=0, blue=0):
            self.color = (red, green, blue)

        def __getitem__(self, idx):
            return self.color[idx]
        
        def Red(self):   return self.color[0]
        def Green(self): return self.color[1]
        def Blue(self):  return self.color[2]

        def SetFromName(self, name):
            colorMapping = {'RED': (255,0,0),
                            'ORANGE': (255,128,0),
                            'YELLOW': (255,255,0),
                            'GREEN': (0,255,0),
                            'BLUE': (0,0,255),
                            'PURPLE': (255,0,255),
                            'WHITE': (255,255,255),
                            'BLACK': (0,0,0)}

            if name.upper() in colorMapping:
                self.color = colorMapping[name.upper()]
            else:
                raise NotImplementedError("Color '%s' not recognized" % name)
            
                

############################################################

# Direction IDs (for defining convexity):
[dir_CW, dir_CCW, dir_CONCAVE] = range(3)

# Region type IDs:
[reg_RECT, reg_POLY] = range(2)

# Selection handle IDs:
[handle_NONE,           handle_TOP_LEFT,        handle_TOP_RIGHT,
 handle_BOTTOM_RIGHT,   handle_BOTTOM_LEFT] = range(-1, 4)

############################################################

class prettierJSONEncoder(json.JSONEncoder):
    """ Subclass of JSONEncoder that stops indenting after 2 levels;
        only seems to work well with python2.6 version, not 2.7 :("""

    def _newline_indent(self, last_indent_level=[None]):
        if self.current_indent_level > 2 or last_indent_level[0] > 2:
            last_indent_level[0] = self.current_indent_level
            return ""
        else:
            last_indent_level[0] = self.current_indent_level
            return super(prettierJSONEncoder, self)._newline_indent()


class RegionFileInterface:
    """
    A wrapper class for handling collections of regions and associated metadata.

    Overview of exposed data structures:

        - background (string): relative path of background image file
        - regions (list): list of Region objects, with properties defined below
        - transitions (list of lists):
            * key1 = Region object index
            * key2 = Region object index
            * values = Lists of faces connecting the two regions
    """

    def __init__(self, background="None", regions=[], transitions=None):
        self.background = background
        self.regions = regions
        self.transitions = transitions
        self.filename = None

    def setToDefaultName(self, region):
        if region.name is '':
            # Find an available name
            region.name = 'r' + str(self.getNextAvailableRegionNumber())

    def indexOfRegionWithName(self, name):
        for i, region in enumerate(self.regions):
            if region.name.lower() == name.lower():
                return i
        print 'WARNING: Region "' + name + '" not found.'
        return -1

    def getCalibrationPoints(self):
        for region in self.regions:
            for index, bool in enumerate(region.alignmentPoints):
                if bool:
                    pts = [x for x in region.getPoints()]
                    yield([region.name, index, pts[index].x, pts[index].y])

    def getNextAvailableRegionNumber(self):
        """
        Look for the smallest region name of form r1, r2, ... available.
        """

        # First get all region names of the form we're interested in
        nums = []
        p = re.compile(r"^r(?P<num>\d+)$")
        
        for region in self.regions:
            m = p.match(region.name)
            if not m: continue
            nums.append(int(m.group('num')))
        
        nums.sort()

        last = 0
        for num in nums:
            if num == last + 1:
                last = num
            elif num == last:
                #print "Warning: Multiple regions with name 'r%d'." % num
                continue
            else:
                # There's a gap we can fill
                return last + 1
        
        # Everything was full, so let's just append to the end
        return last + 1

    def splitSubfaces(self, obj1, obj2):
        """
        If we have a face of region obj1 that overlaps with the face of another region obj2,
        (i.e. the obj1 face is collinear with and has at least one point on the obj2 face)
        we split the larger face into two or three parts as appropriate. 
        """

        # Doesn't make any sense to check against self
        if obj1 is obj2:
            return

        # TODO: Optimize this code; use numpy instead of wx

        COLLINEAR_TOLERANCE = 1  # pixel

        # Iterate over faces in obj1
        for face in obj1.getFaces():
            pta = Point(*face[0])
            ptb = Point(*face[1])

            # Since we are modifying obj2, it's easier to re-run the loop than keep track of indices
            # in the case that we are adding two points.  Lazy, yes, but effective for now.
            clean = False
            while not clean:
                clean = True
                for other_face in obj2.getFaces():
                    other_pta = Point(*other_face[0])
                    other_ptb = Point(*other_face[1])

                    [on_segment_a, d_a, pint_a] = pointLineIntersection(other_pta, other_ptb, pta)
                    [on_segment_b, d_b, pint_b] = pointLineIntersection(other_pta, other_ptb, ptb)
                    if d_a < COLLINEAR_TOLERANCE and d_b < COLLINEAR_TOLERANCE: # Check for collinearity
                        points = [x for x in obj2.getPoints()]

                        # Figure out where we would add a new point
                        idxa = points.index(other_pta)
                        idxb = points.index(other_ptb)
                        if (idxa == 0 and idxb == len(points)-1) or (idxb == 0 and idxa == len(points)-1):  
                            idx = 0
                        else:
                            idx = max(idxa, idxb)

                        # Add a point if appropriate
                        if on_segment_a and not (pta == other_pta or pta == other_ptb): 
                            obj2.addPoint(pta-obj2.position, idx)
                            clean = False
                            break
                        if on_segment_b and not (ptb == other_pta or ptb == other_ptb): 
                            obj2.addPoint(ptb-obj2.position, idx)
                            clean = False
                            break

    def recalcAdjacency(self):
        """
        Calculate the region adjacency matrix and a list of shared faces

        Returns a list of shared faces
        """

        # Calculate adjoining faces:
        self.transitions = [[[] for j in range(len(self.regions))] for i in range(len(self.regions))]

        transitionFaces = {} # This is just a list of faces to draw dotted lines on

        for obj in self.regions:
            for face in obj.getFaces(includeHole=True):                
                if face not in transitionFaces: transitionFaces[face] = []
                ignore = False
                for other_obj in transitionFaces[face]:
                    # Prevent detection of adjoining faces when Duplicate 
                    # command creates object on top of itself
                    if other_obj.position == obj.position and \
                       [x for x in other_obj.getPoints()] == [x for x in obj.getPoints()]:
                        ignore = True
    
                if not ignore:
                    transitionFaces[face].append(obj)

        toDelete = []
        for face, objarray in transitionFaces.iteritems():
            if len(objarray) > 1:
                # If this face is shared by multiple regions
                for obj in objarray:
                    key = self.regions.index(obj)
                    for other_obj in objarray:
                        if other_obj == obj: continue
                        key2 = self.regions.index(other_obj)
                        self.transitions[key][key2].append(face)
            else:
                # Otherwise mark for deletion (we can't delete it in the middle of iteration)
                toDelete.append(face)

        # Delete all those dudes
        for unused_face in toDelete:
            del transitionFaces[unused_face]                    

        return transitionFaces

    def getBoundingBox(self):
        if self.regions == []:
            return None
        
        p = self.regions[0]
        
        leftMargin = min([pt.x for region in self.regions for pt in region.getPoints()])
        topMargin = min([pt.y for region in self.regions for pt in region.getPoints()])
        rightExtent = max([pt.x for region in self.regions for pt in region.getPoints()]) - leftMargin
        downExtent = max([pt.y for region in self.regions for pt in region.getPoints()]) - topMargin

        return (leftMargin, topMargin, rightExtent, downExtent)

    def writeFile(self, filename):
        """
        File format is described inside the comments variable. 
        """

        comments = {"FILE_HEADER":"This is a region definition file for the LTLMoP toolkit.\n" +
                                  "Format details are described at the beginning of each section below.\n" +
                                  "Note that all values are separated by *tabs*.",
                    "Background": "Relative path of background image file",
                    "Regions": "Stored as JSON string",
                    "Transitions": "Region 1 Name, Region 2 Name, Bidirectional transition faces (face1_x1, face1_y1, face1_x2, face1_y2, face2_x1, ...)",
                    "CalibrationPoints": "Vertices to use for map calibration: (vertex_region_name, vertex_index)",
                    "Obstacles": "Names of regions to treat as obstacles"}
    
        regionData = []
        for r in self.regions:
            d = r.getData()
            # We don't want to store the following two attributes inside the regions
            del d['alignmentPoints']
            del d['isObstacle']
            regionData.append(d)
        je = prettierJSONEncoder(indent=4)
        regionData = [je.encode(regionData)]
       
        transitionData = []
        for region1, destinations in enumerate(self.transitions):
            # Note: We are assuming all transitions are bidirectional so we only have to include
            # the parts of the adjacency matrix above the diagonal
            for region2, faces in enumerate(destinations[region1+1:]):
                if faces == []: continue    # No transition between these regions, so skip

                faceData = []
                for face in faces:
                    faceData.extend([face[0][0], face[0][1], face[1][0], face[1][1]])

                transitionData.append("\t".join([self.regions[region1].name,
                                                 self.regions[region1 + 1 + region2].name] +
                                                 map(str, faceData)))

        calibPoints = []
        for region in self.regions:
            for index, isAP in enumerate(region.alignmentPoints):
                if isAP:
                    calibPoints.append("\t".join([region.name, str(index)]))

        calibPointsStr = "\t".join(calibPoints)

        obstacleRegions = [r.name for r in self.regions if r.isObstacle]

        data = {"Background": self.background,
                "Regions": regionData,
                "Transitions": transitionData,
                "CalibrationPoints": calibPoints,
                "Obstacles": obstacleRegions}

        fileMethods.writeToFile(filename, data, comments)

        return True

    def readFile(self, filename):
        """
        For file format information, refer to writeFile() above.
        """

        if not os.path.exists(filename):
            return False

        data = fileMethods.readFromFile(filename)

        if data is None:
            return False

        try:
            self.background = data["Background"][0]
        except KeyError:
            self.background = "None"

        self.regions = []
        rdata = data["Regions"]

        try:
            rdata = json.loads("\n".join(rdata))
            compatMode = False
        except ValueError:
            compatMode = True
        
        # If we are not reading a decomposed file, assume that specEditor is
        # requesting a read of the new file format produced by regionEditor
        # Need to get the corner of the bounding box to offset all regions by
        if not "_decomposed" in filename:
            xOff = -min([float(pt[0]) for rd in rdata for pt in rd['points']])
            yOff = -min([float(pt[1]) for rd in rdata for pt in rd['points']])
            offsets = (xOff, yOff)
        for rd in rdata:
            newRegion = Region()
            if compatMode:
                regionData = rd.split();
                newRegion.setDataOld(regionData)    
            else:
                # If we are not reading a decomposed file, assume that
                # specEditor is requesting a read of the new file format
                # produced by regionEditor
                # New file format has region points in absolute coordinates in
                # real world units, and all regions are polygons
                if not "_decomposed" in filename:
                    rd = self.convertRegionDataNewToOld(rd, offsets)
                newRegion.setData(rd)

            self.regions.append(newRegion)

        # Make an empty adjacency matrix of size (# of regions) x (# of regions)
        self.transitions = [[[] for j in range(len(self.regions))] for i in range(len(self.regions))]
        for transition in data["Transitions"]:
            transition = re.sub('[\[\]\(\)]', '', transition) # Remove brackets
            transData = transition.split();
            region1 = self.indexOfRegionWithName(transData[0])
            region2 = self.indexOfRegionWithName(transData[1])
            faces = []
            
            # If we are not reading a decomposed file, assume that specEditor
            # is requesting a read of the new file format produced by
            # regionEditor
            # Transitions are defined by face index, not point values, and
            # uni-directional transitions are possible
            if not "_decomposed" in filename:
                for i in range(2, len(transData), 2):
                    ip1 = int(transData[i])
                    ip2 = int(transData[i % len(self.regions[region1].pointArray)])
                    p1 = self.regions[region1].pointArray[ip1]
                    p2 = self.regions[region1].pointArray[ip2]
                    faces.append(tuple(sorted((p1, p2))))
                
                # Don't assume bidirectional transitions
                # Adjacency matrix may be nonsymmetric
                self.transitions[region1][region2] = faces
            
            else:
                for i in range(2, len(transData), 4):
                    p1 = Point(int(transData[i]), int(transData[i+1]))
                    p2 = Point(int(transData[i+2]), int(transData[i+3]))
                    faces.append(tuple(sorted((p1, p2))))
                
                # During adjacency matrix reconstruction, we'll mirror over the diagonal
                self.transitions[region1][region2] = faces
                self.transitions[region2][region1] = faces

        if "CalibrationPoints" in data:
            for point in data["CalibrationPoints"]:
                [name, index] = point.split()
                self.regions[self.indexOfRegionWithName(name)].alignmentPoints[int(index)] = True

        if "Obstacles" in data:
            for rname in data["Obstacles"]:
                self.regions[self.indexOfRegionWithName(rname)].isObstacle = True
            
        self.filename = filename

        return True
    
    def convertRegionDataNewToOld(self, rd, offsets):
        """
        Take in data on all the regions that was read in the new file format
        and convert it to data on the regions in the old format. Points will be
        scaled by 1000 (to get milimeter accuracy) and other fields that are
        unused in the new format will be set.
        
        rd - Single region data dictionary
        offsets - Whole map offsets to avoid negative coordinates
                  (xOffset, yOffset)
        """
        # name, position, size, color, type, points, holeList
        # name and color can stay the same
        rdnew = {'name':rd['name'],
                 'color':rd['color']}
        
        # type is always polygon
        rdnew['type'] = 'poly'
        
        
        #########################
        # TODO:
        #   -Don't do the stupid 1000 multiplier
        #   -Make transformation matrix
        #   -Pass in proj into this function
        #   -Save transformation matrix to calibration
        #########################
        
        
        
        # Each point in points and holeList must be converted to pixels
        # Also get bounding box then convert all points to relative coordinates
        points = [(int(1000.0 * pt[0]), int(-1000.0 * pt[1])) \
                  for pt in rd['points']]
        leftMargin = min([pt[0] for pt in points])
        topMargin = min([pt[1] for pt in points])
        rightExtent = max([pt[0] for pt in points]) - leftMargin
        downExtent = max([pt[1] for pt in points]) - topMargin
        points = [(pt[0] - leftMargin, pt[1] - topMargin) for pt in points]
        holeList = [(int(1000.0 * pt[0]) - leftMargin, \
                     int(-1000.0 * pt[1]) - topMargin) \
                    for hole in rd['holeList'] for pt in hole]
        rdnew['points'] = points
        rdnew['holeList'] = holeList
        xOff, yOff = offsets
        xOff = int(xOff * 1000)
        yOff = int(yOff * 1000)
        rdnew['position'] = (leftMargin + xOff, topMargin + yOff)
        rdnew['size'] = (rightExtent, downExtent)
        
        return rdnew
   
############################################################
 
class Region:
    """ A rectangular or polygonal region, defined by the following properties:

            - 'name'          Region name
            - 'type'          What type of region this is (rect or poly)
            - 'position'      The position of the object within the document 
                              (i.e., the top-left corner of the region's bounding-box)
            - 'size'          The size of the object's bounding box
            - 'color'         Color to use for drawing the region
            - 'pointArray'    Polygon points, relative to region position (stored in CW order)
            - 'alignmentPoints'  True/False array indicating for each vertex whether or not to use it as an alignment point
            - 'isObstacle'  Boolean value indicating whether the region should be treated as an obstacle

        NOTE: All coordinates are stored internally with (0,0) at the top left.
              X increases to right, and Y increases downwards.
    """

    def __init__(self, type=reg_POLY, position=Point(0, 0), size=Size(0, 0),
                 color=None, points=[], name=''):

        if color is None:
            # Give a random color
            color = Color()
            color.SetFromName(random.choice(['RED','ORANGE','YELLOW','GREEN','BLUE','PURPLE']))

        self.name              = name
        self.type              = type
        self.position          = position
        self.size              = size
        self.color             = color
        self.pointArray        = points
        self.alignmentPoints   = [False] * len([x for x in self.getPoints()])
        self.isObstacle = False
        self.holeList = []
    # =================================
    # == Region Manipulation Methods ==
    # =================================

    def addPoint(self, point, index):
        """
        Insert a new point at a given index, converting object type if necessary
        """
        if self.type == reg_RECT:
            # Convert from rect to poly :(
            self.pointArray = [p for p in self.getPoints(relative=True)]
            self.type = reg_POLY
        self.pointArray.insert(index, point)
        self.alignmentPoints.insert(index, False)
        self.recalcBoundingBox()

    def removePoint(self, index):
        """
        Remove the point at the given index, converting object type if necessary
        """
        if self.type == reg_POLY and len(self.pointArray) <= 3:
            # Hey now, don't go making lines
            return False
        if self.type == reg_RECT:
            # Convert from rect to poly :(
            self.pointArray = [p for p in self.getPoints(relative=True)]
            self.type = reg_POLY
        self.pointArray.pop(index)
        self.alignmentPoints.pop(index)
        self.recalcBoundingBox()
        return True

    def recalcBoundingBox(self):
        """
        Recalculate the bounding box for our object
        """
        
        # Find how much our bounding box has shifted in relation to the old one
        for i, pt in enumerate(self.getPoints(relative=True)):
            if i == 0:
                # Set initial values
                topLeftX  = pt.x
                topLeftY  = pt.y
                botRightX = pt.x
                botRightY = pt.y
            else:
                # Check for any points that would expand our bounds
                if pt.x > botRightX:
                    botRightX = pt.x
                if pt.x < topLeftX:
                    topLeftX = pt.x
                if pt.y < topLeftY:
                    topLeftY = pt.y
                if pt.y > botRightY:
                    botRightY = pt.y
    
        if self.type == reg_POLY:    
            # Shift our vertices to align with the new bounding box
            self.pointArray = map(lambda x: x-Point(topLeftX, topLeftY), self.pointArray)
            # Shift holes vertices to align with the new bounding box
            for i,hole in enumerate(self.holeList):
                self.holeList[i] =  map(lambda x: x-Point(topLeftX, topLeftY),self.holeList[i])
                

        # Store the new bounding box
        self.position = self.position + Point(topLeftX, topLeftY)
        self.size = Size(botRightX - topLeftX, botRightY - topLeftY)


    # =============================
    # == Object Property Methods ==
    # =============================

    def getDirection(self):
        """
        Determine convexity/concavity and the order of the points stored in the pointArray.
        For details on the algorithm, please refer to:
                 http://local.wasp.uwa.edu.au/~pbourke/geometry/clockwise/index.html
        """

        if self.type == reg_RECT:
            # We define RECTs as CW
            return dir_CW

        last_cp = None
        for [i, pt] in enumerate(self.pointArray):
            # Be sure to account for looping around the edges of the vertex array
            prev = self.pointArray[(i - 1) % len(self.pointArray)]
            next = self.pointArray[(i + 1) % len(self.pointArray)]

            cp = (pt.x - prev.x)*(next.y - pt.y) - (pt.y - prev.y)*(next.x - pt.x)

            if cp != 0:
                if last_cp is not None and (cp > 0) != (last_cp > 0):
                    return dir_CONCAVE
                last_cp = cp

        if last_cp > 0:
            return dir_CW
        else:
            return dir_CCW

    def getData(self):
        """ Return a copy of the object's internal data.
            This is used for undo and to save this region to disk.
        """

        data = {'name': self.name,
                'position': (self.position.x, self.position.y),
                'size': (self.size.width, self.size.height),
                'color': (self.color.Red(), self.color.Green(), self.color.Blue())}

        if self.type == reg_RECT:
            data['type'] = "rect"
        else:
            data['type'] = "poly"

        # Only include points if poly; rects are defined by location+dimension
        if self.type == reg_POLY:
            data['points'] = [(pt.x, pt.y) for pt in self.pointArray]
            data['holeList'] = []
            for hole in self.holeList:
                data['holeList'].append([(pt.x, pt.y) for pt in hole])
                
        data['alignmentPoints'] = [i for i, isAP in enumerate(self.alignmentPoints) if isAP]

        data['isObstacle'] = self.isObstacle

        return data

    def setData(self, data):
        """ Set the object's internal data.

            'data' is a copy of the object's saved data, as returned by
            getData() above.  This is used for undo and to restore a 
            previously saved region.
        """
        self.name = data['name']
        self.position = Point(*data['position'])
        self.size = Size(*data['size'])
        self.color = Color(*data['color'])

        if data['type'].lower() == "rect":
            self.type = reg_RECT
        else:
            self.type = reg_POLY

        # Only load pointArray if type is poly
        if self.type == reg_POLY:
            self.pointArray = [Point(*pt) for pt in data['points']]
            self.holeList = []
            for hole in data['holeList']:
                self.holeList.append([Point(*pt) for pt in hole])
        if 'alignmentPoints' in data:
            self.alignmentPoints = [(i in data['alignmentPoints']) for i, p in enumerate(self.getPoints())]
        else:
            self.alignmentPoints = [False] * len([x for x in self.getPoints()])

        if 'isObstacle' in data:
            self.isObstacle = data['isObstacle']

    def setDataOld(self, data):
        """ Set the object's internal data.

            'data' is a copy of the object's saved data, as returned by
            previous version of region editor.  This function is only for
            backwards compatibility and will eventually be removed.
        """
        self.name = data[0]

        if data[1].lower() == "rect":
            self.type = reg_RECT
        else:
            self.type = reg_POLY

        self.position = Point(int(data[2]), int(data[3]))
        self.size = Size(int(data[4]), int(data[5]))
        self.color = Color(red=int(data[6]),
                                         green=int(data[7]),
                                          blue=int(data[8]))
        if self.type == reg_POLY:
            self.pointArray = []
            for i in range(9, len(data), 2):
                self.pointArray.append(Point(int(data[i]), int(data[i+1])))

        self.alignmentPoints = [False] * len([x for x in self.getPoints()])

    def getFaces(self,includeHole=False):
        """
        Wrapper function to allow for iteration over faces of regions.
        A face is a tuple of the two points (in absolute coordinates) that make up the face,
        sorted so that a given face is defined uniquely.

        FIXME: Make sure we take advantage of this uniqueness elsewhere; I think we check
        too many conditions sometimes
        """

        lastPt = None
        for pt in self.getPoints():

            thisPt = (pt.x, pt.y)

            if lastPt != None:
                yield tuple(sorted((lastPt, thisPt)))
            else:
                firstPt = thisPt

            lastPt = thisPt

        yield tuple(sorted((lastPt, firstPt))) # Closing face
        # also include edges of holes in the get faces for checking adjancency
        if includeHole:
            for i,hole in enumerate(self.holeList):
                lastPt = None
                for pt in self.getPoints(hole_id=i):
                    thisPt = (pt.x, pt.y)

                    if lastPt != None:
                        yield tuple(sorted((lastPt, thisPt)))
                    else:
                        firstPt = thisPt

                    lastPt = thisPt

                yield tuple(sorted((lastPt, firstPt))) # Closing face
                


    def getPoints(self, relative=False, hole_id=None):
        """
        Wrapper function to allow for iteration over the points of a region without
        worrying about whether it's a RECT or POLY.
        When hole_id is None, the boundary points of the region will be returned
        When Otherwise the points of holeList[hole_id] will be returned
        """

        if relative:
            offset = Point(0, 0)
        else:
            offset = self.position
        if self.type == reg_POLY:
            if hole_id == None:
                # using boundary of the region
                for pt in self.pointArray:
                    yield offset + pt
            else:
                # using holeList[hole_id]
                for pt in self.holeList[hole_id]:
                    yield offset + pt
        elif self.type == reg_RECT:
            for pt in [Point(0, 0),
                       Point(self.size.width, 0),
                       Point(self.size.width, self.size.height),
                       Point(0, self.size.height)]:
                yield offset + pt

    def getCenter(self):
        """
        Find the 'center' of a region
        """

        if self.type == reg_POLY:
            # Let's just average the points in the poly; that's easy
            asum = reduce(lambda a,b: a+b, self.pointArray)
            cx = asum.x / len(self.pointArray) + self.position.x
            cy = asum.y / len(self.pointArray) + self.position.y
        else:
            # Rectangles are even easier!
            cx = self.size.GetWidth()/2 + self.position.x
            cy = self.size.GetHeight()/2 + self.position.y
        
        return Point(cx, cy)

    # =======================
    # == Selection Methods ==
    # =======================

    def objectContainsPoint(self, x, y):
        """ Returns True iff this object contains the given point.

            This is used to determine if the user clicked on the object.
        """
        # Firstly, ignore any points outside of the object's bounds.

        if x < self.position.x: return False
        if x > self.position.x + self.size.x: return False
        if y < self.position.y: return False
        if y > self.position.y + self.size.y: return False

        if self.type in [reg_RECT]:
            # Rectangles and text are easy -- they're always selected if the
            # point is within their bounds.
            return True

        return self.polyContainsPoint(self.pointArray, x - self.position.x, y - self.position.y) and \
               not any([self.polyContainsPoint(h_pts, x - self.position.x, y - self.position.y) for h_pts in self.holeList])

    def polyContainsPoint(self, poly_pts, x, y):

        # For polygons, we have to check whether the clicked point is
        # inside or outside the polygon.
        # The algorithm used here was taken from:
        # http://local.wasp.uwa.edu.au/~pbourke/geometry/insidepoly/

        sum = 0
        n = len(poly_pts)
        for i in range(n):
            v1_y = poly_pts[i].y - y
            v1_x = poly_pts[i].x - x
            v2_y = poly_pts[(i+1)%n].y - y
            v2_x = poly_pts[(i+1)%n].x - x
            angle_v1 = math.atan2(v1_y, v1_x)
            angle_v2 = math.atan2(v2_y, v2_x)
            angle = angle_v2 - angle_v1
            while(angle > math.pi):
                angle -= 2 * math.pi
            while(angle < -math.pi):
                angle += 2 * math.pi
            sum += angle

        return not (abs(sum) < math.pi)

    def getSelectionHandleContainingPoint(self, x, y, boundFunc=None):
        """ Return the selection handle containing the given point, if any.

            We return one of the predefined selection handle ID codes (defined at top).
        """
        if boundFunc is None:
            boundFunc = self._pointInSelRect
        
        for index, pt in enumerate(self.getPoints()):
            if boundFunc(x, y, pt.x, pt.y):
                return index    # For rectangles, the index corresponds to the handle ID by definition
        return handle_NONE


    def objectWithinRect(self, x, y, width, height):
        """ Return True iff this object falls completely within the given rect.
        """
        if x          > self.position.x:                    return False
        if x + width  < self.position.x + self.size.width:  return False
        if y          > self.position.y:                    return False
        if y + height < self.position.y + self.size.height: return False
        return True

    # =====================
    # == Private Methods ==
    # =====================

    def _pointInSelRect(self, x, y, rX, rY):
        """ Return True iff (x, y) is within the selection handle at (rX, rY).
        """
        if   x < rX - 3: return False
        elif x > rX + 3: return False
        elif y < rY - 3: return False
        elif y > rY + 3: return False
        else:            return True

    def findPointsNear(self, face, center, distance):
        # find slope of the face line
        x1=face[0][0]
        y1=face[0][1]
        x2=face[1][0]
        y2=face[1][1]
        if x1 == x2: # vertical line
            if center[0]>x1:
                x1_new=x1-distance
                x2_new=x2-distance
            else:
                x1_new=x1+distance
                x2_new=x2+distance
            y1_new=y1
            y2_new=y2
        elif y1 == y2: # horizontal line
            if center[1]>y1:
                
                y1_new=y1-distance
                y2_new=y2-distance
            else:
                y1_new=y1+distance
                y2_new=y2+distance
            x1_new=x1
            x2_new=x2
        else:
            faceSlope=(y1-y2)/(x1-x2*1.0)
            # find slope that is orthogonal to the face
            orthSlope=(-1.0)/faceSlope
            
            # figure out which direction the boundary should be shifted to
            offsetX=distance*math.sqrt(1/(1+orthSlope**2))
            offsetY=distance*math.sqrt(1/(1+1/orthSlope**2))
            if orthSlope>0:
                direction1=math.sqrt((x1+offsetX-center[0])**2+(y1+offsetY-center[1])**2)
                direction2=math.sqrt((x1-offsetX-center[0])**2+(y1-offsetY-center[1])**2)
                if direction1>direction2:
                    x1_new=x1+offsetX
                    y1_new=y1+offsetY
                    x2_new=x2+offsetX
                    y2_new=y2+offsetY
                else:
                    x1_new=x1-offsetX
                    y1_new=y1-offsetY
                    x2_new=x2-offsetX
                    y2_new=y2-offsetY
            else:
                direction1=math.sqrt((x1+offsetX-center[0])**2+(y1-offsetY-center[1])**2)
                direction2=math.sqrt((x1-offsetX-center[0])**2+(y1+offsetY-center[1])**2)
                if direction1>direction2:
                    x1_new=x1+offsetX
                    y1_new=y1-offsetY
                    x2_new=x2+offsetX
                    y2_new=y2-offsetY
                else:
                    x1_new=x1-offsetX
                    y1_new=y1+offsetY
                    x2_new=x2-offsetX
                    y2_new=y2+offsetY
                    
        return Point(int(x1_new), int(y1_new)), Point(int(x2_new), int(y2_new))
        
    def findRegionNear(self, distance, mode='underEstimate',name='newRegion'):
        """
        Given a region object and a distance value, return a new region object which covers
        the area that is within the 'distance' away from the given region.
        """
        
        if distance<0:
            print "The distance cannot be negative."
            return
            
        newRegion = Region()
        # the new region will have most features  same as the old one
        newRegion.name = name
        newRegion.type              = reg_POLY
        newRegion.color             = Color(255-self.color[0], 255-self.color[1], 255-self.color[2])
        newRegion.pointArray        = []
        center=self.getCenter()
        
        if mode == 'overEstimate':
            for i,pt in enumerate(self.getPoints()):
                twoFaces = [face for face in self.getFaces() if pt in face] # faces that connected by pt
                        
                face1_pt1_new,face1_pt2_new = self.findPointsNear(twoFaces[0], center, distance)
                face2_pt1_new,face2_pt2_new = self.findPointsNear(twoFaces[1], center, distance)
                
                
                if math.sqrt((face1_pt1_new.x-pt.x)**2+(face1_pt1_new.y-pt.y)**2) > math.sqrt((face1_pt2_new.x-pt.x)**2+(face1_pt2_new.y-pt.y)**2):
                    pt1 = face1_pt2_new
                else:
                    pt1 = face1_pt1_new
                    
                if math.sqrt((face2_pt1_new.x-pt.x)**2+(face2_pt1_new.y-pt.y)**2) > math.sqrt((face2_pt2_new.x-pt.x)**2+(face2_pt2_new.y-pt.y)**2):
                    pt2 = face2_pt2_new
                else:
                    pt2 = face2_pt1_new
                    
                pt1_new, pt2_new = self.findPointsNear((pt1,pt2), center, distance-math.sqrt(distance**2-((pt1.x-pt2.x)**2+(pt1.y-pt2.y)**2)/4))
        
                #newRegion.pointArray.append(pt1_new)
                #newRegion.pointArray.append(pt2_new)
                
                newTwoFaces = [(face1_pt1_new,face1_pt2_new),
                               (face2_pt1_new,face2_pt2_new)]
                interPoint = self.faceAndFaceIntersection(newTwoFaces[0],tuple(sorted((pt1_new,pt2_new))))
                newRegion.pointArray.append(interPoint)
                interPoint = self.faceAndFaceIntersection(newTwoFaces[1],tuple(sorted((pt1_new,pt2_new))))
                newRegion.pointArray.append(interPoint)
                    
        # TODO: fix this
        # put the vertex in the right order
        poly = Polygon.Utils.convexHull(Polygon.Polygon(newRegion.pointArray))
        #poly = Polygon.Polygon(newRegion.pointArray)
        newRegion.pointArray = [Point(*x) for x in Polygon.Utils.pointList(poly)]
        newRegion.alignmentPoints   = [False] * len([x for x in newRegion.getPoints()])    

        newRegion.recalcBoundingBox()

        return newRegion
        
        
        
    def faceAndFaceIntersection(self,face1,face2):
        # http://www.topcoder.com/tc?module=Static&d1=tutorials&d2=geometry2
        a = [(face[1][1] - face[0][1]) for face in [face1, face2]]
        b = [(face[0][0] - face[1][0]) for face in [face1, face2]]
        c = [a[0]*face1[0][0]+b[0]*face1[0][1],
             a[1]*face2[0][0]+b[1]*face2[0][1]]

        det = a[0]*b[1] - a[1]*b[0]

        if(det == 0):
            print "Lines are parallel"
        else:
            x = float(b[1]*c[0] - b[0]*c[1])/det
            y = float(a[0]*c[1] - a[1]*c[0])/det

        return Point(x,y)
        
    #
    #    # calculate the functions of the lines at the faces
    #    # function is in format of y = Ax+b
    #    
    #    if face1[0][0] == face1[1][0]:
    #        x1=float(face2[0][0])
    #        y1=float(face2[0][1])
    #        x2=float(face2[1][0])
    #        y2=float(face2[1][1])
    #        line2_A = (y2-y1)/(x2-x1)
    #        line2_b = y1-x1*(y2-y1)/(x2-x1)
    #        
    #        interPoint = wx.Point(int(face1[0][0]),int(line2_A*face1[0][0]+line2_b))
    #    elif face2[0][0] == face2[1][0]:
    #        x1=float(face1[0][0])
    #        y1=float(face1[0][1])
    #        x2=float(face1[1][0])
    #        y2=float(face1[1][1])
    #        line1_A = (y2-y1)/(x2-x1)
    #        line1_b = y1-x1*(y2-y1)/(x2-x1)
    #        
    #        interPoint = wx.Point(int(face2[0][0]),int(line1_A*face2[0][0]+line1_b))
    #    else:
    #        x1=float(face1[0][0])
    #        y1=float(face1[0][1])
    #        x2=float(face1[1][0])
    #        y2=float(face1[1][1])
    #        line1_A = (y2-y1)/(x2-x1)
    #        line1_b = y1-x1*(y2-y1)/(x2-x1)
    #        
    #        x1=float(face2[0][0])
    #        y1=float(face2[0][1])
    #        x2=float(face2[1][0])
    #        y2=float(face2[1][1])
    #        line2_A = (y2-y1)/(x2-x1)
    #        line2_b = y1-x1*(y2-y1)/(x2-x1)
    #        
    #        interPoint_x = (line2_b-line1_b)/(line2_A-line1_A)
    #        interPoint = wx.Point(int(interPoint_x),int(line1_A*interPoint_x+line1_b))
    #        
    #    return interPoint


############################################################
def findRegionBetween(regionA, regionB,name = 'newRegion'):
    """
    Find the region between two given regions (doesn't include the given regions)
    """

    newRegion = Region()
    # the new region will have most features  same as the old one
    newRegion.name = name
    newRegion.type              = reg_POLY
    
    polyA = Polygon.Polygon([x for x in regionA.getPoints()])
    polyB = Polygon.Polygon([x for x in regionB.getPoints()])
    
    betw_AB = Polygon.Utils.convexHull(polyA+polyB)-polyA-polyB
    
    newRegion.pointArray = [Point(*x) for x in Polygon.Utils.pointList(betw_AB)]
    newRegion.alignmentPoints   = [False] * len([x for x in newRegion.getPoints()])    
    
    newRegion.recalcBoundingBox()

    return newRegion

def pointLineIntersection(pt1, pt2, test_pt):
    """
    Given two points (pt1, pt2), find the point on the line formed by those points that is nearest
    to test_pt and give the distance.
    """

    [x1, y1] = [pt1.x, pt1.y]
    [x2, y2] = [pt2.x, pt2.y]
    [xm, ym] = [test_pt.x, test_pt.y]
    if x1 == x2: # vertical line
        xi = x1
        yi = ym
        d = (xm - xi)**2
        on_segment = min(y1,y2) < yi < max(y1,y2)
    elif y1 == y2: # horizontal line
        yi = y1
        xi = xm
        d = (ym - yi)**2
        on_segment = min(x1,x2) < xi < max(x1,x2)
    else:
        c1 = 1.0*(y2 - y1)/(x2 - x1)
        c2 = 1.0*-(x2 - x1)/(y2 - y1)
        xi = (x1*c1 - xm*c2 - y1 + ym)/(c1 - c2)
        yi = y1 + c1*(xi - x1)
        d = (xm - xi)**2 + (ym - yi)**2
        on_segment = min(x1,x2) < xi < max(x1,x2) 

    return [on_segment, d, Point(xi, yi)]

