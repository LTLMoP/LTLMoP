import math
import random
import wx

class Region:
    def __init__(self, points=[], name="", holes=[], rgb=None, obst=False):
        """Create an object to represent a region.

        points - List of Points containing vertex information
                 [Point(x1, y1), Point(x2, y2), ...]
        name - String defining region name
        holes - List of lists of points representing holes cut in the region
        rgb - List of integers defining color
              [red, green, blue], each with value in range [0 255]
        obst - Boolean, True indicates that the region is an obstacle
        """
        # TODO: Add convex/concave
        self.pointArray = points
        self.name = name
        self.holeList = holes
        if not rgb:
            rgb = [random.randint(0, 255), random.randint(0, 255), \
                random.randint(0,255)]
        self.color = wx.Colour(rgb[0], rgb[1], rgb[2])
        self.isObstacle = obst
    
    def __str__(self):
        """Representation of object."""
        s = "%s\t{%03d\t%03d\t%03d}\t[" % (self.name, self.color.Red(), \
            self.color.Green(), self.color.Blue())
        for iPt, pt in enumerate(self.pointArray):
            s += str(pt)
            if iPt < len(self.pointArray) - 1:
                s += "\t"
        s += "]"
        return s
    
    def getData(self):
        """Return a copy of the object's internal data.
        This is used to save this region to disk.
        """

        data = {'name': self.name, 'color': (self.color.Red(), \
            self.color.Green(), self.color.Blue())}
        
        data['points'] = [(pt.x, pt.y) for pt in self.pointArray]
        
        data['holeList'] = []
        for hole in self.holeList:
            data['holeList'].append([(pt.x, pt.y) for pt in hole])

        data['isObstacle'] = self.isObstacle

        return data

    def setData(self, data):
        """Set the object's internal data.
        
        'data' is a copy of the object's saved data, as returned by
        getData() above. This is used to restore a previously saved region.
        """
        
        self.name = data['name']
        self.color = wx.Colour(*data['color'])
        
        self.pointArray = [Point(*pt) for pt in data['points']]
        
        if 'holeList' in data:
            self.holeList = []
            for hole in data['holeList']:
                self.holeList.append([Point(*pt) for pt in hole])
        
        if 'isObstacle' in data:
            self.isObstacle = data['isObstacle']
    
    def PtInRegion(self, pt):
        """Check if a point is inside of the region.
        Algorithm taken from C# version of Solution 1 from
        http://local.wasp.uwa.edu.au/~pbourke/geometry/insidepoly/
        
        pt - Point to check
        returns - Boolean, True if the point is inside of the region
        """
        result = False
        n = len(self.pointArray)
        for i in range(n):
            j = (i + 1) % n
            if ((self.pointArray[j].y <= pt.y and pt.y < self.pointArray[i].y) or \
                    (self.pointArray[i].y <= pt.y and pt.y < self.pointArray[j].y)) and \
                    pt.x < (self.pointArray[i].x - self.pointArray[j].x) * \
                    (pt.y - self.pointArray[j].y) / \
                    (self.pointArray[i].y - self.pointArray[j].y) + self.pointArray[j].x:
                result = not result
        return result
# end of class Region


class Point:
    def __init__(self, x, y):
        """Create an object that allows floating point vector operations.

        x - Float, first coordinate of point
        y - Float, second coordinate of point
        """
        if isinstance(x, float) and isinstance(y, float):
            self.x = x
            self.y = y
        else:
            raise TypeError('\'Point\' creation takes only \'float\' values')

    def __str__(self):
        """Representation of object."""
        return '(%.3f\t%.3f)' % (self.x, self.y)

    def __hash__(self):
        """Hashtable representation of object."""
        tup = (self.x, self.y)
        return tup.__hash__()

    def __eq__(self, other):
        """Checks equality (self == other)."""
        return isinstance(other, Point) and self.x == other.x and self.y == other.y

    def __ne__(self, other):
        """Checks non-equality (self != other)."""
        return not self.__eq__(other)

    def __add__(self, other):
        """Addition operator (self + other)."""
        if isinstance(other, Point):
            return Point(self.x + other.x, self.y + other.y)
        elif isinstance(other, tuple) and len(other) == 2:
            return Point(self.x + other[0], self.y + other[1])
        else:
            raise TypeError('cannot add \'Point\' and \'' + \
                other.__class__.__name__ + '\' objects')

    def __radd__(self, other):
        """Right addition operator (other + self)."""
        return self.__add__(other)

    def __sub__(self, other):
        """Subtraction operator (self - other)."""
        if isinstance(other, Point):
            return Point(self.x - other.x, self.y - other.y)
        elif isinstance(other, tuple) and len(other) == 2:
            return Point(self.x - other[0], self.y - other[1])
        else:
            raise TypeError('cannot subtract \'Point\' and \'' + \
                other.__class__.__name__ + '\' objects')

    def __rsub__(self, other):
        """Right subtraction operator (other - self)."""
        if isinstance(other, Point):
            return Point(other.x - self.x, other.y - self.y)
        elif isinstance(other, tuple) and len(other) == 2:
            return Point(other[0] - self.x, other[1] - self.y)
        else:
            raise TypeError('cannot subtract \'Point\' and \'' + \
                other.__class__.__name__ + '\' objects')

    def __mul__(self, other):
        """Multiplication operator (self * other)."""
        if isinstance(other, float):
            return Point(self.x * other, self.y * other)
        else:
            raise TypeError('cannot multiply \'Point\' and \'' + \
                other.__class__.__name__ + '\' objects')

    def __rmul__(self, other):
        """Right multiplication operator (other * self)."""
        return self.__mul__(other)

    def __div__(self, other):
        """Division operator (self / other)."""
        if isinstance(other, float):
            return Point(self.x / other, self.y / other)
        else:
            raise TypeError('cannot divide \'Point\' and \'' + \
                other.__class__.__name__ + '\' objects')

    def __truediv__(self, other):
        """True division operator."""
        return self.__div__(other)
    
    def Set(self, x, y):
        """Change the value of the point."""
        self.x = x
        self.y = y

    def Dot(self, other):
        """Dot product."""
        if isinstance(other, Point):
            return self.x * other.x + self.y * other.y
        elif isinstance(other, tuple) and len(other) == 2:
            return self.x * other[0] + self.y * other[1]
        else:
            raise TypeError('cannot compute dot product of \'Point\' and \'' + \
                other.__class__.__name__ + '\' object')

    def Dist(self, other):
        """Euclidean distance from this point to the other."""
        if isinstance(other, Point):
            return math.sqrt((self.x - other.x) ** 2 + \
                (self.y - other.y) ** 2)
        elif isinstance(other, tuple) and len(other) == 2:
            return math.sqrt((self.x - other[0]) ** 2 + \
                (self.y - other[1]) ** 2)
        else:
            raise TypeError('cannot compute distance from \'Point\' to \'' + \
                other.__class__.__name__ + '\' object')

    def Norm(self):
        """Length of the vector."""
        return math.sqrt(self.x ** 2 + self.y ** 2)
# end of class Point