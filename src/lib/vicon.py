import time
import threading
import socket
import struct
from regionsPoints import Region, Point

class ViconMarkerListener(threading.Thread):
    def __init__(self, parent, trackMarkers=False):
        """Create the a socket to receive Vicon data.
        
        When processing the pose data, this thread will set the field
        parent.markerPoses with the new marker positions and will call the
        function parent.RedrawVicon().
        
        trackMarkers indicates whether the moving markers should be identified
        and retained (mostly used for audio feedback).
        """
        super(ViconMarkerListener, self).__init__()

        # Communication parameters
        self.parent = parent        # regionEditor or CalibrationFrame
        self.addr = ("0.0.0.0", 7500)
        self.bufsize = 65536
        self.udpSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lock = threading.Lock()
        self.close = threading.Event()
        self.updateFreq = 20    # Hz

        # Tracking parameters
        # TODO: Set all paremeters in configuration of GUI
        self.minDist = 0.01     # Distance to move to qualify as movement
        self.maxDist = 0.1      # Qualified as new point
        self.movMaxTime = 15 * self.updateFreq  # (sec)*(Hz)
                                # Iterations during which marker is not moving
                                # before stopping tracking
        self.invMaxTime = 1 * self.updateFreq   # (sec)*(Hz)
                                # Iterations during which marker is not found
                                # before stopping tracking

        # Tracking containers
        self.trackMarkers = trackMarkers
                                # Flag indicating if moving poses should be 
                                # looked for during processing
        self.oldPoses = []      # Previous marker positions
        self.movingPoses = []   # Markers that are moving
        self.movingTimeout = [] # Iterations left before movement timeout
        self.invisTimeout = []  # Iteractions left before not-found timeout

    def run(self):
        """Open the socket to start communication. Process messages."""
        # Open socket for communication
        self.udpSock.bind(self.addr)

        # Receive communication until stopped
        self.close.clear()
        delay = 1 / self.updateFreq
        while not self.close.isSet():
            self.lock.acquire()
            data = self.udpSock.recv(self.bufsize)
            self.lock.release()
            self.ProcessData(data)
            time.sleep(delay)

        # Close socket
        self.udpSock.close()
        self.oldPoses = []
        self.movingPoses = []
        self.movingTimeout = []
        self.invisTimeout = []
    
    def stop(self):
        """Close the socket to end UDP communication."""
        self.close.set()
    
    def ProcessData(self, data):
        """Extract marker positions and pass them on to be mapped.
        
        data - Byte array encoded from multiple pairs of doubles [x1 y1 ...]
        """
        # Check for valid data (not null or incomplete)
        if data and len(data)%16 == 0:
            poses = []
            for i in range(0, len(data), 16):
                x, y = struct.unpack('dd', data[i:i+16])
                poses.append(Point(x, y))
            # Save and plot marker positions
            # Both regionEditor GUI and CalibrationFrame GUI have
            # markerPoses field and RedrawVicon method
            self.parent.markerPoses = poses
            if self.trackMarkers:
                self.UpdateMovingMarkers(poses)
            self.parent.RedrawVicon()      # Force map redraw

    def UpdateMovingMarkers(self, poses):
        """Note which markers are newly moving and update position of old
        moving markers based on change in position from previous list.

        poses - List of Points, marker positions.
        """
        # Not first time through
        if self.oldPoses:
            # Check all currently tracked markers first
            for iMarker, markerPose in enumerate(self.movingPoses):
                closestIdx = self.FindClosest(poses, markerPose)
                closestDist = markerPose.Dist(poses[closestIdx])
                # Same marker
                if closestDist < self.maxDist:
                    self.invisTimeout[iMarker] = self.invMaxTime
                    # Is moving
                    if closestDist > self.minDist:
                        self.movingTimeout[iMarker] = self.movMaxTime + 1
                    self.movingPoses[iMarker] = poses[closestIdx]
                # Marker not seen
                else:
                    self.invisTimeout[iMarker] -= 1
                self.movingTimeout[iMarker] -= 1
                # Marker has stopped moving or not been seen for some time
                if self.invisTimeout[iMarker] == 0 or \
                        self.movingTimeout[iMarker] == 0:
                    self.invisTimeout.pop(iMarker)
                    self.movingTimeout.pop(iMarker)
                    self.movingPoses.pop(iMarker)
            # Check through all new markers next
            for markerPose in poses:
                closestIdx = self.FindClosest(self.oldPoses, markerPose)
                closestDist = markerPose.Dist(self.oldPoses[closestIdx])
                # Point is moving
                if closestDist < self.maxDist and closestDist > self.minDist:
                    self.movingPoses.append(markerPose)
                    self.movingTimeout.append(self.movMaxTime)
                    self.invisTimeout.append(self.invMaxTime)
        self.oldPoses = poses

    def FindClosest(self, points, target):
        """Find the closest point to the target.

        points - List of Points, marker positions.
        target - Point, point to find marker closest to.
        returns - Integer, index of closest point in points.
        """
        # Iterate through all points
        minDist = float('inf')
        minIdx = 0
        for i, pt in enumerate(points):
            dist = target.Dist(pt)
            if dist < minDist:
                minIdx = i
                minDist = dist
        return minIdx
# end of class ViconMarkerListener