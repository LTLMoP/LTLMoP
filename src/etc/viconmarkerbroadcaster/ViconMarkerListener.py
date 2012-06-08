import time
import struct
import threading
import socket

class ViconMarkerListener(threading.Thread):
    def __init__(self, freq=20.0, ip="0.0.0.0", port=7500, parent=None):
        """Create the a socket to receive Vicon data.
        
        freq - Update frequency in Hz.
        ip - IP address to listen on.
             Default is local computer.
        port - Port to listen on.
               Default matches ViconMarkerBroadcaster (C#) default
        parent - Object that may be useful. Can change ProcessData method to
                 make it do something with this object when data is received.
        """
        super(ViconMarkerListener, self).__init__()

        # Communication parameters
        self.updateFreq = freq
        self.addr = (ip, port)
        self.bufsize = 65536
        self.udpSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lock = threading.Lock()
        self.close = threading.Event()

        # Container for marker positions
        self.poses = []         # List of tuples [(x1, y1), (x2, y2), ...]
        
        # Creator of this object
        self.parent = parent

    def run(self):
        """Open the socket to start communication. Process messages."""
        # Open socket for communication
        self.udpSock.bind(self.addr)

        # Receive communication until stopped
        self.close.clear()
        delay = 1.0 / self.updateFreq
        while not self.close.isSet():
            self.lock.acquire()
            data = self.udpSock.recv(self.bufsize)
            self.lock.release()
            self.ProcessData(data)
            time.sleep(delay)

        # Close socket
        self.udpSock.close()

    def stop(self):
        """Close the socket to end UDP communication."""
        self.close.set()
    
    # Deserialize and save data
    def ProcessData(self, data):
        """Extract marker positions and keep them.

        data - Byte array encoded from multiple pairs of doubles [x1 y1 ...]
        """
        # Check for valid data (not null or incomplete)
        if data and len(data)%16 == 0:
            self.poses = []
            for i in range(0, len(data), 16):
                x, y = struct.unpack('dd', data[i:i+16])
                self.poses.append((x, y))
            # If you want something to happen every time poses are recieved
            # put that in here. You may need to pass in parameter parent.

# Running by itself will just print out first 10 markers every second forever
if __name__ == "__main__":
    vicon = ViconMarkerListener()
    vicon.start()
    j = 0
    while (1):
        time.sleep(1)
        print "Vicon Markers:"
        for i in range(min(len(vicon.poses), 10)):
            print "(%.3f, %.3f)" % vicon.poses[i]
        print ""
    vicon.stop()