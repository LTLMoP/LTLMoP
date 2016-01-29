#!/usr/bin/env python
"""
=====================================
dummySensor.py - Dummy Sensor Handler
=====================================

Displays a silly little window for faking sensor values by clicking on buttons.
"""

import threading, subprocess, os, time, socket
import numpy, math
import sys

import lib.handlers.handlerTemplates as handlerTemplates

from __is_inside import *

class DummySensorHandler(handlerTemplates.SensorHandler):
    def __init__(self, executor, shared_data):
        """
        Start up sensor handler subwindow and create a new thread to listen to it.
        """

        # Since we don't want to have to poll the subwindow for each request,
        # we need a data structure to cache sensor states:
        self.sensorValue = {}
        self.proj = executor.proj
        self.sensorListenInitialized = False
        self._running = True
        self.p_sensorHandler = None
        self.executor = executor

    def _stop(self):
        if self.p_sensorHandler is not None:
            print >>sys.__stderr__, "(SENS) Killing dummysensor GUI..."
            self.p_sensorHandler.stdin.write(":QUIT\n")
            self.p_sensorHandler.stdin.close()

            print >>sys.__stderr__, "(SENS) Terminating dummysensor GUI listen thread..."
            self._running = False
            self.sensorListenThread.join()

    def _createSubwindow(self):
            # Create a subprocess
            print "(SENS) Starting sensorHandler window and listen thread..."
            self.p_sensorHandler = subprocess.Popen([sys.executable, "-u", os.path.join(self.proj.ltlmop_root,"lib","handlers","share","Sensor","_SensorHandler.py")], stdin=subprocess.PIPE)

            # Create new thread to communicate with subwindow
            self.sensorListenThread = threading.Thread(target = self._sensorListen)
            self.sensorListenThread.daemon = True
            self.sensorListenThread.start()

            # Block until the sensor listener gets the go-ahead from the subwindow
            while not self.sensorListenInitialized:
                time.sleep(0.05) # Yield cpu

    def regionBit(self,name,init_region,bit_num,initial=False):
        """
        Return the value of bit #bit_num in the bit-vector encoding of the currently selected region

        name (string): Unique identifier for region sensor (default="target")
        init_region (region): Name of the sensor whose state is interested
        bit_num (int): The index of the bit to return
        """
        if initial:
            if not self.sensorListenInitialized:
                self._createSubwindow()

            if name not in self.sensorValue.keys():
                # create a new map element
                # choose an initial (decomposed) region inside the desired one
                self.sensorValue[name] = self.proj.regionMapping[init_region][0]
                self.p_sensorHandler.stdin.write("loadproj," + self.proj.getFilenamePrefix() + ".spec,\n")
                self.p_sensorHandler.stdin.write(",".join(["region", name, self.sensorValue[name]]) + "\n")
            return True
        else:
            if name in self.sensorValue:
                reg_idx = self.proj.rfi.indexOfRegionWithName(self.sensorValue[name])
                numBits = int(math.ceil(math.log(len(self.proj.rfi.regions),2)))
                reg_idx_bin = numpy.binary_repr(reg_idx, width=numBits)
                #print name, bit_num, (reg_idx_bin[bit_num] == '1')
                return (reg_idx_bin[bit_num] == '1')
            else:
                print "(SENS) WARNING: Region sensor %s is unknown!" % button_name
                return None

    def buttonPress(self,button_name,init_value,initial=False):
        """
        Return a boolean value corresponding to the state of the sensor with name ``sensor_name``
        If such a sensor does not exist, returns ``None``

        button_name (string): Name of the sensor whose state is interested
        init_value (bool): The initial state of the sensor (default=False)
        """
        if initial:

            if not self.sensorListenInitialized:
                self._createSubwindow()

            if button_name not in self.sensorValue.keys():
                self.sensorValue[button_name] = init_value
                if init_value:
                    self.p_sensorHandler.stdin.write("button," + button_name + ",1\n")
                else:
                    self.p_sensorHandler.stdin.write("button," + button_name + ",0\n")
            return self.sensorValue[button_name]
        else:
            if button_name in self.sensorValue:
                return self.sensorValue[button_name]
            else:
                print "(SENS) WARNING: Sensor %s is unknown!" % button_name
                return None

    def _sensorListen(self):
        """
        Processes messages from the sensor handler subwindow, and updates our cache appropriately
        """
        host = 'localhost'
        port = 23459
        buf = 1024
        addr = (host,port)

        UDPSock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        UDPSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        UDPSock.settimeout(1)
        try:
            UDPSock.bind(addr)
        except:
            print "ERROR: Cannot bind to port.  Try killing all Python processes and trying again."
            return

        while self._running:
            # Wait for and receive a message from the subwindow
            try:
                input,addrFrom = UDPSock.recvfrom(1024)
            except socket.timeout:
                continue

            if input == '':  # EOF indicates that the connection has been destroyed
                print "(SENS) Sensor handler listen thread is shutting down."
                break

            # Check for the initialization signal, if necessary
            if not self.sensorListenInitialized and input.strip() == "Hello!":
                self.sensorListenInitialized = True
                continue

            # Get the data out of the message
            args = input.strip().split("=")

            if len(args) != 2:
                continue

            # Update our internal cache
            if args[1] == "True":
                self.sensorValue[args[0]] = True
            elif args[1] == "False":
                self.sensorValue[args[0]] = False
            else:
                self.sensorValue[args[0]] = args[1]

    def inRegion(self, regionName , initial = False):
        """
        Check if the robot is in this region
        regionName (string): Name of the region
        """    

        if initial:
            return True
        
        else: 
            pose = self.executor.hsub.coordmap_lab2map(self.executor.hsub.getPose())
            #########################################
            ### Copied from vectorController.py #####
            #########################################            

            regionNo = self.proj.rfiold.indexOfRegionWithName(regionName)
            pointArray = [x for x in self.proj.rfiold.regions[regionNo].getPoints()]
            pointArray = map(self.executor.hsub.coordmap_map2lab, pointArray)
            vertices = numpy.mat(pointArray).T 
            #print >>sys.__stdout__, self.proj.rfiold.regions[regionNo].name +": " +  str(is_inside([pose[0], pose[1]], vertices))    
            return is_inside([pose[0], pose[1]], vertices)
