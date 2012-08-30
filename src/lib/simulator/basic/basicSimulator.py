#!/usr/bin/env python
"""
================================================================
basicSimulator.py -- A simple robot simulator provides pose by integrating given locomotion cmd
================================================================
"""
from numpy import array,sqrt,dot
from math import atan2,log10,ceil
import time, sys
import thread

class basicSimulator:
    def __init__(self, init_pose):
        """
        Initialization handler for pioneer ode simulated robot.

        init_pose is a 1-by-3 vector [x,y,orintation]
        """

        print "(Basic Simulator) Initializing Basic Simulator..."
        self.pose = array(init_pose) # current pose
        self.curVel = array([0.0,0.0]) # current velocity
        self.time = 0.0 # used to calculate time elapsed
        self.inertia = 1 # scale from 0 to 1, the bigger the scale the smaller the "inertia" is
        self.setVel_called = False

        # Choose a timer func with maximum accuracy for given platform
        if sys.platform in ['win32', 'cygwin']:
            self.timer_func = time.clock
        else:
            self.timer_func = time.time

        print "(Basic Simulator) Start Basic Simulator..."
        thread.start_new_thread(self.runSimulation, () )

    def setVel(self,cmd):
        """
        Set the velocity of the robot, update the pose by simply inegrate the velocity

        cmd is a 1-by-2 vector represents the velocity
        """

        # the orintation is kept the same (rad)
        # TODO: allows more robot models
        # update the velocity, assume the velocity takes times to change (to avoid local minimum)
        self.curVel = self.inertia*array(cmd)+(1-self.inertia)*self.curVel
        self.setVel_called = True

    def runSimulation(self):
        if self.time == 0.0:
            self.time = self.timer_func()
        while 1:
            if self.setVel_called:
                time_span = (self.timer_func()-self.time)
                time_span = time_span*10**ceil(log10(0.03/time_span))
                vel = array(self.curVel)*time_span
                self.pose[0:2] = self.pose[0:2]+vel
                self.setVel_called=False
            else:
                self.pose[0:2] = self.pose[0:2]+array([0.0,0.0])*(self.timer_func()-self.time)
            self.time = self.timer_func()
            time.sleep(0.1)

    def getPose(self):
        """
        Returns the current pose of the robot
        """
        return self.pose
